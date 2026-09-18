#include "Indirect/LoopCloser.h"
#include "Indirect/MapPoint.h"
#include "Indirect/Frame.h"
#include "Indirect/Map.h"
#include "Indirect/Matcher.h"
#include "Indirect/Optimizer.h"
#include "Indirect/Sim3Solver.h"

#include "FullSystem/FullSystem.h"
#include <iostream>
#include <cmath>


namespace HSLAM {

    LoopCloser::LoopCloser(FullSystem *fullsystem) : fullSystem(fullsystem)
    {
        globalMap = fullSystem->globalMap;
        wpmatcher = fullSystem->matcher;
        mainLoop = boost::thread(&LoopCloser::Run, this);
        currMaxMp = 0;
        currMaxKF = 0;
        minActId = 0;
        lc_Vocabpnt = nullptr;
    }

    void LoopCloser::InsertKeyFrame(shared_ptr<Frame> &frame, int maxMpId)
    {
        std::vector<std::shared_ptr<MapPoint>> curActMP;
        std::vector<std::shared_ptr<Frame>> curActKF;

        boost::unique_lock<boost::mutex> lock(mutexKFQueue);
        frame->SetNotErase();
        copyActiveMapData(curActKF, curActMP);
        KFqueue.push_back(std::make_tuple(frame, curActKF, curActMP, frame->fs->KfId, maxMpId));
    }

    void LoopCloser::Run() {
        finished = false;

        while (1) {

            if (needFinish) {break; }

            {
                // get the oldest one
                boost::unique_lock<boost::mutex> lock(mutexKFQueue);
                if (KFqueue.empty()) {
                    lock.unlock();
                    usleep(5000);
                    continue;
                }
                
                if (KFqueue.size() > 5)
                { //can happen if optimization took too long!! in this case just add the accumulated kfs to the database and move on
                    
                    for (auto it : KFqueue)
                    {
                        auto frame = std::get<0>(it);
                        frame->ComputeBoVW(lc_Vocabpnt);
                        globalMap.lock()->KfDB->add(frame);
                        frame->SetErase(); //allow mapper to erase it if deemed it not useful already!
                    }
                        
                    KFqueue.clear();
                    continue;
                }

                //copy a snapshot of active frames and mapPoints when the candidate was inserted!! use the max KFIds to keep out all new data from the loop closure process (newer MapPoints or Kfs should be helf fixed to preseve gauge!)
                std::tie(currentKF, ActiveFrames, ActivePoints, currMaxKF, currMaxMp) = KFqueue.front();
                
                currentKF->ComputeBoVW(lc_Vocabpnt);
                KFqueue.pop_front(); 

            }

            bool loopDetected = DetectLoop();
            if (loopDetected)
            {

                if (computeSim3())
                {
                    static Timer loopCorrTime("loopCorr");
                    loopCorrTime.startTime();
                    auto gMap = globalMap.lock();
                    if (gMap->isIdle()) //prevent from doing a loop closure correction when another is taking place!
                    {
                        gMap->setBusy(true);
                        CorrectLoop();
                        gMap->setBusy(false);
                    }
                    loopCorrTime.endTime(true);
                }
            }

            usleep(5000);
        }

        finished = true;
    }

    void LoopCloser::lc_setVocab(DBoW3::Vocabulary* _Vocabpnt){
        lc_Vocabpnt = _Vocabpnt;
    }

    DBoW3::Vocabulary* LoopCloser::getVocab()
    {;
        return lc_Vocabpnt;
    }

    void LoopCloser::copyActiveMapData(std::vector<std::shared_ptr<Frame>> & _KFs ,std::vector<std::shared_ptr<MapPoint>> & _MPs)
    {
        // boost::unique_lock<boost::mutex> lock(fullSystem->mapMutex);
        auto gMap = globalMap.lock();
        minActId = UINT_MAX;
        for (int i = 0, iend = fullSystem->frameHessians.size(); i < iend; ++i)
        {
            std::shared_ptr<Frame> actKF = fullSystem->frameHessians[i]->shell->frame;
            _KFs.push_back(actKF);

            std::vector<std::shared_ptr<MapPoint>> kfPts = actKF->getMapPointsV();
            for (int j = 0, jend = kfPts.size(); j < jend; ++j)
            {
                if (!kfPts[j])
                    continue;
                if(kfPts[j]->isBad() || kfPts[j]->getDirStatus() != MapPoint::active)
                    continue;
                _MPs.push_back(kfPts[j]);
            }

            if(actKF->fs->KfId < minActId)
                minActId = actKF->fs->KfId;
        }
    }


    bool LoopCloser::DetectLoop()
    {

        auto gMap = globalMap.lock();
        //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
        if (currentKF->fs->KfId < mLastLoopKFid + kfGap)
        {
            gMap->KfDB->add(currentKF);
            currentKF->SetErase();
            return false;
        }

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        const std::vector<std::shared_ptr<Frame>> vpConnectedKeyFrames = currentKF->GetVectorCovisibleKeyFrames();
        const DBoW3::BowVector &CurrentBowVec = currentKF->mBowVec;
        float minScore = 1;
        for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++)
        {
            std::shared_ptr<Frame> pKF = vpConnectedKeyFrames[i];
            if (pKF->isBad())
                continue;
            const DBoW3::BowVector &BowVec = pKF->mBowVec;

            float score = lc_Vocabpnt->score(CurrentBowVec, BowVec);

            if (score < minScore)
                minScore = score;
        }

        // Query the database imposing the minimum score
        std::vector<std::shared_ptr<Frame>> vpCandidateKFs = gMap->KfDB->DetectLoopCandidates(currentKF, minScore);
        // If there are no loop candidates, just add new keyframe and return false
        if (vpCandidateKFs.empty())
        {
            gMap->KfDB->add(currentKF);
            mvConsistentGroups.clear();
            currentKF->SetErase();
            return false;
        }

        // For each loop candidate check consistency with previous loop candidates
        // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
        // A group is consistent with a previous group if they share at least a keyframe
        // We must detect a consistent loop in several consecutive keyframes to accept it
        mvpEnoughConsistentCandidates.clear();

        std::vector<ConsistentGroup> vCurrentConsistentGroups;
        std::vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);
        for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++)
        {
            std::shared_ptr<Frame> pCandidateKF = vpCandidateKFs[i];

            std::set<std::shared_ptr<Frame>, std::owner_less<std::shared_ptr<Frame>>> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
            spCandidateGroup.insert(pCandidateKF);

            bool bEnoughConsistent = false;
            bool bConsistentForSomeGroup = false;
            for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; iG++)
            {
                std::set<std::shared_ptr<Frame>,std::owner_less<std::shared_ptr<Frame>>> sPreviousGroup = mvConsistentGroups[iG].first;

                bool bConsistent = false;
                for (std::set<std::shared_ptr<Frame>,std::owner_less<std::shared_ptr<Frame>>>::iterator sit = spCandidateGroup.begin(), send = spCandidateGroup.end(); sit != send; sit++)
                {
                    if (sPreviousGroup.count(*sit))
                    {
                        bConsistent = true;
                        bConsistentForSomeGroup = true;
                        break;
                    }
                }

                if (bConsistent)
                {
                    int nPreviousConsistency = mvConsistentGroups[iG].second;
                    int nCurrentConsistency = nPreviousConsistency + 1;
                    if (!vbConsistentGroup[iG])
                    {
                        ConsistentGroup cg = std::make_pair(spCandidateGroup, nCurrentConsistency);
                        vCurrentConsistentGroups.push_back(cg);
                        vbConsistentGroup[iG] = true; //this avoid to include the same group more than once
                    }
                    if (nCurrentConsistency >= mnCovisibilityConsistencyTh && !bEnoughConsistent)
                    {
                        mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                        bEnoughConsistent = true; //this avoid to insert the same candidate more than once
                    }
                }
            }

            // If the group is not consistent with any previous group insert with consistency counter set to zero
            if (!bConsistentForSomeGroup)
            {
                ConsistentGroup cg = std::make_pair(spCandidateGroup, 0);
                vCurrentConsistentGroups.push_back(cg);
            }
        }

        // Update Covisibility Consistent Groups
        mvConsistentGroups = vCurrentConsistentGroups;

        // Add Current Keyframe to database
        gMap->KfDB->add(currentKF);

        if (mvpEnoughConsistentCandidates.empty())
        {
            currentKF->SetErase();
            return false;
        }
        else
        {
            return true;
        }

        currentKF->SetErase();
        return false;
    }

    bool LoopCloser::computeSim3()
    {
        const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

        auto matcher = wpmatcher.lock();

        std::vector<std::shared_ptr<Sim3Solver>> vpSim3Solvers;
        vpSim3Solvers.resize(nInitialCandidates);

        std::vector<std::vector<std::shared_ptr<MapPoint> >> vvpMapPointMatches;
        vvpMapPointMatches.resize(nInitialCandidates);

        std::vector<bool> vbDiscarded;
        vbDiscarded.resize(nInitialCandidates);

        int nCandidates = 0; //candidates with enough matches

        for (int i = 0; i < nInitialCandidates; i++)
        {
            std::shared_ptr<Frame> pKF = mvpEnoughConsistentCandidates[i];

            // avoid that local mapping erase it while it is being processed in this thread
            pKF->SetNotErase();

            if (pKF->isBad())
            {
                vbDiscarded[i] = true;
                continue;
            }

            std::vector<std::shared_ptr<MapPoint>> matches;
            int nmatches = matcher->SearchByBow(pKF, currentKF , 0.75, true, vvpMapPointMatches[i]); //0.75 def currentKF, pKF
            if (nmatches < 20)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                std::shared_ptr<Sim3Solver> pSolver = std::make_shared<Sim3Solver>(pKF, currentKF, vvpMapPointMatches[i], false); // def currentKF, pKF
                pSolver->SetRansacParameters(0.99, 20, 300);
                vpSim3Solvers[i] = pSolver;
            }

            nCandidates++;
        }

        bool bMatch = false;
        
        // Perform alternatively RANSAC iterations for each candidate
        // until one is succesful or all fail
        while (nCandidates > 0 && !bMatch)
        {
            for (int i = 0; i < nInitialCandidates; i++)
            {
                if (vbDiscarded[i])
                    continue;

                std::shared_ptr<Frame> pKF = mvpEnoughConsistentCandidates[i];

                // Perform 5 Ransac Iterations
                std::vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                std::shared_ptr<Sim3Solver> pSolver = vpSim3Solvers[i];
                cv::Mat Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
                
                // If Ransac reachs max. iterations discard keyframe
                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                if (!Scm.empty())
                {
                    std::vector<std::shared_ptr<MapPoint>> vpMapPointMatches(vvpMapPointMatches[i].size(), nullptr);
                    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++)
                    {
                        if (vbInliers[j])
                            vpMapPointMatches[j] = vvpMapPointMatches[i][j];
                    }

                    Mat33f R = pSolver->GetEstimatedRotation();
                    Vec3f t = pSolver->GetEstimatedTranslation();
                    const float s = pSolver->GetEstimatedScale();
                    if(s < 0 )
                    {
                        vbDiscarded[i] = true;
                        nCandidates--;
                        continue;
                    }

                    int numatches = matcher->SearchBySim3(pKF, currentKF, vpMapPointMatches, s, R, t, 7.5); //def: currentKf, pKF
                    // cv::Mat Output;
                    // cv::hconcat(pKF->Image, currentKF->Image, Output);
                    // cv::cvtColor(Output, Output, CV_GRAY2RGB);
                    // static int count = 0;
                    // count = 0;
                    // for (int j = 0, jend = pKF->nFeatures; j < jend; ++j)
                    // {
                    //     if (vpMapPointMatches[j])
                    //     {
                    //         cv::Point2f Pt1 = pKF->mvKeys[j].pt;
                    //         cv::Point2f Pt2 = currentKF->mvKeys[vpMapPointMatches[j]->getIndexInKF(currentKF)].pt + cv::Point2f(640, 0);
                    //         cv::circle(Output, Pt1, 1, cv::Scalar(0, 255, 0), -1);
                    //         cv::circle(Output, Pt2, 1, cv::Scalar(0, 255, 0), -1);
                    //         cv::line(Output, Pt1, Pt2, cv::Scalar(255, 0, 0));
                    //         count++;
                    //     }
                    // }

                    // cv::namedWindow("matches", cv::WINDOW_KEEPRATIO);
                    // cv::imshow("matches", Output);

                    // cv::waitKey(1);
                    // std::cout << "candid: " << i << " inliers: " << numatches << " count " << count << std::endl;

                    Sim3 gScm = Sim3(SE3(R.cast<double>(), t.cast<double>()).matrix());
                    gScm.setScale(s);

                    // Indirect.H1 (May 8, 2026): optionally seed OptimizeSim3 scale with ML-derived blend.
                    // s_seed = alpha * s_RANSAC + (1 - alpha) * s_ml; alpha=1.0 (default) is no-op identity.
                    // Per lit-audit reframing as a *seed-sensitivity diagnostic* (plan §6.2), we sweep alpha
                    // ∈ {0, 0.3, 0.5, 0.7, 0.9, 1.0} and measure |s_post_optimize - s_RANSAC| / s_RANSAC. Predicted
                    // null per LM convergence theory: RANSAC inlier sets typically place s_RANSAC near the inlier-
                    // energy minimum, and LM should converge to the same minimum regardless of starting scale.
                    // Uses new ?: old fallback identical to H2's policy. Only fires when seed has data (n>=5).
                    float s_h1_seed_used = -1.f, s_h1_ml_used = -1.f;
                    if (setting_indirectSim3MlSeed) {
                        std::vector<float> h1_ratios_new, h1_ratios_old;
                        const bool haveBothMl = pKF->mlDepthImage && !pKF->mlDepthImage->empty()
                            && currentKF->mlDepthImage && !currentKF->mlDepthImage->empty();
                        for (size_t j = 0; j < vpMapPointMatches.size(); j++) {
                            auto mpCur = vpMapPointMatches[j];
                            if (!mpCur) continue;
                            // Old fallback: source-frame MP idepth ratios
                            auto mpCand = pKF->getMapPoint(j);
                            if (mpCand && mpCur->getHasMLDepth() && mpCand->getHasMLDepth()
                                && mpCur->getMLIdepth() > 0 && mpCand->getMLIdepth() > 0)
                                h1_ratios_old.push_back(mpCand->getMLIdepth() / mpCur->getMLIdepth());
                            // New: per-pixel current-KF ML depth at matched features
                            if (haveBothMl) {
                                const int idxCur = mpCur->getIndexInKF(currentKF);
                                if (idxCur >= 0 && idxCur < currentKF->nFeatures && j < (size_t)pKF->nFeatures) {
                                    const cv::Point2f& ppKF = pKF->mvKeys[j].pt;
                                    const cv::Point2f& pCur = currentKF->mvKeys[idxCur].pt;
                                    const int yp = (int)ppKF.y, xp = (int)ppKF.x;
                                    const int yc = (int)pCur.y, xc = (int)pCur.x;
                                    if (yp >= 0 && yp < pKF->mlDepthImage->rows && xp >= 0 && xp < pKF->mlDepthImage->cols
                                        && yc >= 0 && yc < currentKF->mlDepthImage->rows && xc >= 0 && xc < currentKF->mlDepthImage->cols) {
                                        const float d_pKF = pKF->mlDepthImage->at<float>(yp, xp);
                                        const float d_cur = currentKF->mlDepthImage->at<float>(yc, xc);
                                        if (std::isfinite(d_pKF) && std::isfinite(d_cur) && d_pKF > 0.f && d_cur > 0.f)
                                            h1_ratios_new.push_back(d_cur / d_pKF);
                                    }
                                }
                            }
                        }
                        std::vector<float>* chosen = nullptr; const char* h1_src = "none";
                        if (h1_ratios_new.size() >= 5) { chosen = &h1_ratios_new; h1_src = "new"; }
                        else if (h1_ratios_old.size() >= 5) { chosen = &h1_ratios_old; h1_src = "old"; }
                        if (chosen) {
                            std::sort(chosen->begin(), chosen->end());
                            const float s_ml_h1 = (*chosen)[chosen->size() / 2];
                            const float alpha = setting_indirectSim3MlSeedAlpha;
                            const float s_blend = alpha * s + (1.0f - alpha) * s_ml_h1;
                            printf("[INDIRECT.SIM3_SEED] cur=%d cand=%d s_ransac=%.3f s_ml=%.3f source=%s alpha=%.2f s_seed=%.3f n=%zu\n",
                                   (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                                   s, s_ml_h1, h1_src, alpha, s_blend, chosen->size());
                            gScm.setScale((double)s_blend);
                            s_h1_seed_used = s_blend; s_h1_ml_used = s_ml_h1;
                        }
                    }

                    const int nInliers = OptimizeSim3(pKF, currentKF, vpMapPointMatches, gScm, 10, false); //def: currentKf, pKF
                    // If optimization is succesful stop ransacs and continue
                    if (nInliers >= 30) //20
                    {
                        const float s_optimized = gScm.scale();

                        // Indirect.H1 sensitivity diagnostic: how much did the seed actually move s_post?
                        // If LM is convergent (predicted), |s_post - s_RANSAC| / s_RANSAC < 1% across all alpha.
                        if (setting_indirectSim3MlSeed && s_h1_seed_used > 0.f) {
                            const float sensitivity = std::abs(s_optimized - s) / std::max(s, 1e-6f);
                            printf("[INDIRECT.SIM3_SEED] cur=%d cand=%d s_seed=%.3f s_post=%.3f s_ransac=%.3f sensitivity=%.4f (vs_ransac=%.2f%%)\n",
                                   (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                                   s_h1_seed_used, s_optimized, s, sensitivity, sensitivity * 100.0f);
                        }

                        // Indirect.H0 (May 7, 2026): compute s_ml two ways for comparison.
                        // - s_ml_old: legacy ratio of source-frame MapPoint ML idepths (semantically conflated; see audit R2).
                        // - s_ml_new: ratio of current-KF ML depth images at matched feature pixels (correct inter-KF scale,
                        //   indexing convention: vpMapPointMatches indexed by pKF feature; the stored MapPoint is currentKF's
                        //   matched MP, located in currentKF via getIndexInKF). Direction matches legacy: candidate→current,
                        //   i.e. d_currentKF / d_pKF (in idepth: i_pKF / i_currentKF).
                        // The SML_COMPARE diagnostic always prints both regardless of any flag so we can audit before
                        // shipping the new estimator. The P2 rejection gate (when enabled) consumes whichever the
                        // setting_indirectMlSemanticFix flag selects (default: new).
                        std::vector<float> ml_ratios_old;
                        std::vector<float> ml_ratios_new;

                        const bool haveBothMlImages = pKF->mlDepthImage && !pKF->mlDepthImage->empty()
                            && currentKF->mlDepthImage && !currentKF->mlDepthImage->empty();

                        for (size_t j = 0; j < vpMapPointMatches.size(); j++) {
                            auto mpCurrent = vpMapPointMatches[j];   // MP stored on currentKF, matched to pKF feature j
                            auto mpCandidate = pKF->getMapPoint(j);  // MP stored on pKF at feature j

                            // Legacy s_ml: source-frame idepth ratio
                            if (mpCurrent && mpCandidate &&
                                mpCurrent->getHasMLDepth() && mpCandidate->getHasMLDepth() &&
                                mpCurrent->getMLIdepth() > 0 && mpCandidate->getMLIdepth() > 0) {
                                ml_ratios_old.push_back(mpCandidate->getMLIdepth() / mpCurrent->getMLIdepth());
                            }

                            // New s_ml: per-pixel ML depth at the matched features in each KF's own ML image.
                            if (haveBothMlImages && mpCurrent) {
                                const int idxCur = mpCurrent->getIndexInKF(currentKF);
                                if (idxCur >= 0 && idxCur < currentKF->nFeatures &&
                                    j < (size_t)pKF->nFeatures) {
                                    const cv::Point2f& ppKF = pKF->mvKeys[j].pt;
                                    const cv::Point2f& pCur = currentKF->mvKeys[idxCur].pt;
                                    const int yp = (int)ppKF.y, xp = (int)ppKF.x;
                                    const int yc = (int)pCur.y, xc = (int)pCur.x;
                                    if (yp >= 0 && yp < pKF->mlDepthImage->rows &&
                                        xp >= 0 && xp < pKF->mlDepthImage->cols &&
                                        yc >= 0 && yc < currentKF->mlDepthImage->rows &&
                                        xc >= 0 && xc < currentKF->mlDepthImage->cols) {
                                        const float d_pKF = pKF->mlDepthImage->at<float>(yp, xp);
                                        const float d_cur = currentKF->mlDepthImage->at<float>(yc, xc);
                                        if (std::isfinite(d_pKF) && std::isfinite(d_cur) && d_pKF > 0.f && d_cur > 0.f) {
                                            ml_ratios_new.push_back(d_cur / d_pKF);
                                        }
                                    }
                                }
                            }
                        }

                        auto medianOf = [](std::vector<float>& v) -> float {
                            if (v.empty()) return -1.f;
                            std::sort(v.begin(), v.end());
                            return v[v.size() / 2];
                        };
                        const float s_ml_old = medianOf(ml_ratios_old);
                        const float s_ml_new = medianOf(ml_ratios_new);

                        // [INDIRECT.SML_COMPARE]: always print on accepted (≥30-inlier) loop events for audit.
                        // degenerate=YES tags Sim3 RANSAC scales outside [1/3, 3] — physically implausible inter-KF
                        // scale changes (e.g. cur=3106↔cand=17 on KITTI 00 produced s_ransac=29.85). Lets C0_post
                        // grep "would the gate have rejected this?" per estimator without rerunning. Threshold is
                        // intentionally loose (factor of 3) so ordinary scale drift through the trajectory is not
                        // flagged. See docs/indirect_depth_integration/EXECUTION_LOG.md Session 2 finding.
                        const bool degenerateRansac = (s_optimized < (1.f/3.f)) || (s_optimized > 3.f);
                        printf("[INDIRECT.SML_COMPARE] cur=%d cand=%d s_ransac=%.3f s_ml_old=%.3f s_ml_new=%.3f n_old=%zu n_new=%zu degenerate=%s\n",
                               (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                               s_optimized, s_ml_old, s_ml_new,
                               ml_ratios_old.size(), ml_ratios_new.size(),
                               degenerateRansac ? "YES" : "no");

                        // Indirect.S.1 (May 8, 2026): ML-confidence (κ) gating wrapper around the H2 P2 gate.
                        // Per lit audit §3.6 LR4: Metric3D's AngMF κ over matched-feature pixels indicates
                        // ML reliability for this loop pair. Below κ < τ, H2 should NOT veto a loop (ML is
                        // unreliable here; fall back to RANSAC-only behavior). Mean κ computed over matched
                        // pixels in currentKF's confidence map. Default off (--s1-confidence-gate=false);
                        // wraps H2 only — has no effect when --p2-gate=false.
                        bool s1_bypass_h2 = false;
                        if (setting_indirectS1ConfidenceGate && setting_indirectP2RejectGate) {
                            auto confImg = currentKF->mlConfidenceImage;
                            if (confImg && !confImg->empty()) {
                                double sum_kappa = 0.0; int n_kappa = 0;
                                for (size_t j = 0; j < vpMapPointMatches.size(); j++) {
                                    auto mpCur = vpMapPointMatches[j];
                                    if (!mpCur) continue;
                                    const int idxCur = mpCur->getIndexInKF(currentKF);
                                    if (idxCur < 0 || idxCur >= currentKF->nFeatures) continue;
                                    const cv::Point2f& pCur = currentKF->mvKeys[idxCur].pt;
                                    const int yc = (int)pCur.y, xc = (int)pCur.x;
                                    if (yc >= 0 && yc < confImg->rows && xc >= 0 && xc < confImg->cols) {
                                        const float kappa = confImg->at<float>(yc, xc);
                                        if (std::isfinite(kappa) && kappa > 0.f) { sum_kappa += kappa; n_kappa++; }
                                    }
                                }
                                const double mean_kappa = (n_kappa > 0) ? sum_kappa / n_kappa : -1.0;
                                s1_bypass_h2 = (n_kappa > 0 && mean_kappa < setting_indirectS1ConfidenceThresh);
                                printf("[INDIRECT.S1] cur=%d cand=%d mean_kappa=%.3f n=%d thresh=%.3f decision=%s\n",
                                       (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                                       mean_kappa, n_kappa, setting_indirectS1ConfidenceThresh,
                                       s1_bypass_h2 ? "BYPASS_H2" : "ALLOW_H2");
                            } else {
                                printf("[INDIRECT.S1] cur=%d cand=%d no_confidence_map (no S1 effect)\n",
                                       (int)currentKF->fs->KfId, (int)pKF->fs->KfId);
                            }
                        }

                        // Indirect.H2 (May 8, 2026): loop-closure Sim3 scale-disagreement rejection gate.
                        // Fallback policy decided by C0_post finding (plan §12.2): prefer new s_ml (≥5 samples) for
                        // semantic correctness, fall back to old s_ml (≥5 samples) when production ML cadence
                        // (every-Nth-KF) leaves new with insufficient coverage on this loop pair. Bypass only when
                        // BOTH estimators are < 5 samples — that's true coverage_low. Diagnostic always prints
                        // the gate's decision (REJECTED / ACCEPTED / BYPASS_COVERAGE_LOW) so §6.3 measurement can
                        // count, per estimator, would-the-gate-have-caught-this. Default ON since cad6539 (--p2-gate=true).
                        if (setting_indirectP2RejectGate && !s1_bypass_h2) {
                            bool mlScaleValid = true;
                            float s_ml_used = -1.f; size_t n_used = 0; const char* source_used = "none";
                            if (ml_ratios_new.size() >= 5) {
                                s_ml_used = s_ml_new; n_used = ml_ratios_new.size(); source_used = "new";
                            } else if (ml_ratios_old.size() >= 5) {
                                s_ml_used = s_ml_old; n_used = ml_ratios_old.size(); source_used = "old";
                            }
                            if (n_used >= 5) {
                                const float scale_disagreement = std::abs(s_optimized - s_ml_used) / std::max(s_optimized, s_ml_used);
                                const bool rejected = (scale_disagreement > setting_indirectP2RejectThresh);
                                printf("[INDIRECT.P2_GATE] cur=%d cand=%d s_ransac=%.3f s_ml=%.3f disagreement=%.1f%% n=%zu source=%s thresh=%.2f decision=%s\n",
                                       (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                                       s_optimized, s_ml_used, scale_disagreement * 100.0f,
                                       n_used, source_used, setting_indirectP2RejectThresh,
                                       rejected ? "REJECTED" : "ACCEPTED");
                                ++stat_lcScaleGateEvals;
                                if (rejected) { mlScaleValid = false; ++stat_lcScaleGateRejects; }
                            } else {
                                ++stat_lcScaleGateBypass;
                                printf("[INDIRECT.P2_GATE] cur=%d cand=%d s_ransac=%.3f decision=BYPASS_COVERAGE_LOW n_new=%zu n_old=%zu\n",
                                       (int)currentKF->fs->KfId, (int)pKF->fs->KfId,
                                       s_optimized, ml_ratios_new.size(), ml_ratios_old.size());
                            }
                            if (!mlScaleValid) continue;
                        }

                        bMatch = true;
                        candidateKF = pKF;
                        Sim3 gSmw = currentKF->fs->getPoseOpti(); //Sim3(currentKF->fs->getPoseInverse().matrix()); //pKF
                        mScw = gScm * gSmw;
                       
                        mvpCurrentMatchedPoints = vpMapPointMatches;
                        break;
                    }
                }
            }
        }

        if (!bMatch)
        {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            currentKF->SetErase();
            return false;
        }

        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        std::vector<std::shared_ptr<Frame>> vpLoopConnectedKFs = currentKF->GetVectorCovisibleKeyFrames(); //candidateKF
        vpLoopConnectedKFs.push_back(currentKF); //candidateKF
        mvpLoopMapPoints.clear();
        for (std::vector<std::shared_ptr<Frame>>::iterator vit = vpLoopConnectedKFs.begin(); vit != vpLoopConnectedKFs.end(); vit++)
        {
            std::shared_ptr<Frame> pKF = *vit;
            std::vector<std::shared_ptr<MapPoint>> vpMapPoints = pKF->getMapPointsV();
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
            {
                std::shared_ptr<MapPoint> pMP = vpMapPoints[i];
                if (pMP)
                {
                    if (!pMP->isBad() && pMP->mnLoopPointForKF != currentKF->fs->KfId)
                    {
                        mvpLoopMapPoints.push_back(pMP);
                        pMP->mnLoopPointForKF = currentKF->fs->KfId;
                    }
                }
            }
        }

        // Find more matches projecting with the computed Sim3
        int curmatches = matcher->SearchBySim3Projection(candidateKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints, 10); // currentKF
        // If enough matches accept Loop
        int nTotalMatches = 0;
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
        {
            if (mvpCurrentMatchedPoints[i])
                nTotalMatches++;
        }

        if (nTotalMatches >= 20) //20
        {
            for (int i = 0; i < nInitialCandidates; i++)
                if (mvpEnoughConsistentCandidates[i] != candidateKF)
                    mvpEnoughConsistentCandidates[i]->SetErase();
            return true;
        }
        else
        {
            for (int i = 0; i < nInitialCandidates; i++)
                mvpEnoughConsistentCandidates[i]->SetErase();
            currentKF->SetErase();
            return false;
        }
    }

    void LoopCloser::SetFinish(bool finish)
    {
        needFinish = finish;
        std::shared_ptr<Map> gMap = globalMap.lock();
        std::cout << "wait for loop closing thread to finish" << std::endl;
        
        while(!globalMap.lock()->isIdle())
        {
            usleep(10000);
        }
        mainLoop.join();
        usleep(5000);

        KFqueue.clear();
        mLastLoopKFid = 0;
    }

    void LoopCloser::CorrectLoop()
    {
        printf("Loop detected, to be corrected!");//debugNA

        // boost::unique_lock<boost::mutex> lck(fullSystem->mapMutex);  //
        auto gMap = globalMap.lock();
        std::vector<std::shared_ptr<Frame>> allKFrames;
        std::vector<std::shared_ptr<MapPoint>> allMapPoints;

        {
            // boost::unique_lock<boost::mutex> lck(fullSystem->mapMutex);
            globalMap.lock()->GetAllKeyFrames(allKFrames);
            globalMap.lock()->GetAllMapPoints(allMapPoints);
        }

        // Ensure current keyframe is updated
        currentKF->UpdateConnections();
        candidateKF->UpdateConnections();

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        mvpCurrentConnectedKFs = candidateKF->GetVectorCovisibleKeyFrames(); //currentKF
        mvpCurrentConnectedKFs.push_back(candidateKF);                       //currentKF

        auto LatestConnected = currentKF->GetConnectedKeyFrames(); //currentKF

        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        CorrectedSim3[candidateKF] = mScw; //currentKF, mg2oScw

        auto TempTwc = candidateKF->fs->getPoseOptiInv();

        std::set<std::shared_ptr<Frame>> TempFixed; //these frames are old but found few matches to the latest KF- fix them here but don't fix them in poseGraph!

        for (std::vector<std::shared_ptr<Frame>>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
        {
            std::shared_ptr<Frame> pKFi = *vit;

            if (pKFi->fs->KfId > currMaxKF) //make sure keyframes added after loop detection are not modified!!
                continue;

            if (pKFi->fs->KfId >= minActId) //if the connected keyframe to the loop candidate was recently added don't update its pose!! this is like a bubble that protects the recently added keyframes from being changed!
                continue;

            Sim3 TiwTemp = pKFi->fs->getPoseOpti();
            
            if (pKFi != candidateKF) //currentKF - below not changed..
            {
                Sim3 g2oSic = TiwTemp * TempTwc; //SE3 Tic = Tiw * Twc

                Sim3 g2oCorrectedSiw = g2oSic * mScw;
                //Pose corrected with the Sim3 of the loop closure
                // if (!LatestConnected.count(pKFi))
                // if(LatestConnected.count(pKFi) == 0 )
                    CorrectedSim3[pKFi] = g2oCorrectedSiw;
                // else
                // {
                    // CorrectedSim3[pKFi] = TiwTemp;
                    // TempFixed.insert(pKFi);
                // }
            }
            //Pose without correction
            NonCorrectedSim3[pKFi] = TiwTemp;
        }

        // Correct all MapPoints obsrved by candidate keyframe and neighbors, so that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
        {
            std::shared_ptr<Frame> pKFi = mit->first;
            Sim3 g2oCorrectedSiw = mit->second;
            pKFi->fs->setPoseOpti(g2oCorrectedSiw);
            Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            std::vector<std::shared_ptr<MapPoint>> vpMPsi = pKFi->getMapPointsV();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                std::shared_ptr<MapPoint> pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;

                if (pMPi->id > currMaxMp || pMPi->sourceFrame->fs->KfId > currMaxKF) //prevent new data from being modified
                    continue;

                if (pMPi->isBad())
                    continue;
                if (pMPi->mnCorrectedByKF == pKFi->fs->KfId) 
                    continue;

                if (pMPi->getDirStatus() == MapPoint::active) //prevent active data from being modified!
                    continue;
                if (pMPi->sourceFrame->getState() == Frame::active)
                    continue;

                pMPi->updateGlobalPose();
                pMPi->mnCorrectedByKF = pKFi->fs->KfId;
                pMPi->mnCorrectedReference = currentKF->fs->KfId;
                pMPi->UpdateNormalAndDepth();
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < mvpCurrentMatchedPoints.size(); i++)
        {
            if (mvpCurrentMatchedPoints[i])
            {
                std::shared_ptr<MapPoint> pLoopMP = candidateKF->getMapPoint(i); //mvpCurrentMatchedPoints[i];
                std::shared_ptr<MapPoint> pCurMP = mvpCurrentMatchedPoints[i];   //currentKF->getMapPoint(i);
                if (pLoopMP)                                                     //pCurMP  //Replace old mapPoints with new ones! should experiment with the other way around!
                    pLoopMP->Replace(pCurMP);                                    //pCurMP->Replace(pLoopMP);
                else                                                             //if old kf does not contain match, add it!
                {
                    candidateKF->addMapPointMatch(pCurMP, i);     //mpCurrentKF, pLoopMP
                    pCurMP->AddObservation(candidateKF, i);       // pLoopMP,  mpCurrentKF
                    pCurMP->ComputeDistinctiveDescriptors(false); //pLoopMP
                }
            }
        }

        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        SearchAndFuse(CorrectedSim3);

        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        std::map<std::shared_ptr<Frame>, std::set<std::shared_ptr<Frame>, std::owner_less<std::shared_ptr<Frame>>>, std::owner_less<std::shared_ptr<Frame>>>  LoopConnections;

        for (std::vector<std::shared_ptr<Frame>>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
        {
            std::shared_ptr<Frame> pKFi = *vit;
            if(pKFi->fs->KfId > currMaxKF)
                continue;
            std::vector<std::shared_ptr<Frame>> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

            // Update connections. Detect new links.
            pKFi->UpdateConnections();
            LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
            for (std::vector<std::shared_ptr<Frame>>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end(); vit_prev != vend_prev; vit_prev++)
            {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            for (std::vector<std::shared_ptr<Frame>>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end(); vit2 != vend2; vit2++)
            {
                LoopConnections[pKFi].erase(*vit2);
            }
        }

        // Optimize graph 
        //this allows future data captured since the candidate detection to also fix the gauge freedom and prevent unwanted errors!
        OptimizeEssentialGraph(fullSystem->allKeyFramesHistory, allMapPoints, TempFixed, currentKF ,candidateKF , NonCorrectedSim3, CorrectedSim3, LoopConnections, 
                                fullSystem->ef->connectivityMap, currMaxKF, minActId, currMaxMp, false); //mpMatchedKF, mpCurrentKF,
        
        for (auto it : fullSystem->allKeyFramesHistory)
            it->setRefresh(true);
        gMap->InformNewBigChange();

        // Add loop edge
        candidateKF->AddLoopEdge(currentKF);
        currentKF->AddLoopEdge(candidateKF);

        // Launch a new thread to perform Global Bundle Adjustment
        // mbRunningGBA = true;
        // mbFinishedGBA = false;
        mbStopGBA = false;
        // mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, mpCurrentKF->mnId);
        
        // boost::unique_lock<boost::mutex> lock(fullSystem->trackMutex);
        
        // BundleAdjustment(allKFrames, allMapPoints, ActiveFrames, ActivePoints, 10, &mbStopGBA, true, true, maxKfId, currMaxKF, currMaxMp);
        // BundleAdjustment(allKFrames, allMapPoints, 10, &mbStopGBA, true, true, currMaxKF, minActId, currMaxMp);
        // // Loop closed. Release Local Mapping.
        // mpLocalMapper->Release();

        for (auto it : fullSystem->allKeyFramesHistory)
            it->setRefresh(true);

        mLastLoopKFid = currentKF->fs->KfId;
        cout << "Loop correction complete!" << endl;
    }

    void LoopCloser::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
    {
        // ORBmatcher matcher(0.8);
        auto matcher = wpmatcher.lock();
        for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend; mit++)
        {
            std::shared_ptr<Frame> pKF = mit->first;

            Sim3 Scw = mit->second;
            // cv::Mat cvScw = Converter::toCvMat(g2oScw);

            std::vector<std::shared_ptr<MapPoint>> vpReplacePoints(mvpLoopMapPoints.size(), nullptr);
            matcher->Fuse(pKF, Scw, mvpLoopMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            // unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            const int nLP = mvpLoopMapPoints.size();
            for (int i = 0; i < nLP; i++)
            {
                std::shared_ptr<MapPoint> pRep = vpReplacePoints[i];
                if (pRep)
                {
                    pRep->Replace(mvpLoopMapPoints[i]);
                }
            }
        }
    }

}
