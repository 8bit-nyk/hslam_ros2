/**
* This file is part of DSO, written by Jakob Engel.
* It has been modified by Georges Younes, Daniel Asmar, John Zelek, and Yan Song Hu
*
* Copyright 2024 University of Waterloo and American University of Beirut.
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * KFBuffer.cpp
 *
 *  Created on: Jan 7, 2014
 *      Author: engelj
 */


#include "FullSystem/FullSystem.h"
 
#include "stdio.h"
#include "util/globalFuncs.h"
#include "util/FPSLogger.h"
#include <Eigen/LU>
#include <algorithm>
#include "IOWrapper/ImageDisplay.h"
#include "util/globalCalib.h"
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include "FullSystem/PixelSelector.h"
#include "FullSystem/PixelSelector2.h"
#include "FullSystem/ResidualProjections.h"
#include "FullSystem/ImmaturePoint.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "FullSystem/CoarseTracker.h"
#include "FullSystem/CoarseInitializer.h"

#include "Indirect/Frame.h"
#include "Indirect/Detector.h"
#include "Indirect/MapPoint.h"
#include "Indirect/Map.h"
#include "Indirect/Matcher.h"
#include "Indirect/Optimizer.h"
#include "Indirect/LoopCloser.h"

#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>  // For std::this_thread::sleep_for

#include "OptimizationBackend/EnergyFunctional.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include "IOWrapper/Output3DWrapper.h"

#include "util/ImageAndExposure.h"
#include "util/DepthLogger.h"

#include "Indirect/IndirectTracker.h"

#include <cmath>
#include <chrono>
#include <thread>
#include "ML/MLInference.h"
#include "ML/MLDepthProcessor.h"

// MLDepthProcessor integration (former feature flag removed - using simplified processor only)

namespace HSLAM
{
int FrameHessian::instanceCounter=0;
int PointHessian::instanceCounter=0;
unsigned long PointHessian::totalInstantCounter=0;
int CalibHessian::instanceCounter=0;


/**
 * @brief Construct a new FullSystem Object
 * 
 * @param linearizeOperationPassed 
 * @param imuCalibration 
 * @param imuSettings 
 */
FullSystem::FullSystem()
{

	int retstat =0;
	if(setting_logStuff)
	{

		retstat += system("rm -rf logs");
		retstat += system("mkdir logs");

		// retstat += system("rm -rf mats");  // DSO legacy: mats/ directory unused in HSLAM
		// retstat += system("mkdir mats");

		calibLog = new std::ofstream();
		calibLog->open("logs/calibLog.txt", std::ios::trunc | std::ios::out);
		calibLog->precision(12);

		numsLog = new std::ofstream();
		numsLog->open("logs/numsLog.txt", std::ios::trunc | std::ios::out);
		numsLog->precision(10);

		coarseTrackingLog = new std::ofstream();
		coarseTrackingLog->open("logs/coarseTrackingLog.txt", std::ios::trunc | std::ios::out);
		coarseTrackingLog->precision(10);

		// DSO legacy: Hessian eigenvalue/nullspace diagnostics — printEigenValLine() is commented out, these are never written
		// eigenAllLog = new std::ofstream();
		// eigenAllLog->open("logs/eigenAllLog.txt", std::ios::trunc | std::ios::out);
		// eigenAllLog->precision(10);

		// eigenPLog = new std::ofstream();
		// eigenPLog->open("logs/eigenPLog.txt", std::ios::trunc | std::ios::out);
		// eigenPLog->precision(10);

		// eigenALog = new std::ofstream();
		// eigenALog->open("logs/eigenALog.txt", std::ios::trunc | std::ios::out);
		// eigenALog->precision(10);

		// DiagonalLog = new std::ofstream();
		// DiagonalLog->open("logs/diagonal.txt", std::ios::trunc | std::ios::out);
		// DiagonalLog->precision(10);

		// variancesLog = new std::ofstream();
		// variancesLog->open("logs/variancesLog.txt", std::ios::trunc | std::ios::out);
		// variancesLog->precision(10);

		// nullspacesLog = new std::ofstream();
		// nullspacesLog->open("logs/nullspacesLog.txt", std::ios::trunc | std::ios::out);
		// nullspacesLog->precision(10);
		eigenAllLog=0; eigenPLog=0; eigenALog=0; DiagonalLog=0; variancesLog=0; nullspacesLog=0;
	}
	else
	{
		nullspacesLog=0;
		variancesLog=0;
		DiagonalLog=0;
		eigenALog=0;
		eigenPLog=0;
		eigenAllLog=0;
		numsLog=0;
		calibLog=0;
	}

	assert(retstat!=293847);



	selectionMap = new float[wG[0]*hG[0]];

	coarseDistanceMap = new CoarseDistanceMap(wG[0], hG[0]);
	coarseTracker = new CoarseTracker(wG[0], hG[0]);
	coarseTracker_forNewKF = new CoarseTracker(wG[0], hG[0]);
	coarseInitializer = new CoarseInitializer(wG[0], hG[0]);
	pixelSelector = new PixelSelector(wG[0], hG[0]);

	// indirect!: Additional classes for loop closure
	detector = std::make_shared<FeatureDetector>();
	globalMap = std::make_shared<Map>();
	matcher = std::make_shared<Matcher>();
	if(LoopClosure)
		{
		// std::make_shared will not use the eigen alignment macro and cause an alignment error
		loopCloser = std::shared_ptr<LoopCloser> (new LoopCloser(this));
	}

	Velocity = SE3();

	statistics_lastNumOptIts=0;
	statistics_numDroppedPoints=0;
	statistics_numActivatedPoints=0;
	statistics_numCreatedPoints=0;
	statistics_numForceDroppedResBwd = 0;
	statistics_numForceDroppedResFwd = 0;
	statistics_numMargResFwd = 0;
	statistics_numMargResBwd = 0;

	lastCoarseRMSE.setConstant(100);

	currentMinActDist=2;
	initialized=false;

	// Initialize ML reference variables to prevent uninitialized access
	ml_reference_depth_ = cv::Mat(0, 0, CV_32F);  // Explicitly create empty Mat with 0 rows/cols
	ml_reference_frame_id_ = -1;      // Already initialized in header, but be explicit
	ml_reference_confidence_ = 0.0f;
	ml_reference_time_ = 0.0;

	ef = new EnergyFunctional();
	ef->red = &this->treadReduce;

	isLost=false;
	initFailed=false;


	needNewKFAfter = -1;

	linearizeOperation=true;
	runMapping=true;
	// MAPPING THREAD IS STARTED HERE
	mappingThread = boost::thread(&FullSystem::mappingLoop, this);
	lastRefStopID = 0;

	minIdJetVisDebug = -1;
	maxIdJetVisDebug = -1;
	minIdJetVisTracker = -1;
	maxIdJetVisTracker = -1;
}

/**
 * @brief Destroy the FullSystem Object
 * 
 */
FullSystem::~FullSystem()
{
	blockUntilMappingIsFinished();

	if(setting_logStuff)
	{
		calibLog->close(); delete calibLog;
		numsLog->close(); delete numsLog;
		coarseTrackingLog->close(); delete coarseTrackingLog;
		//errorsLog->close(); delete errorsLog;
		// DSO legacy logs disabled (never written — printEigenValLine is commented out)
		// eigenAllLog->close(); delete eigenAllLog;
		// eigenPLog->close(); delete eigenPLog;
		// eigenALog->close(); delete eigenALog;
		// DiagonalLog->close(); delete DiagonalLog;
		// variancesLog->close(); delete variancesLog;
		// nullspacesLog->close(); delete nullspacesLog;

	}

	delete[] selectionMap;

	for(FrameShell* s : allFrameHistory)
	{
		delete s;
		s = nullptr;
	}
	for(FrameHessian* fh : unmappedTrackedFrames)
	{
		if (fh->shell->frame)
			fh->shell->frame.reset();
		if(fh)
		{
			delete fh;
			fh = nullptr;
		}
	}
	delete coarseDistanceMap;
	delete coarseTracker;
	delete coarseTracker_forNewKF;
	delete coarseInitializer;
	delete pixelSelector;
	delete ef;
	
	// Phase 2: Thread-safe shutdown sequence for ML components
	
	loopCloser.reset();
	matcher.reset();
	detector.reset();
	globalMap.reset();
}

void FullSystem::setOriginalCalib(const VecXf &originalCalib, int originalW, int originalH)
{

}

void FullSystem::setGammaFunction(float* BInv)
{
	if(BInv==0) return;

	// copy BInv.
	memcpy(Hcalib.Binv, BInv, sizeof(float)*256);


	// invert.
	for(int i=1;i<255;i++)
	{
		// find val, such that Binv[val] = i.
		// I dont care about speed for this, so do it the stupid way.

		for(int s=1;s<255;s++)
		{
			if(BInv[s] <= i && BInv[s+1] >= i)
			{
				Hcalib.B[i] = s+(i - BInv[s]) / (BInv[s+1]-BInv[s]);
				break;
			}
		}
	}
	Hcalib.B[0] = 0;
	Hcalib.B[255] = 255;
}



void FullSystem::printResult(std::string file, bool printSim)
{
	int removed = 0;
	int marginalized = 0;
	int active = 0;
	int immature = 0;

	// indirect!: Count indirect points for printing
	for (auto it : allKeyFramesHistory)
	{
		if(!it->frame)
			continue;
		auto kfPts = it->frame->getMapPointsV();
		for (int i = 0, iend = kfPts.size(); i < iend; ++i)
			if (kfPts[i])
			{
				auto status = kfPts[i]->getDirStatus(); //active, marginalized, removed
				if (status == MapPoint::active)
					active += 1;
				else if (status == MapPoint::marginalized)
					marginalized += 1;
				else if (status == MapPoint::removed)
					removed += 1;
			}
		}


	boost::unique_lock<boost::mutex> lock(trackMutex);

	std::ofstream myfile;
	myfile.open (file.c_str());
	myfile << std::setprecision(15);

	for(FrameShell* s : allFrameHistory)
	{
		if(!s->poseValid) 
			continue;

		if(setting_onlyLogKFPoses && s->marginalizedAt == s->id) 
			continue;
		SE3 Twc = s->getPose();
		// Twc = s->getPose();

		myfile << s->timestamp <<
			" " << Twc.translation().transpose()<<
			" " << Twc.so3().unit_quaternion().x()<<
			" " << Twc.so3().unit_quaternion().y()<<
			" " << Twc.so3().unit_quaternion().z()<<
			" " << Twc.so3().unit_quaternion().w() << "\n";
	}
	myfile.close();
}

void FullSystem::printPC(std::string file)
{
	boost::unique_lock<boost::mutex> lock(trackMutex);
	boost::unique_lock<boost::mutex> crlock(shellPoseMutex);

	std::ofstream myfile;
	myfile.open (file.c_str());
	myfile << std::setprecision(9);

	unsigned long totalpcs = allMargPointsHistory.size();
	//unsigned long totalpcs = allMargPointsHistory.size()+allFrameHistory.size();
	
	myfile << std::string("# .PCD v.6 - Point Cloud Data file format\n");
	myfile << std::string("FIELDS x y z rgb\n");
	myfile << std::string("SIZE 4 4 4 4\n");
	myfile << std::string("TYPE F F F F\n");
	myfile << std::string("COUNT 1 1 1 1\n");
	myfile << std::string("WIDTH ") << totalpcs << std::string("\n");
	myfile << std::string("HEIGHT 1\n");
	myfile << std::string("#VIEWPOINT 0 0 0 1 0 0 0\n");
	myfile << std::string("POINTS ") << totalpcs << std::string("\n");
	myfile << std::string("DATA ascii\n");
	
	std::unordered_map<unsigned long, PC_output>::iterator itr; 
	for (itr = allMargPointsHistory.begin(); itr != allMargPointsHistory.end(); itr++)  
	{
		PC_output tmp_PC = itr->second;
		float rgb;
		unsigned char b[] = {tmp_PC.r, tmp_PC.g, tmp_PC.b, 0};
		memcpy(&rgb, &b, sizeof(rgb));

		myfile << tmp_PC.x << " " << tmp_PC.y << " " << tmp_PC.z << " " << rgb << "\n";
	} 

	// Show trajectory in point cloud
	for (FrameShell* s : allFrameHistory)  
	{
		Sophus::SE3d camToWorld = s->getPose();
		Sophus::SE3d camToFirst = camToWorld;
		float rgb;
		unsigned char b[] = {0, 255, 0, 0};
		memcpy(&rgb, &b, sizeof(rgb));
		myfile << camToFirst.translation().x() <<
            " " << camToFirst.translation().y() <<
            " " << camToFirst.translation().z() << " " << rgb << "\n";
	} 

	myfile.close();
}

/**
 * @brief Determines best track by tracking possible poses using the coarseTracker class
 * Has three steps
 * 1. Creates a list of likely poses to test
 * 2. Passes possible poses to be tracked by the coarseTracker class till a good track is returned
 * 3. Update variables
 * 
 * If a good track cannot be found, either use the estimated pose from the IMU or assume constant motion
 * If the tracking residual grows too large, this function will kill the program
 * 
 * @param fh 						Current frame
 * @param referenceToFrameHint 		Pose derived from IMU measurements
 * @return std::pair<Vec4, bool> 	Achieved residual, flow vector, and if the tracking is good
 */
Vec5 FullSystem::trackNewCoarse(FrameHessian* fh, bool writePose)
{
	// TEMPORARY DEBUG: Block if ML processing is active
	if (ml_depth_enabled_) {
		boost::unique_lock<boost::mutex> lock(ml_processing_mutex_);
		if (ml_processing_active_) {
			printf("DEBUG: Frame %d waiting for ML processing to complete...\n", fh->frameID);
			ml_processing_done_.wait(lock, [this]{ return !ml_processing_active_; });
			printf("DEBUG: Frame %d resuming after ML processing done\n", fh->frameID);
		}
	}

	// ML depth forwarding moved to keyframe processing (makeKeyFrame) to avoid per-frame overhead
	// and ensure proper dual-instance handling

	assert(allFrameHistory.size() > 0);
	// If there is at least one frame in history, set pose initialization.
	FrameHessian* lastF = coarseTracker->lastRef;
	AffLight aff_last_2_l = AffLight(0,0);
	std::vector<SE3,Eigen::aligned_allocator<SE3>> lastF_2_fh_tries;
	
	// =====Pose Hypothesis Generation======
	// The function generates a set of possible relative poses (lastF_2_fh_tries)
	// between the last reference frame and the new frame. This is crucial for robustness, 
	//as tracking can fail if the initial guess is poor.
	//If only two frames, use identity transformation as the only guess.
	if(allFrameHistory.size() == 2)
		for(unsigned int i=0;i<lastF_2_fh_tries.size();i++) lastF_2_fh_tries.push_back(SE3());
	else
	{	// If there are more than two frames, generate a set of possible relative poses.
		FrameShell* slast = allFrameHistory[allFrameHistory.size()-2];
		FrameShell* sprelast = allFrameHistory[allFrameHistory.size()-3];
		SE3 slast_2_sprelast;
		SE3 lastF_2_slast;
		{	// lock on global pose consistency!
			boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
			slast_2_sprelast = sprelast->getPose().inverse() * slast->getPose();
			lastF_2_slast = slast->getPose().inverse() * lastF->shell->getPose();
			aff_last_2_l = slast->aff_g2l;
		}
		SE3 fh_2_slast = slast_2_sprelast;// assumed to be the same as fh_2_slast.

		if(nIndmatches > 20 && isUsable) // indirect!: if indirect tracking is good use it as prior for Direct tracking
		{
			lastF_2_fh_tries.push_back(fh->shell->getPoseInverse() * lastF->shell->getPose());
		}
		// Add several motion hypotheses
		// get last delta-movement.
		lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast);							// assume constant motion.
		lastF_2_fh_tries.push_back(fh_2_slast.inverse() * fh_2_slast.inverse() * lastF_2_slast);	// assume double motion (frame skipped)
		lastF_2_fh_tries.push_back(SE3::exp(fh_2_slast.log()*0.5).inverse() * lastF_2_slast); // assume half motion.
		lastF_2_fh_tries.push_back(lastF_2_slast); // assume zero motion.
        lastF_2_fh_tries.push_back(SE3()); 															// assume zero motion from KF.

		// Add several rotation hypotheses
		// just try a TON of different initializations (all rotations). In the end,
		// if they don't work they will only be tried on the coarsest level, which is super fast anyway.
		// also, if tracking rails here we loose, so we really, really want to avoid that.
		// for(float rotDelta=0.02; rotDelta < 0.05; rotDelta++) //rotDelta+=0.01
		// {
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,0), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,0), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,rotDelta), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,0), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,0), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,0,-rotDelta), Vec3(0,0,0)));			// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,0,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,0), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,0,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,0,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,-rotDelta,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,-rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,-rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// 	lastF_2_fh_tries.push_back(fh_2_slast.inverse() * lastF_2_slast * SE3(Sophus::Quaterniond(1,rotDelta,rotDelta,rotDelta), Vec3(0,0,0)));	// assume constant motion.
		// }

		if(!slast->poseValid || !sprelast->poseValid || !lastF->shell->poseValid)
		{
			lastF_2_fh_tries.clear();
			lastF_2_fh_tries.push_back(SE3());
		}
	}


	// ============== Test all of the pose guesses ===================
	// ===Initialize variables===
	Vec3 flowVecs = Vec3(100,100,100); // Flow vector of tracked motion
	SE3 lastF_2_fh = SE3();
	AffLight aff_g2l = AffLight(0,0);


	// As long as maxResForImmediateAccept is not reached, it will continue through the options.
	// Keep track of the so-far best achieved residual for each level in achievedRes.
	// If on a coarse level, tracking is WORSE than achievedRes, we will not continue to save time.


	Vec5 achievedRes = Vec5::Constant(NAN);
	bool haveOneGood = false;
	int tryIterations=0;

	// ===Hypothesis Evaluation Loop===
	
	for(unsigned int i=0;i<lastF_2_fh_tries.size();i++) // Try tracking for all poses in lastF_2_fh_tries
	{
		AffLight aff_g2l_this = aff_last_2_l;
		SE3 lastF_2_fh_this = lastF_2_fh_tries[i];
		// bool trackIndirect = trackNewestCoarse(fh->shell->frame, lastF->shell->frame, Test, pyrLevelsUsed - 1);
		// Tracking is done by the coarseTracker class		
		bool trackingIsGood = coarseTracker->trackNewestCoarse(
			fh, lastF_2_fh_this, aff_g2l_this,
			pyrLevelsUsed - 1,
			achievedRes);	// in each level this has to be at least as good as the last try.
		tryIterations++;

		if(i != 0)
		{
			printf("RE-TRACK ATTEMPT %d with initOption %d and start-lvl %d (ab %f %f): %f %f %f %f %f -> %f %f %f %f %f \n",
					i,
					i, pyrLevelsUsed-1,
					aff_g2l_this.a,aff_g2l_this.b,
					achievedRes[0],
					achievedRes[1],
					achievedRes[2],
					achievedRes[3],
					achievedRes[4],
					coarseTracker->lastResiduals[0],
					coarseTracker->lastResiduals[1],
					coarseTracker->lastResiduals[2],
					coarseTracker->lastResiduals[3],
					coarseTracker->lastResiduals[4]);
		}


		// ============== Update variables if there is a good track ===================
		// Track for given motion is sucessful is:
		// 1. Tracking does not return invalid values
		// 2. Residual is finite
		// 3. Residual is smaller than last achieved residual
		if(trackingIsGood && std::isfinite((float)coarseTracker->lastResiduals[0]) && !(coarseTracker->lastResiduals[0] >=  achievedRes[0]))
		{
			//printf("take over. minRes %f -> %f!\n", achievedRes[0], coarseTracker->lastResiduals[0]);
			flowVecs = coarseTracker->lastFlowIndicators;
			aff_g2l = aff_g2l_this;
			lastF_2_fh = lastF_2_fh_this;
			haveOneGood = true;
		}

		// Set achieved residuals to new values
		if(haveOneGood)
		{
			for(int j=0;j<5;j++)
			{
				if(!std::isfinite((float)achievedRes[j]) || achievedRes[j] > coarseTracker->lastResiduals[j])
					achievedRes[j] = coarseTracker->lastResiduals[j];
			}
		}


		// Stop testing other tracks if result is good enough
        if(haveOneGood &&  achievedRes[0] < lastCoarseRMSE[0]*setting_reTrackThreshold)
            break;

	}

	if(!haveOneGood)
	{
        printf("BIG ERROR! tracking failed entirely. Take predictred pose and hope we may somehow recover.\n");
		flowVecs = Vec3(0,0,0);
		aff_g2l = aff_last_2_l;
		lastF_2_fh = lastF_2_fh_tries[0];
	}
	
	// NUMERICAL STABILITY FIX: Clamp extreme affine parameters to prevent Sophus scale exceptions
	if (!std::isfinite(aff_g2l.a) || !std::isfinite(aff_g2l.b) || 
	    std::abs(aff_g2l.a) > 1e6 || std::abs(aff_g2l.b) > 1e6) {
		printf("WARNING: Extreme affine parameters detected (a=%f, b=%f), clamping for stability\n", 
		       aff_g2l.a, aff_g2l.b);
		aff_g2l.a = std::max(-100.0, std::min(100.0, (double)aff_g2l.a));
		aff_g2l.b = std::max(-1000.0, std::min(1000.0, (double)aff_g2l.b));
		printf("WARNING: Clamped to (a=%f, b=%f)\n", aff_g2l.a, aff_g2l.b);
	}

	lastCoarseRMSE = achievedRes;

	// no lock required, as fh is not used anywhere yet.

	fh->shell->trackingRefId = lastF->shell->id;
	
	// Add bounds checking to prevent numerical instability
	if (!std::isfinite(aff_g2l.a) || std::abs(aff_g2l.a) > 1e6) {
		printf("WARNING: Extreme affine parameter a=%f, clamping to reasonable range\n", aff_g2l.a);
		aff_g2l.a = std::max(-10.0, std::min(10.0, (double)aff_g2l.a));
	}
	if (!std::isfinite(aff_g2l.b) || std::abs(aff_g2l.b) > 1e6) {
		printf("WARNING: Extreme affine parameter b=%f, clamping to reasonable range\n", aff_g2l.b);
		aff_g2l.b = std::max(0.0, std::min(255.0, (double)aff_g2l.b));
	}
	
	fh->shell->aff_g2l = aff_g2l;

	if(writePose || tryIterations < 2)
		fh->shell->setPose(lastF->shell->getPose() * lastF_2_fh.inverse());

	if(coarseTracker->firstCoarseRMSE < 0)
		coarseTracker->firstCoarseRMSE = achievedRes[0];

    if(!setting_debugout_runquiet)
        printf("Coarse Tracker tracked ab = %f %f (exp %f). Res %f!\n", aff_g2l.a, aff_g2l.b, fh->ab_exposure, achievedRes[0]);



	if(setting_logStuff)
	{
		(*coarseTrackingLog) << std::setprecision(16)
						<< fh->shell->id << " "
						<< fh->shell->timestamp << " "
						<< fh->ab_exposure << " "
						<< fh->shell->getPose().log().transpose() << " "
						<< aff_g2l.a << " "
						<< aff_g2l.b << " "
						<< achievedRes[0] << " "
						<< tryIterations << "\n";
	}

	// ===Output===
	Vec5 Output;
	Output << achievedRes[0], flowVecs[0], flowVecs[1], flowVecs[2], (double)(tryIterations > 1 ? -1.0: +1.0);
	return Output;
}

/**
 * @brief Traces all the immature points
 * 
 * Because immature points are only made for new frames,
 * the number of immature points per frame will decrease over time
 * 
 * @param fh Current frame
 */
void FullSystem::traceNewCoarse(FrameHessian* fh)
{
	boost::unique_lock<boost::mutex> lock(mapMutex);

	int trace_total=0, trace_good=0, trace_oob=0, trace_out=0, trace_skip=0, trace_badcondition=0, trace_uninitialized=0;

	Mat33f K = Mat33f::Identity();
	K(0,0) = Hcalib.fxl();
	K(1,1) = Hcalib.fyl();
	K(0,2) = Hcalib.cxl();
	K(1,2) = Hcalib.cyl();

	for(FrameHessian* host : frameHessians)	// Go through all active frames
	{

		// Get SE(3) matrix from old to new position
		SE3 hostToNew = fh->PRE_worldToCam * host->PRE_camToWorld;
		// Seperate into rotation and translation parts, multiply by K at this stage for calculational efficency
		Mat33f KRKi = K * hostToNew.rotationMatrix().cast<float>() * K.inverse();
		Vec3f Kt = K * hostToNew.translation().cast<float>();

		// Get matrix that transforms points from one frame to another
		Vec2f aff = AffLight::fromToVecExposure(host->ab_exposure, fh->ab_exposure, host->aff_g2l(), fh->aff_g2l()).cast<float>();

		for(ImmaturePoint* ph : host->immaturePoints) // For all immature points in active host frame
		{
			ph->traceOn(fh, KRKi, Kt, aff, &Hcalib, false ); // trace the immature point

			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_GOOD) trace_good++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_BADCONDITION) trace_badcondition++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OOB) trace_oob++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_OUTLIER) trace_out++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_SKIPPED) trace_skip++;
			if(ph->lastTraceStatus==ImmaturePointStatus::IPS_UNINITIALIZED) trace_uninitialized++;
			trace_total++;
		}
	}
//	printf("ADD: TRACE: %'d points. %'d (%.0f%%) good. %'d (%.0f%%) skip. %'d (%.0f%%) badcond. %'d (%.0f%%) oob. %'d (%.0f%%) out. %'d (%.0f%%) uninit.\n",
//			trace_total,
//			trace_good, 100*trace_good/(float)trace_total,
//			trace_skip, 100*trace_skip/(float)trace_total,
//			trace_badcondition, 100*trace_badcondition/(float)trace_total,
//			trace_oob, 100*trace_oob/(float)trace_total,
//			trace_out, 100*trace_out/(float)trace_total,
//			trace_uninitialized, 100*trace_uninitialized/(float)trace_total);
}




/**
 * @brief Helper function for activatePointsMT
 * 
 * Does the type conversion from ImmaturePoints to PointHessians
 * 
 * @param optimized 
 * @param toOptimize 
 * @param min 
 * @param max 
 * @param stats 
 * @param tid 
 */
void FullSystem::activatePointsMT_Reductor(
		std::vector<PointHessian*>* optimized,
		std::vector<ImmaturePoint*>* toOptimize,
		int min, int max, Vec10* stats, int tid)
{
	ImmaturePointTemporaryResidual* tr = new ImmaturePointTemporaryResidual[frameHessians.size()];
	for(int k=min;k<max;k++)
	{
		(*optimized)[k] = optimizeImmaturePoint((*toOptimize)[k],1,tr);
	}
	delete[] tr;
}



/**
 * @brief Converts immature points to active points
 * 
 */
void FullSystem::activatePointsMT()
{

	// ============== Update variables for desired point density ===================
	// Change the currentMinActDist in order to achieve the desired point number
	if(ef->nPoints < setting_desiredPointDensity*0.66)
		currentMinActDist -= 0.8;
	if(ef->nPoints < setting_desiredPointDensity*0.8)
		currentMinActDist -= 0.5;
	else if(ef->nPoints < setting_desiredPointDensity*0.9)
		currentMinActDist -= 0.2;
	else if(ef->nPoints < setting_desiredPointDensity)
		currentMinActDist -= 0.1;

	if(ef->nPoints > setting_desiredPointDensity*1.5)
		currentMinActDist += 0.8;
	if(ef->nPoints > setting_desiredPointDensity*1.3)
		currentMinActDist += 0.5;
	if(ef->nPoints > setting_desiredPointDensity*1.15)
		currentMinActDist += 0.2;
	if(ef->nPoints > setting_desiredPointDensity)
		currentMinActDist += 0.1;

	// Max and min currentMinActDist values
	if(currentMinActDist < 0) currentMinActDist = 0;
	if(currentMinActDist > 4) currentMinActDist = 4;

    if(!setting_debugout_runquiet)
        printf("SPARSITY:  MinActDist %f (need %d points, have %d points)!\n",
                currentMinActDist, (int)(setting_desiredPointDensity), ef->nPoints);



	// ============== Loop through all of the immature points in active frames ===================
	FrameHessian* newestHs = frameHessians.back();

	// Make dist map.
	coarseDistanceMap->makeK(&Hcalib);
	coarseDistanceMap->makeDistanceMap(frameHessians, newestHs);

	//coarseTracker->debugPlotDistMap("distMap");

	std::vector<ImmaturePoint*> toOptimize; toOptimize.reserve(20000);


	for(FrameHessian* host : frameHessians)		// go through all active frames
	{
		if(host == newestHs) continue; // exclude newest frame

		SE3 fhToNew = newestHs->PRE_worldToCam * host->PRE_camToWorld; // Transformation matrix from host to newest frame
		// Seperate transformation matrix to rotation and translation parts
		Mat33f KRKi = (coarseDistanceMap->K[1] * fhToNew.rotationMatrix().cast<float>() * coarseDistanceMap->Ki[0]);
		Vec3f Kt = (coarseDistanceMap->K[1] * fhToNew.translation().cast<float>());


		for(unsigned int i=0;i<host->immaturePoints.size();i+=1) // for every immature point in the frame
		{
			ImmaturePoint* ph = host->immaturePoints[i];
			ph->idxInImmaturePoints = i;

			// ============== Delete invalid immature points ===================
			// Delete points that have never been traced successfully, or that are outlier on the last trace.
			if(!std::isfinite(ph->idepth_max) || ph->lastTraceStatus == IPS_OUTLIER)
			{
//				immature_invalid_deleted++;
				// remove point.
				delete ph;
				host->immaturePoints[i] = 0;
				continue;
			}

			// Activate only if this is true.
			bool canActivate = (ph->lastTraceStatus == IPS_GOOD
					|| ph->lastTraceStatus == IPS_SKIPPED
					|| ph->lastTraceStatus == IPS_BADCONDITION
					|| ph->lastTraceStatus == IPS_OOB )
							&& ph->lastTracePixelInterval < 8
							&& ph->quality > setting_minTraceQuality
							&& (ph->idepth_max+ph->idepth_min) > 0;


			// if I cannot activate the point, skip it. Maybe also delete it.
			if(!canActivate)
			{
				// if point will be out afterwards, delete it instead.
				if(ph->host->flaggedForMarginalization || ph->lastTraceStatus == IPS_OOB)
				{
//					immature_notReady_deleted++;
					delete ph;
					host->immaturePoints[i] = 0;
				}
//				immature_notReady_skipped++;
				continue;
			}


			// ============== Add immature point to activation list if it meets conditions ===================
			// Determine if the point is far away enough from other points
			// Points are only activated if it has good spacing compared to other active points
			// once projected on the newest keyframe
			Vec3f ptp = KRKi * Vec3f(ph->u, ph->v, 1) + Kt*(0.5f*(ph->idepth_max+ph->idepth_min));
			int u = ptp[0] / ptp[2] + 0.5f;
			int v = ptp[1] / ptp[2] + 0.5f;

			if((u > 0 && v > 0 && u < wG[1] && v < hG[1])) // delete point if it is out of bounds
			{

				float dist = coarseDistanceMap->fwdWarpedIDDistFinal[u+wG[1]*v] + (ptp[0]-floorf((float)(ptp[0])));

				// indirect!: Also account for indirect map points
				if(dist>=currentMinActDist* ((ph->my_type <= 4) ? ph->my_type : 1))
				{
					coarseDistanceMap->addIntoDistFinal(u,v);
					toOptimize.push_back(ph); // add point to list of points to be optimized
				}
			}
			else
			{
				delete ph;
				host->immaturePoints[i] = 0;
			}
		}
	}


//	printf("ACTIVATE: %d. (del %d, notReady %d, marg %d, good %d, marg-skip %d)\n",
//			(int)toOptimize.size(), immature_deleted, immature_notReady, immature_needMarg, immature_want, immature_margskip);

	std::vector<PointHessian*> optimized; optimized.resize(toOptimize.size());

	// ============== Activate points in activation list ===================
	// Activate points by optimizing them and converting them into PointHessians
	if(multiThreading)
		treadReduce.reduce(boost::bind(&FullSystem::activatePointsMT_Reductor, this, &optimized, &toOptimize, _1, _2, _3, _4), 0, toOptimize.size(), 50);

	else
		activatePointsMT_Reductor(&optimized, &toOptimize, 0, toOptimize.size(), 0, 0);


	// Check if all the points are valid after optimization
	for(unsigned k=0;k<toOptimize.size();k++)
	{
		PointHessian* newpoint = optimized[k];
		ImmaturePoint* ph = toOptimize[k];

		if(newpoint != 0 && newpoint != (PointHessian*)((long)(-1)))
		{
			// Add new point into the optimization and active point list
			newpoint->host->immaturePoints[ph->idxInImmaturePoints]=0;
			newpoint->host->pointHessians.push_back(newpoint);

			// indirect!: Process indirect points
			if (newpoint->my_type > 4)
			{
				// if(ph->initIdepth_min > 0)
				// 	std::cout << "idepth min " << ph->initIdepth_min << " idepth max " << ph->initIdepth_max << " idepth final " << newpoint->idepth << std::endl;
				// if(ph->priorFromInd > 0.0)
				// 	newpoint->priorFromInd = ph->priorFromInd;

				{ //strategy: block new indirect mapPoint from being created:
					auto oldMp = newpoint->host->shell->frame->getMapPoint(newpoint->my_type - 5);
					if (!oldMp)
					{
						std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint>(newpoint, globalMap);
						newpoint->host->shell->frame->addMapPoint(pMP);
						newpoint->Mp = pMP;
						pMP->AddObservation(pMP->sourceFrame, pMP->index);
						globalMap->AddMapPoint(pMP);

					}
					else
					{
						// std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint>(newpoint, globalMap);
						// // newpoint->host->shell->frame->addMapPoint(pMP);
						// newpoint->Mp = pMP;
						// pMP->AddObservation(pMP->sourceFrame, pMP->index);
						// globalMap->AddMapPoint(pMP);
						// oldMp->Replace(pMP);
						// // newpoint->priorFromInd =  oldMp->getidepthHessian(); //This point hessian was traced from an indirect map Point depth prior
					}
				}

				// { //strategy 2: replace old with new mappoint depth estimate and transfer connectivity information of old!
				// 	std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint>(newpoint, globalMap);
				// 	if (!newpoint->host->shell->frame->getMapPoint(newpoint->my_type - 5))
				// 	{
				// 		newpoint->host->shell->frame->addMapPoint(pMP);
				// 		pMP->AddObservation(pMP->sourceFrame, pMP->index);
				// 	}
				// 	else
				// 	{
				// 		newpoint->host->shell->frame->getMapPoint(newpoint->my_type - 5)->Replace(pMP);
				// 		// newpoint->hasDepthPrior = true; //This point hessian was traced from an indirect map Point depth prior
				// 	}
				// 	newpoint->Mp = pMP;
				// 	globalMap->AddMapPoint(pMP);
				// }


				//Some immature points could have been initialized from a known indirect map Point that was later removed due to KF culling, use its data:
				
			}
		

			ef->insertPoint(newpoint);
			for(PointFrameResidual* r : newpoint->residuals)
				ef->insertResidual(r);
			assert(newpoint->efPoint != 0);
			delete ph;
		}
		else if(newpoint == (PointHessian*)((long)(-1)) || ph->lastTraceStatus==IPS_OOB)
		{
			delete ph;
			if(!ph->Mp.expired())
				ph->Mp.lock()->setDirStatus(MapPoint::removed);
			ph->host->immaturePoints[ph->idxInImmaturePoints] = nullptr;
		}
		else
		{
			assert(newpoint == 0 || newpoint == (PointHessian*)((long)(-1)));
		}
	}

	// Removes immature points that have been deleted from the frames
	for (FrameHessian *host : frameHessians)
	{
		for(int i=0;i<(int)host->immaturePoints.size();i++)
		{
			if(host->immaturePoints[i]==0)
			{
				host->immaturePoints[i] = host->immaturePoints.back();
				host->immaturePoints.pop_back();
				i--;
			}
		}
	}

#ifdef ENABLE_DEPTH_DEBUG
    // DEPTH_DEBUG: ML vs HSLAM depth comparison after point activation (Better timing!)
    if (!currentMLDepthImage.empty() && frameHessians.size() > 0) {
        debugMLDepthComparison(frameHessians.back(), currentMLDepthImage);
    }
#endif

}


void FullSystem::activatePointsOldFirst()
{
	assert(false);
}

/**
 * @brief Flags bad points for removal
 * 
 */
void FullSystem::flagPointsForRemoval()
{
	assert(EFIndicesValid);

	std::vector<FrameHessian*> fhsToKeepPoints;
	std::vector<FrameHessian*> fhsToMargPoints;

	//if(setting_margPointVisWindow>0)
	{
		for(int i=((int)frameHessians.size())-1;i>=0 && i >= ((int)frameHessians.size());i--)
			if(!frameHessians[i]->flaggedForMarginalization) fhsToKeepPoints.push_back(frameHessians[i]);

		for(int i=0; i< (int)frameHessians.size();i++)
			if(frameHessians[i]->flaggedForMarginalization) fhsToMargPoints.push_back(frameHessians[i]);
	}



	//ef->setAdjointsF();
	//ef->setDeltaF(&Hcalib);
	int flag_oob=0, flag_in=0, flag_inin=0, flag_nores=0;

	for(FrameHessian* host : frameHessians)		// go through all active frames
	{
		for(unsigned int i=0;i<host->pointHessians.size();i++)
		{
			PointHessian* ph = host->pointHessians[i];
			if(ph==0) continue;

			// Remove points that have invalid depths
			if(ph->idepth_scaled < 0 || ph->residuals.size()==0)
			{
				host->pointHessiansOut.push_back(ph);
				ph->efPoint->stateFlag = EFPointStatus::PS_DROP;
				if(!ph->Mp.expired())
					ph->Mp.lock()->setDirStatus(MapPoint::removed);
				host->pointHessians[i] = 0;
				flag_nores++;
			}
			// Remove points that are out of bounds or are in frames that are to be marginalized
			else if(ph->isOOB(fhsToKeepPoints, fhsToMargPoints) || host->flaggedForMarginalization)
			{
				flag_oob++;
				if(ph->isInlierNew()) // Case for points with low residual values
				{
					flag_in++;
					int ngoodRes=0;
					// Linearize all
					for(PointFrameResidual* r : ph->residuals)
					{
						r->resetOOB();
						r->linearize(&Hcalib);
						r->efResidual->isLinearized = false;
						r->applyRes(true);
						if(r->efResidual->isActive())
						{
							r->efResidual->fixLinearizationF(ef);
							ngoodRes++;
						}
					}
                    if(ph->idepth_hessian > setting_minIdepthH_marg)
					{
						flag_inin++;
						ph->efPoint->stateFlag = EFPointStatus::PS_MARGINALIZE;
						host->pointHessiansMarginalized.push_back(ph);
						
						// indirect!: Remove indirect point
						if(!ph->Mp.expired())
							ph->Mp.lock()->setDirStatus(MapPoint::marginalized);
					}
					else
					{
						ph->efPoint->stateFlag = EFPointStatus::PS_DROP;
						host->pointHessiansOut.push_back(ph);
						// indirect!: Remove indirect point
						if(!ph->Mp.expired())
							ph->Mp.lock()->setDirStatus(MapPoint::removed);
					}
				}
				else
				{
					host->pointHessiansOut.push_back(ph);
					ph->efPoint->stateFlag = EFPointStatus::PS_DROP;
					// indirect!: Remove indirect point
					if(!ph->Mp.expired())
						ph->Mp.lock()->setDirStatus(MapPoint::removed);

					//printf("drop point in frame %d (%d goodRes, %d activeRes)\n", ph->host->idx, ph->numGoodResiduals, (int)ph->residuals.size());
				}

				host->pointHessians[i]=0;
			}
		}


		// Remove points that are flagged for removal from the pointHessians list
		for(int i=0;i<(int)host->pointHessians.size();i++)
		{
			if(host->pointHessians[i]==0)
			{
				host->pointHessians[i] = host->pointHessians.back();
				host->pointHessians.pop_back();
				i--;
			}
		}
	}

}

/**
 * @brief Executes odometry when new frame is added
 * 
 * This function is the starting point for most of the other functions
 * 
 * The function is passed the IMU-data from the previous frame until the current frame.
 * 
 * @param image 
 * @param id 
 * @param imuData 
 * @param gtData 
 */
/**
 * @brief RGB-D tracking method - establishes the pipeline for future depth integration
 * 
 * For now, this method simply calls the existing monocular tracking pipeline.
 * The depth image is loaded and validated but not yet used in the tracking process.
 * This establishes the interface for future depth integration in Milestone 2.2.
 * 
 * @param rgb_img RGB image as CV_32FC1
 * @param depth_img Depth image as CV_32FC1 in meters
 * @param timestamp Frame timestamp
 */
void FullSystem::TrackRGBD(const cv::Mat& rgb_color, const cv::Mat& rgb_image, const cv::Mat& depth_image, const double timestamp)
{
    if(isLost) return;
    
    // Validate input images
    if(rgb_color.empty() || rgb_image.empty() || depth_image.empty()) {
        printf("ERROR: Empty images in TrackRGBD\n");
        return;
    }

    // Log frame processing details
    if (!setting_debugout_runquiet) {
        printf("TrackRGBD: Processing frame with timestamp %.6f, RGB %dx%d, Depth %dx%d\n",
               timestamp, rgb_image.cols, rgb_image.rows, depth_image.cols, depth_image.rows);
    }
    
    // Store depth image for use in makeNewTraces (thread-safe)
    {
        boost::unique_lock<boost::mutex> lock(rgbd_depth_mutex_);
        currentDepthImage = depth_image.clone();
    }
    
    // Store RGB color for ML keyframe submissions (thread-safe)
    {
        boost::unique_lock<boost::mutex> lock(ml_depth_mutex_);
        last_bgr_frame_ = rgb_color.clone();  // Store actual RGB for ML
    }
    
    // Synchronize depth with coarse trackers immediately
    synchronizeDepthWithTracking();
    
    // Create ImageAndExposure from RGB for compatibility with existing pipeline
    ImageAndExposure* img = new ImageAndExposure(rgb_image.cols, rgb_image.rows, timestamp);
    memcpy(img->image, rgb_image.data, rgb_image.cols * rgb_image.rows * sizeof(float));
    
    
    static int frame_id = 0;
    addActiveFrame(img, frame_id++);
    
    delete img;
}

/**
 * @brief Enhanced monocular tracking with ML depth estimation (Phase 2)
 * 
 * Uses asynchronous MLDepthService with multiple inference strategies.
 * Submits frames for ML processing and retrieves results when available.
 * Falls back to standard monocular tracking if ML depth unavailable.
 * 
 * @param rgb_color RGB color image for ML processing
 * @param original_img Original ImageAndExposure with both image and PhoUncalibImage buffers
 */
void FullSystem::TrackMonocularWithML(const cv::Mat& rgb_color, ImageAndExposure* original_img)
{
    if(isLost) return;
    
    // Extract data from original ImageAndExposure
    if(!original_img || !original_img->image) {
        printf("ERROR: Invalid ImageAndExposure in TrackMonocularWithML\n");
        return;
    }
    
    double timestamp = original_img->timestamp;
    cv::Mat rgb_img(original_img->h, original_img->w, CV_32FC1, original_img->image);
    
    // Validate extracted image
    if(rgb_img.empty()) {
        printf("ERROR: Empty grayscale image extracted from ImageAndExposure\n");
        return;
    }
    
    // Store RGB color frame for keyframe ML submissions
    {
        boost::unique_lock<boost::mutex> lock(ml_depth_mutex_);
        if (!rgb_color.empty() && rgb_color.channels() == 3) {
            last_bgr_frame_ = rgb_color.clone();  // Store actual RGB for ML
            // DEBUG: Stored RGB for ML %dx%d channels=%d\n", 
            //        rgb_color.cols, rgb_color.rows, rgb_color.channels());
        } else {
            // Fallback: convert grayscale to RGB
            printf("WARNING: No RGB color available, using grayscale->RGB conversion %dx%d\n",
                   rgb_img.cols, rgb_img.rows);
            cv::cvtColor(rgb_img, last_bgr_frame_, cv::COLOR_GRAY2RGB);
        }
    }
    
    // Bounds checking: Skip ML depth retrieval if not enough frames (but keep ML enabled for future)
    if (frameHessians.size() < 2) {
        // DEBUG: Not enough frames for ML retrieval (%zu), skipping ML for this frame only\n", frameHessians.size());
        // NOTE: ml_depth_enabled_ stays true - we just skip this frame, don't permanently disable ML
    }
    
    cv::Mat ml_depth_image;
    bool ml_depth_available = false;
    
    // Phase 2: Frame-based ML depth processing  
    ml_frame_counter_++;
    
    // FIXED: Single atomic access to ML reference data (thread-safe)
    if (ml_depth_enabled_) {
        float ml_confidence = 0.0f;
        double ml_inference_time = 0.0;
        int ref_frame_id = -1;
        
        // Single atomic access to ML reference with minimal cloning
        {
            boost::unique_lock<boost::mutex> lock(ml_reference_mutex_);
            
            // DEBUG: Log variable values to understand why defensive checks pass
            // DEBUG: Frame %d - Checking ML reference: frame_id=%d, depth.empty=%d, depth.data=%p, rows=%d, cols=%d\n",
            //        ml_frame_counter_, ml_reference_frame_id_, ml_reference_depth_.empty(), 
            //        ml_reference_depth_.data, ml_reference_depth_.rows, ml_reference_depth_.cols());
            
            // Force ml_reference to be invalid if frame_id is negative (safety check)
            if (ml_reference_frame_id_ < 0) {
                ml_reference_depth_ = cv::Mat();  // Force empty
                // DEBUG: Frame %d - Forced ML reference to empty (frame_id < 0)\n", ml_frame_counter_);
            }
            
            // Defensive checks: ensure ML reference is valid and not corrupted
            if (!ml_reference_depth_.empty() && 
                ml_reference_depth_.data != nullptr &&
                ml_reference_frame_id_ >= 0 && 
                ml_reference_frame_id_ < ml_frame_counter_ &&
                ml_reference_depth_.rows > 0 && 
                ml_reference_depth_.cols > 0) {
                
                // Single clone operation with all data accessed atomically
                ml_depth_image = ml_reference_depth_.clone();
                ref_frame_id = ml_reference_frame_id_;
                ml_confidence = ml_reference_confidence_;
                ml_inference_time = ml_reference_time_;
                ml_depth_available = true;
            }
        } // Release lock early
        
        if (ml_depth_available) {
            
            // Update metrics
            ml_metrics_.avg_ml_inference_time_ms = ml_inference_time;
            
            // Store ML depth for other components (direct assignment, no redundant clone)
            {
                boost::unique_lock<boost::mutex> lock(ml_depth_mutex_);
                current_ml_depth_image_ = ml_depth_image;
            }
            
            // Comprehensive validation before printf to prevent crash from corrupted data
            if (ref_frame_id < 0 || ref_frame_id > 100000) {
                printf("ERROR: Invalid ref_frame_id=%d at Frame %d\n", ref_frame_id, ml_frame_counter_);
                ml_depth_available = false;
            } else if (ml_frame_counter_ < 0 || ml_frame_counter_ > 100000) {
                printf("ERROR: Invalid ml_frame_counter=%d\n", ml_frame_counter_);
                ml_depth_available = false;
            } else if (!ml_depth_image.data || ml_depth_image.empty()) {
                printf("ERROR: Invalid ml_depth_image at Frame %d\n", ml_frame_counter_);
                ml_depth_available = false;
            } else if (ml_depth_image.rows <= 0 || ml_depth_image.cols <= 0) {
                printf("ERROR: Invalid ml_depth_image dimensions %dx%d at Frame %d\n", 
                       ml_depth_image.cols, ml_depth_image.rows, ml_frame_counter_);
                ml_depth_available = false;
            } else {
                // Safe to print - all variables validated
                // DEBUG: Frame %d - ML available: YES (from frame %d), SLAM initialized: %s\n",
                //        ml_frame_counter_, ref_frame_id,
                //        initialized ? "YES" : "NO");
                
                // printf("TrackMonocularWithML: Using ML depth from keyframe %d (%.1fms), size %dx%d, conf=%.2f\n", 
                //        ref_frame_id, ml_inference_time, 
                //        ml_depth_image.cols, ml_depth_image.rows, ml_confidence);
            }
        }
    }

    if (!ml_depth_available) {
        // DEBUG: Frame %d - ML available: NO, SLAM initialized: %s\n",
        //        ml_frame_counter_, initialized ? "YES" : "NO");
    }
    
    // Update frame counter for ML processing synchronization
    
    // Update ML utilization rate
    if (!allKeyFramesHistory.empty()) {
        ml_metrics_.ml_depth_utilization = (float)ml_metrics_.ml_keyframes_successful / allKeyFramesHistory.size();
    }
    
    // Store ML depth if available for unified pipeline
    if(ml_depth_available) {
        // Check resolution compatibility with HSLAM processing
        if(ml_depth_image.cols != wG[0] || ml_depth_image.rows != hG[0]) {
            // Fallback: Resize ML depth if dimensions don't match (should be rare now)
            cv::Mat resized_ml_depth;
            cv::resize(ml_depth_image, resized_ml_depth, 
                      cv::Size(wG[0], hG[0]), 0, 0, cv::INTER_LINEAR);
            currentMLDepthImage = resized_ml_depth;
            printf("WARNING: ML depth resolution mismatch - resized from %dx%d to %dx%d (consider using undistorted RGB input)\n",
                   ml_depth_image.cols, ml_depth_image.rows, wG[0], hG[0]);
        } else {
            // Optimal case: Resolutions match, no resizing needed
            currentMLDepthImage = ml_depth_image;
            static bool first_match_logged = false;
            if (!first_match_logged) {
                printf("ML depth resolution matches HSLAM (%dx%d) - optimal pipeline\n", wG[0], hG[0]);
                first_match_logged = true;
            }
        }
    } else {
        // Clear any existing ML depth for pure monocular
        currentMLDepthImage = cv::Mat();
        // DEBUG: No ML depth available, using pure monocular tracking\n");
    }
    
    // Clear any existing RGB-D depth to use pure monocular/ML approach
    currentDepthImage = cv::Mat();
    
    // FIX: Use original ImageAndExposure which has both image and PhoUncalibImage buffers properly populated
    // This preserves the PhoUncalibImage buffer needed for indirect SLAM (ORB feature detection)
    static int frame_id_tracking = 0;
    addActiveFrame(original_img, frame_id_tracking++);
}


void FullSystem::addActiveFrame( ImageAndExposure* image, int id )
{
	// ML processor warmup is now handled in main.cpp during initialization

    if(isLost) return;
	boost::unique_lock<boost::mutex> lock(trackMutex);

	// =========================== Add new frame into allFrameHistory =========================
	// Initialize variables
	FrameHessian* fh = new FrameHessian();
	FrameShell* shell = new FrameShell();

	// indirect!: Create indirect frame
	// Indirect point are detected when frame is made
	
	shell->frame = std::make_shared<Frame>(image->PhoUncalibImage, detector, &Hcalib, fh, shell, globalMap);
	
	// std::cout << shell->frame->nFeatures << std::endl;
	// cv::Mat Output;
	// shell->frame->Image.convertTo(Output, CV_8UC3);
	// cv::drawKeypoints(Output, shell->frame->mvKeys, Output, cv::Scalar(0, 255, 0));
	// cv::namedWindow("test2", cv::WINDOW_KEEPRATIO);
	// cv::imshow("test2", Output);
	// cv::waitKey(1);

	shell->marginalizedAt = shell->id = allFrameHistory.size();
    shell->timestamp = image->timestamp;
    shell->incoming_id = id;
	fh->shell = shell;
	allFrameHistory.push_back(shell);


    // =========================== Place image and image settings into frame =========================
	fh->ab_exposure = image->exposure_time;
    	fh->makeImages(image->image, &Hcalib); // Image derivative and gradient is also calculated
	if(image->useColour){
		fh->makeColourImages(image->r_image, image->g_image, image->b_image);
	}



	// =========================== Process Image =========================
	if(!initialized) // Need initialization case
	{
		// =========================== Init using image if needed =========================
		if(coarseInitializer->frameID<0)	// first frame set. fh is kept by coarseInitializer.
		{
			// Start initialization timing
			init_start_time_ = std::chrono::high_resolution_clock::now();
			printf("\n=== INITIALIZATION STARTED ===\n");
			
			coarseInitializer->setFirst(&Hcalib, fh);
			
			// Process ML depth for initialization to establish metric scale
			if (ml_depth_enabled_ && ml_processor_ && ml_processor_->isReady()) {
				cv::Mat rgb_image;
				{
					boost::unique_lock<boost::mutex> lock(ml_depth_mutex_);
					if (!last_bgr_frame_.empty()) {
						rgb_image = last_bgr_frame_.clone();
					}
				}
				
				// Check if we have warmup results available first (optimization)
				if (warmup_results_available_) {
					printf("[SCALE_INIT] Using GPU warmup results for metric scale...\n");
					coarseInitializer->setMLDepth(warmup_depth_map_, 
												 warmup_confidence_,
												 warmup_mean_depth_);
					printf("[SCALE_INIT] Metric scale set from warmup: %.2fm (saved ~70ms processing)\n", 
						   warmup_mean_depth_);
					
					// Seed initializer points with ML inverse depth
					coarseInitializer->seedPointsWithMLDepth();
					// Clear the stored results to free memory
					warmup_depth_map_.release();
					warmup_results_available_ = false;
				} else if (!rgb_image.empty()) {
					printf("[SCALE_INIT] Processing ML depth for metric scale initialization...\n");
					auto ml_result = ml_processor_->processKeyframeDetailed(rgb_image);
					if (ml_result.success) {
						coarseInitializer->setMLDepth(ml_result.depth_map, 
													 ml_result.confidence,
													 ml_result.mean_depth);
						printf("[SCALE_INIT] ML depth set for initialization (mean=%.2fm, confidence=%.2f)\n", 
							   ml_result.mean_depth, ml_result.confidence);
						// Seed initializer points with ML inverse depth
						coarseInitializer->seedPointsWithMLDepth();
					} else {
						printf("[SCALE_INIT] ML inference failed, falling back to photometric scale\n");
					}
				} else {
					printf("[SCALE_INIT] No RGB image available, falling back to photometric scale\n");
				}
			} else {
				printf("Initialization: Using standard monocular initialization (ML depth disabled)\n");
			}
		}
		else if(coarseInitializer->trackFrame(fh, outputWrapper))	// if SNAPPED
		{

			initializeFromInitializer(fh);
			
			// Calculate total initialization time
			auto init_end = std::chrono::high_resolution_clock::now();
			total_init_time_ms_ = std::chrono::duration<double, std::milli>(
				init_end - init_start_time_).count();
			
			// Calculate pure tracking time (total - ML processing)
			double tracking_time_ms = total_init_time_ms_ - ml_init_processing_time_ms_;
			
			// Log initialization performance using FPSLogger
			FPSLogger::logInitializationPerformance(
				total_init_time_ms_,
				ml_init_processing_time_ms_,
				tracking_time_ms,
				using_metric_scale_ ? "Direct Metric" : "Photometric",
				init_scale_factor_,
				init_points_count_,
				using_metric_scale_ ? coarseInitializer->mlConfidence : 0.0f
			);
			
			// CRITICAL FIX: DEFER metric conversion until AFTER RMSE validation passes
			// The metric conversion must happen AFTER the RMSE check, not before
			// if (using_metric_scale_ && !scale_aligned_) {
			//     printf("[METRIC_CONVERSION] Initialization successful - applying metric conversion\n");
			//     applyMetricScaleConversion();
			// }
			// NOTE: Metric conversion now happens in makeKeyFrame() after RMSE validation
			
			lock.unlock();
			deliverTrackedFrame(fh, true);
		}
		else
		{
			// if still initializing
			fh->shell->poseValid = false;
			if(fh->shell->frame)
				fh->shell->frame.reset();
			delete fh;
			fh = nullptr;
		}
		return;
	}
	else // do standard front-end operation.
	{
		
		// =========================== Swap tracking reference =========================
		if(coarseTracker_forNewKF->refFrameID > coarseTracker->refFrameID)
		{
			boost::unique_lock<boost::mutex> crlock(coarseTrackerSwapMutex);
			CoarseTracker* tmp = coarseTracker; coarseTracker=coarseTracker_forNewKF; coarseTracker_forNewKF=tmp;
			
			// Safety check: Ensure the swapped tracker is initialized
			// This should not be needed anymore with Fix 1, but good to have as backup
			if(coarseTracker->w[0] == 0 || coarseTracker->h[0] == 0) {
				coarseTracker->makeK(&Hcalib);
				if (!setting_debugout_runquiet) {
					printf("CoarseTracker: Initialized swapped tracker with calibration data\n");
				}
			}
		}

		// Velocity = cumulativeForm();
		
		shell->setPose(mLastFrame->fs->getPose() * Velocity.inverse()); //Velocity * LastFrameTcw
		nIndmatches = 0;
		isUsable = false;
		bool computedBoW = false;
		
		
		CheckReplacedInLastFrame();

		int nMatches;
		// indirect!: Update the loop closer and indirect points
		nMatches = matcher->SearchByProjectionFrameToFrame(shell->frame, mLastFrame, 15, true);

		if (nMatches < 20)
		{
			DBoW3::Vocabulary* weakVocanpnt = loopCloser->getVocab();
			nMatches  = matcher->SearchByBoWTracking(mpReferenceKF, shell->frame, 0.7, true, shell->frame->tMapPoints, weakVocanpnt);
			computedBoW = true;
		}

		isUsable = PoseOptimization(shell->frame, &Hcalib);

		if (!isUsable && !computedBoW)
		{
			DBoW3::Vocabulary* weakVocanpnt = loopCloser->getVocab();
			nMatches  = matcher->SearchByBoWTracking(mpReferenceKF, shell->frame, 0.7, true, shell->frame->tMapPoints, weakVocanpnt);
			isUsable = PoseOptimization(shell->frame, &Hcalib);
			computedBoW = true;
		}

		nIndmatches = updatePoseOptimizationData(shell->frame, nMatches, true);

		// int nFrametoLocalMapMatches = SearchLocalPoints(shell->frame);
		// PoseOptimization(shell->frame, &Hcalib, isUsable); //isUsable
		// nIndmatches = updatePoseOptimizationData(shell->frame, nFrametoLocalMapMatches, false);


		// =========================== Do coarse tracking =========================
		// Synchronize depth information before coarse tracking
		synchronizeDepthWithTracking();
		
		// Coarse Tracking is only done on only the new frame and reference frame
		// The reference frame should be the latest keyframe
		Vec5 tres = trackNewCoarse(fh, ! (isUsable && computedBoW) );
		

		int nFrametoLocalMapMatches = SearchLocalPoints(shell->frame);
		// checkOutliers(shell->frame, &Hcalib);
		// PoseOptimization(shell->frame, &Hcalib, isUsable); //isUsable
		nIndmatches = updatePoseOptimizationData(shell->frame, nFrametoLocalMapMatches, false);

		shell->frame->mpReferenceKF = mpReferenceKF;

		if (!std::isfinite((double)tres[0]) || !std::isfinite((double)tres[1]) || !std::isfinite((double)tres[2]) || !std::isfinite((double)tres[3]))
		{
			printf("Initial Tracking failed: LOST!\n");
			isLost = true;
			return;
		}
	
		// =========================== Make keyframe or non-keyframe =========================
		bool needToMakeKF = false;
		// Decide if keyframe or non-keyframe needs to be made
		// If setting_keyframesPerSecond is set, the keyframe is made at a specific frequency
		// Otherwise, the keyframe is made depending on specific conditions

		if(setting_keyframesPerSecond > 0) // fixed keyframe rate
		{
			needToMakeKF = allFrameHistory.size() == 1 ||
						   (fh->shell->timestamp - allKeyFramesHistory.back()->timestamp) > 0.95f / setting_keyframesPerSecond;
			needToMakeKF = needToMakeKF && (tres[4] > 0.0);

		}
		else // makes keyframe under specific conditions
		{
			Vec2 refToFh = AffLight::fromToVecExposure(coarseTracker->lastRef->ab_exposure, fh->ab_exposure,
													   coarseTracker->lastRef_aff_g2l, fh->shell->aff_g2l);

			// Make keyframe if:
			// 1. If the field of view changes too much. FOV change is measured by the mean optical flow
			// 2. If motion causes occlusions and disocclusions. This is measured by mean translation flow
			// 3. If camera exposure is changed significantly.
			// 4. Residual is too high.
			// 5. If max time between keyframes is surpassed.
			// 6. The IMU system needs a keyframe.
			needToMakeKF = allFrameHistory.size() == 1 ||
						   setting_kfGlobalWeight * setting_maxShiftWeightT * sqrtf((double)tres[1]) / (wG[0] + hG[0]) +
								   setting_kfGlobalWeight * setting_maxShiftWeightR * sqrtf((double)tres[2]) / (wG[0] + hG[0]) +
								   setting_kfGlobalWeight * setting_maxShiftWeightRT * sqrtf((double)tres[3]) / (wG[0] + hG[0]) +
								   setting_kfGlobalWeight * setting_maxAffineWeight * fabs(logf((float)refToFh[0])) >
							   1 ||
						   2 * coarseTracker->firstCoarseRMSE < tres[0];
			needToMakeKF = needToMakeKF && (tres[4] > 0.0);
		}

		//if frame succesfully tracked, update global motion model and set it to become the reference frame for the next frame

		Velocity = shell->getPoseInverse() * mLastFrame->fs->getPose(); //currentTcw * LastTwc
		// vVelocity.push(Velocity);

		mLastFrame = shell->frame;



		for (IOWrap::Output3DWrapper *ow : outputWrapper)
		{
			ow->publishCamPose(fh->shell, &Hcalib);
			ow->pushLiveFrame(fh, nIndmatches);
		}
		
		lock.unlock();
		deliverTrackedFrame(fh, needToMakeKF);
		return;
	}
}


/**
 * @brief Creates a keyframe or non-keyframe
 * 
 * Also parses outputted frame for GUI and debugging
 * Helper function for addActiveFrame
 * 
 * @param fh 
 * @param needKF 
 */
void FullSystem::deliverTrackedFrame(FrameHessian* fh, bool needKF)
{


	if(linearizeOperation)
	{
		if(goStepByStep && lastRefStopID != coarseTracker->refFrameID)
		{
			MinimalImageF3 img(wG[0], hG[0], fh->dI);
			IOWrap::displayImage("frameToTrack", &img);
			while(true)
			{
				char k=IOWrap::waitKey(0);
				if(k==' ') break;
				handleKey( k );
			}
			lastRefStopID = coarseTracker->refFrameID;
		}
		else handleKey( IOWrap::waitKey(1) );



		if(needKF) makeKeyFrame(fh);
		else makeNonKeyFrame(fh);
	}
	else
	{
		boost::unique_lock<boost::mutex> lock(trackMapSyncMutex);
		unmappedTrackedFrames.push_back(fh);
		if(needKF) needNewKFAfter=fh->shell->trackingRefId;
		trackedFrameSignal.notify_all();

		while(coarseTracker_forNewKF->refFrameID == -1 && coarseTracker->refFrameID == -1 )
		{
			mappedFrameSignal.wait(lock);
		}

		lock.unlock();
	}
}

/**
 * @brief Map using the set of active frames
 * 
 * Mapping is done when a keyframe is created
 * Runs on its own thread
 * 
 */
void FullSystem::mappingLoop()
{
	boost::unique_lock<boost::mutex> lock(trackMapSyncMutex);

	while(runMapping)
	{
		while(unmappedTrackedFrames.size()==0) // do not run if there are no frames
		{
			trackedFrameSignal.wait(lock);
			if(!runMapping) return;
		}

		FrameHessian* fh = unmappedTrackedFrames.front();
		unmappedTrackedFrames.pop_front();


        // Guaranteed to make a KF for the very first two tracked frames.
		if(allKeyFramesHistory.size() <= 2)
		{
			lock.unlock();
			makeKeyFrame(fh);
			lock.lock();
			mappedFrameSignal.notify_all();
			continue;
		}

		if(unmappedTrackedFrames.size() > 3)
			needToKetchupMapping=true;


		// if there are other frames to track, do that first.
		if(unmappedTrackedFrames.size() > 0)
		{
			lock.unlock();
			makeNonKeyFrame(fh);
			lock.lock();

			if(needToKetchupMapping && unmappedTrackedFrames.size() > 0)
			{
				FrameHessian* fh = unmappedTrackedFrames.front();
				unmappedTrackedFrames.pop_front();
				{
					// boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
					fh->setEvalPT_scaled(fh->shell->getPose().inverse(),fh->shell->aff_g2l);
				}

				if(fh->shell->frame)
					fh->shell->frame.reset();
				delete fh;
				fh = nullptr;
			}
		}
		else
		{
			if(setting_realTimeMaxKF || needNewKFAfter >= frameHessians.back()->shell->id)
			{
				lock.unlock();
				makeKeyFrame(fh);
				needToKetchupMapping=false;
				lock.lock();
			}
			else
			{
				lock.unlock();
				makeNonKeyFrame(fh);
				lock.lock();
			}
		}
		mappedFrameSignal.notify_all();
	}
	printf("MAPPING FINISHED!\n");
}

void FullSystem::blockUntilMappingIsFinished()
{
	boost::unique_lock<boost::mutex> lock(trackMapSyncMutex);
	runMapping = false;
	trackedFrameSignal.notify_all();
	lock.unlock();

	// indirect!: Handle loop closure
	if (loopCloser)
	{
		loopCloser->SetFinish(true);
		// if (globalMap->NumFrames() > 4)
		// {
		// 	globalMap->lastOptimizeAllKFs();
		// }
	}

	mappingThread.join();
	printf("Mapping Thread has been joined\n"); //debugNA

}

/**
 * @brief Creates a non-keyframe
 * 
 * @param fh 
 */
void FullSystem::makeNonKeyFrame( FrameHessian* fh)
{
	// needs to be set by mapping thread. no lock required since we are in mapping thread.
	{
		// boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
		fh->setEvalPT_scaled(fh->shell->getPose().inverse(),fh->shell->aff_g2l);
	}

	traceNewCoarse(fh);

	if(fh->shell->frame)
		fh->shell->frame.reset();
	delete fh;
	fh = nullptr;
}

/**
 * @brief Create a keyframe and optimize all of the active frames
 * 
 * @param fh 
 */
void FullSystem::makeKeyFrame( FrameHessian* fh)
{
	// needs to be set by mapping thread
	{
		// boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
		fh->setEvalPT_scaled(fh->shell->getPoseInverse(),fh->shell->aff_g2l);
		fh->shell->setPoseOpti(Sim3(fh->shell->getPoseInverse().matrix()));
	}


	traceNewCoarse(fh);

	

	boost::unique_lock<boost::mutex> lock(mapMutex);
	
	// =========================== Flag Frames to be Marginalized. =========================
	flagFramesForMarginalization(fh);


	// =========================== add New Frame to Hessian Struct. =========================
	
	fh->idx = frameHessians.size();
	frameHessians.push_back(fh);
	
	// CRITICAL FIX: Set frameID BEFORE ML submission to ensure correct synchronization
	fh->shell->KfId = allKeyFramesHistory.back()->KfId + 1;
	fh->frameID = allKeyFramesHistory.size();
	

	// =================== ABLATION STUDY: ML FREQUENCY CONTROL ===================
	// Increment keyframe counter for ablation tracking
	keyframe_counter_++;
	
	// Determine if ML inference should run based on strategy
	bool should_run_ml = false;
	
	if (ml_config_.inference_strategy == "snapshot_mode") {
		// Snapshot mode: Run ML every N keyframes
		should_run_ml = ((keyframe_counter_ - 1) % ml_config_.snapshot_rate == 0);
		
		if (should_run_ml) {
			printf("[ML_ABLATION] Keyframe %zu: RUNNING ML inference (snapshot rate=%d)\n", 
			       keyframe_counter_, ml_config_.snapshot_rate);
			last_ml_keyframe_ = keyframe_counter_;
		} else {
			size_t next_ml_kf = ((keyframe_counter_ - 1) / ml_config_.snapshot_rate + 1) 
			                    * ml_config_.snapshot_rate + 1;
			printf("[ML_ABLATION] Keyframe %zu: SKIPPING ML (last ML at KF %zu, next at KF %zu)\n", 
			       keyframe_counter_, last_ml_keyframe_, next_ml_kf);
		}
	} else {
		// Default mode: Run ML every Nth keyframe (Paper Table V: N=2 optimal)
		should_run_ml = ((keyframe_counter_ - 1) % setting_mlInferenceEveryN == 0);
		if (should_run_ml)
			last_ml_keyframe_ = keyframe_counter_;
	}

	// =================== ML DEPTH PROCESSOR INTEGRATION ===================
	// Direct synchronous ML depth processing for current keyframe
	if (ml_depth_enabled_ && ml_processor_ && ml_processor_->isReady() && should_run_ml) {
		// Signal that ML processing is starting
		{
			boost::unique_lock<boost::mutex> lock(ml_processing_mutex_);
			ml_processing_active_ = true;
			// DEBUG: ML processing STARTED for keyframe %d\n", fh->frameID);
		}
		cv::Mat rgb_image;
		{
			boost::unique_lock<boost::mutex> lock(ml_depth_mutex_);
			if (!last_bgr_frame_.empty()) {
				rgb_image = last_bgr_frame_.clone();
			} else {
				printf("makeKeyFrame: Warning - no BGR frame available for ML processing\n");
			}
		}
		
		if (!rgb_image.empty()) {
			// TEMP_DEBUG_REPETITIVE: Commented out for cleaner testing output
			// printf("makeKeyFrame: Processing keyframe %d with ML processor...\n", fh->frameID);
			// DEBUG: makeKeyFrame - RGB image %dx%d channels=%d\n", 
			//        rgb_image.cols, rgb_image.rows, rgb_image.channels());
		
			// Validate inputs before ML processing
			if (!fh) {
				printf("ERROR: makeKeyFrame - FrameHessian is null, skipping ML processing\n");
				return;
			}
			
			if (rgb_image.cols <= 0 || rgb_image.rows <= 0 || rgb_image.channels() != 3) {
				printf("ERROR: makeKeyFrame - Invalid RGB image dimensions %dx%d channels=%d\n",
					   rgb_image.cols, rgb_image.rows, rgb_image.channels());
				return;
			}
			
			if (!ml_processor_->isReady()) {
				printf("ERROR: makeKeyFrame - ML processor not ready\n");
				return;
			}
			
			// === DEBUG_ML_PHASE1: Pre-inference validation ===
			// printf("DEBUG_ML_PHASE1: About to call second ML inference (keyframe processing)\n");
			// printf("DEBUG_ML_PHASE1: RGB image validation - size: %dx%d, channels: %d, type: %d\n", 
			//        rgb_image.cols, rgb_image.rows, rgb_image.channels(), rgb_image.type());
			// printf("DEBUG_ML_PHASE1: ML processor state - isReady: %s\n", 
			//        ml_processor_->isReady() ? "true" : "false");
			// === END DEBUG_ML_PHASE1 ===
			
			// Count this as an attempted keyframe (after validation passes)
			ml_metrics_.ml_keyframes_attempted++;
			
			// Track ML inference for ablation study
			ml_inference_counter_++;
			
			try {
				auto start_time = std::chrono::high_resolution_clock::now();
				// printf("DEBUG_ML_PHASE1: Calling ml_processor_->processKeyframeDetailed()...\n");
				auto ml_result = ml_processor_->processKeyframeDetailed(rgb_image);
				auto end_time = std::chrono::high_resolution_clock::now();
				auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
				
				// printf("DEBUG_ML_PHASE1: ml_processor_->processKeyframeDetailed() returned successfully!\n");
				// printf("DEBUG: makeKeyFrame - ML processing completed, success=%s\n", 
				//        ml_result.success ? "true" : "false");
					   
				// Continue with ML result processing if successful
				if (ml_result.success && !ml_result.depth_map.empty()) {
					fh->setMLDepth(ml_result.depth_map, ml_result.confidence, ml_result.inference_time_ms, ml_result.confidence_map);
					fh->setMLPending(false);  // Clear pending flag
					
					// Update ML visualization immediately
					for (IOWrap::Output3DWrapper *ow : outputWrapper) {
						IOWrap::PangolinDSOViewer* pangolin_viewer = dynamic_cast<IOWrap::PangolinDSOViewer*>(ow);
						if (pangolin_viewer) {
							pangolin_viewer->updateMLVisualization(fh);
						}
					}
					
					// DEPRECATED: Immediate scale alignment - kept for debugging/monitoring
					// With ML at initialization, scale should be correct from start
					// alignScalesImmediately(ml_result.depth_map);
					
					// NEW: Set as ML reference for tracking (CoarseTracker-style pattern)
					setMLReference(fh->frameID, ml_result.depth_map, ml_result.confidence, ml_result.inference_time_ms);
				
					// PHASE 2: Confidence map now stored directly in FrameHessian (no duplicate storage)
					if (!ml_result.confidence_map.empty()) {
						// printf("PHASE2_DEBUG: Confidence map %dx%d stored in FrameHessian (no duplicate)\n", 
						//        ml_result.confidence_map.cols, ml_result.confidence_map.rows);
					} else {
						// printf("PHASE2_DEBUG: No confidence map available from ML inference\n");
					}
				
					// Direct.P3: Forward ML depth to CoarseTracker for frame-to-frame tracking (KEYFRAMES ONLY)
					if(initialized && !currentMLDepthImage.empty()) {
						// Ensure ML depth dimensions match processing resolution
						cv::Mat ml_depth_for_tracker;
						if(currentMLDepthImage.cols != wG[0] || currentMLDepthImage.rows != hG[0]) {
							cv::resize(currentMLDepthImage, ml_depth_for_tracker, 
									  cv::Size(wG[0], hG[0]), 0, 0, cv::INTER_LINEAR);
						} else {
							ml_depth_for_tracker = currentMLDepthImage;
						}
						
						// Forward to BOTH CoarseTracker instances (CRITICAL for trajectory quality)
						coarseTracker->setExternalDepthImage(ml_depth_for_tracker);
						coarseTracker_forNewKF->setExternalDepthImage(ml_depth_for_tracker);

						// Scale drift diagnostics (pure instrumentation, no trajectory impact)
						monitorScaleDrift(fh, ml_depth_for_tracker);

						// Per-scene ML weight calibration (after warmup, sample CALIBRATION_WINDOW keyframes)
						// Skip calibration when Direct.P2 BA is disabled (which it permanently is)
						if (!setting_disableDirectP2BA && !ml_weight_calib_.is_calibrated && ef) {
							ml_weight_calib_.kf_count++;
							if (ml_weight_calib_.kf_count <= MLWeightCalibration::WARMUP_KEYFRAMES) {
								printf("[WEIGHT_CALIB] Warmup KF%d (skipping)\n", ml_weight_calib_.kf_count);
							} else {
								double photo_e = ef->last_photometric_energy_;
								double ml_e = ef->last_ml_energy_;
								// Require minimum photometric energy to filter init transient
								if (photo_e >= MLWeightCalibration::MIN_PHOTO_ENERGY) {
									ml_weight_calib_.photo_energy_samples.push_back(photo_e);
									ml_weight_calib_.ml_energy_samples.push_back(ml_e);
									int n = ml_weight_calib_.photo_energy_samples.size();
									double total = photo_e + ml_e;
									printf("[WEIGHT_CALIB] Sample %d: photo=%.1f, ml=%.1f, ratio=%.1f%%\n",
										   n, photo_e, ml_e, (total > 0) ? ml_e/total*100.0 : 0.0);
									if (n >= MLWeightCalibration::CALIBRATION_WINDOW) {
										std::vector<double> ratios;
										for (int i = 0; i < n; i++) {
											double t = ml_weight_calib_.photo_energy_samples[i]
														 + ml_weight_calib_.ml_energy_samples[i];
											if (t > 0) ratios.push_back(ml_weight_calib_.ml_energy_samples[i] / t);
											else ratios.push_back(0.0);
										}
										std::sort(ratios.begin(), ratios.end());
										double median_ratio = ratios[ratios.size() / 2];
										float multiplier;
										if (median_ratio < 1e-6) {
											// ML energy effectively zero — apply max boost
											multiplier = MLWeightCalibration::MAX_WEIGHT_MULTIPLIER;
										} else {
											multiplier = MLWeightCalibration::TARGET_ML_RATIO / median_ratio;
											multiplier = std::min(multiplier, MLWeightCalibration::MAX_WEIGHT_MULTIPLIER);
										}
										float new_weight = setting_mlDepthWeight * multiplier;
										new_weight = std::max(new_weight, MLWeightCalibration::MIN_WEIGHT);
										printf("[WEIGHT_CALIB] Calibrated: median_ratio=%.4f, multiplier=%.1f, "
											   "weight %.2f -> %.2f\n", median_ratio, multiplier,
											   setting_mlDepthWeight, new_weight);
										setting_mlDepthWeight = new_weight;
										ml_weight_calib_.calibrated_weight = new_weight;
										ml_weight_calib_.is_calibrated = true;
									}
								} else {
									printf("[WEIGHT_CALIB] KF%d: photo=%.1f too low, skipping\n",
										   ml_weight_calib_.kf_count, photo_e);
								}
							}
						}
					}

					// TEMP_DEBUG_REPETITIVE: Commented out for cleaner testing output
					// printf("makeKeyFrame: ✅ ML processing successful for frame %d (%.1fms inference, conf=%.2f, total=%.1fms)\n",
					//	   fh->frameID, ml_result.inference_time_ms, ml_result.confidence, (float)processing_time);
					
					// Update metrics
					ml_metrics_.ml_keyframes_successful++;
					ml_metrics_.ml_depth_utilization = (float)ml_metrics_.ml_keyframes_successful / allKeyFramesHistory.size();
					
					// Update ablation study metrics
					ml_metrics_.frames_skipped = keyframe_counter_ - ml_inference_counter_;
					
					// Print ablation statistics periodically (snapshot_mode only)
					if (ml_config_.inference_strategy == "snapshot_mode" &&
					    (keyframe_counter_ % 10 == 0 || keyframe_counter_ <= 5)) {
						printf("[ML_ABLATION_STATS] Progress: %zu keyframes, %zu ML inferences (%.1f%% coverage)\n",
						       keyframe_counter_, ml_inference_counter_,
						       100.0f * ml_inference_counter_ / keyframe_counter_);

						float expected_coverage = 100.0f / ml_config_.snapshot_rate;
						float actual_coverage = 100.0f * ml_inference_counter_ / keyframe_counter_;
						printf("[ML_ABLATION_STATS] Expected coverage: %.1f%%, Actual: %.1f%%\n",
						       expected_coverage, actual_coverage);
					}
			
				// DEBUG: Save ML depth maps for visual inspection (when --save flag is enabled)
				if (debugSaveImages) {
					static int ml_depth_saved_count = 0;
					const int max_saves = 10; // Save first 10 samples to avoid disk space issues
				
					if (ml_depth_saved_count < max_saves) {
						try {
							// Create filename for colorized depth visualization
							char depth_file[256];
							snprintf(depth_file, 256, "images_out/ml_depth_%05d_%02d.png", 
									 fh->frameID, ml_depth_saved_count);
							
							// Normalize depth for visualization (0-255)
							cv::Mat depth_viz;
							cv::normalize(ml_result.depth_map, depth_viz, 0, 255, cv::NORM_MINMAX, CV_8U);
							
							// Apply JET colormap for better depth visualization
							cv::Mat depth_colored;
							cv::applyColorMap(depth_viz, depth_colored, cv::COLORMAP_JET);
							cv::imwrite(depth_file, depth_colored);
							
							// Get depth range for diagnostic info
							double min_depth, max_depth;
							cv::minMaxLoc(ml_result.depth_map, &min_depth, &max_depth);
							
							// Save confidence map if available
							if (!ml_result.confidence_map.empty()) {
								// Create filename for confidence visualization  
								char confidence_file[256];
								snprintf(confidence_file, 256, "images_out/ml_confidence_%05d_%02d.png",
										 fh->frameID, ml_depth_saved_count);
								
								// Normalize confidence [0,1] to [0,255] for visualization
								cv::Mat conf_viz;
								cv::normalize(ml_result.confidence_map, conf_viz, 0, 255, cv::NORM_MINMAX, CV_8U);
								
								// Apply HOT colormap (high confidence = bright)
								cv::Mat conf_colored;
								cv::applyColorMap(conf_viz, conf_colored, cv::COLORMAP_HOT);
								cv::imwrite(confidence_file, conf_colored);
								
								// Get confidence statistics
								cv::Scalar conf_mean, conf_std;
								cv::meanStdDev(ml_result.confidence_map, conf_mean, conf_std);
								
								printf("makeKeyFrame: [SAVE] ML confidence sample %d: %s (mean=%.3f, std=%.3f)\n",
									   ml_depth_saved_count, confidence_file, conf_mean[0], conf_std[0]);
							}
							
							printf("makeKeyFrame: [SAVE] ML depth sample %d: %s (depth range: %.2f-%.2f)\n", 
								   ml_depth_saved_count, depth_file, min_depth, max_depth);
							ml_depth_saved_count++;
							
						} catch (const cv::Exception& e) {
							printf("makeKeyFrame: WARNING - Failed to save ML depth map: %s\n", e.what());
						}
					}
				}
				} else {
					printf("makeKeyFrame: ERROR - ML processing failed for frame %d: %s\n", 
						   fh->frameID, ml_result.error_message.c_str());
					fh->setMLPending(false);  // Clear pending flag on failure
				}
				
			} catch (const cv::Exception& e) {
				printf("ERROR: makeKeyFrame - OpenCV exception during ML processing: %s\n", e.what());
			} catch (const std::exception& e) {
				printf("ERROR: makeKeyFrame - Exception during ML processing: %s\n", e.what());
			} catch (...) {
				printf("ERROR: makeKeyFrame - Unknown exception during ML processing\n");
			}
		}
		
		// Signal that ML processing is completed
		{
			boost::unique_lock<boost::mutex> lock(ml_processing_mutex_);
			ml_processing_active_ = false;
			ml_processing_done_.notify_all();
			// DEBUG: ML processing COMPLETED for keyframe %d\n", fh->frameID);
		}
	}

	allKeyFramesHistory.push_back(fh->shell);
	ef->insertFrame(fh, &Hcalib);

	IndirectMapper(fh->shell->frame);

	setPrecalcValues();



	// =========================== add new residuals for old points =========================
	int numFwdResAdde=0;
	for(FrameHessian* fh1 : frameHessians)		// go through all active frames
	{
		if(fh1 == fh) continue;
		for(PointHessian* ph : fh1->pointHessians)
		{
			PointFrameResidual* r = new PointFrameResidual(ph, fh1, fh);
			r->setState(ResState::IN);
			ph->residuals.push_back(r);
			ef->insertResidual(r);
			ph->lastResiduals[1] = ph->lastResiduals[0];
			ph->lastResiduals[0] = std::pair<PointFrameResidual*, ResState>(r, ResState::IN);
			numFwdResAdde+=1;
		}
	}



	// =========================== Activate Points (& flag for marginalization). =========================
	activatePointsMT();
	ef->makeIDX();
	




	// =========================== OPTIMIZE ALL =========================

	fh->frameEnergyTH = frameHessians.back()->frameEnergyTH;
	
	// DEBUG_INIT: Track optimization before/after metric scale initialization
	// printf("[DEBUG_INIT] Before optimization: keyframes=%zu, using_metric_scale=%s, scale_factor=%.3f\n", 
	// 	   allKeyFramesHistory.size(), using_metric_scale_ ? "YES" : "NO", init_scale_factor_);
	
	float rmse = optimize(setting_maxOptIterations);
	
	// DEBUG_INIT: Log optimization results for analysis
	// printf("[DEBUG_INIT] Optimization result: RMSE=%.2f, keyframes=%zu, points=%zu/%zu\n", 
	//	   rmse, allKeyFramesHistory.size(), ef->nPoints, ef->nFrames * setting_desiredPointDensity);





	// =========================== Figure Out if INITIALIZATION FAILED =========================
	// ML-aware initialization failure detection
	bool use_ml_tolerant_thresholds = ml_depth_enabled_ && ml_processor_ && ml_processor_->isReady();
	int max_keyframes = use_ml_tolerant_thresholds ? 6 : 4; // Allow more keyframes with ML
	
	// DEBUG_INIT: Show failure detection parameters
	// printf("[DEBUG_INIT] Failure detection: RMSE=%.2f, keyframes=%zu/%d, ML_mode=%s, slack=%.1f\n", 
	//	   rmse, allKeyFramesHistory.size(), max_keyframes, 
	//	   use_ml_tolerant_thresholds ? "YES" : "NO", benchmark_initializerSlackFactor);
	
	if(allKeyFramesHistory.size() <= max_keyframes)
	{
		bool should_reset = false;
		
		float threshold = -1.0f; // Track which threshold applies
		
		if (use_ml_tolerant_thresholds) {
			// More lenient thresholds when ML depth is available
			// Increased by ~20% to account for ML depth integration complexity
			if(allKeyFramesHistory.size()==2) {
				threshold = 42*benchmark_initializerSlackFactor;  // Was 35
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==3) {
				threshold = 30*benchmark_initializerSlackFactor;  // Was 25
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==4) {
				threshold = 22*benchmark_initializerSlackFactor;  // Was 18
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==5) {
				threshold = 18*benchmark_initializerSlackFactor;  // Was 15
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==6) {
				threshold = 15*benchmark_initializerSlackFactor;  // Was 12
				if(rmse > threshold) should_reset = true;
			}
		} else {
			// Original strict thresholds for pure monocular mode
			if(allKeyFramesHistory.size()==2) {
				threshold = 20*benchmark_initializerSlackFactor;
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==3) {
				threshold = 13*benchmark_initializerSlackFactor;
				if(rmse > threshold) should_reset = true;
			} else if(allKeyFramesHistory.size()==4) {
				threshold = 9*benchmark_initializerSlackFactor;
				if(rmse > threshold) should_reset = true;
			}
		}
		
		// DEBUG_INIT: Show threshold comparison
		// printf("[DEBUG_INIT] Threshold check: RMSE=%.2f vs threshold=%.2f (%s)\n", 
		//	   rmse, threshold, should_reset ? "EXCEEDED - RESET" : "OK");
		
		if (should_reset) {
			printf("I THINK INITIALIZATINO FAILED! Resetting. (ML-tolerant mode: %s, RMSE: %.2f, keyframes: %zu)\n", 
			       use_ml_tolerant_thresholds ? "YES" : "NO", rmse, allKeyFramesHistory.size());
			initFailed=true;
		} else {
			// CRITICAL FIX: Apply metric conversion AFTER RMSE validation passes
			// This ensures conversion happens only when initialization is stable
			if (using_metric_scale_ && ml_to_slam_scale_factor_ > 0.0f) {
				// Check if full metric conversion has been applied (not just scale alignment)
				bool needs_full_conversion = true;
				for (FrameHessian* fh : frameHessians) {
					if (fh->shell->getPose().translation().norm() < 0.01f) {
						needs_full_conversion = false; // Already converted to metric scale
						break;
					}
				}
				
				if (needs_full_conversion) {
					printf("[METRIC_CONVERSION] RMSE validation passed (%.2f) - now applying metric conversion\n", rmse);
					printf("[METRIC_CONVERSION] Forcing conversion despite scale_aligned_=%s\n", scale_aligned_ ? "true" : "false");
					// Temporarily override scale_aligned_ to force conversion
					bool original_scale_aligned = scale_aligned_;
					scale_aligned_ = false;
					applyMetricScaleConversion();
					// Don't restore scale_aligned_ - let the conversion function set it
				}
			}
		}
	}



    if(isLost) return;




	// =========================== REMOVE OUTLIER =========================
	removeOutliers();




	{
		boost::unique_lock<boost::mutex> crlock(coarseTrackerSwapMutex);
		coarseTracker_forNewKF->makeK(&Hcalib);
		
		// Synchronize depth information before setting coarse tracking reference
		synchronizeDepthWithTracking();
		
		coarseTracker_forNewKF->setCoarseTrackingRef(frameHessians);



        coarseTracker_forNewKF->debugPlotIDepthMap(&minIdJetVisTracker, &maxIdJetVisTracker, outputWrapper);
        coarseTracker_forNewKF->debugPlotIDepthMapFloat(outputWrapper);
	}


	debugPlot("post Optimize");






	// =========================== (Activate-)Marginalize Points =========================
	flagPointsForRemoval();
	ef->dropPointsF();
	getNullspaces(
			ef->lastNullspaces_pose,
			ef->lastNullspaces_scale,
			ef->lastNullspaces_affA,
			ef->lastNullspaces_affB);
	ef->marginalizePointsF();



	// =========================== add new Immature points & new residuals =========================
	// UNIFIED depth handling: Use makeNewTraces for all cases with ML depth guidance
	{
		// Prepare depth information for unified processing
		float* depthData = nullptr;
		bool hasMLDepth = false;
		
		// Check for ML depth first (preferred)
		if(!currentMLDepthImage.empty() && initialized) {
			hasMLDepth = true;
			if(!setting_debugout_runquiet) {
				printf("makeNewTraces: Using ML depth (%dx%d) as guidance for enhanced tracking\n", 
				       currentMLDepthImage.cols, currentMLDepthImage.rows);
			}
		}
		
		// Check for RGB-D depth as fallback
		{
			boost::unique_lock<boost::mutex> lock(rgbd_depth_mutex_);
			if(!currentDepthImage.empty() && currentDepthImage.type() == CV_32FC1) {
				depthData = (float*)currentDepthImage.data;
				if(!hasMLDepth && !setting_debugout_runquiet) {
					printf("makeNewTraces: Using RGB-D depth (%dx%d)\n", 
					       currentDepthImage.cols, currentDepthImage.rows);
				}
			}
		}
		
		// Check for ML depth first, then RGB-D depth
		if(hasMLDepth) {
			// Use ML depth for enhanced point creation
			if(!setting_debugout_runquiet) {
				printf("makeNewTraces: Using ML depth (%dx%d) for enhanced tracking\n", 
				       currentMLDepthImage.cols, currentMLDepthImage.rows);
			}
			makeNewTracesWithMLDepth(fh, currentMLDepthImage);
			
			// Clear ML depth after use (confidence stored in frame, no clearing needed)
			currentMLDepthImage = cv::Mat();
		} else {
			// Use RGB-D depth if available, otherwise pure monocular
			makeNewTraces(fh, depthData);
			if(!depthData && !setting_debugout_runquiet) {
				printf("makeNewTraces: Using pure monocular (no depth available)\n");
			}
		}
	}





    for(IOWrap::Output3DWrapper* ow : outputWrapper)
    {
        ow->publishGraph(ef->connectivityMap);
        ow->publishKeyframes(frameHessians, false, &Hcalib);
		ow->publishGlobalMap(globalMap);
	}


	// =========================== Marginalize Frames =========================

	for (unsigned int i = 0; i < frameHessians.size(); i++)
		if (frameHessians[i]->flaggedForMarginalization)
		{
			marginalizeFrame(frameHessians[i]);
			i = 0;
		}

	
	// SearchInNeighbors(fh->shell->frame);
	// KeyFrameCulling(fh->shell->frame);

	// indirect!: Update indirect structs
	updateLocalKeyframes(fh->shell->frame);
	updateLocalPoints(fh->shell->frame);

	printLogLine();
	//printEigenValLine();
}


/**
 * @brief Initializes the system if coarseInitializer finds good initial systems
 * 
 * @param newFrame 
 */
void FullSystem::initializeFromInitializer(FrameHessian* newFrame)
{
	boost::unique_lock<boost::mutex> lock(mapMutex);

	// add firstframe.
	FrameHessian* firstFrame = coarseInitializer->firstFrame;
	
	{
		firstFrame->idx = frameHessians.size();
		frameHessians.push_back(firstFrame);
	}
	
	firstFrame->frameID = allKeyFramesHistory.size();
	firstFrame->shell->KfId = 0;
	newFrame->shell->KfId = 1;
	allKeyFramesHistory.push_back(firstFrame->shell);
	ef->insertFrame(firstFrame, &Hcalib);
	setPrecalcValues();

	//int numPointsTotal = makePixelStatus(firstFrame->dI, selectionMap, wG[0], hG[0], setting_desiredDensity);
	//int numPointsTotal = pixelSelector->makeMaps(firstFrame->dIp, selectionMap,setting_desiredDensity);

	firstFrame->pointHessians.reserve(wG[0]*hG[0]*0.2f);
	firstFrame->pointHessiansMarginalized.reserve(wG[0]*hG[0]*0.2f);
	firstFrame->pointHessiansOut.reserve(wG[0]*hG[0]*0.2f);


	// FIXED: Compute BOTH photometric and ML scales, decouple geometry from metric info
	float rescaleFactor;
	bool usingMetricScale = false;
	float ml_mean_depth = -1.0f;
	
	// Always compute photometric scale (for good initialization geometry)
	// Only use well-triangulated points (same filter as computeMetricScaleFactor)
	float sumID=1e-5, numID=1e-5;
	int goodPointCount = 0;
	for(int i=0;i<coarseInitializer->numPoints[0];i++)
	{
		if (!coarseInitializer->points[0][i].isGood) continue;
		if (coarseInitializer->points[0][i].lastHessian < 0.1f) continue;
		if (fabsf(coarseInitializer->points[0][i].iR - 1.0f) < 0.01f) continue;
		sumID += coarseInitializer->points[0][i].iR;
		numID++;
		goodPointCount++;
	}
	float photometricScale = 1 / (sumID / numID);

	// Quality gate: check fraction of well-triangulated points
	int totalPoints = coarseInitializer->numPoints[0];
	float goodRatio = (float)goodPointCount / totalPoints;

	// Check if ML scale is available
	float metricScaleFactor = coarseInitializer->computeMetricScaleFactor();
	bool scaleReliable = (metricScaleFactor > 0) && (goodRatio >= setting_mlInitMinGoodRatio);

	printf("[INIT_QUALITY] %d/%d points well-triangulated (%.1f%%). Scale %s.\n",
		   goodPointCount, totalPoints, goodRatio * 100.0f,
		   scaleReliable ? "RELIABLE" : "UNRELIABLE - falling back to photometric");

	if (setting_useMLForInitialization && scaleReliable &&
		coarseInitializer->mlConfidence > setting_mlInitConfidenceThreshold) {
		// Metric scaling applies to both seeded and non-seeded init paths.
		// With relative seeding (iR=mlMeanDepth/mlDepth), computeMetricScaleFactor()
		// returns mlMeanDepth directly — the ratio is clean and metric scaling is valid.
		rescaleFactor = photometricScale;  // Good geometry
		ml_mean_depth = metricScaleFactor;  // Metric information
		usingMetricScale = true;
		
		printf("=== INITIALIZATION WITH DIRECT METRIC APPROACH ===\n");
		printf("Initial triangulation scale: %.3f (computed for metric conversion)\n", rescaleFactor);
		printf("ML mean depth (metric scale): %.3f meters\n", ml_mean_depth);
		printf("ML confidence: %.2f (system will be initialized directly in metric scale)\n", coarseInitializer->mlConfidence);
		
	} else {
		// Standard photometric initialization
		rescaleFactor = photometricScale;
		
		printf("=== INITIALIZATION WITH PHOTOMETRIC SCALE ===\n");
		printf("Warning: ML depth unavailable or low confidence\n");
		printf("Using photometric scale: %.3f\n", rescaleFactor);
	}
	
	// Store initialization method and scale factor for logging
	if (usingMetricScale) {
		using_metric_scale_ = true;
		init_scale_factor_ = ml_mean_depth;  // Metric scale used directly
		
		// DIRECT METRIC: No conversion needed - system initialized in metric scale
		initial_ml_to_slam_scale_factor_ = 1.0f;  // No conversion factor needed
		ml_to_slam_scale_factor_ = 1.0f;         // Already in metric scale
		
		// Mark as ALIGNED - system initialized directly in metric scale
		scale_aligned_ = true;
		
	} else {
		using_metric_scale_ = false;
		init_scale_factor_ = rescaleFactor;
		ml_to_slam_scale_factor_ = 1.0f;
		scale_aligned_ = true;  // Standard monocular, no conversion needed
	}
	
	// VALIDATION DEBUG: Show initial triangulation quality  
	printf("[VALIDATION] Triangulation Scale = %.4f (%.1f%% difference from 1.0 - good for geometry)\n", 
		   rescaleFactor, fabs(rescaleFactor - 1.0) * 100.0);
	printf("[VALIDATION] Using %s initialization\n",
		   usingMetricScale ? "DIRECT METRIC (ML depth sets scale from start)" : "PHOTOMETRIC");
	printf("[VALIDATION] ML Initialization %s\n",
		   setting_useMLForInitialization ? "ENABLED" : "DISABLED");
	
	// SCALE MONITORING: Track direct metric initialization
	if (usingMetricScale) {
		// printf("[SCALE_MONITOR] ✅ DIRECT METRIC INITIALIZATION ENABLED\n");
		// printf("[SCALE_MONITOR] System initialized directly in metric scale: %.3f meters\n", ml_mean_depth);
		// printf("[SCALE_MONITOR] Translation and depths scaled to metric from start\n");
		// printf("[SCALE_MONITOR] Scale factor: 1.0 (no conversion needed)\n");
		// printf("[SCALE_MONITOR] ML confidence: %.2f, threshold: %.2f\n", 
			//    coarseInitializer->mlConfidence, setting_mlInitConfidenceThreshold);
		// printf("[SCALE_MONITOR] Expected: Direct ML depth usage, consistent RMSE\n");
	} else {
		// printf("[SCALE_MONITOR] Using standard photometric scale: %.3f\n", rescaleFactor);
		// printf("[SCALE_MONITOR] Scale alignment may be needed if ML depth becomes available\n");
	}

	// randomly sub-select the points I need.
	float keepPercentage = setting_desiredPointDensity / coarseInitializer->numPoints[0];

    if(!setting_debugout_runquiet)
        printf("Initialization: keep %.1f%% (need %d, have %d)!\n", 100*keepPercentage,
                (int)(setting_desiredPointDensity), coarseInitializer->numPoints[0] );


	SE3 firstToNew = coarseInitializer->thisToNext;
	
	// DIRECT METRIC INITIALIZATION: Scale translation to metric from start
	if (usingMetricScale) {
		// Convert translation to metric scale during initialization
		float metric_scale_factor = ml_mean_depth / photometricScale;
		// printf("[INIT_DIAG] photometricScale=%.6f, metricScaleFactor(median)=%.6f\n", photometricScale, metricScaleFactor);
		// printf("[INIT_DIAG] metric_scale_factor (ml/photo) = %.6f\n", metric_scale_factor);
		// printf("[INIT_DIAG] Pre-metric translation norm: %.6f\n", firstToNew.translation().norm());

		firstToNew.translation() *= metric_scale_factor;

		// printf("[INIT_DIAG] Post-metric translation norm: %.6f\n", firstToNew.translation().norm());
	} else {
		// Standard monocular - apply photometric scale normalization
		firstToNew.translation() /= rescaleFactor;
	}


	// really no lock required, as we are initializing.
	{
		// boost::unique_lock<boost::mutex> crlock(shellPoseMutex);
		firstFrame->shell->setPose(SE3());
		firstFrame->shell->aff_g2l = AffLight(0,0);
		firstFrame->setEvalPT_scaled(firstFrame->shell->getPose().inverse(),firstFrame->shell->aff_g2l);
		firstFrame->shell->setPoseOpti(Sim3(firstFrame->shell->getPoseInverse().matrix()));

		newFrame->shell->setPose(firstToNew.inverse());
		newFrame->shell->aff_g2l = AffLight(0,0);
		newFrame->setEvalPT_scaled(newFrame->shell->getPose().inverse(),newFrame->shell->aff_g2l);
	}

	for (int i = 0; i < coarseInitializer->numPoints[0]; i++)
	{

		Pnt *point = coarseInitializer->points[0] + i;
		// indirect!: Indirect point handling
		if (point->my_type <= 4)
			if (rand() / (float)RAND_MAX > keepPercentage)
				continue;

		ImmaturePoint *pt = new ImmaturePoint(point->u + 0.5f, point->v + 0.5f, firstFrame, point->my_type, &Hcalib);

		if (!std::isfinite(pt->energyTH)) { delete pt; pt = nullptr; continue; }

		pt->idepth_max = pt->idepth_min = 1;
		PointHessian *ph = new PointHessian(pt, &Hcalib);
		delete pt;

		if (!std::isfinite(ph->energyTH)){ delete ph; continue; }

		// DIRECT METRIC INITIALIZATION: Scale point depths to metric from start
		float original_idepth = point->iR;
		float final_idepth;
		
		if (usingMetricScale) {
			// DIRECT METRIC: Scale inverse depths to metric during initialization
			float metric_scale_factor = ml_mean_depth / photometricScale;
			final_idepth = original_idepth / metric_scale_factor;  // Inverse depth scales inversely
			
			// DEBUG_INIT: Log direct metric approach
			static int debug_point_count_metric = 0;
			// if (debug_point_count_metric < 5) {
			// 	printf("[DEBUG_INIT] DIRECT METRIC Point %d: original_iR=%.6f, metric_idepth=%.6f (factor=%.3f, depth=%.3fm)\n", 
			// 		   debug_point_count_metric, original_idepth, final_idepth, metric_scale_factor, 1.0f/final_idepth);
			// 	debug_point_count_metric++;
			// }
		} else {
			// Standard monocular - apply photometric scaling
			final_idepth = original_idepth * rescaleFactor;
			// DEBUG_INIT: Log standard scaling 
			static int debug_point_count_std = 0;
			// if (debug_point_count_std < 5) {
			// 	printf("[DEBUG_INIT] STANDARD Point %d: original_iR=%.6f, scaled_idepth=%.6f (factor=%.3f, depth=%.3fm)\n", 
			// 		   debug_point_count_std, original_idepth, final_idepth, rescaleFactor, 1.0f/final_idepth);
			// 	debug_point_count_std++;
			// }
		}
		
		ph->setIdepthScaled(final_idepth);
		ph->setIdepthZero(ph->idepth);
		ph->setPointStatus(PointHessian::ACTIVE);

		// INIT_DIAG: Log first 5 points to verify scale math
		static int init_diag_count = 0;
		if (init_diag_count < 5 && usingMetricScale) {
			int u = (int)(point->u + 0.5f);
			int v = (int)(point->v + 0.5f);
			float mlD = 0;
			if (u >= 0 && v >= 0 && u < coarseInitializer->firstFrameMLDepth.cols &&
				v < coarseInitializer->firstFrameMLDepth.rows)
				mlD = coarseInitializer->firstFrameMLDepth.at<float>(v, u);
			// printf("[INIT_DIAG] Point %d: iR=%.6f, final_idepth=%.6f (depth=%.2fm), mlDepth=%.2fm, ratio=%.3f\n",
			//        init_diag_count, original_idepth, final_idepth,
			//        final_idepth > 0 ? 1.0f/final_idepth : -1,
			//        mlD, mlD > 0 && final_idepth > 0 ? (1.0f/final_idepth)/mlD : -1);
			init_diag_count++;
		}
		
		// Set appropriate depth prior flags based on point type
		if (ph->my_type > 4) {
			// Indirect point from MapPoint - set indirect prior
			ph->hasDepthPrior = true;
		}
		
		// NEW: If using metric scale, also set ML depth prior (independent)
		if (usingMetricScale && coarseInitializer->hasMLDepth) {
			int u = (int)(point->u + 0.5f);
			int v = (int)(point->v + 0.5f);
			
			if (u >= 0 && v >= 0 && 
				u < coarseInitializer->firstFrameMLDepth.cols && 
				v < coarseInitializer->firstFrameMLDepth.rows) {
				
				float mlDepth = coarseInitializer->firstFrameMLDepth.at<float>(v, u);
				if (mlDepth > 0 && std::isfinite(mlDepth)) {
					// FIXED: Convert ML depth to photometric scale for consistent optimization
					float mlDepthInPhotoScale = mlDepth * ml_to_slam_scale_factor_;
					
					// Set ML depth information (separate from indirect prior)
					ph->hasMLDepth = true;
					ph->ml_idepth_reference = 1.0f / mlDepthInPhotoScale;  // Now in same scale as photometric
					ph->ml_uncertainty = 0.1f; // Default uncertainty for initialization
					ph->ml_weight = setting_idepthFixPrior; // Use default weight for initialization
				}
			}
		}

		firstFrame->pointHessians.push_back(ph);
		ef->insertPoint(ph);

		// indirect!: Indirect point handling
		if (ph->my_type > 4)
		{
			std::shared_ptr<MapPoint> pMP = std::make_shared<MapPoint>(ph, globalMap);
			ph->host->shell->frame->addMapPoint(pMP);
			ph->Mp = pMP;
			pMP->AddObservation(firstFrame->shell->frame, pMP->index);
			globalMap->AddMapPoint(pMP);
		}
	}
	
	// Store points count for logging
	init_points_count_ = firstFrame->pointHessians.size();


	// indirect!: Add indirect point to global map
	globalMap->AddKeyFrame(firstFrame->shell->frame);
	globalMap->mvpKeyFrameOrigins.push_back(firstFrame->shell->frame);
	
	
	mnLastKeyFrameId = newFrame->shell->id;
	mpLastKeyFrame = newFrame->shell->frame;

	mvpLocalKeyFrames.push_back(newFrame->shell->frame);
	mvpLocalKeyFrames.push_back(firstFrame->shell->frame);
	globalMap->GetAllMapPoints(mvpLocalMapPoints);
	
	mpReferenceKF = newFrame->shell->frame;

	newFrame->shell->frame->mpReferenceKF = newFrame->shell->frame;
	mLastFrame = newFrame->shell->frame;
	globalMap->SetReferenceMapPoints(mvpLocalMapPoints);
	
	int nmatches = SearchLocalPoints(newFrame->shell->frame, 5, 0.8);
	for (int i = 0; i < newFrame->shell->frame->nFeatures; ++i)
	{
		std::shared_ptr<MapPoint> pMP = newFrame->shell->frame->tMapPoints[i];
		if(pMP)
			pMP->increaseFound();
	}

	// indirect!: Add point to loop closer
	if (loopCloser)
		loopCloser->InsertKeyFrame(firstFrame->shell->frame, globalMap->GetMaxMPid());

	initialized = true;
	
	// Initialize both CoarseTracker instances with calibration data
	// This prevents dimension mismatch warnings when setting ML depth
	coarseTracker->makeK(&Hcalib);
	coarseTracker_forNewKF->makeK(&Hcalib);
	
	printf("INITIALIZE FROM INITIALIZER (%d pts)!\n", (int)firstFrame->pointHessians.size());
}

/**
 * @brief Create new immature points
 * 
 * Only made for the newest keyframe
 * 
 * @param newFrame 
 * @param gtDepth 
 */
void FullSystem::makeNewTraces(FrameHessian* newFrame, float* gtDepth)
{
	//int numPointsTotal = makePixelStatus(newFrame->dI, selectionMap, wG[0], hG[0], setting_desiredDensity);
	int numPointsTotal = pixelSelector->makeMaps(newFrame, selectionMap,setting_desiredImmatureDensity);

	newFrame->pointHessians.reserve(numPointsTotal*1.2f);
	//fh->pointHessiansInactive.reserve(numPointsTotal*1.2f);
	newFrame->pointHessiansMarginalized.reserve(numPointsTotal*1.2f);
	newFrame->pointHessiansOut.reserve(numPointsTotal*1.2f);

	SE3 Tcw = newFrame->shell->getPoseInverse();
	for (int y = PATTERNPADDING + 1; y < hG[0] - PATTERNPADDING - 2; y++)
		for (int x = PATTERNPADDING + 1; x < wG[0] - PATTERNPADDING - 2; x++)
		{
			int i = x + y * wG[0];
			if (selectionMap[i] == 0)
				continue;

			
			ImmaturePoint *impt = new ImmaturePoint(x, y, newFrame, selectionMap[i], &Hcalib);
			
			// RGB-D Depth Integration: Use ground truth depth if available
			if (gtDepth != nullptr)
			{
				float depth = gtDepth[i];  // Access depth at pixel (x, y)
				
				// Validate depth value - adaptive for any dataset range  
				if (depth > 0.0f && std::isfinite(depth))
				{
					float idepth = 1.0f / depth;
					float uncertainty = 0.01f * idepth;  // Small uncertainty for ground truth depth
					
					impt->idepth_min = std::max(0.0f, idepth - uncertainty);
					impt->idepth_max = idepth + uncertainty;
					
					// Debug-only logging for depth-integrated point details
					static int depthPointCount = 0;
					if (depthPointCount < 10)
					{
						HSLAM::DepthLogger::logDepthPoint(depthPointCount, x, y, depth, 
														  idepth, impt->idepth_min, impt->idepth_max);
						depthPointCount++;
					}
				}
			}
			
			// indirect!: Indirect point handling
			if (selectionMap[i] > 4) //getPriors
			{
				// if(newFrame->shell->frame->getMapPoint(selectionMap[i]-5))
				// {
				// 	delete impt;
				// 	continue;
				// }
				int index = selectionMap[i] - 5;
				auto pMP = newFrame->shell->frame->getMapPoint(index);
				if (pMP)
				{
					if (!pMP->isBad())
					{
						Vec3 PointinFrame =  (Tcw * pMP->getWorldPose().cast<double>());
						float invz = (1.0 / (float)PointinFrame[2]);
						if (invz > 0)
						{
							float devi = pMP->getStdDev();
							float idepthmin = invz - 15 * devi; //15
							impt->idepth_min = idepthmin > 0 ? idepthmin : 0;
							impt->idepth_max = invz + 15 * devi; //15
							
						}
					}
				}
			}

		if (!std::isfinite(impt->energyTH))
			delete impt;
		else newFrame->immaturePoints.push_back(impt);

	}

	int depthIntegratedCount = 0;
	if (gtDepth != nullptr)
	{
		for (const auto& impt : newFrame->immaturePoints)
		{
			if (impt->idepth_max != NAN && impt->idepth_max > impt->idepth_min)
			{
				depthIntegratedCount++;
			}
		}
	}
	
	// Log point creation statistics using DepthLogger
	HSLAM::DepthLogger::logPointCreation((int)newFrame->immaturePoints.size(), depthIntegratedCount);
}

void FullSystem::makeNewTracesWithMLDepth(FrameHessian* newFrame, const cv::Mat& ml_depth) {
    // ML depth integration with confidence maps - get confidence directly from frame
    auto ml_confidence_ptr = newFrame->getMLConfidenceMap();
    cv::Mat ml_confidence = (ml_confidence_ptr && !ml_confidence_ptr->empty()) ? *ml_confidence_ptr : cv::Mat();
    
    int numPointsTotal = pixelSelector->makeMaps(newFrame, selectionMap, setting_desiredImmatureDensity);
    
    newFrame->pointHessians.reserve(numPointsTotal * 1.2f);
    newFrame->pointHessiansMarginalized.reserve(numPointsTotal * 1.2f);
    newFrame->pointHessiansOut.reserve(numPointsTotal * 1.2f);
    
    int ml_depth_points = 0;
    int total_points = 0;
    SE3 Tcw = newFrame->shell->getPoseInverse();
    
    for(int y = PATTERNPADDING + 1; y < hG[0] - PATTERNPADDING - 2; y++) {
        for(int x = PATTERNPADDING + 1; x < wG[0] - PATTERNPADDING - 2; x++) {
            int i = x + y * wG[0];
            if(selectionMap[i] == 0) continue;
            
            ImmaturePoint* impt = new ImmaturePoint(x, y, newFrame, selectionMap[i], &Hcalib);
            
            // ML depth integration with scale alignment and validation
            if(!ml_depth.empty() && y < ml_depth.rows && x < ml_depth.cols) {
                float raw_ml_depth = ml_depth.at<float>(y, x);
                
                // Validate ML depth before processing
                if(isMLDepthValid(raw_ml_depth, 50.0f)) {
                    // DIRECT METRIC: System always initialized in metric scale when ML enabled
                    float depth = raw_ml_depth;  // Use ML depth directly (system already metric)
                    
                    // Monitor first ML point usage
                    if(ml_depth_points == 0) {
                        // printf("[SCALE_MONITOR] First ML point: raw=%.2fm, used=%.2fm (direct usage, system in metric)\n", 
                        //        raw_ml_depth, depth);
                    }
                    // PHASE 2: Extract per-pixel ML confidence if available
                    float pixel_confidence = 1.0f;  // Default confidence
                    if(!ml_confidence.empty() && y < ml_confidence.rows && x < ml_confidence.cols) {
                        pixel_confidence = ml_confidence.at<float>(y, x);
                        pixel_confidence = std::max(0.1f, std::min(1.0f, pixel_confidence));  // Clamp to [0.1, 1.0]
                    }
                    
                    // Confidence-based uncertainty model (uniform across all depths)
                    // This approach uses the ML confidence map directly without problematic depth² scaling
                    const float idepth = 1.0f / depth;
                    
                    // Base uncertainty in inverse depth space (not metric)
                    // This provides consistent bounds regardless of depth
                    const float base_idepth_uncertainty = 0.05f;
                    
                    // Scale by confidence: high confidence = tighter bounds, low confidence = looser bounds
                    // pixel_confidence is already clamped to [0.1, 1.0] at line 2764
                    float effective_idepth_uncertainty = base_idepth_uncertainty / pixel_confidence;
                    
                    // Optional gentle depth-adaptive scaling for numerical stability at extreme distances
                    float depth_factor = 1.0f + (depth / 100.0f);
                    effective_idepth_uncertainty *= depth_factor;
                    
                    // PHASE 2: Store ML bounds with proper uncertainty propagation
                    impt->idepth_min = idepth - effective_idepth_uncertainty;
                    impt->idepth_max = idepth + effective_idepth_uncertainty;
                    impt->idepth_GT = idepth;
                    impt->ml_confidence = pixel_confidence;           // Store per-pixel confidence
                    impt->ml_uncertainty_m = effective_idepth_uncertainty;  // Store effective inverse depth uncertainty (not metric)
                    
                    // ML depth integration complete for this point
                    
                    // Debug output for new confidence-based bounds
                    // if(ml_depth_points < 10 || (ml_depth_points % 500 == 0)) {
                    //     printf("ML_BOUNDS_CONFIDENCE Point %d: depth=%.2fm, confidence=%.2f, "
                    //            "idepth=%.6f, uncertainty=%.6f, bounds=[%.6f, %.6f]\n",
                    //            ml_depth_points, depth, pixel_confidence,
                    //            idepth, effective_idepth_uncertainty,
                    //            impt->idepth_min, impt->idepth_max);
                    // }
                    
                    ml_depth_points++;                    // unchanged
                }
            }
            
            // Handle indirect point priors (existing HSLAM functionality)
            if(selectionMap[i] > 4) {
                int index = selectionMap[i] - 5;
                auto pMP = newFrame->shell->frame->getMapPoint(index);
                if(pMP && !pMP->isBad()) {
                    Vec3 PointinFrame = (Tcw * pMP->getWorldPose().cast<double>());
                    float invz = (1.0 / (float)PointinFrame[2]);
                    if(invz > 0) {
                        float devi = pMP->getStdDev();
                        float idepthmin = invz - 15 * devi;
                        impt->idepth_min = idepthmin > 0 ? idepthmin : 0;
                        impt->idepth_max = invz + 15 * devi;
                    }
                }
            }
            
            if(!std::isfinite(impt->energyTH)) {
                delete impt;
                continue;
            }
            
            newFrame->immaturePoints.push_back(impt);
            total_points++;
        }
    }
    
    // ML bounds set at creation - hybrid combination happens during tracing
    
    // TEMP_DEBUG_REPETITIVE: Commented out ML integration statistics for cleaner output
    // if(ml_depth_points > 0) {
    //     printf("ML Depth Integration: %d/%d points (%.1f%%)\n",
    //            ml_depth_points, total_points, 
    //            100.0f * ml_depth_points / total_points);
    // }
    
    // DEPTH_DEBUG: Comprehensive ML depth integration logging (TEMPORARY)
    #ifdef ENABLE_DEPTH_DEBUG
    if(ml_depth_points > 0) {
        HSLAM::DepthLogger::logIntegrationStats(ml_depth_points, ml_depth_points, 
                                               total_points, 
                                               100.0f * ml_depth_points / total_points, 
                                               "ML_Depth_Point_Creation");
        printf("DEPTH_DEBUG: Logged ML depth integration statistics (%d points)\n", ml_depth_points);
        
        // DEPTH_DEBUG: ML vs HSLAM depth comparison moved to activatePointsMT() for better timing
    } else {
        printf("DEPTH_DEBUG: WARNING - No ML depth points integrated despite ML depth available\n");
    }
    #endif
    
    // Log using existing depth logger
    HSLAM::DepthLogger::logPointCreation(total_points, ml_depth_points);
}

/**
 * @brief Compute scale alignment between ML depth (metric) and SLAM depth (arbitrary)
 * 
 * This function analyzes existing points to determine the scale factor needed to convert
 * ML depths from metric units to the arbitrary scale used by SLAM.
 */
void FullSystem::computeScaleAlignment() {
    if (scale_aligned_ || frameHessians.empty()) {
        return;  // Already aligned or no frames to analyze
    }
    
    // Collect depth samples from mature points
    std::vector<float> slam_depths;
    std::vector<float> ml_depths_at_points;
    
    // Use the most recent frame with mature points
    for (int fh_idx = frameHessians.size() - 1; fh_idx >= 0 && slam_depths.size() < 50; fh_idx--) {
        FrameHessian* fh = frameHessians[fh_idx];
        
        for (PointHessian* ph : fh->pointHessians) {
            if (ph->idepth > 1e-6f && ph->hasMLDepth) {
                float slam_depth = 1.0f / ph->idepth;
                float ml_depth = ph->ml_idepth_reference > 0 ? (1.0f / ph->ml_idepth_reference) : -1.0f;
                
                if (ml_depth > 0.1f && ml_depth < 100.0f && slam_depth > 0.1f && slam_depth < 100.0f) {
                    slam_depths.push_back(slam_depth);
                    ml_depths_at_points.push_back(ml_depth);
                }
            }
        }
    }
    
    // Need at least 10 points for reliable scale estimation
    if (slam_depths.size() >= 10) {
        // Compute scale factor as median ratio to be robust to outliers
        std::vector<float> scale_ratios;
        for (size_t i = 0; i < slam_depths.size(); ++i) {
            scale_ratios.push_back(slam_depths[i] / ml_depths_at_points[i]);
        }
        
        std::sort(scale_ratios.begin(), scale_ratios.end());
        ml_to_slam_scale_factor_ = scale_ratios[scale_ratios.size() / 2];  // Median
        
        // Store reference values for monitoring
        reference_slam_depth_ = slam_depths[slam_depths.size() / 2];
        reference_ml_depth_ = ml_depths_at_points[slam_depths.size() / 2];
        
        scale_aligned_ = true;
        
        printf("[SCALE_ALIGNMENT] Computed scale factor: %.3f (SLAM/ML ratio)\n", ml_to_slam_scale_factor_);
        printf("[SCALE_ALIGNMENT] Reference: SLAM=%.2fm, ML=%.2fm, samples=%zu\n", 
               reference_slam_depth_, reference_ml_depth_, slam_depths.size());
    }
}

/**
 * @brief Immediate scale alignment on first ML keyframe to prevent mixed scale artifacts
 * 
 * This method computes and applies scale alignment as soon as the first ML depth is available,
 * preventing the creation of points at different scales which causes visual artifacts.
 * 
 * @param ml_depth ML depth map from first keyframe
 */
void FullSystem::alignScalesImmediately(const cv::Mat& ml_depth) {
    if (scale_aligned_ || ml_depth.empty()) {
        return;  // Already aligned or no ML depth available
    }
    
    // Collect ML depth samples for robust scale estimation
    std::vector<float> ml_depths;
    ml_depths.reserve(1000);  // Pre-allocate for efficiency
    
    // Sample every 10th pixel to get representative depths without too much computation
    for(int y = 5; y < ml_depth.rows; y += 10) {
        for(int x = 5; x < ml_depth.cols; x += 10) {
            float depth = ml_depth.at<float>(y, x);
            // Validate depth (reasonable range for most scenes)
            if(depth > 0.1f && depth < 100.0f && std::isfinite(depth)) {
                ml_depths.push_back(depth);
            }
        }
    }
    
    // Need sufficient samples for reliable estimation
    if(ml_depths.size() < 50) {
        printf("[IMMEDIATE_SCALE] Insufficient ML depth samples (%zu), alignment skipped\n", ml_depths.size());
        return;
    }
    
    // Compute median ML depth (robust to outliers)
    std::sort(ml_depths.begin(), ml_depths.end());
    float ml_median_depth = ml_depths[ml_depths.size() / 2];
    
    // The scale factor converts ML metric depths to SLAM arbitrary scale
    // ml_to_slam_scale_factor = slam_scale / ml_scale
    // Since initialization scale ≈ 1.0, we need to scale ML depths down to match
    ml_to_slam_scale_factor_ = init_scale_factor_ / ml_median_depth;
    
    // Store reference values for monitoring and debugging
    reference_ml_depth_ = ml_median_depth;
    reference_slam_depth_ = init_scale_factor_;
    
    // Mark as aligned
    scale_aligned_ = true;
    
    printf("[IMMEDIATE_SCALE] Scale aligned on first ML keyframe\n");
    printf("[IMMEDIATE_SCALE] Init scale: %.3f, ML median: %.2fm, factor: %.3f\n", 
           init_scale_factor_, ml_median_depth, ml_to_slam_scale_factor_);
    printf("[IMMEDIATE_SCALE] Samples used: %zu/%d pixels\n", 
           ml_depths.size(), ml_depth.rows * ml_depth.cols);
}

/**
 * @brief Convert ML depth from metric units to SLAM scale
 * 
 * @param ml_depth_meters ML depth in meters
 * @return float Equivalent depth in SLAM arbitrary scale
 */
float FullSystem::convertMLDepthToSLAMScale(float ml_depth_meters) {
    if (!scale_aligned_) {
        // If not aligned yet, attempt alignment (should rarely happen now)
        printf("[SCALE_FIX] WARNING: Scale not aligned during conversion, attempting late alignment\n");
        computeScaleAlignment();
    }
    
    if (scale_aligned_) {
        return ml_depth_meters * ml_to_slam_scale_factor_;
    }
    
    // Fallback: return original ML depth (no conversion)
    return ml_depth_meters;
}

/**
 * @brief Validate ML depth for reasonableness
 * 
 * @param ml_depth ML depth value to validate
 * @param expected_range Maximum expected depth for scene
 * @return true if depth is valid, false otherwise
 */
bool FullSystem::isMLDepthValid(float ml_depth, float expected_range) {
    // Basic sanity checks
    if (!std::isfinite(ml_depth) || ml_depth <= 0.0f) {
        return false;
    }
    
    // Remove hard cutoff - only validate minimum depth for numerical stability
    // Allow any reasonable positive depth value
    if (ml_depth < 0.1f) {
        return false;
    }
    
    // No upper limit - let the system handle far points naturally
    // For EuRoC (indoor aerial): typical range 1-15m
    // For KITTI (outdoor): typical range 2-80m  
    // For TUM (indoor): typical range 0.5-8m
    return true;
}

/**
 * @brief Apply metric scale conversion to entire map after successful initialization
 * 
 * This method converts all poses and landmarks from photometric scale (used for good geometry)
 * to metric scale (from ML depth) while preserving the same world structure.
 */
void FullSystem::applyMetricScaleConversion() {
    if (scale_aligned_ || ml_to_slam_scale_factor_ <= 0.0f) {
        return;  // Already converted or invalid conversion factor
    }
    
    printf("[METRIC_CONVERSION] Applying metric scale conversion...\n");
    printf("[METRIC_CONVERSION] Conversion factor: %.6f (photometric/metric)\n", ml_to_slam_scale_factor_);
    
    // Convert all frame poses - scale translations, keep rotations
    for (FrameHessian* fh : frameHessians) {
        SE3 pose = fh->shell->getPose();
        Vec3 translation = pose.translation();
        
        // Convert translation from photometric to metric scale
        translation /= ml_to_slam_scale_factor_;
        
        // Keep rotation unchanged, update translation
        SE3 metric_pose = SE3(pose.so3(), translation);
        fh->shell->setPose(metric_pose.inverse());
        fh->shell->setPoseOpti(Sim3(fh->shell->getPoseInverse().matrix()));
        
        // Update evaluation point for consistency
        fh->setEvalPT_scaled(fh->shell->getPose().inverse(), fh->shell->aff_g2l);
    }
    
    // Convert all point depths - scale inverse depths
    for (FrameHessian* fh : frameHessians) {
        for (PointHessian* ph : fh->pointHessians) {
            // Scale inverse depth to convert depth scale
            ph->setIdepth(ph->idepth * ml_to_slam_scale_factor_);
            ph->setIdepthZero(ph->idepth);
        }
    }
    
    // Mark as aligned and log completion
    scale_aligned_ = true;
    
    printf("[METRIC_CONVERSION] Conversion complete - map now in metric coordinates\n");
    printf("[METRIC_CONVERSION] All poses and landmarks converted to meter units\n");
}

#ifdef ENABLE_DEPTH_DEBUG
/**
 * @brief DEPTH_DEBUG: Compare ML depth values with mature HSLAM points after activation
 * 
 * @param newFrame Frame being processed
 * @param ml_depth ML depth map
 */
void FullSystem::debugMLDepthComparison(FrameHessian* newFrame, const cv::Mat& ml_depth) {
    static int debug_call_count = 0;
    debug_call_count++;
    
    // Show first 10 comparisons for initial analysis, then every 5th for periodic monitoring
    if (debug_call_count > 10 && debug_call_count % 5 != 0) {
        return;
    }
    
    printf("\n=== DEPTH_DEBUG: ML vs HSLAM Depth Comparison (Call #%d) ===\n", debug_call_count);
    
    // Count mature PointHessian objects across all frames
    size_t total_mature_points = 0;
    int ml_vs_hslam_comparisons = 0;
    float ml_depth_sum = 0.0f;
    float hslam_depth_sum = 0.0f;
    float total_absolute_error = 0.0f;
    float total_relative_error = 0.0f;
    float min_ml_depth = std::numeric_limits<float>::max();
    float max_ml_depth = 0.0f;
    float min_hslam_depth = std::numeric_limits<float>::max();
    float max_hslam_depth = 0.0f;
    
    // Compare ML depth with mature HSLAM points from all frames
    for(FrameHessian* fh : frameHessians) {
        total_mature_points += fh->pointHessians.size();
        
        for(PointHessian* ph : fh->pointHessians) {
            // Get pixel coordinates of the point
            float u = ph->u;
            float v = ph->v;
            
            // Check bounds and get ML depth at this location
            if(u >= 0 && v >= 0 && u < ml_depth.cols && v < ml_depth.rows) {
                float ml_depth_val = ml_depth.at<float>((int)v, (int)u);
                
                if(ml_depth_val > 0.0f && std::isfinite(ml_depth_val)) {
                    // Get HSLAM depth (convert from inverse depth)
                    float hslam_depth = 1.0f / ph->idepth;
                    
                    if(hslam_depth > 0.0f && std::isfinite(hslam_depth)) {
                        ml_depth_sum += ml_depth_val;
                        hslam_depth_sum += hslam_depth;
                        
                        min_ml_depth = std::min(min_ml_depth, ml_depth_val);
                        max_ml_depth = std::max(max_ml_depth, ml_depth_val);
                        min_hslam_depth = std::min(min_hslam_depth, hslam_depth);
                        max_hslam_depth = std::max(max_hslam_depth, hslam_depth);
                        
                        float abs_error = std::abs(ml_depth_val - hslam_depth);
                        float rel_error = abs_error / hslam_depth * 100.0f;
                        
                        total_absolute_error += abs_error;
                        total_relative_error += rel_error;
                        
                        ml_vs_hslam_comparisons++;
                        
                        // Sample points for detailed logging to avoid massive log files (every ~100th point)
                        int sample_interval = std::max(1, (int)(total_mature_points / 20));  // Target ~20 samples per comparison
                        if (ml_vs_hslam_comparisons % sample_interval == 0) {
                            HSLAM::DepthLogger::logMLvsHSLAMComparison(u, v, ml_depth_val, hslam_depth, abs_error, rel_error);
                        }
                    }
                }
            }
        }
    }
    
    // Print comprehensive comparison statistics
    printf("DEPTH_DEBUG: Mature Points Analysis:\n");
    printf("  Total mature PointHessian objects: %zu\n", total_mature_points);
    printf("  Successful ML vs HSLAM comparisons: %d\n", ml_vs_hslam_comparisons);
    
    if(ml_vs_hslam_comparisons > 0) {
        float avg_ml_depth = ml_depth_sum / ml_vs_hslam_comparisons;
        float avg_hslam_depth = hslam_depth_sum / ml_vs_hslam_comparisons;
        float avg_abs_error = total_absolute_error / ml_vs_hslam_comparisons;
        float avg_rel_error = total_relative_error / ml_vs_hslam_comparisons;
        
        printf("DEPTH_DEBUG: ML Depth - Range: [%.3f, %.3f]m, Avg: %.3fm\n",
               min_ml_depth, max_ml_depth, avg_ml_depth);
        printf("DEPTH_DEBUG: HSLAM Depth - Range: [%.3f, %.3f]m, Avg: %.3fm\n",
               min_hslam_depth, max_hslam_depth, avg_hslam_depth);
        printf("DEPTH_DEBUG: Error Analysis - Avg Abs: %.3fm, Avg Rel: %.1f%%\n",
               avg_abs_error, avg_rel_error);
        
        // Log structured comparison to DepthLogger for detailed analysis
        HSLAM::DepthLogger::logIntegrationStats(
            ml_vs_hslam_comparisons, 
            0,  // fused_pixels 
            total_mature_points, 
            (float)ml_vs_hslam_comparisons / total_mature_points * 100.0f,
            "ML_vs_HSLAM_Comparison");
            
    } else {
        printf("DEPTH_DEBUG: No successful ML vs HSLAM depth comparisons found\n");
        printf("DEPTH_DEBUG: This may indicate timing issues or coordinate mismatches\n");
    }
    
    printf("=== End ML vs HSLAM Depth Comparison ===\n\n");
}
#endif

/**
 * @brief Sets pre-calculation values for the active frames
 * 
 */
void FullSystem::setPrecalcValues()
{
	for(FrameHessian* fh : frameHessians)
	{
		fh->targetPrecalc.resize(frameHessians.size());
		for(unsigned int i=0;i<frameHessians.size();i++)
			fh->targetPrecalc[i].set(fh, frameHessians[i], &Hcalib);
	}

	ef->setDeltaF(&Hcalib);
}


/**
 * @brief For debugging
 * 
 */
void FullSystem::printLogLine()
{
	if(frameHessians.size()==0) return;

    if(!setting_debugout_runquiet)
        printf("LOG %d: %.3f fine. Res: %d A, %d L, %d M; (%'d / %'d) forceDrop. a=%f, b=%f. Window %d (%d)\n",
                allKeyFramesHistory.back()->id,
                statistics_lastFineTrackRMSE,
                ef->resInA,
                ef->resInL,
                ef->resInM,
                (int)statistics_numForceDroppedResFwd,
                (int)statistics_numForceDroppedResBwd,
                allKeyFramesHistory.back()->aff_g2l.a,
                allKeyFramesHistory.back()->aff_g2l.b,
                frameHessians.back()->shell->id - frameHessians.front()->shell->id,
                (int)frameHessians.size());


	if(!setting_logStuff) return;

	if(numsLog != 0)
	{
		(*numsLog) << allKeyFramesHistory.back()->id << " "  <<
				statistics_lastFineTrackRMSE << " "  <<
				(int)statistics_numCreatedPoints << " "  <<
				(int)statistics_numActivatedPoints << " "  <<
				(int)statistics_numDroppedPoints << " "  <<
				(int)statistics_lastNumOptIts << " "  <<
				ef->resInA << " "  <<
				ef->resInL << " "  <<
				ef->resInM << " "  <<
				statistics_numMargResFwd << " "  <<
				statistics_numMargResBwd << " "  <<
				statistics_numForceDroppedResFwd << " "  <<
				statistics_numForceDroppedResBwd << " "  <<
				frameHessians.back()->aff_g2l().a << " "  <<
				frameHessians.back()->aff_g2l().b << " "  <<
				frameHessians.back()->shell->id - frameHessians.front()->shell->id << " "  <<
				(int)frameHessians.size() << " "  << "\n";
		numsLog->flush();
	}


}



/**
 * @brief For debugging the energy function
 * 
 */
void FullSystem::printEigenValLine()
{
	if(!setting_logStuff) return;
	if(ef->lastHS.rows() < 12) return;


	MatXX Hp = ef->lastHS.bottomRightCorner(ef->lastHS.cols()-CPARS,ef->lastHS.cols()-CPARS);
	MatXX Ha = ef->lastHS.bottomRightCorner(ef->lastHS.cols()-CPARS,ef->lastHS.cols()-CPARS);
	int n = Hp.cols()/8;
	assert(Hp.cols()%8==0);

	// sub-select
	for(int i=0;i<n;i++)
	{
		MatXX tmp6 = Hp.block(i*8,0,6,n*8);
		Hp.block(i*6,0,6,n*8) = tmp6;

		MatXX tmp2 = Ha.block(i*8+6,0,2,n*8);
		Ha.block(i*2,0,2,n*8) = tmp2;
	}
	for(int i=0;i<n;i++)
	{
		MatXX tmp6 = Hp.block(0,i*8,n*8,6);
		Hp.block(0,i*6,n*8,6) = tmp6;

		MatXX tmp2 = Ha.block(0,i*8+6,n*8,2);
		Ha.block(0,i*2,n*8,2) = tmp2;
	}

	VecX eigenvaluesAll = ef->lastHS.eigenvalues().real();
	VecX eigenP = Hp.topLeftCorner(n*6,n*6).eigenvalues().real();
	VecX eigenA = Ha.topLeftCorner(n*2,n*2).eigenvalues().real();
	VecX diagonal = ef->lastHS.diagonal();

	std::sort(eigenvaluesAll.data(), eigenvaluesAll.data()+eigenvaluesAll.size());
	std::sort(eigenP.data(), eigenP.data()+eigenP.size());
	std::sort(eigenA.data(), eigenA.data()+eigenA.size());

	int nz = std::max(100,setting_maxFrames*10);

	if(eigenAllLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenvaluesAll.size()) = eigenvaluesAll;
		(*eigenAllLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenAllLog->flush();
	}
	if(eigenALog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenA.size()) = eigenA;
		(*eigenALog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenALog->flush();
	}
	if(eigenPLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(eigenP.size()) = eigenP;
		(*eigenPLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		eigenPLog->flush();
	}

	if(DiagonalLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(diagonal.size()) = diagonal;
		(*DiagonalLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		DiagonalLog->flush();
	}

	if(variancesLog != 0)
	{
		VecX ea = VecX::Zero(nz); ea.head(diagonal.size()) = ef->lastHS.inverse().diagonal();
		(*variancesLog) << allKeyFramesHistory.back()->id << " " <<  ea.transpose() << "\n";
		variancesLog->flush();
	}

	std::vector<VecX> &nsp = ef->lastNullspaces_forLogging;
	(*nullspacesLog) << allKeyFramesHistory.back()->id << " ";
	for(unsigned int i=0;i<nsp.size();i++)
		(*nullspacesLog) << nsp[i].dot(ef->lastHS * nsp[i]) << " " << nsp[i].dot(ef->lastbS) << " " ;
	(*nullspacesLog) << "\n";
	nullspacesLog->flush();

}

/**
 * @brief For debugging the frames
 * 
 */
void FullSystem::printFrameLifetimes()
{
	if(!setting_logStuff) return;


	boost::unique_lock<boost::mutex> lock(trackMutex);

	std::ofstream* lg = new std::ofstream();
	lg->open("logs/lifetimeLog.txt", std::ios::trunc | std::ios::out);
	lg->precision(15);

	for(FrameShell* s : allFrameHistory)
	{
		(*lg) << s->id
			<< " " << s->marginalizedAt
			<< " " << s->statistics_goodResOnThis
			<< " " << s->statistics_outlierResOnThis
			<< " " << s->movedByOpt;



		(*lg) << "\n";
	}





	lg->close();
	delete lg;

}


void FullSystem::printEvalLine()
{
	return;
}


void FullSystem::IndirectMapper(std::shared_ptr<Frame> frame)
{

	for(size_t i=0, iend = frame->tMapPoints.size(); i < iend; ++i)
    {
		std::shared_ptr<MapPoint> Mp = frame->tMapPoints[i]; 
		if (Mp)
		{	if(Mp->isBad())
				continue;
			if (!Mp->isInKeyframe(frame))
			{
				frame->addMapPointMatch(Mp, i);
				Mp->AddObservation(frame, i);
				Mp->ComputeDistinctiveDescriptors(true);
				Mp->UpdateNormalAndDepth();
			}
		}
	}

	if (frame->fs->KfId == 1) //this is the second keyframe being added to the map: need to update first kf connections.
		for (auto it: globalMap->mvpKeyFrameOrigins)
			it->UpdateConnections();

	frame->UpdateConnections();
	
	globalMap->AddKeyFrame(frame);
	mpReferenceKF = frame;

	mLastFrame->mpReferenceKF = frame;

	mnLastKeyFrameId = frame->fs->id;
    mpLastKeyFrame = frame;

	if (loopCloser)
		loopCloser->InsertKeyFrame(frame, globalMap->GetMaxMPid());

}

int FullSystem::SearchLocalPoints(std::shared_ptr<Frame> frame, int th, float nnratio)
{
	int nmatches = 0;

	for (int i = 0; i < frame->nFeatures; ++i)
	{
		std::shared_ptr<MapPoint> pMP = frame->tMapPoints[i];
		if (pMP)
		{
			if (pMP->isBad())
			{
				frame->tMapPoints[i].reset(); 
			}
			else
            {
				nmatches += 1;
                pMP->increaseVisible();
                pMP->mnLastFrameSeen = frame->fs->id;
                pMP->mbTrackInView = false;
            }
		}
	}


	int nToMatch = 0;

	boost::unique_lock<boost::mutex> lock(localMapMtx);

	for (int i = 0, iend = mvpLocalMapPoints.size(); i < iend;++i)
	{
		std::shared_ptr<MapPoint> pMP = mvpLocalMapPoints[i];

        if(pMP->mnLastFrameSeen == frame->fs->id)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(frame->isInFrustum(pMP,0.5))
        {
            pMP->increaseVisible();
            nToMatch++;
        }
	}

	if (nToMatch > 0)
	{
		// If the camera has been relocalised recently, perform a coarser search
		// if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
		// 	th = 5;
		nmatches += matcher->SearchLocalMapByProjection(frame, mvpLocalMapPoints, th, nnratio);
	}
	return nmatches;
}

bool FullSystem::TrackLocalMap(std::shared_ptr<Frame> frame)
{

	SearchLocalPoints(frame);

	// Optimize Pose
	// Optimizer::PoseOptimization(&mCurrentFrame);
	int mnMatchesInliers = 0;

	// Update MapPoints Statistics
	for (int i = 0; i < frame->nFeatures; ++i)
	{
		if (frame->tMapPoints[i])
		{
			if (!frame->mvbOutlier[i])
			{
				frame->tMapPoints[i]->increaseFound();
		
					if (frame->tMapPoints[i]->getNObservations() > 0)
						mnMatchesInliers++;
			}
		}
	}

	// Decide if the tracking was succesful
	// More restrictive if there was a relocalization recently
	// if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
	// 	return false;

	if (mnMatchesInliers < 30)
		return false;
	else
		return true;
}

void FullSystem::updateLocalKeyframes(std::shared_ptr<Frame> frame)
{
	mvpLocalKeyFrames.clear();

	//include currently active frames
	for (auto it : frameHessians)
	{
		assert(it->shell->frame != nullptr);

		mvpLocalKeyFrames.push_back(it->shell->frame);
		it->shell->frame->mnTrackReferenceForFrame = frame->fs->id;
		mpReferenceKF = it->shell->frame; // this will settle on the latest keyframe added in the map but might be changed later if we found one with more matches
		const std::vector<std::shared_ptr<Frame>> vNeighs = it->shell->frame->GetBestCovisibilityKeyFrames(10); //2

		for (auto it2 : vNeighs)
			if (it2->mnTrackReferenceForFrame != frame->fs->id && it->shell->frame)
			{
				mvpLocalKeyFrames.push_back(it2);
				it2->mnTrackReferenceForFrame = frame->fs->id;
			}
	}

	std::map<std::shared_ptr<Frame>, int> keyframeCounter;
	auto mapPoints = frame->getMapPointsV();

	for (auto it : mapPoints)
	{
		if (!it || it->isBad())
			continue;

		const std::map<std::shared_ptr<Frame>, size_t> observations = it->GetObservations();
		for (std::map<std::shared_ptr<Frame>, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
			keyframeCounter[it->first]++;
	}

	int max = 0;
	std::shared_ptr<Frame> pKFmax = nullptr;

	for (auto it : keyframeCounter)
	{
		if (it.first->isBad() || it.second < 10)
			continue;

		if (it.second > max)
		{
			max = it.second;
			pKFmax = it.first;
		}

		if (it.first->mnTrackReferenceForFrame == frame->fs->id)
			continue;

		mvpLocalKeyFrames.push_back(it.first);
		it.first->mnTrackReferenceForFrame = frame->fs->id;
	}

	if (pKFmax)
		mpReferenceKF = pKFmax;
	frame->mpReferenceKF = mpReferenceKF;
}

void FullSystem::updateLocalPoints(std::shared_ptr<Frame> frame)
{
	// Update local MapPoints:
	boost::unique_lock<boost::mutex> lock(localMapMtx);

	mvpLocalMapPoints.clear();

	for (auto itKF : mvpLocalKeyFrames)
	{
		std::shared_ptr<Frame> pKF = itKF;
		const std::vector<std::shared_ptr<MapPoint>> vpMPs = pKF->getMapPointsV();

		for (auto pMP : vpMPs)
		{
			if (!pMP || pMP->mnTrackReferenceForFrame == frame->fs->KfId)
				continue;

			if (!pMP->isBad() ) //&& pMP->checkVar()
			{
				mvpLocalMapPoints.push_back(pMP);
				pMP->mnTrackReferenceForFrame = frame->fs->KfId;
			}
		}
	}

	globalMap->SetReferenceMapPoints(mvpLocalMapPoints);
}

void FullSystem::updateLocalKeyframesOld(std::shared_ptr<Frame> frame)
{	
	//Update Local Keyframes
	// Each map point vote for the keyframes in which it has been observed
	std::map<std::shared_ptr<Frame>, int> keyframeCounter;
	auto mapPoint = frame->getMapPointsV();

	for (int i = 0; i < frame->nFeatures; ++i)
	{
		std::shared_ptr<MapPoint> pMP = mapPoint[i]; //frame->tMapPoints[i];
		if (pMP)
		{
			if (!pMP->isBad())
			{
				const std::map<std::shared_ptr<Frame>, size_t> observations = pMP->GetObservations();
				for (std::map<std::shared_ptr<Frame>, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
					keyframeCounter[it->first]++;
			}
			// else
			// {
			// 	frame->tMapPoints[i] = nullptr;
			// }
		}
	}

	if (keyframeCounter.empty())
		return;

	auto sortedKfs = sortLocalKFs(keyframeCounter, true);


	int max = 0;
	std::shared_ptr<Frame> pKFmax;

	mvpLocalKeyFrames.clear();
	mvpLocalKeyFrames.reserve(3 * sortedKfs.size());
	
	// All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
	// for (std::map<std::shared_ptr<Frame>, int>::const_iterator it = sortedKfs.begin(), itEnd = sortedKfs.end(); it != itEnd; it++)
	for (std::vector<std::pair<std::shared_ptr<Frame>, int>>::const_iterator it = sortedKfs.begin(), itEnd = sortedKfs.end(); it != itEnd; it++)
	{
		if( it - sortedKfs.begin() > 80)
			break;

		std::shared_ptr<Frame> pKF = it->first;

		if (pKF->isBad())
			continue;

		if (it->second > max)
		{
			max = it->second;
			pKFmax = pKF;
		}

		mvpLocalKeyFrames.push_back(it->first);
		pKF->mnTrackReferenceForFrame = frame->fs->id;
	}

	// Include also some not-already-included keyframes that are neighbors to already-included keyframes
	for (std::vector<std::shared_ptr<Frame>>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
	{
		// Limit the number of keyframes
		if (mvpLocalKeyFrames.size() > 80)
			break;

		std::shared_ptr<Frame> pKF = *itKF;

		const std::vector<std::shared_ptr<Frame>> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

		for (std::vector<std::shared_ptr<Frame>>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
		{
			std::shared_ptr<Frame> pNeighKF = *itNeighKF;
			if (!pNeighKF->isBad())
			{
				if (pNeighKF->mnTrackReferenceForFrame != frame->fs->id)
				{
					mvpLocalKeyFrames.push_back(pNeighKF);
					pNeighKF->mnTrackReferenceForFrame = frame->fs->id;
					break;
				}
			}
		}

		const std::set<std::shared_ptr<Frame>> spChilds = pKF->GetChilds();
		for (std::set<std::shared_ptr<Frame>>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
		{
			std::shared_ptr<Frame> pChildKF = *sit;
			if (!pChildKF->isBad())
			{
				if (pChildKF->mnTrackReferenceForFrame != frame->fs->id)
				{
					mvpLocalKeyFrames.push_back(pChildKF);
					pChildKF->mnTrackReferenceForFrame = frame->fs->id;
					break;
				}
			}
		}

		std::shared_ptr<Frame> pParent = pKF->GetParent();
		if (pParent)
		{
			if (pParent->mnTrackReferenceForFrame != frame->fs->id)
			{
				mvpLocalKeyFrames.push_back(pParent);
				pParent->mnTrackReferenceForFrame = frame->fs->id;
				break;
			}
		}
	}

	if (pKFmax)
	{
		mpReferenceKF = pKFmax;
		frame->mpReferenceKF = mpReferenceKF;
	}
}


void FullSystem::updateLocalPointsOld(std::shared_ptr<Frame> frame)
{
	
	// Update local MapPoints:
	boost::unique_lock<boost::mutex> lock(localMapMtx);

	mvpLocalMapPoints.clear();

	for (std::vector<std::shared_ptr<Frame>>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
	{
		std::shared_ptr<Frame> pKF = *itKF;
		const std::vector<std::shared_ptr<MapPoint>> vpMPs = pKF->getMapPointsV();

		for (std::vector<std::shared_ptr<MapPoint>>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
		{
			std::shared_ptr<MapPoint> pMP = *itMP;
			if (!pMP)
				continue;
			if (pMP->mnTrackReferenceForFrame == frame->fs->KfId)
				continue;
			if (!pMP->isBad() && pMP->checkVar())
			{
				mvpLocalMapPoints.push_back(pMP);
				pMP->mnTrackReferenceForFrame = frame->fs->KfId;
			}
		}
	}

	for (int i = 0, iend = frameHessians.size(); i < iend; ++i)
	{
		if(!frameHessians[i]->shell->frame)
			continue;


		for (int j = 0, jend = frameHessians[i]->pointHessiansMarginalized.size(); j < jend; ++j)
		{
			std::shared_ptr<MapPoint> Mp = frameHessians[i]->pointHessiansMarginalized[j]->Mp.lock();
			if (!Mp)
				continue;
			
			if (Mp->mnTrackReferenceForFrame == frame->fs->KfId)
				continue;
			if (!Mp->isBad()&& Mp->checkVar())
			{
				mvpLocalMapPoints.push_back(Mp);
				Mp->mnTrackReferenceForFrame = frame->fs->KfId;
			}

		}

		for (int j = 0, jend = frameHessians[i]->pointHessians.size(); j < jend; ++j)
		{
			std::shared_ptr<MapPoint> Mp = frameHessians[i]->pointHessians[j]->Mp.lock();
			if (!Mp)
				continue;
			
			if (Mp->mnTrackReferenceForFrame == frame->fs->KfId)
				continue;
			if (!Mp->isBad()&& Mp->checkVar())
			{
				mvpLocalMapPoints.push_back(Mp);
				Mp->mnTrackReferenceForFrame = frame->fs->KfId;
			}

		}
	}
	globalMap->SetReferenceMapPoints(mvpLocalMapPoints);
}

void FullSystem::CheckReplacedInLastFrame()
{
	for (int i = 0; i < mLastFrame->nFeatures; ++i)
	{
		std::shared_ptr<MapPoint> pMP = mLastFrame->tMapPoints[i];

		if (pMP)
		{
			std::shared_ptr<MapPoint> pRep = pMP->GetReplaced();
			if (pRep)
			{
				mLastFrame->tMapPoints[i] = pRep;
			}
		}
	}
}

int FullSystem::updatePoseOptimizationData(std::shared_ptr<Frame> frame, int & nmatches ,bool istrackingLastFrame)
{
	int nmatchesMap = 0;
	int outliers = 0;
	for (int i = 0; i < frame->nFeatures; ++i)
	{
		if (frame->tMapPoints[i])
		{
			if (istrackingLastFrame)
			{
				if (frame->mvbOutlier[i])
				{
					std::shared_ptr<MapPoint> pMP = frame->tMapPoints[i];

					frame->tMapPoints[i].reset();
					frame->mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = frame->fs->id;
					nmatches--;
					outliers++;
				}

				else if (frame->tMapPoints[i]->getNObservations() > 0)
					nmatchesMap++;
			}
			else //if tracking the localmap
			{
				if (!frame->mvbOutlier[i])
				{
					frame->tMapPoints[i]->increaseFound();

					if (frame->tMapPoints[i]->getNObservations() > 0)
						nmatchesMap++;
				}
				else //stop outliers from going to the mapping thread
				{
					frame->tMapPoints[i].reset();
					outliers++;
				}
			}
		}
	}
	// std::string out = istrackingLastFrame ? "last frame " : "local map ";
	// std::cout << "rejected outliers " + out << outliers << " total matches "<< nmatchesMap<< std::endl;
	return nmatchesMap;
}

void FullSystem::SearchInNeighbors(std::shared_ptr<Frame> currKF)
{
	// Retrieve neighbor keyframes
	int nn = 3; //5 10 20
	
	const std::vector<std::shared_ptr<Frame>> vpNeighKFs = currKF->GetBestCovisibilityKeyFrames(nn);
	std::vector<std::shared_ptr<Frame>> vpTargetKFs;
	for (std::vector<std::shared_ptr<Frame>>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
	{
		std::shared_ptr<Frame> pKFi = *vit;
		if (pKFi->isBad() || pKFi->mnFuseTargetForKF == currKF->fs->KfId)
			continue;
		vpTargetKFs.push_back(pKFi);
		pKFi->mnFuseTargetForKF = currKF->fs->KfId;

		// Extend to some second neighbors
		// const std::vector<std::shared_ptr<Frame>> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(2); //5
		// for (std::vector<std::shared_ptr<Frame>>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++)
		// {
		// 	std::shared_ptr<Frame> pKFi2 = *vit2;
		// 	if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == currKF->fs->KfId || pKFi2->fs->KfId == currKF->fs->KfId)
		// 		continue;
		// 	vpTargetKFs.push_back(pKFi2);
		// }
	}

	// Search matches by projection from current KF in target KFs
	std::vector<std::shared_ptr<MapPoint>> vpMapPointMatches = currKF->getMapPointsV();
	for (std::vector<std::shared_ptr<Frame>>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
	{
		std::shared_ptr<Frame> pKFi = *vit;
		matcher->Fuse(pKFi, vpMapPointMatches, 3.0); //th = 3.0
	}
	
	// Search matches by projection from target KFs in current KF
	std::vector<std::shared_ptr<MapPoint>> vpFuseCandidates;
	vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

	for (std::vector<std::shared_ptr<Frame>>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++)
	{
		std::shared_ptr<Frame> pKFi = *vitKF;

		std::vector<std::shared_ptr<MapPoint>> vpMapPointsKFi = pKFi->getMapPointsV();

		for (std::vector<std::shared_ptr<MapPoint>>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++)
		{
			std::shared_ptr<MapPoint> pMP = *vitMP;
			if (!pMP)
				continue;
			if (pMP->isBad() || pMP->mnFuseCandidateForKF == currKF->fs->KfId)
				continue;
			pMP->mnFuseCandidateForKF = currKF->fs->KfId;
			vpFuseCandidates.push_back(pMP);
		}
	}

	matcher->Fuse(currKF, vpFuseCandidates, 3.0); //th = 3.0

	// Update points
	vpMapPointMatches = currKF->getMapPointsV();
	for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
	{
		std::shared_ptr<MapPoint> pMP = vpMapPointMatches[i];
		if (pMP)
		{
			if (!pMP->isBad())
			{
				pMP->ComputeDistinctiveDescriptors(false);
				pMP->UpdateNormalAndDepth();
			}
		}
	}

	// Update connections in covisibility graph
	currKF->UpdateConnections();
}


void FullSystem::KeyFrameCulling(std::shared_ptr<Frame> currKF)
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 80% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    std::vector<std::shared_ptr<Frame>> vpLocalKeyFrames = currKF->GetVectorCovisibleKeyFrames();
	int KfsChecked = 0;
	for (std::vector<std::shared_ptr<Frame>>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend; vit++)
	{	
		std::shared_ptr<Frame> pKF = *vit;
		if(pKF->fs->KfId==0 || (pKF->getState()== Frame::kfstate::active))
            continue;

		int age = mpLastKeyFrame->fs->KfId - pKF->fs->KfId;
		if (age > 20)
			continue;

		KfsChecked += 1;
		if(KfsChecked > 30)
			return;

		const std::vector<std::shared_ptr<MapPoint>> vpMapPoints = pKF->getMapPointsV();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            std::shared_ptr<MapPoint> pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    if(pMP->getNObservations()>thObs)
                    {
                        // const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const std::map<std::shared_ptr<Frame>, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(std::map<std::shared_ptr<Frame>, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            std::shared_ptr<Frame> pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            // const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            // if(scaleLeveli<=scaleLevel+1)
                            // {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            // }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>0.9*nMPs) //0.9
            {
				if(pKF->getState() != Frame::kfstate::active)
				{
					pKF->setBadFlag();		
					pKF->fs->frame.reset();
				}
			}
	}
}

void FullSystem::BAatExit()
{
	std::vector<std::shared_ptr<Frame>> allKFrames;
	std::vector<std::shared_ptr<MapPoint>> allMapPoints;

	globalMap->GetAllKeyFrames(allKFrames);
	globalMap->GetAllMapPoints(allMapPoints);


	bool stopGBA = false;

	size_t currMaxKF = allKeyFramesHistory.back()->KfId;
	size_t currMaxMp = globalMap->GetMaxMPid();

	BundleAdjustment(allKFrames, allMapPoints, 10, &stopGBA, true, true, currMaxKF, currMaxKF - 15, currMaxMp);
	for (auto it : allKeyFramesHistory)
	    it->setRefresh(true);
}

void FullSystem::setVocab(DBoW3::Vocabulary* _Vocabpnt)
{
	loopCloser->lc_setVocab(_Vocabpnt);
	globalMap->m_setVocab(_Vocabpnt);
}


// SE3 FullSystem::cumulativeForm()
// {
// 	auto v = vVelocity;
// 	if (vVelocity.size() == 4)
// 	{
// 		int u = 3;

// 		SE3 t1 = v.front(); v.pop();
// 		SE3 t2 = v.front(); v.pop();
// 		SE3 t3 = v.front(); v.pop();
// 		SE3 t4 = v.front(); v.pop();
// 		return SE3::exp(t1.log()) *
// 			   SE3::exp(((5 + 3 * u - 3 * u * u + u * u * u) / 6) * SE3(t1.inverse() * t2).log()) *
// 			   SE3::exp(((1 + 3 * u + 3 * u * u - 2 * u * u * u) / 6) * SE3(t2.inverse() * t3).log()) *
// 			   SE3::exp(((u * u * u) / 6) * SE3(t3.inverse() * t4).log());
// 	}
// 	else
// 	{
// 		if(vVelocity.size() == 0 )
// 			return SE3();

// 		return vVelocity.back();
// 	}
// }

// =================== COARSE TRACKER DEPTH INTEGRATION COORDINATION ===================

/**
 * @brief Update CoarseTracker instances with current depth information
 * 
 * Synchronizes external depth information with both coarseTracker and 
 * coarseTracker_forNewKF instances to ensure consistent depth integration.
 */
void FullSystem::updateCoarseTrackerDepth()
{
    // Skip if not initialized yet or trackers are not ready
    if(!initialized || !coarseTracker || !coarseTracker_forNewKF) {
        return;
    }
    
    // Ensure depth synchronization with tracking
    {
        boost::unique_lock<boost::mutex> lock(rgbd_depth_mutex_);
        if(!currentDepthImage.empty()) {
            // Validate depth image format before setting
            if(currentDepthImage.type() == CV_32FC1) {
                // Update main coarse tracker
                coarseTracker->setExternalDepthImage(currentDepthImage);
                
                // Update coarse tracker for new keyframes
                coarseTracker_forNewKF->setExternalDepthImage(currentDepthImage);
            
            if(!setting_debugout_runquiet) {
                printf("FullSystem: Updated CoarseTracker instances with depth image (%dx%d)\n", 
                       currentDepthImage.cols, currentDepthImage.rows);
            }
            } else {
                printf("WARNING: Invalid depth image format for CoarseTracker integration\n");
            }
        } else {
            // CRITICAL FIX for Phase 3: Don't clear ML depth when currentDepthImage is empty
            // currentDepthImage is for RGB-D depth, but we might have ML depth
            // Only clear if we know there's no depth source at all
            bool main_has_ml_depth = coarseTracker->hasExternalDepth();
            bool kf_has_ml_depth = coarseTracker_forNewKF->hasExternalDepth();
            
            if(!main_has_ml_depth && !kf_has_ml_depth) {
                // Only clear if neither tracker has any depth (ML or RGB-D)
                coarseTracker->clearExternalDepthImage();
                coarseTracker_forNewKF->clearExternalDepthImage();
                
                if(!setting_debugout_runquiet) {
                    printf("FullSystem: Cleared depth from CoarseTracker instances\n");
                }
            }
            // Otherwise preserve ML depth that was set during keyframe processing
        }
    }
}

/**
 * @brief Synchronize depth with tracking operations
 * 
 * Ensures proper timing and coordination between depth updates and 
 * tracking operations to maintain system consistency.
 */
void FullSystem::synchronizeDepthWithTracking()
{
    // Update depth information before tracking operations
    updateCoarseTrackerDepth();
    
    // Ensure both trackers have consistent depth information
    if(coarseTracker && coarseTracker_forNewKF) {
        bool main_has_depth = coarseTracker->hasExternalDepth();
        bool kf_has_depth = coarseTracker_forNewKF->hasExternalDepth();
        
        if(main_has_depth != kf_has_depth) {
            printf("WARNING: Depth synchronization mismatch between CoarseTracker instances\n");
            // Re-synchronize to ensure consistency
            updateCoarseTrackerDepth();
        }
        
        // Log depth integration statistics from the main tracker
        if(main_has_depth) {
            auto stats = coarseTracker->getLastIntegrationStats();
            HSLAM::DepthLogger::logIntegrationStats(
                stats.pixels_from_external_depth,
                stats.pixels_fused,
                stats.total_valid_pixels,
                stats.integration_rate,
                "FullSystem"
            );
        }
    }
}

// =================== ML DEPTH SERVICE INTEGRATION (Phase 2) ===================





FullSystem::MLMetrics FullSystem::getMLMetrics() const
{
    MLMetrics metrics = ml_metrics_;
    
    // ML processor statistics are directly maintained in ml_metrics_
    
    return metrics;
}

bool FullSystem::shouldCreateKeyframe() const
{
    // Use existing HSLAM keyframe creation logic
    // This is a simplified version - the actual logic is in addActiveFrame
    // For now, use a basic heuristic based on frame count and tracking quality
    
    if (allFrameHistory.size() == 1) {
        return true;  // Always make first frame a keyframe
    }
    
    if (setting_keyframesPerSecond > 0) {
        // Fixed keyframe rate mode
        if (allKeyFramesHistory.empty()) {
            return true;
        }
        double time_since_last_kf = allFrameHistory.back()->timestamp - allKeyFramesHistory.back()->timestamp;
        return time_since_last_kf > 0.95f / setting_keyframesPerSecond;
    }
    
    // For adaptive keyframe creation, use conservative estimate
    // In practice, this would use more sophisticated tracking quality metrics
    return (allFrameHistory.size() % 10 == 0);  // Every 10th frame as rough estimate
}

// Keyframe statistics methods
float FullSystem::getKeyframeRatio() const
{
	boost::lock_guard<boost::mutex> lock(trackMutex);
	
	if (allFrameHistory.empty()) {
		return 0.0f;
	}
	
	size_t keyframeCount = allKeyFramesHistory.size();
	size_t totalFrameCount = allFrameHistory.size();
	
	return static_cast<float>(keyframeCount) / static_cast<float>(totalFrameCount);
}


size_t FullSystem::getTotalFrameCount() const
{
	boost::lock_guard<boost::mutex> lock(trackMutex);
	return allFrameHistory.size();
}

void FullSystem::printKeyframeStats() const
{
	size_t keyframeCount = getKeyframeCount();
	size_t totalFrameCount = getTotalFrameCount();
	float keyframeRatio = getKeyframeRatio();
	
	// Console output following existing patterns
	if (!setting_debugout_runquiet) {
		printf("Keyframe Statistics: %zu keyframes / %zu total frames (%.2f%% ratio)\n",
			   keyframeCount, totalFrameCount, keyframeRatio * 100.0f);
	}
	
	// File logging if enabled
	if (setting_logStuff && numsLog != nullptr) {
		(*numsLog) << "KEYFRAME_STATS " << keyframeCount << " " << totalFrameCount 
				   << " " << std::fixed << std::setprecision(4) << keyframeRatio << "\n";
		numsLog->flush();
	}
}

void FullSystem::printInitializationPerformance() const
{
	printf("\n=== Initialization Performance ===\n");
	printf("Method: %s\n", using_metric_scale_ ? "Metric (ML-based)" : "Photometric");
	printf("Total Time: %.1f ms\n", total_init_time_ms_);
	if (ml_init_processing_time_ms_ > 0) {
		printf("  ML Processing: %.1f ms (%.1f%%)\n", 
			   ml_init_processing_time_ms_,
			   (ml_init_processing_time_ms_/total_init_time_ms_)*100);
	}
	printf("  Tracking: %.1f ms\n", total_init_time_ms_ - ml_init_processing_time_ms_);
	printf("Scale Factor: %.4f\n", init_scale_factor_);
	printf("Initial Points: %d\n", init_points_count_);
	printf("=================================\n");
}

void FullSystem::printAblationStatistics() const
{
	if (ml_config_.inference_strategy != "snapshot_mode") {
		return;
	}
	
	printf("\n");
	printf("========================================\n");
	printf("   ML INFERENCE ABLATION STUDY RESULTS  \n");
	printf("========================================\n");
	printf("Strategy: %s\n", ml_config_.inference_strategy.c_str());
	printf("Snapshot Rate: Every %d keyframes\n", ml_config_.snapshot_rate);
	printf("\n");
	printf("KEYFRAME STATISTICS:\n");
	printf("  Total Keyframes: %zu\n", keyframe_counter_);
	printf("  ML Inferences: %zu\n", ml_inference_counter_);
	printf("  Skipped: %zu\n", keyframe_counter_ - ml_inference_counter_);
	printf("\n");
	printf("COVERAGE ANALYSIS:\n");
	printf("  Expected Coverage: %.1f%%\n", 100.0f / ml_config_.snapshot_rate);
	printf("  Actual Coverage: %.1f%%\n", 
	       keyframe_counter_ > 0 ? 100.0f * ml_inference_counter_ / keyframe_counter_ : 0.0f);
	printf("  Inference Reduction: %.1f%%\n", 
	       100.0f * (keyframe_counter_ - ml_inference_counter_) / std::max(keyframe_counter_, size_t(1)));
	printf("\n");
	printf("PERFORMANCE METRICS:\n");
	printf("  ML Success Rate: %.1f%%\n", 
	       ml_inference_counter_ > 0 ? 100.0f * ml_metrics_.ml_keyframes_successful / ml_inference_counter_ : 0.0f);
	printf("  Avg ML Time: %.1f ms\n", ml_metrics_.avg_ml_inference_time_ms);
	printf("========================================\n");
}

// =================== MLDepthProcessor Initialization ===================
bool FullSystem::initializeMLDepthProcessor(const MLConfig& config)
{
	try {
		// Store configuration for use in ablation study logic
		ml_config_ = config;
		
		// Convert FullSystem::MLConfig to ML::MLInference::InferenceConfig
		ML::MLInference::InferenceConfig ml_config;
		ml_config.model_path = config.model_path;
		ml_config.model_type = ML::MLInference::METRIC3D_V2;
		ml_config.enable_gpu = config.enable_gpu;
		ml_config.num_threads = config.num_threads;
		
		// GPU-specific parameters
		ml_config.enable_fp16 = config.enable_fp16;
		ml_config.gpu_device_id = config.gpu_device_id;
		ml_config.gpu_memory_limit = config.gpu_memory_limit_mb * 1024 * 1024;  // Convert MB to bytes
		
		ml_config.input_width = 518;   // Metric3D model requirement for quality inference
		ml_config.input_height = 518;  // Metric3D model requirement for quality inference
		ml_config.benchmark_enabled = config.benchmark_enabled;
		
		// Create MLDepthProcessor instance
		ml_processor_ = std::make_unique<ML::MLDepthProcessor>(ml_config);
		
		// Initialize the processor
		if (!ml_processor_->initialize()) {
			printf("initializeMLDepthProcessor: ERROR - Failed to initialize ML processor\n");
			ml_processor_.reset();
			return false;
		}
		
		printf("initializeMLDepthProcessor: MLDepthProcessor initialized successfully\n");
		
		// CRITICAL: Enable ML depth processing flag
		ml_depth_enabled_ = true;
		
		return true;
		
	} catch (const std::exception& e) {
		printf("initializeMLDepthProcessor: ERROR - Exception during initialization: %s\n", e.what());
		ml_processor_.reset();
		ml_depth_enabled_ = false;
		return false;
	} catch (...) {
		printf("initializeMLDepthProcessor: ERROR - Unknown exception during initialization\n");
		ml_processor_.reset();
		ml_depth_enabled_ = false;
		return false;
	}
}

bool FullSystem::performMLWarmup(const cv::Mat& warmup_image)
{
	if (!ml_processor_ || !ml_depth_enabled_) {
		printf("ERROR: ML processor not initialized for warmup\n");
		return false;
	}
	
	try {
		// Validate warmup image
		if (warmup_image.empty() || warmup_image.channels() != 3) {
			printf("ERROR: Invalid warmup image format\n");
			return false;
		}
		
		// Perform detailed warmup inference and STORE results for initialization reuse
		auto warmup_result = ml_processor_->processKeyframeDetailed(warmup_image);
		
		if (warmup_result.success && !warmup_result.depth_map.empty()) {
			// STORE the warmup results for initialization
			warmup_depth_map_ = warmup_result.depth_map.clone();
			warmup_mean_depth_ = warmup_result.mean_depth;
			warmup_confidence_ = warmup_result.confidence;
			warmup_results_available_ = true;
			
			printf("GPU warmup complete - stored metric scale: %.2fm\n", warmup_mean_depth_);
			return true;
		} else {
			printf("ERROR: ML warmup inference failed\n");
			return false;
		}
		
	} catch (const std::exception& e) {
		printf("ERROR: Exception during ML warmup: %s\n", e.what());
		return false;
	}
}

// Phase 5: Scale drift diagnostics — pure instrumentation, no trajectory impact
void FullSystem::monitorScaleDrift(FrameHessian* newKF, const cv::Mat& mlDepth)
{
	if (mlDepth.empty() || !newKF) return;

	Mat33f K = Mat33f::Identity();
	K(0,0) = Hcalib.fxl(); K(1,1) = Hcalib.fyl();
	K(0,2) = Hcalib.cxl(); K(1,2) = Hcalib.cyl();
	Mat33f Ki = K.inverse();

	std::vector<float> scale_ratios;  // Signal 1: SLAM_depth / ML_depth
	float total_residual = 0.0f;      // Signal 2: mean |idepth - ml_ref| / ml_ref
	int residual_count = 0;

	for (FrameHessian* host : frameHessians) {
		SE3 hostToNew = newKF->PRE_worldToCam * host->PRE_camToWorld;
		Mat33f KRKi = K * hostToNew.rotationMatrix().cast<float>() * Ki;
		Vec3f Kt = K * hostToNew.translation().cast<float>();

		for (PointHessian* ph : host->pointHessians) {
			if (!ph->hasMLDepth || ph->idepth <= 0) continue;

			// Signal 2: residual magnitude for ALL ML points
			if (ph->ml_idepth_reference > 0) {
				float rel_residual = std::abs(ph->idepth - ph->ml_idepth_reference) / ph->ml_idepth_reference;
				total_residual += rel_residual;
				residual_count++;
			}

			// Signal 1: project into new KF's ML depth map
			float Ku, Kv;
			if (!projectPoint(ph->u, ph->v, ph->idepth_scaled, KRKi, Kt, Ku, Kv)) continue;
			int ui = (int)(Ku + 0.5f), vi = (int)(Kv + 0.5f);
			if (ui < 1 || vi < 1 || ui >= mlDepth.cols - 1 || vi >= mlDepth.rows - 1) continue;

			float mlDepthVal = mlDepth.at<float>(vi, ui);
			if (mlDepthVal <= 0 || !std::isfinite(mlDepthVal)) continue;

			float slamDepth = 1.0f / ph->idepth;
			float ratio = slamDepth / mlDepthVal;
			if (std::isfinite(ratio) && ratio > 0) scale_ratios.push_back(ratio);
		}
	}

	// Compute median live scale ratio
	float live_ratio = 1.0f;
	if (scale_ratios.size() >= 10) {
		size_t mid = scale_ratios.size() / 2;
		std::nth_element(scale_ratios.begin(), scale_ratios.begin() + mid, scale_ratios.end());
		live_ratio = scale_ratios[mid];
	}

	// EMA update
	scale_ema_ = 0.9f * scale_ema_ + 0.1f * live_ratio;
	float drift_pct = (scale_ema_ - 1.0f) * 100.0f;
	float mean_residual = (residual_count > 0) ? (total_residual / residual_count) : 0.0f;
	scale_monitor_count_++;

	// Log scale drift every 10 keyframes (pure diagnostic, no trajectory impact)
	if (scale_monitor_count_ % 10 == 0) {
		// printf("[SCALE_DRIFT] KF%d: live_ratio=%.4f ema=%.4f drift=%+.1f%% | mean_residual=%.4f (%d pts) | %zu projections\n",
		//        newKF->frameID, live_ratio, scale_ema_, drift_pct, mean_residual, residual_count, scale_ratios.size());
	}

	// ML probation mechanism — GATED
	// Originally designed to auto-enable Direct.P2 after ML consistency check passes,
	// or disable Direct.P1 if ML is unreliable (e.g., EuRoC calibration flight).
	// Gated because Direct.P2 is permanently disabled (proved inert). The concept of
	// auto-detecting unreliable ML sequences is valuable and may be repurposed for the
	// indirect pipeline. See KEY_INSIGHTS.md §2.1, IMPLEMENTATION_PLAN.md (indirect).
	//
	// if (!ml_fallback_triggered_ && scale_ratios.size() >= 10) {
	// 	ml_fallback_ratios_.push_back(live_ratio);
	// 	if (ml_fallback_ratios_.size() == 20) {
	// 		float sum = 0, sum_sq = 0;
	// 		for (float r : ml_fallback_ratios_) { sum += r; sum_sq += r * r; }
	// 		float mean = sum / ml_fallback_ratios_.size();
	// 		float variance = sum_sq / ml_fallback_ratios_.size() - mean * mean;
	// 		bool ratio_unreliable = (mean < 0.65f || mean > 1.5f);
	// 		if (variance > 0.3f || mean_residual > 0.5f || ratio_unreliable) {
	// 			printf("[ML_FALLBACK] ML predictions unreliable (var=%.3f, mean_residual=%.3f, mean_ratio=%.3f). "
	// 			       "Direct.P2 remains DISABLED, disabling Direct.P1 bounds.\n", variance, mean_residual, mean);
	// 			setting_enableDirectP1Bounds = false;
	// 			ml_fallback_triggered_ = true;
	// 		} else {
	// 			printf("[ML_FALLBACK] ML predictions OK (var=%.3f, mean_residual=%.3f, mean_ratio=%.3f). "
	// 			       "ENABLING Direct.P2 self-gating BA.\n", variance, mean_residual, mean);
	// 			setting_disableDirectP2BA = false;
	// 			ml_fallback_triggered_ = true;
	// 		}
	// 	}
	// }
}

// ML Reference Frame Management (CoarseTracker-style thread safety)
void FullSystem::setMLReference(int frame_id, const cv::Mat& depth, float confidence, double inference_time) {
	boost::unique_lock<boost::mutex> lock(ml_reference_mutex_);
	
	if (!depth.empty()) {
		ml_reference_depth_ = depth.clone();  // Safe copy like CoarseTracker
		ml_reference_frame_id_ = frame_id;
		ml_reference_confidence_ = confidence;
		ml_reference_time_ = inference_time;
	}
}

bool FullSystem::hasMLReference() const {
	boost::unique_lock<boost::mutex> lock(ml_reference_mutex_);
	return !ml_reference_depth_.empty() && ml_reference_frame_id_ >= 0;
}

cv::Mat FullSystem::getMLReferenceDepth() const {
	boost::unique_lock<boost::mutex> lock(ml_reference_mutex_);
	if (!ml_reference_depth_.empty()) {
		return ml_reference_depth_.clone();  // SAFE: deep copy following RGB-D pattern
	}
	return cv::Mat();  // Return empty Mat if no depth available
}

int FullSystem::getMLReferenceFrameId() const {
	boost::unique_lock<boost::mutex> lock(ml_reference_mutex_);
	return ml_reference_frame_id_;
}

}
