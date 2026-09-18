#pragma once

#include <string.h>
#include <string>
#include <cmath>
#include "DBoW3/Vocabulary.h"

namespace HSLAM
{

    extern bool Pause; //not a good idea:)
    extern bool LoopClosure;
    extern int gridSize;
    extern int mnGridCols, mnGridRows;
    extern float mnMinX, mnMaxX, mnMinY, mnMaxY, mfGridElementWidthInv, mfGridElementHeightInv;
    extern int EDGE_THRESHOLD;

    extern int minThFAST;
    extern int minIndDist;
    extern int minDirDist;
    extern int indFeaturesToExtract;

    extern int minKfIdDist_LoopCandidate;
    extern int kfGap;
    extern int mnCovisibilityConsistencyTh;


    extern bool normalizeInfoWithVariance;

#define SOLVER_SVD (int)1
#define SOLVER_ORTHOGONALIZE_SYSTEM (int)2
#define SOLVER_ORTHOGONALIZE_POINTMARG (int)4
#define SOLVER_ORTHOGONALIZE_FULL (int)8
#define SOLVER_SVD_CUT7 (int)16
#define SOLVER_REMOVE_POSEPRIOR (int)32
#define SOLVER_USE_GN (int)64
#define SOLVER_FIX_LAMBDA (int)128
#define SOLVER_ORTHOGONALIZE_X (int)256
#define SOLVER_MOMENTUM (int)512
#define SOLVER_STEPMOMENTUM (int)1024
#define SOLVER_ORTHOGONALIZE_X_LATER (int)2048


// ============== PARAMETERS TO BE DECIDED ON COMPILE TIME =================
#define PYR_LEVELS 6
extern int pyrLevelsUsed;



extern float setting_keyframesPerSecond;
extern bool setting_realTimeMaxKF;
extern float setting_maxShiftWeightT;
extern float setting_maxShiftWeightR;
extern float setting_maxShiftWeightRT;
extern float setting_maxAffineWeight;
extern float setting_kfGlobalWeight;



extern float setting_idepthFixPrior;
extern float setting_idepthFixPriorMargFac;
extern float setting_initialRotPrior;
extern float setting_initialTransPrior;
extern float setting_initialAffBPrior;
extern float setting_initialAffAPrior;
extern float setting_initialCalibHessian;

extern int setting_solverMode;
extern double setting_solverModeDelta;


extern float setting_minIdepthH_act;
extern float setting_minIdepthH_marg;



extern float setting_maxIdepth;
extern float setting_maxPixSearch;
extern float setting_desiredImmatureDensity;			// done
extern float setting_desiredPointDensity;			// done
extern float setting_minPointsRemaining;
extern float setting_maxLogAffFacInWindow;
extern int setting_minFrames;
extern int setting_maxFrames;
extern int setting_minFrameAge;
extern int setting_maxOptIterations;
extern int setting_minOptIterations;
extern float setting_thOptIterations;
extern float setting_outlierTH;
extern float setting_outlierTHSumComponent;



extern int setting_pattern;
extern float setting_margWeightFac;
extern int setting_GNItsOnPointActivation;


extern float setting_minTraceQuality;
extern int setting_minTraceTestRadius;
extern float setting_reTrackThreshold;


extern int   setting_minGoodActiveResForMarg;
extern int   setting_minGoodResForMarg;
extern int   setting_minInlierVotesForMarg;




extern int setting_photometricCalibration;
extern bool setting_useExposure;
extern float setting_affineOptModeA;
extern float setting_affineOptModeB;
extern int setting_gammaWeightsPixelSelect;



extern bool setting_forceAceptStep;



extern float setting_huberTH;
extern float setting_huberTH_Ind;

extern bool setting_logStuff;
extern float benchmarkSetting_fxfyfac;
extern int benchmarkSetting_width;
extern int benchmarkSetting_height;
extern float benchmark_varNoise;
extern float benchmark_varBlurNoise;
extern int benchmark_noiseGridsize;
extern float benchmark_initializerSlackFactor;

extern float setting_frameEnergyTHConstWeight;
extern float setting_frameEnergyTHN;

extern float setting_frameEnergyTHFacMedian;
extern float setting_overallEnergyTHWeight;
extern float setting_coarseCutoffTH;

extern float setting_minGradHistCut;
extern float setting_minGradHistAdd;
extern float setting_gradDownweightPerLevel;
extern bool  setting_selectDirectionDistribution;



extern float setting_trace_stepsize;
extern int setting_trace_GNIterations;
extern float setting_trace_GNThreshold;
extern float setting_trace_extraSlackOnTH;
extern float setting_trace_slackInterval;
extern float setting_trace_minImprovementFactor;


extern bool setting_render_displayCoarseTrackingFull;
extern bool setting_render_renderWindowFrames;
extern bool setting_render_plotTrackingFull;


extern bool setting_fullResetRequested;

extern bool setting_debugout_runquiet;

extern bool disableAllDisplay;
extern bool outputPC;
extern bool disableReconfigure;


extern bool setting_onlyLogKFPoses;

// ML Depth Integration Settings — Direct Pipeline (Photometric/DSO)
// Naming: "Direct.P0" = metric init, "Direct.P1" = depth bounds, "Direct.P2" = BA energy, "Direct.P3" = tracker fusion
extern bool setting_enableDirectP1Bounds;     // Direct.P1: ML depth bounds on ImmaturePoint tracing
extern float setting_mlDepthWeight;
extern float setting_mlGaussianScale;
extern bool setting_disableDirectP2BA;        // Direct.P2: ML energy in photometric BA (INERT — see KEY_INSIGHTS.md §2.1)
extern bool setting_disableDirectP3Tracker;   // Direct.P3: ML depth fusion in CoarseTracker (marginal)
extern float setting_mlSelfGateTau;           // Direct.P2: self-gating width (Paper Eq. 5) — currently gated
extern int setting_mlInferenceEveryN;         // Paper Table V: run ML every Nth keyframe (N=2 optimal)

// ML Depth Integration Settings — Direct Virtual Stereo (Step 4)
// Naming: "Direct.VS" = virtual stereo constraint in DSO BA (image-space, self-gating via gradient)
extern bool setting_disableDirectVS;     // Direct.VS: kill switch (default false)
extern float setting_vsBaseline;         // Direct.VS: virtual baseline in meters (default 0.1m)
extern float setting_vsWeight;           // Direct.VS: weight multiplier (default 0.001, tune per eval)

// ML Depth Integration Settings — Indirect Pipeline (Geometric/ORB)
// Naming: "Indirect.P0" = MapPoint storage, "Indirect.P1" = BA depth prior, "Indirect.P2" = loop scale validation
extern float setting_indirectMLDepthWeight;      // Indirect.P1: Weight multiplier for g2o depth prior edges (INERT — consumer is dead BundleAdjustment)
extern bool setting_disableIndirectP2LoopCloser; // Indirect.P2: DEPRECATED. Was the original P2 rejection-gate kill-switch; superseded by setting_indirectP2RejectGate below. Retained as-declared for paper-doc traceability but no longer consumed in code.
extern bool setting_indirectMlSemanticFix;       // Indirect.H0: when true, s_ml in LoopCloser is computed from current-KF ML depth images at matched feature pixels (correct inter-KF scale) instead of source-frame MapPoint ML idepth ratios. Default true. SML_COMPARE diagnostic always prints both regardless of this flag. CLI: --indirect-ml-semantic-fix.
extern bool setting_indirectP2RejectGate;        // Indirect.H2: when true, reject loop-closure Sim3 candidates with |s_RANSAC − s_ML|/max > setting_indirectP2RejectThresh. Uses new s_ml when available (n≥5), else falls back to old s_ml (n≥5), else bypasses (coverage_low). Default true (shipped May 8, 2026). CLI: --p2-gate.
// WP0 (RA-L v2): [LC_SCALE_GATE] counters for the Indirect.H2 scale-disagreement gate. Written only
// by the loop-closure thread; read once at shutdown, after that thread is joined.
// WP0 (RA-L v2): run provenance captured at PARSE time. A failed ML warmup flips ml_depth_enabled_
// and a subsequent init reset builds a fresh FullSystem whose ml_config_ is empty, so the live
// instance cannot be trusted to report what the run was asked to do. These can.
extern std::string setting_mlModelPathResolved;  // after any --ml-fp16 substitution
extern bool setting_mlGpuRequested;
extern bool setting_mlFp16Requested;

extern int stat_lcScaleGateEvals;    // candidate Sim3 reached the gate with usable ML coverage
extern int stat_lcScaleGateRejects;  // rejected as scale-degenerate (the mechanism firing)
extern int stat_lcScaleGateBypass;   // skipped: ML coverage < 5 on both estimators
extern float setting_indirectP2RejectThresh;     // Indirect.H2: disagreement threshold for the gate. Default 0.5 (i.e. reject if |s_RANSAC − s_ML| / max > 50%). CLI: --p2-gate-thresh.
extern bool setting_indirectH3AbsScalePrior;     // Indirect.H3-abs: per-KF unary EdgeSim3ScalePrior in OptimizeEssentialGraph. Default false (opt-in). Requires bias-corrected s_target (multiplied by setting_indirectH3BiasCorrection); without correction, anchors gauge to Metric3D's biased floor. CLI: --h3-abs.
extern bool setting_indirectH3RelScalePrior;     // Indirect.H3-rel: pairwise EdgeSim3RelScalePrior (covisible KF pairs) in OptimizeEssentialGraph. Bias cancels by construction, no correction needed. Default false (opt-in). CLI: --h3-rel.
extern float setting_indirectH3InfoWeight;       // Indirect.H3: information-matrix weight for both abs and rel scale priors. Greene & Roy ICRA 2020 left this open; sweep {1e-3, 1e-1, 1, 1e1, 1e3} per plan §6.4. Default 1.0. CLI: --h3-weight.
extern float setting_indirectH3BiasCorrection;   // Indirect.H3-abs: scalar bias multiplier applied to s_ml_implied before use as unary-prior target. Default 0.55 (Phase C empirical Metric3D effective bias factor mapping outdoor scale 0.55 floor → 1.0). For bias-cancelling H3-rel this is unused. CLI: --h3-bias.
extern bool setting_indirectSim3MlSeed;          // Indirect.H1 (seed-sensitivity diagnostic): when true, blend RANSAC scale with ML-derived s_ml before OptimizeSim3. s_seed = alpha*s_RANSAC + (1-alpha)*s_ml. Default false. Per lit-audit reframing as diagnostic, not ATE-test. CLI: --sim3-seed.
extern float setting_indirectSim3MlSeedAlpha;    // Indirect.H1: blend weight on s_RANSAC. alpha=1.0 (default) is no-op (pure RANSAC). alpha=0.0 is pure ML. Sweep {0.0, 0.3, 0.5, 0.7, 0.9, 1.0} per plan §6.2. CLI: --sim3-seed-alpha.
extern bool setting_indirectS1ConfidenceGate;    // Indirect.S.1 (lit audit §3.6 LR4): wrap H2 gate with ML κ-confidence check. When mean κ over matched-feature pixels < setting_indirectS1ConfidenceThresh, BYPASS H2 (ML unreliable; fall back to RANSAC-only). Default false; no effect when --p2-gate=false. CLI: --s1-confidence-gate.
extern float setting_indirectS1ConfidenceThresh; // Indirect.S.1: AngMF κ threshold (raw, not sigmoid). Sweep {1, 5, 10, 25, 50}. Default 5.0 (loose initial bar). CLI: --s1-confidence-thresh.
extern bool setting_indirectMapPointMLStorage;   // Indirect.P0 kill switch: when false, MapPoints are constructed WITHOUT ml_idepth/ml_uncertainty/hasMLDepth fields populated. Disables H2's old-estimator fallback and H3's old-estimator fallback. Step2a/2b matchers unaffected (they read live ML image, not MP fields). Default true (preserves existing behavior). CLI: --indirect-mp-ml-storage.

// GT Depth Validation (Phase B) — research-only, NOT a shipped feature
// When depthSource=GT, load GT depth from --associations PNG files and route it through
// the ML depth integration path (idepth_GT, bounds, Phase 0/1/2/VS hooks all fire identically).
// Purpose: decouple integration-correctness testing from Metric3D bias. See docs/gt_depth_validation/PLAN.md.
enum DepthSource { DEPTH_SOURCE_ML = 0, DEPTH_SOURCE_GT = 1, DEPTH_SOURCE_NONE = 2 };
extern int setting_depthSource;                  // default DEPTH_SOURCE_ML; flip via --depth-source=ml|gt|none

// ML Initialization Settings
extern bool setting_useMLForInitialization;      // Enable ML-based initialization
extern float setting_mlInitConfidenceThreshold;  // Min confidence for ML scale
extern int setting_mlInitMinPoints;              // Min points for scale estimation
extern float setting_mlInitMinGoodRatio;         // Min fraction of well-triangulated points for ML init
extern bool setting_mlReinitializationEnabled;   // Enable for re-init
extern float setting_alphaWForMLInit;            // Phase 0 alpha-prior strength when ML-seeded (default 50*50; mono uses 150*150)
extern float setting_idepthUncertaintyForMLInit; // Phase 1 base idepth-uncertainty for ML bounds (default 0.05; larger = looser bounds)
extern int setting_mlMeanDepthStrategy;          // mlMeanDepth computation: 0=arith mean of valid, 1=median, 2=trimmed 5-95% (default 0 for backward compat; recommended: 2)
extern int setting_mlInferenceMode;              // ML inference cadence: 0=every Nth KF (default), 1=init_only, 2=disabled
extern bool setting_indirectMatcherUseML;        // Indirect.Step2: gate ML depth-ratio filter in feature matchers (Matcher.cpp). Default true (legacy).

// Map PLY export (Sprint 0d) — optional end-of-run export of marginalized PointHessians
extern bool setting_exportMapPly;        // default false; set via --export-map-ply
extern std::string setting_mapPlyOut;    // default "": derive from result file path

// Surface Normal Integration (Sprint 1) — master gate for downstream normal consumers.
// Normals are always extracted from ONNX output[1] when model_type=METRIC3D_V2 and the
// multi-output IoBinding is active (Sprint 0c). This flag gates any downstream code that
// READS the normals. Default false until Sprint 2 ships and is validated.
// CLI: --use-normal-integration

// Sprint 2 — CD-H5: foreshortening factor |n·z_hat| in idepth uncertainty formula.
// Widens bounds on edge-on surfaces (geometrically ill-conditioned for depth).
// Default true after Sprint 2 win verdict. CLI: --ml-foreshortening
// WP0 (RA-L v2): setting_useNormalIntegration and setting_disableIndirectMLDepth were
// deleted. The first gated no mechanism (parse + printf only); the second had zero
// consumers anywhere in the tree. Neither was an ablation.
extern bool setting_useNormalForeshortening;

// Sprint 3 — A.2: κ → AngMF expected-angle confidence formula.
// Replaces the mis-oriented sigmoid 1/(1+κ/5) with 1 - E_angle(κ)/π ∈ [0.5, 1.0].
// Default false (KILL verdict: KITTI +107% ATE; Sprint 3a re-sweeps --ml-idepth-uncertainty).
// CLI: --ml-angmf-confidence
extern bool setting_useAngmfConfidence;

// Sprint 4 — CD-H3: gate CoarseTracker lvl=0 gap-fill dilation on normal-cosine agreement.
// Skips a neighbour's depth contribution when its surface normal disagrees with the center
// pixel's normal by more than acos(setting_normalGapfillCosThreshold).  Prevents phantom
// medium-depth pixels at object boundaries from entering the semi-dense reference depth.
// KILL verdict (2026-05-07): cos_min=0.873 across TUM; threshold 0.866 catches nothing.
// Default false (KILL). CLI: --ml-normal-gapfill-mask / --ml-normal-gapfill-cos
extern bool setting_useNormalGapfillMask;
extern bool setting_useNormalOptReg;   // Sprint 5 (CD-H7/B-NEW-1): normal-cosine-weighted iR smoothing in CoarseInitializer::optReg
extern bool setting_useNormalPixelGate;    // Sprint 6 (CD-H2): gate lvl-0 pixel selection on |n·z_hat|
extern float setting_normalPixelGateCos;   // Sprint 6 (CD-H2): cosine threshold (default 0.5 = 60deg)
extern float setting_normalGapfillCosThreshold;  // cos(30°) ≈ 0.866

// Sprint 7 (C-NEW-2 / H7): normal-viewing-angle information weighting in Indirect::Optimizer::
// PoseOptimization. Each observation edge's information matrix is scaled by max(|n·z_hat|, floor),
// where n is the MapPoint's cached host-keyframe ML surface normal rotated into the current camera
// frame. Soft weight (not a gate) — the Sprint 2 category, not the Sprints 4/5/6 category.
// CLI: --ml-normal-indirect-info / --ml-normal-indirect-info-floor
extern bool setting_useNormalIndirectInfo;
extern float setting_normalIndirectInfoFloor;

// Sprint 10 (E3 / Phase B.4 / H3.A-B-C): depth-normal surface-consistency residual in the
// photometric BA. For each active point j with an ML normal n_j, the local tangent plane implied by
// n_j predicts j's inverse depth from each spatial neighbour i's CURRENT (frozen) inverse depth:
//     rho_hat_j^(i) = rho_i * (n_j . v_j) / (n_j . v_i),   v = K^-1 * p_tilde
// rho_hat_j is the median over neighbours, and the residual r = rho_j - rho_hat_j is added as a
// point-diagonal prior (exactly like ml_priorF / vs_h), so the Schur block-diagonal structure is
// untouched. Two properties matter:
//   * ML DEPTH NEVER ENTERS — only the normal direction (the tilt ratio). The residual's zero set is
//     scale-invariant, so the Metric3D 1.7x/2.94x bias cannot be imported (unlike Direct.P2/VS,
//     which pulled idepth toward the biased ML value and died on the 0.55 scale floor).
//   * The weight is PROPORTIONAL TO THE POINT'S OWN PHOTOMETRIC HESSIAN (Hdd_accAF+Hdd_accLF), so
//     the DN/photometric stiffness ratio is uniform across depths by construction. This is the
//     direct antidote to the documented Phase-2 failure mode (constant-weight priors made far-point
//     Hessians 833x stiffer than photometric). Points with no photometric conditioning get no DN
//     prior at all.
// Default OFF. CLI: --ml-dn-weight (0 disables) / --ml-dn-radius / --ml-dn-min-neighbors
// Sprint 8 (CD-H6) gauge probe: apply a FIXED world-frame rotation of N degrees about the camera
// x-axis at initialization, in place of the card's RANSAC floor normal. HSLAM's BA has a gauge
// freedom (the frame-0 prior in FrameHessian::getPrior() constrains the frame's state INCREMENT,
// not an absolute pose), so a one-time world rotation should propagate rigidly and leave both
// pre-registered metrics — ATE under evo's Umeyama alignment, and RPE over relative poses —
// mathematically unchanged. This probe tests that equivariance directly and decides whether the
// full Sprint 8 (floor RANSAC) can be measurable at all. 0 = disabled.
// CLI: --ml-gravity-test-rot
extern float setting_gravityAlignTestRotDeg;

// Phase 0 / plan red-team — MASTER OFF SWITCH for the entire normal channel.
// Until now no configuration could turn the normal head off: `normal_confidence` (kappa) is an output
// of the NORMAL head, and processUncertaintyToConfidence consumes it in BOTH the AngMF path and the
// legacy-sigmoid path. So kappa was live in every arm ever run, including arms labelled "normals off"
// (--ml-foreshortening=false only disables |n.z|). Every ablation of the normal channel to date is
// therefore a partial ablation, and the question "does the normal channel add value, or is it the
// depth fix?" was structurally unanswerable.
// When true: the confidence map is forced to all-ones and the normal map is not extracted, so
// getMLNormalMap() returns nullptr and every downstream normal consumer no-ops. This is the A0/A4
// control arm. Default false.
// CLI: --ml-normal-channel  (pass =off to disable)
extern bool setting_mlNormalChannelOff;
extern bool setting_diagTraceStats;
enum MLIdepthPrior { ML_IDEPTH_PRIOR_BOX = 0, ML_IDEPTH_PRIOR_NONE = 1, ML_IDEPTH_PRIOR_RELATIVE = 2 };
extern int   setting_mlIdepthPrior;           // Sprint 13: box (shipped) | none (true P1 ablation) | relative
extern float setting_mlIdepthRelQ;            // Sprint 13: log-depth half-width for the relative arm
extern bool setting_mlPriorCentredTrace;      // Sprint 12: centre the truncated epipolar window on rho_ML           // [TRACE_STATS]/[ACT_STATS] diagnostics; no behavioural effect

// Sprint 11 (INTEGRATION_DAMAGE_AUDIT D0/D1/D2) — Metric3D-v2 input-geometry and depth-scale
// correctness. Three confirmed integration defects, each independently gateable, all default OFF so
// the off path reproduces every recorded baseline exactly.
//   F0  setting_mlMetric3dRefGeometry — production runs Metric3D at 518x518, which is
//       Depth-Anything-V2's geometry (main.cpp's own help text says "for metric3d/dav2"). The
//       Metric3D-v2 ViT recipe is 616x1064. Only overrides for model_type=METRIC3D_V2.
//   F1  setting_mlCanonicalScale — Metric3D-v2 emits CANONICAL-camera depth (focal 1000 px) and the
//       ONNX graph has no intrinsics input, so D*fx_eff*s/1000 must be applied externally. It never
//       was. Offline against dense GT: at 616x1064 this takes ICL's depth bias 1.645x -> 1.016x.
//       MUST be combined with F0 — at 518x518 the same factor over-corrects to 0.845x.
//   F2  setting_mlIsotropicInput — HSLAM's KITTI calib yields a rectified fx=368.88, fy=703.52
//       (1.9x non-square pixels). Simulating that squeeze on ICL degrades predicted-normal accuracy
//       3.09deg -> 12.50deg and nearly doubles depth error.
// CLI: --ml-input-geometry={legacy,metric3d} / --ml-canonical-scale / --ml-isotropic-input
extern bool setting_mlMetric3dRefGeometry;
extern bool setting_mlCanonicalScale;
extern bool setting_mlIsotropicInput;

extern bool setting_useDepthNormalBA;
extern float setting_dnWeight;          // lambda: DN energy as a fraction of photometric stiffness
extern float setting_dnNeighborRadius;  // px, lvl-0; neighbourhood for the local-plane prediction
extern int setting_dnMinNeighbors;      // below this many valid neighbours, no DN prior for a point
extern float setting_dnMaxRatio;        // reject neighbour predictions outside [1/r, r] x rho_j
extern float setting_dnMinCosRay;       // reject neighbours whose ray is edge-on to the plane

extern bool debugSaveImages;


extern int sparsityFactor;
extern bool goStepByStep;
extern bool plotStereoImages;
extern bool multiThreading;

extern float freeDebugParam1;
extern float freeDebugParam2;
extern float freeDebugParam3;
extern float freeDebugParam4;
extern float freeDebugParam5;


void handleKey(char k);
void set_frame_sz(int size_x, int size_y);




extern int staticPattern[10][40][2];
extern int staticPatternNum[10];
extern int staticPatternPadding[10];




//#define PATTERNNUM staticPatternNum[setting_pattern]
//#define PATTERNP staticPattern[setting_pattern]
//#define PATTERNPADDING staticPatternPadding[setting_pattern]

//
#define PATTERNNUM 8
#define PATTERNP staticPattern[8]
#define PATTERNPADDING 2













}
