/**
* This file is part of DSO.
* 
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



#include "util/settings.h"
#include <boost/bind/bind.hpp>


namespace HSLAM
{

bool Pause = false;
bool LoopClosure = false;
int gridSize = 10;
int mnGridCols = 64;
int mnGridRows = 48;
float mnMinX = 0;
float mnMaxX = 640;
float mnMinY = 0;
float mnMaxY = 480;
float mfGridElementWidthInv = (float)mnGridCols/(float)(mnMaxX-mnMinX);
float mfGridElementHeightInv = (float)mnGridRows/(float)(mnMaxY-mnMinY);
//NA: change minKfIdDist from 50 to 10 and kfGap from 60 to 15
int minKfIdDist_LoopCandidate = 50; // min nmbre of keyframes between current and candidate to consider the candidate as potential loop closure.
int kfGap = 60; // minimum number of keyframes have passed since last loop closure performed.
int mnCovisibilityConsistencyTh = 1; //nbre of candidates connected to the loop candidate must also confirm for loop closure to take place

int EDGE_THRESHOLD = 19; //15?

int minThFAST = 7; //8
int minIndDist = 4; //9x9
int minDirDist = 1; //3x3
int indFeaturesToExtract = 1000;

bool normalizeInfoWithVariance = true; //turn this off to normalize the information matrix for Indirect data with the max (taken from the variance of the inverse depth estimates.)

int pyrLevelsUsed = PYR_LEVELS;

/* Parameters controlling when KF's are taken */
float setting_keyframesPerSecond = 0;   // if !=0, takes a fixed number of KF per second.
bool setting_realTimeMaxKF = false;   // if true, takes as many KF's as possible (will break the system if the camera stays stationary)
float setting_maxShiftWeightT= 0.04f * (mnMaxY + mnMaxX); //(640+480);
float setting_maxShiftWeightR= 0.0f * (mnMaxY + mnMaxX);//(640+480);
float setting_maxShiftWeightRT= 0.02f * (mnMaxY + mnMaxX);//(640+480);
float setting_kfGlobalWeight = 1;   // general weight on threshold, the larger the more KF's are taken (e.g., 2 = double the amount of KF's).
float setting_maxAffineWeight= 2;


/* initial hessian values to fix unobservable dimensions / priors on affine lighting parameters.
 */
float setting_idepthFixPrior = 50*50; //50*50;
float setting_idepthFixPriorMargFac = 600*600;
float setting_initialRotPrior = 1e11;
float setting_initialTransPrior = 1e10;
float setting_initialAffBPrior = 1e14;
float setting_initialAffAPrior = 1e14;
float setting_initialCalibHessian = 5e9;





/* some modes for solving the resulting linear system (e.g. orthogonalize wrt. unobservable dimensions) */
int setting_solverMode = SOLVER_FIX_LAMBDA | SOLVER_ORTHOGONALIZE_X_LATER;
double setting_solverModeDelta = 0.00001;
bool setting_forceAceptStep = false;  // CRITICAL FIX: Allow ML energy evaluation in bundle adjustment



/* some thresholds on when to activate / marginalize points */
float setting_minIdepthH_act = 100;
float setting_minIdepthH_marg = 50;



float setting_desiredImmatureDensity = 1500; // immature points per frame
float setting_desiredPointDensity = 2000; // aimed total points in the active window.
float setting_minPointsRemaining = 0.05;  // marg a frame if less than X% points remain.
float setting_maxLogAffFacInWindow = 0.7; // marg a frame if factor between intensities to current frame is larger than 1/X or X.


int   setting_minFrames = 5; // min frames in window.
int   setting_maxFrames = 7; // max frames in window.
int   setting_minFrameAge = 1;
int   setting_maxOptIterations=6; // max GN iterations.
int   setting_minOptIterations=1; // min GN iterations.
float setting_thOptIterations=1.2; // factor on break threshold for GN iteration (larger = break earlier)





/* Outlier Threshold on photometric energy */
float setting_outlierTH = 12*12;					// higher -> less strict
float setting_outlierTHSumComponent = 50*50; 		// higher -> less strong gradient-based reweighting .




int setting_pattern = 8;						// point pattern used. DISABLED.
float setting_margWeightFac = 0.5*0.5;          // factor on hessian when marginalizing, to account for inaccurate linearization points.


/* when to re-track a frame */
float setting_reTrackThreshold = 1.5; // (larger = re-track more often)



/* require some minimum number of residuals for a point to become valid */
int   setting_minGoodActiveResForMarg=3;
int   setting_minGoodResForMarg=4;






// 0 = nothing.
// 1 = apply inv. response.
// 2 = apply inv. response & remove V.
int setting_photometricCalibration = 2;
bool setting_useExposure = true;
float setting_affineOptModeA = 1e12; //-1: fix. >=0: optimize (with prior, if > 0).
float setting_affineOptModeB = 1e8; //-1: fix. >=0: optimize (with prior, if > 0).

int setting_gammaWeightsPixelSelect = 1; // 1 = use original intensity for pixel selection; 0 = use gamma-corrected intensity.




float setting_huberTH = 9; // Huber Threshold
float setting_huberTH_Ind = 5.991; // Huber Threshold




// parameters controlling adaptive energy threshold computation.
float setting_frameEnergyTHConstWeight = 0.5;
float setting_frameEnergyTHN = 0.7f;
float setting_frameEnergyTHFacMedian = 1.5;
float setting_overallEnergyTHWeight = 1;
float setting_coarseCutoffTH = 20;





// parameters controlling pixel selection
float setting_minGradHistCut = 0.5;
float setting_minGradHistAdd = 0.005;
float setting_gradDownweightPerLevel = 0.75;
bool  setting_selectDirectionDistribution = true;






/* settings controling initial immature point tracking */
float setting_maxPixSearch = 0.027; // max length of the ep. line segment searched during immature point tracking. relative to image resolution.
float setting_minTraceQuality = 3;
int setting_minTraceTestRadius = 2;
int setting_GNItsOnPointActivation = 3;
float setting_trace_stepsize = 1.0;				// stepsize for initial discrete search.
int setting_trace_GNIterations = 3;				// max # GN iterations
float setting_trace_GNThreshold = 0.1;				// GN stop after this stepsize.
float setting_trace_extraSlackOnTH = 1.2;			// for energy-based outlier check, be slightly more relaxed by this factor.
float setting_trace_slackInterval = 1.5;			// if pixel-interval is smaller than this, leave it be.
float setting_trace_minImprovementFactor = 2;		// if pixel-interval is smaller than this, leave it be.




// for benchmarking different undistortion settings
float benchmarkSetting_fxfyfac = 0;
int benchmarkSetting_width = 0;
int benchmarkSetting_height = 0;
float benchmark_varNoise = 0;
float benchmark_varBlurNoise = 0;
float benchmark_initializerSlackFactor = 1;
int benchmark_noiseGridsize = 3;


float freeDebugParam1 = 1;
float freeDebugParam2 = 1;
float freeDebugParam3 = 1;
float freeDebugParam4 = 1;
float freeDebugParam5 = 1;



bool disableReconfigure=false;
bool debugSaveImages = false;
bool multiThreading = true;
bool disableAllDisplay = false;
bool outputPC = false;
bool setting_onlyLogKFPoses = true;
bool setting_logStuff = true;

// ML Depth Integration Settings — Direct Pipeline (Photometric/DSO)
// Direct.P0 = metric init (always on), Direct.P1 = depth bounds, Direct.P2 = BA energy, Direct.P3 = tracker
bool setting_enableDirectP1Bounds = true;    // Direct.P1: Config D — ML depth bounds on ImmaturePoint tracing
float setting_mlDepthWeight = 2.0f;         // Base weight for ML depth constraints (calibrated per-scene at runtime)
float setting_mlGaussianScale = 0.01f;      // Near-flat Gaussian: ML constraints stay active. Validated on KITTI (-10% ATE)
bool setting_disableDirectP2BA = true;      // Direct.P2: PERMANENTLY DISABLED — proved inert (0.2-0.4% energy). See KEY_INSIGHTS.md §2.1
bool setting_disableDirectP3Tracker = true; // Direct.P3: Disabled — marginal benefit in ablation study
float setting_mlSelfGateTau = 0.01f;        // Direct.P2: Paper Eq. 5 self-gating width (gated — P2 disabled)
int setting_mlInferenceEveryN = 2;          // Paper Table V: ML every 2nd keyframe is optimal

// Direct Virtual Stereo ML Depth Settings (Step 4)
bool setting_disableDirectVS = true;     // Direct.VS: DISABLED BY DEFAULT (Apr 25, 2026). Futility test on TUM freiburg1_room (V0-V4 matrix, 5 weights × 3 reps) showed VS produces no measurable scale anchoring under production ML conditions — all weights leave Sim(3) scale in [0.554, 0.573], same band as VS-off. The 0.55 floor is set by Metric3D bias and VS at any tested weight cannot move it. Joining Phase 2/Phase 3 in "kept disabled until outdoor / sweet-spot evidence justifies re-enabling". See docs/gt_depth_validation/PHASE_C_CONSOLIDATED.md §9.
float setting_vsBaseline = 0.1f;         // Direct.VS: virtual baseline in meters
float setting_vsWeight = 1e-5f;          // Direct.VS: weight multiplier (intentionally inert baseline per fa6af55; Phase C found VS not load-bearing on indoor — proper sweet-spot evaluation pending)

// Indirect Pipeline ML Depth Settings
bool setting_disableIndirectMLDepth = false;     // Indirect: Global kill switch
float setting_indirectMLDepthWeight = 0.3f;      // Indirect.P1v2: INERT — consumer is dead BundleAdjustment() in Optimizer.cpp (heap corruption from g2o port; only caller in main.cpp:915 is commented out). See feedback_bundleadjustment_dead_code memory entry.
bool setting_disableIndirectP2LoopCloser = true; // Indirect.P2: DISABLED BY DEFAULT (May 5, 2026). Loop-closure ML/RANSAC scale-disagreement rejection (commit ea9c5e2) was implemented in LoopCloser.cpp::computeSim3 but the rejection branch was never experimentally validated — only fires when there's a loop closure with ≥30 inliers, observed ~1 event in TUM and 0 in KITTI/EuRoC across all eval runs. Kept in code for paper documentation; gated off pending real loop-closure exposure in evaluation.

// GT Depth Validation (Phase B) — research-only. Default = ML (unchanged production behavior).
// See docs/gt_depth_validation/PLAN.md
int setting_depthSource = DEPTH_SOURCE_ML;

// ML Initialization Settings
bool setting_useMLForInitialization = true;        // Enable ML-based metric scale initialization
float setting_mlInitConfidenceThreshold = 0.5f;   // Min confidence for ML scale usage
int setting_mlInitMinPoints = 20;                  // Min points for robust scale estimation
float setting_mlInitMinGoodRatio = 0.3f;           // Min fraction of well-triangulated points for reliable ML init
bool setting_mlReinitializationEnabled = true;    // Enable ML depth for re-initialization
// Phase 0 alpha-prior strength when ML-seeded (CoarseInitializer.cpp:120).
// Default 10000 (100*100) — uniformly improves ATE on TUM (-16%), KITTI (-11%),
// and EuRoC MH_01 (-23%) compared to the prior 2500 (50*50) used through April 2026.
// alphaW sweep 2026-04-27 showed 50000 best on TUM/KITTI but EuRoC regresses there;
// 10000 is the no-tradeoff cross-regime sweet spot. Mono uses 22500 (150*150).
// Configurable via --ml-alpha-w for further experimentation.
float setting_alphaWForMLInit = 100.0f * 100.0f;

// mlMeanDepth computation strategy (MLDepthProcessor.cpp).
// 0 = arithmetic mean over valid pixels (legacy + filtered for [0.1, 80]m, NaN-safe)
// 1 = median (robust to outliers)
// 2 = trimmed mean (5-95% percentile)
// Default 0 keeps legacy behavior; sweep planned 2026-04-27 to find best across regimes.
int setting_mlMeanDepthStrategy = 0;

// Indirect.Step2: ML depth-ratio filter in feature matchers (Matcher.cpp:363, 1103).
// Default true keeps legacy behavior. Set false to disable when ML is OOD-poisoned
// on a regime (April 27 finding: OOD ML feed via this filter is the dominant
// EuRoC regression mechanism). CLI: --ml-indirect-filter.
bool setting_indirectMatcherUseML = true;

// ML inference cadence (FullSystem.cpp::makeKeyFrame).
// 0 = every Nth keyframe (default, controlled by setting_mlInferenceEveryN)
// 1 = init_only (only first keyframe; tests "is ongoing ML inference actually useful?")
// 2 = disabled (no ML inference at all in makeKeyFrame; warmup at startup still runs)
// Init-only mode is the "lean production" candidate: if Phase 0 metric init is the
// dominant ATE driver (per April 27 phase ablation), ongoing keyframe inference is
// wasted compute. Frees GPU for downstream models (YOLO, DINO, SAM).
int setting_mlInferenceMode = 0;

// Phase 1 base idepth uncertainty for ML depth bounds (FullSystem.cpp:3068).
// April 27, 2026: 4-value sweep (0.05, 0.10, 0.20, 0.50) showed:
//   - KITTI 07 ATE drops -35% (5.31m -> 3.42m, matching mono baseline 3.39m) at 0.20
//   - TUM fr1_room slight regression (+12%, 0.344m -> 0.384m) — both still far below mono
//   - EuRoC MH_01 marginal -5% improvement
// Sharp non-monotonic optimum at 0.20 on KITTI: 0.05 too tight (cascades outlier rejection
// when no photo-cal), 0.50 too loose (looses ML depth's anchoring value). 0.20 is the
// cross-regime sweet spot. Configurable via --ml-idepth-uncertainty.
float setting_idepthUncertaintyForMLInit = 0.20f;

bool goStepByStep = false;


bool setting_render_displayCoarseTrackingFull=false;
bool setting_render_renderWindowFrames=true;
bool setting_render_plotTrackingFull = false;


bool setting_fullResetRequested = false;

bool setting_debugout_runquiet = false;

int sparsityFactor = 5;	// not actually a setting, only some legacy stuff for coarse initializer.


void handleKey(char k)
{
	char kkk = k;
	switch(kkk)
	{
	case 'd': case 'D':
		freeDebugParam5 = ((int)(freeDebugParam5+1))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	case 's': case 'S':
		freeDebugParam5 = ((int)(freeDebugParam5-1+10))%10;
		printf("new freeDebugParam5: %f!\n", freeDebugParam5);
		break;
	}

}

void set_frame_sz(int size_x, int size_y)
{
	mnGridCols = (int) size_x/gridSize;
	mnGridRows = (int) size_y/gridSize;
	mnMinX = 0;
	mnMaxX = size_x;
	mnMinY = 0;
	mnMaxY = size_y;
	mfGridElementWidthInv = (float)mnGridCols/(float)(mnMaxX-mnMinX);
	mfGridElementHeightInv = (float)mnGridRows/(float)(mnMaxY-mnMinY);;

	setting_maxShiftWeightT= 0.04f * (mnMaxY + mnMaxX);
	setting_maxShiftWeightR= 0.0f * (mnMaxY + mnMaxX);
	setting_maxShiftWeightRT= 0.02f * (mnMaxY + mnMaxX);
}




int staticPattern[10][40][2] = {
		{{0,0}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// .
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-1},	  {-1,0},	   {0,0},	    {1,0},	     {0,1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// +
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {1,1},	   {0,0},	    {-1,1},	     {1,-1}, 	  {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},	// x
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-1,-1},	  {-1,0},	   {-1,1},		{-1,0},		 {0,0},		  {0,1},	   {1,-1},		{1,0},		 {1,1},       {-100,-100},	// full-tight
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-100,-100},	// full-spread-9
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-13
		 {-2,2},      {2,-2},      {2,2},       {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-2,-2},     {-2,-1}, {-2,-0}, {-2,1}, {-2,2}, {-1,-2}, {-1,-1}, {-1,-0}, {-1,1}, {-1,2}, 										// full-25
		 {-0,-2},     {-0,-1}, {-0,-0}, {-0,1}, {-0,2}, {+1,-2}, {+1,-1}, {+1,-0}, {+1,1}, {+1,2},
		 {+2,-2}, 	  {+2,-1}, {+2,-0}, {+2,1}, {+2,2}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{1,1},		 {0,2},       {-2,-2},   // full-spread-21
		 {-2,2},      {2,-2},      {2,2},       {-3,-1},     {-3,1},      {3,-1}, 	   {3,1},       {1,-3},      {-1,-3},     {1,3},
		 {-1,3},      {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{0,-2},	  {-1,-1},	   {1,-1},		{-2,0},		 {0,0},		  {2,0},	   {-1,1},		{0,2},		 {-100,-100}, {-100,-100},	// 8 for SSE efficiency
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100},
		 {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}, {-100,-100}},

		{{-4,-4},     {-4,-2}, {-4,-0}, {-4,2}, {-4,4}, {-2,-4}, {-2,-2}, {-2,-0}, {-2,2}, {-2,4}, 										// full-45-SPREAD
		 {-0,-4},     {-0,-2}, {-0,-0}, {-0,2}, {-0,4}, {+2,-4}, {+2,-2}, {+2,-0}, {+2,2}, {+2,4},
		 {+4,-4}, 	  {+4,-2}, {+4,-0}, {+4,2}, {+4,4}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200},
		 {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}, {-200,-200}},
};

int staticPatternNum[10] = {
		1,
		5,
		5,
		9,
		9,
		13,
		25,
		21,
		8,
		25
};

int staticPatternPadding[10] = {
		1,
		1,
		1,
		1,
		2,
		2,
		2,
		3,
		2,
		4
};


}
