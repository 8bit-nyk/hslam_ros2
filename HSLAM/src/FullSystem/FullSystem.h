#pragma once
#define MAX_ACTIVE_FRAMES 100

#include <deque>
#include "util/NumType.h"
#include "util/globalCalib.h"
#include "vector"
 
#include <iostream>
#include <fstream>
#include <unordered_map> 
#include "util/NumType.h"
#include "FullSystem/Residuals.h"
#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include "util/IndexThreadReduce.h"
#include "OptimizationBackend/EnergyFunctional.h"
#include "FullSystem/PixelSelector2.h"

#include <math.h>
#include <chrono>
#include <opencv2/opencv.hpp>


namespace HSLAM
{
namespace IOWrap
{
class Output3DWrapper;
}

namespace ML
{
class MLInference;
class MLDepthProcessor;
}

class PixelSelector;
class PCSyntheticPoint;
class CoarseTracker;
struct FrameHessian;
struct PointHessian;
class CoarseInitializer;
struct ImmaturePointTemporaryResidual;
class ImageAndExposure;
class CoarseDistanceMap;

class FeatureDetector;

class EnergyFunctional;

class Map;
class Matcher;
class LoopCloser;

template<typename T> inline void deleteOut(std::vector<T*> &v, const int i)
{
	delete v[i];
	v[i] = v.back();
	v.pop_back();
}
template<typename T> inline void deleteOutPt(std::vector<T*> &v, const T* i)
{
	delete i;

	for(unsigned int k=0;k<v.size();k++)
		if(v[k] == i)
		{
			v[k] = v.back();
			v.pop_back();
		}
}
template<typename T> inline void deleteOutOrder(std::vector<T*> &v, const int i)
{
	delete v[i];
	for(unsigned int k=i+1; k<v.size();k++)
		v[k-1] = v[k];
	v.pop_back();
}
template<typename T> inline void deleteOutOrder(std::vector<T*> &v, const T* element)
{
	int i=-1;
	for(unsigned int k=0; k<v.size();k++)
	{
		if(v[k] == element)
		{
			i=k;
			break;
		}
	}
	assert(i!=-1);

	for(unsigned int k=i+1; k<v.size();k++)
		v[k-1] = v[k];
	v.pop_back();

	delete element;
	element = nullptr;
}

inline bool eigenTestNan(const MatXX &m, std::string msg)
{
	bool foundNan = false;
	for(int y=0;y<m.rows();y++)
		for(int x=0;x<m.cols();x++)
		{
			if(!std::isfinite((double)m(y,x))) foundNan = true;
		}

	if(foundNan)
	{
		printf("NAN in %s:\n",msg.c_str());
		std::cout << m << "\n\n";
	}


	return foundNan;
}

inline Eigen::Vector3d convert_uv_xyz(float u, float v, float idepth, float fxi, float fyi, float cxi, float cyi, SE3 camToWorld)
{
	float x = (u * fxi + cxi) / idepth;
	float y = (v * fyi + cyi) / idepth;
	float z = (1 + 2 * fxi) / idepth;

	Eigen::Vector4d camPoint(x, y, z, 1.f);
	return camToWorld.matrix3x4() * camPoint;
}

struct PC_output
{
	float x=-1;
	float y=-1;
	float z=-1;

	unsigned char r=0;
	unsigned char g=0;
	unsigned char b=0;
};

class FullSystem
{
	friend class LoopCloser;  // Allow LoopCloser access to private members
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	FullSystem();
	virtual ~FullSystem();

	// adds a new frame, and creates point & residual structs.
	void addActiveFrame(ImageAndExposure* image, int id);

	// RGB-D tracking method - for now, calls TrackMonocular
	void TrackRGBD(const cv::Mat& rgb_color, const cv::Mat& rgb_img, const cv::Mat& depth_img, const double timestamp);

	// ML Depth tracking method - enhanced monocular tracking with ML depth
	void TrackMonocularWithML(const cv::Mat& rgb_color, ImageAndExposure* original_img);
	

	// Phase 2.5: ML service persistence through SLAM resets
	void restartSLAMSystem();
	
	// ML Reference Frame Management (CoarseTracker-style thread safety)
	void setMLReference(int frame_id, const cv::Mat& depth, float confidence, double inference_time);
	bool hasMLReference() const;
	cv::Mat getMLReferenceDepth() const;
	int getMLReferenceFrameId() const;
	
	// Performance monitoring
	int getKeyframeCount() const { return static_cast<int>(frameHessians.size()); }

	// Run-end performance summary — call after blockUntilMappingIsFinished()
	// avg_ml_inference_ms: mean ML forward-pass time. pipeline_wall_ms / pipeline_frames: end-to-end
	// wall clock and frame count for the WHOLE pipeline (tracking + mapping/BA + ML + loop closure),
	// measured in main.cpp after the mapping backlog drains. Pass <=0 to omit the pipeline figures.
	void printPerfSummary(double avg_ml_inference_ms = -1.0,
	                      double pipeline_wall_ms = -1.0, int pipeline_frames = -1);

	// Perf accumulators (updated per-frame in TrackMonocularWithML)
	struct timeval perf_last_frame_time_;
	bool           perf_timing_initialized_ = false;
	double         perf_total_tracking_ms_  = 0.0;
	int            perf_tracking_frame_count_ = 0;
	
	// Keyframe history for performance metrics
	std::vector<FrameShell*> allKeyFramesHistory;

	// =================== COARSE TRACKER DEPTH INTEGRATION ===================
	/**
	 * @brief Update CoarseTracker instances with current depth information
	 * 
	 * Synchronizes external depth information with both coarseTracker and 
	 * coarseTracker_forNewKF instances to ensure consistent depth integration.
	 */
	void updateCoarseTrackerDepth();
	
	/**
	 * @brief Synchronize depth with tracking operations
	 * 
	 * Ensures proper timing and coordination between depth updates and 
	 * tracking operations to maintain system consistency.
	 */
	void synchronizeDepthWithTracking();

	// marginalizes a frame. drops / marginalizes points & residuals.
	void marginalizeFrame(FrameHessian* frame);
	void blockUntilMappingIsFinished();

	float optimize(int mnumOptIts);

	// Sprint 10 (E3 / Phase B.4): recompute the per-point depth-normal surface-consistency prior
	// (EFPoint::dn_h / dn_target) for every active point. Called once per BA round from optimize().
	// Returns the number of points that received a prior; fills the [ENERGY_DN] diagnostic counters.
	int computeDepthNormalPriors();
	// Current DN energy, sum over active points of dn_h * (idepth - dn_target)^2.
	double calcDepthNormalEnergy() const;
	// [ENERGY_DN] counters, filled by computeDepthNormalPriors()
	int dn_last_applied_ = 0, dn_last_candidates_ = 0, dn_last_frames_ = 0, dn_last_toofew_ = 0;
	double dn_last_mean_rel_res_ = 0.0;
	int dn_collapse_iters_ = 0, dn_total_iters_ = 0;   // Phase-2-collapse kill-criterion tracking

	void printResult(std::string file, bool printSim = false);
	void printPC(std::string file);
	void printMapPly(std::string file);  // [MAP_EXPORT] Sprint 0d: ASCII PLY of marginalized PointHessians

	void debugPlot(std::string name);

	void printFrameLifetimes();
	// contains pointers to active frames

	void setVocab(DBoW3::Vocabulary* _Vocabpnt);

    std::vector<IOWrap::Output3DWrapper*> outputWrapper;

	bool isLost;
	bool initFailed;
	bool initialized;
	bool linearizeOperation;

	// RGB-D depth integration support
	cv::Mat currentDepthImage;     // RGB-D sensor depth for direct component
	cv::Mat currentMLDepthImage;   // Dedicated ML depth storage (NEW)

	// ML Depth integration support
	std::unique_ptr<ML::MLDepthProcessor> ml_processor_;
	cv::Mat current_ml_depth_image_;
	cv::Mat last_bgr_frame_;  // Store last BGR frame for ML processing
	mutable boost::mutex ml_depth_mutex_;
	mutable boost::mutex rgbd_depth_mutex_;  // Protect RGB-D depth access
	bool ml_depth_enabled_ = false;
	
	// ML Reference Frame (CoarseTracker-style pattern for thread safety)
	cv::Mat ml_reference_depth_;         // Copy of ML depth map
	int ml_reference_frame_id_ = -1;     // Frame ID for reference
	float ml_reference_confidence_ = 0;  // Confidence score
	double ml_reference_time_ = 0;       // Inference time
	int ml_frame_counter_ = 0;           // Frame counter for ML processing
	mutable boost::mutex ml_reference_mutex_; // Thread safety
	
	// GPU Warmup Result Caching (optimization to avoid duplicate processing)
	cv::Mat warmup_depth_map_;           // Stored depth map from GPU warmup
	cv::Mat warmup_normal_map_;          // Sprint 5 (CD-H7): stored normal map from GPU warmup (for optReg)
	float warmup_mean_depth_ = -1.0f;    // Mean depth from warmup inference
	float warmup_confidence_ = 0.0f;     // Confidence score from warmup
	bool warmup_results_available_ = false; // Flag indicating stored results are available
	
	// ML processing synchronization (temporary debugging solution)
	mutable boost::mutex ml_processing_mutex_;
	bool ml_processing_active_ = false;
	boost::condition_variable ml_processing_done_;
	
	// ML performance tracking
	struct MLMetrics {
		size_t ml_keyframes_attempted = 0;
		size_t ml_keyframes_successful = 0;
		float ml_depth_utilization = 0.0f;
		float avg_ml_inference_time_ms = 0.0f;
		size_t frames_skipped = 0;  // Added for strategy tracking
	};
	MLMetrics ml_metrics_;
	
	// Ablation study tracking counters
	size_t keyframe_counter_ = 0;        // Total keyframes created
	size_t ml_inference_counter_ = 0;    // ML inferences performed
	size_t last_ml_keyframe_ = 0;        // Last keyframe that had ML inference
	
	// Initialization performance tracking
	std::chrono::high_resolution_clock::time_point init_start_time_;
	double ml_init_processing_time_ms_ = 0.0;
	double total_init_time_ms_ = 0.0;
	bool using_metric_scale_ = false;
	float init_scale_factor_ = 1.0f;
	int init_points_count_ = 0;
	
	// Scale alignment system for ML depth integration
	float ml_to_slam_scale_factor_ = 1.0f;  // Conversion factor from ML (metric) to SLAM scale
	bool scale_aligned_ = false;            // Whether scales have been aligned
	float reference_slam_depth_ = -1.0f;    // Reference SLAM depth for alignment
	float reference_ml_depth_ = -1.0f;      // Corresponding ML depth for alignment
	float initial_ml_to_slam_scale_factor_ = 1.0f;  // Store initial factor for protection
	
	// RGB storage for first frame ML processing
	cv::Mat first_frame_rgb_;
	
	// ML Depth Processor Management
public:
	// Simple ML configuration struct to avoid forward declaration issues
	struct MLConfig {
		std::string model_path;
		bool enable_gpu = false;
		int num_threads = 4;
		
		// GPU-specific parameters
		bool enable_fp16 = false;          // FP16 optimization for GPU
		int gpu_device_id = 0;             // GPU device selection
		size_t gpu_memory_limit_mb = 2048; // GPU memory limit in MB
		
		int input_width = 518;
		int input_height = 518;
		bool benchmark_enabled = false;

		// Ablation study parameters
		std::string inference_strategy = "keyframe_only";  // "keyframe_only" or "snapshot_mode"
		int snapshot_rate = 1;  // ML inference every N keyframes (1 = every keyframe)

		// Vector A: model-type selection (default = metric3d). Other options:
		//   "depth_anything_v2" -> ML::MLInference::DEPTH_ANYTHING (relative inverse depth)
		//   "midas"             -> ML::MLInference::MIDAS_V3 (placeholder, not implemented)
		std::string model_type = "metric3d";

		// Multiplier applied to relative-depth model output (DA V2 only). 1.0 = pure relative.
		float output_scale = 1.0f;
	};
	
	// ML configuration storage  
	MLConfig ml_config_;
	
	                              
	/**
	 * @brief Initialize simplified ML depth processor for synchronous processing
	 * @param config ML inference configuration
	 * @return Initialization success status
	 */
	bool initializeMLDepthProcessor(const MLConfig& config);
	
	/**
	 * @brief Perform ML processor warmup to prevent blocking during frame processing
	 * @param warmup_image RGB image for warmup inference (result discarded)
	 * @return true if warmup successful, false otherwise
	 */
	bool performMLWarmup(const cv::Mat& warmup_image);
	
	
	/**
	 * @brief Get current ML depth processor performance statistics
	 * @return Performance statistics structure
	 */
	MLMetrics getMLMetrics() const;
	
	// Public utility methods
	void setGammaFunction(float* BInv);
	void setOriginalCalib(const VecXf &originalCalib, int originalW, int originalH);
	
	// Keyframe statistics methods
	float getKeyframeRatio() const;
	size_t getTotalFrameCount() const;
	void printKeyframeStats() const;
	void printInitializationPerformance() const;
	void printAblationStatistics() const;
	
	// Scale drift diagnostics: pure instrumentation, no trajectory impact
	void monitorScaleDrift(FrameHessian* newKF, const cv::Mat& mlDepth);
	float scale_ema_ = 1.0f;
	int scale_monitor_count_ = 0;
	// ML probation (GATED — see monitorScaleDrift). Kept for potential indirect pipeline reuse.
	std::vector<float> ml_fallback_ratios_;
	bool ml_fallback_triggered_ = false;

	// Per-scene ML weight calibration (Step 3): compute once during first N keyframes
	struct MLWeightCalibration {
		std::vector<double> photo_energy_samples;
		std::vector<double> ml_energy_samples;
		float calibrated_weight = -1.0f;
		bool is_calibrated = false;
		int kf_count = 0;
		static constexpr int WARMUP_KEYFRAMES = 5;
		static constexpr int CALIBRATION_WINDOW = 10;
		static constexpr float MIN_PHOTO_ENERGY = 10.0f;
		static constexpr float TARGET_ML_RATIO = 0.20f;
		static constexpr float MAX_WEIGHT_MULTIPLIER = 50.0f;
		static constexpr float MIN_WEIGHT = 0.1f;
	};
	MLWeightCalibration ml_weight_calib_;

private:
	// Helper methods for ML integration
	bool shouldCreateKeyframe() const;  // Forward declare existing keyframe logic

	boost::mutex mapMutex;

	CalibHessian Hcalib;
	std::shared_ptr<Matcher> matcher;
	std::shared_ptr<Map> globalMap;
	std::vector<FrameHessian*> frameHessians;	// ONLY changed in marginalizeFrame and addFrame.
	EnergyFunctional* ef;
	std::unordered_map<unsigned long, PC_output> allMargPointsHistory;
	mutable boost::mutex trackMutex;
	void BAatExit();


private:

	// opt single point
	int optimizePoint(PointHessian* point, int minObs, bool flagOOB);
	PointHessian* optimizeImmaturePoint(ImmaturePoint* point, int minObs, ImmaturePointTemporaryResidual* residuals);

	double linAllPointSinle(PointHessian* point, float outlierTHSlack, bool plot);

	// mainPipelineFunctions
	Vec5 trackNewCoarse(FrameHessian *fh, bool writePose = false);
	void traceNewCoarse(FrameHessian* fh);
	void activatePoints();
	void activatePointsMT();
	void activatePointsOldFirst();
	void flagPointsForRemoval();
	void makeNewTraces(FrameHessian* newFrame, float* gtDepth);
	void makeNewTracesWithMLDepth(FrameHessian* newFrame, const cv::Mat& ml_depth);  // PHASE 2: Gets confidence from frame
	
	// Scale alignment functions for ML depth integration
	void computeScaleAlignment();
	void alignScalesImmediately(const cv::Mat& ml_depth);  // NEW: Immediate alignment on first ML keyframe
	void applyMetricScaleConversion();  // FIXED: Post-initialization metric conversion
	float convertMLDepthToSLAMScale(float ml_depth_meters);
	bool isMLDepthValid(float ml_depth, float expected_range = 50.0f);
	#ifdef ENABLE_DEPTH_DEBUG
	void debugMLDepthComparison(FrameHessian* newFrame, const cv::Mat& ml_depth);  // DEPTH_DEBUG: TEMPORARY
	#endif
	void initializeFromInitializer(FrameHessian* newFrame);
	void flagFramesForMarginalization(FrameHessian* newFH);


	void removeOutliers();


	// set precalc values.
	void setPrecalcValues();


	// solce. eventually migrate to ef.
	void solveSystem(int iteration, double lambda);
	Vec3 linearizeAll(bool fixLinearization);
	bool doStepFromBackup(float stepfacC,float stepfacT,float stepfacR,float stepfacA,float stepfacD);
	void backupState(bool backupLastStep);
	void loadSateBackup();
	double calcLEnergy();
	double calcMEnergy();
	void linearizeAll_Reductor(bool fixLinearization, std::vector<PointFrameResidual*>* toRemove, int min, int max, Vec10* stats, int tid);
	void activatePointsMT_Reductor(std::vector<PointHessian*>* optimized,std::vector<ImmaturePoint*>* toOptimize,int min, int max, Vec10* stats, int tid);
	void applyRes_Reductor(bool copyJacobians, int min, int max, Vec10* stats, int tid);

	void printOptRes(const Vec3 &res, double resL, double resM, double resPrior, double LExact, float a, float b);

	void debugPlotTracking();

	std::vector<VecX> getNullspaces(
			std::vector<VecX> &nullspaces_pose,
			std::vector<VecX> &nullspaces_scale,
			std::vector<VecX> &nullspaces_affA,
			std::vector<VecX> &nullspaces_affB);

	void setNewFrameEnergyTH();


	void printLogLine();
	void printEvalLine();
	void printEigenValLine();
	std::ofstream* calibLog;
	std::ofstream* numsLog;
	std::ofstream* errorsLog;
	std::ofstream* eigenAllLog;
	std::ofstream* eigenPLog;
	std::ofstream* eigenALog;
	std::ofstream* DiagonalLog;
	std::ofstream* variancesLog;
	std::ofstream* nullspacesLog;

	std::ofstream* coarseTrackingLog;

	// statistics
	long int statistics_lastNumOptIts;
	long int statistics_numDroppedPoints;
	long int statistics_numActivatedPoints;
	long int statistics_numCreatedPoints;
	long int statistics_numForceDroppedResBwd;
	long int statistics_numForceDroppedResFwd;
	long int statistics_numMargResFwd;
	long int statistics_numMargResBwd;
	float statistics_lastFineTrackRMSE;







	// =================== changed by tracker-thread. protected by trackMutex ============
	std::vector<FrameShell*> allFrameHistory;
	CoarseInitializer* coarseInitializer;
	Vec5 lastCoarseRMSE;


	// ================== changed by mapper-thread. protected by mapMutex ===============
	

	IndexThreadReduce<Vec10> treadReduce;

	float* selectionMap;
	PixelSelector* pixelSelector;
	CoarseDistanceMap* coarseDistanceMap;


	std::vector<PointFrameResidual*> activeResiduals;
	float currentMinActDist;


	std::vector<float> allResVec;



	// mutex etc. for tracker exchange.
	boost::mutex coarseTrackerSwapMutex;			// if tracker sees that there is a new reference, tracker locks [coarseTrackerSwapMutex] and swaps the two.
	CoarseTracker* coarseTracker_forNewKF;			// set as as reference. protected by [coarseTrackerSwapMutex].
	CoarseTracker* coarseTracker;					// always used to track new frames. protected by [trackMutex].
	float minIdJetVisTracker, maxIdJetVisTracker;
	float minIdJetVisDebug, maxIdJetVisDebug;





	// mutex for camToWorl's in shells (these are always in a good configuration).
	boost::mutex shellPoseMutex;



/*
 * tracking always uses the newest KF as reference.
 *
 */

	void makeKeyFrame( FrameHessian* fh);
	void makeNonKeyFrame( FrameHessian* fh);
	void deliverTrackedFrame(FrameHessian* fh, bool needKF);
	void mappingLoop();

	// tracking / mapping synchronization. All protected by [trackMapSyncMutex].
	boost::mutex trackMapSyncMutex;
	boost::condition_variable trackedFrameSignal;
	boost::condition_variable mappedFrameSignal;
	std::deque<FrameHessian*> unmappedTrackedFrames;
	int needNewKFAfter;	// Otherwise, a new KF is *needed that has ID bigger than [needNewKFAfter]*.
	boost::thread mappingThread;
	bool runMapping;
	bool needToKetchupMapping;

	int lastRefStopID;



	void IndirectMapper(std::shared_ptr<Frame> frame);
	bool TrackLocalMap(std::shared_ptr<Frame> frame);

	void updateLocalKeyframes(std::shared_ptr<Frame> frame);
	void updateLocalKeyframesOld(std::shared_ptr<Frame> frame);
	void updateLocalPoints(std::shared_ptr<Frame> frame);
	void updateLocalPointsOld(std::shared_ptr<Frame> frame);
	void CheckReplacedInLastFrame();
	int SearchLocalPoints(std::shared_ptr<Frame> frame, int th = 1, float nnratio = 0.8);
	int updatePoseOptimizationData(std::shared_ptr<Frame> frame, int & nmatches, bool istrackingLastFrame = true);
	void SearchInNeighbors(std::shared_ptr<Frame> currKF);
	void KeyFrameCulling(std::shared_ptr<Frame> currKF);
	// SE3 cumulativeForm();


	std::shared_ptr<FeatureDetector> detector;

	size_t mnLastKeyFrameId;
	std::shared_ptr<Frame> mpLastKeyFrame;
	std::vector<std::shared_ptr<MapPoint>> mvpLocalMapPoints;
	std::vector<std::shared_ptr<Frame>> mvpLocalKeyFrames;
	std::shared_ptr<Frame> mpReferenceKF;
	std::shared_ptr<Frame> mLastFrame;

	SE3 Velocity;
	FixedQueue<SE3, 4> vVelocity;
	boost::mutex localMapMtx;

	int nIndmatches;
	bool isUsable;

	std::shared_ptr<LoopCloser> loopCloser;

	//sort localKeyframes while updating localkeyframes from best to worst:
	static bool cmpAscending(std::pair<std::shared_ptr<Frame>, int> &a, std::pair<std::shared_ptr<Frame>, int> &b)
	{
		return a.second < b.second;
	}
	static bool cmpDescending(std::pair<std::shared_ptr<Frame>, int> &a, std::pair<std::shared_ptr<Frame>, int> &b)
	{
		return a.second > b.second;
	}
	static std::vector<std::pair<std::shared_ptr<Frame>, int>> sortLocalKFs(std::map<std::shared_ptr<Frame>, int> &M, bool descendingOrder = false)
	{
		std::vector<std::pair<std::shared_ptr<Frame>, int>> A;
		for (auto &it : M)
		{
			A.push_back(it);
		}
		if(descendingOrder)
			std::sort(A.begin(), A.end(), cmpDescending);
		else
			std::sort(A.begin(), A.end(), cmpAscending);
		return A;
	}
};
}

