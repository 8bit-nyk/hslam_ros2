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
class MLDepthService;
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
	void TrackMonocularWithML(const cv::Mat& rgb_color, const cv::Mat& rgb_img, double timestamp);
	
	// Core ML-enhanced SLAM tracking method - uses ML depth for enhanced tracking
	void TrackFrameWithMLDepth(const cv::Mat& rgb_img, const cv::Mat& ml_depth, double timestamp);

	// Phase 2.5: ML service persistence through SLAM resets
	void restartSLAMSystem();

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

	void printResult(std::string file, bool printSim = false);
	void printPC(std::string file);

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

	// ML Depth integration support (Phase 2)
	std::unique_ptr<ML::MLDepthService> ml_depth_service_;
	cv::Mat current_ml_depth_image_;
	cv::Mat last_bgr_frame_;  // Store last BGR frame for keyframe ML submissions
	mutable std::mutex ml_depth_mutex_;
	mutable std::mutex rgbd_depth_mutex_;  // NEW: Protect RGB-D depth access
	bool ml_depth_enabled_ = false;
	
	// ML performance tracking
	struct MLMetrics {
		size_t total_frames = 0;
		size_t frames_with_ml_depth = 0;
		float ml_depth_utilization = 0.0f;
		float avg_ml_inference_time_ms = 0.0f;
		size_t frames_skipped = 0;  // Added for strategy tracking
	};
	MLMetrics ml_metrics_;
	
	// Phase 2.5: ML Service Persistence State
	bool ml_service_persistent_;
	bool slam_restart_in_progress_;
	std::chrono::steady_clock::time_point slam_last_restart_;
	int slam_restart_count_;
	
	// ML Depth Service Management (Phase 2 additions)
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
		float min_depth = 0.1f;
		float max_depth = 10.0f;
		bool benchmark_enabled = false;
	};
	
	/**
	 * @brief Initialize ML depth service with asynchronous processing
	 * @param config ML inference configuration
	 * @param strategy Initial inference strategy
	 * @param snapshot_interval Frames between inference in snapshot mode
	 * @return Initialization success status
	 */
	bool initializeMLDepthService(const MLConfig& config,
	                              const std::string& strategy_name = "every_frame",
	                              int snapshot_interval = 5);
	
	/**
	 * @brief Stop and cleanup ML depth service
	 */
	void shutdownMLDepthService();
	
	/**
	 * @brief Wait for ML warm-up completion with timeout
	 * @param timeout_ms Maximum wait time in milliseconds
	 * @return True if warm-up completed, false if timeout
	 */
	bool waitForMLWarmup(int timeout_ms = 5000);
	
	/**
	 * @brief Change ML inference strategy during runtime
	 * @param strategy_name Strategy name ("every_frame", "keyframe_only", "snapshot_mode")
	 * @param snapshot_interval Interval for snapshot mode
	 * @return Success status
	 */
	bool setMLInferenceStrategy(const std::string& strategy_name, int snapshot_interval = 5);
	
	/**
	 * @brief Get current ML depth service performance statistics
	 * @return Performance statistics structure
	 */
	MLMetrics getMLMetrics() const;
	
	// Public utility methods
	void setGammaFunction(float* BInv);
	void setOriginalCalib(const VecXf &originalCalib, int originalW, int originalH);
	
	// Keyframe statistics methods
	float getKeyframeRatio() const;
	size_t getKeyframeCount() const;
	size_t getTotalFrameCount() const;
	void printKeyframeStats() const;
	
private:
	// Helper methods for ML integration
	bool shouldCreateKeyframe() const;  // Forward declare existing keyframe logic

	boost::mutex mapMutex;

	CalibHessian Hcalib;
	std::shared_ptr<Matcher> matcher;
	std::shared_ptr<Map> globalMap;
	std::vector<FrameHessian*> frameHessians;	// ONLY changed in marginalizeFrame and addFrame.
	EnergyFunctional* ef;
	std::vector<FrameShell*> allKeyFramesHistory;
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
	void makeNewTracesWithMLDepth(FrameHessian* newFrame, const cv::Mat& ml_depth);
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

