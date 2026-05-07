#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>

#include "IOWrapper/Output3DWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include <pangolin/pangolin.h>
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "util/DepthLogger.h"
#include "FullSystem/FullSystem.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <cxxopts.hpp>

// RGB-D pipeline includes
#include "tum_benchmark/tum_benchmark.hpp"
#include <opencv2/opencv.hpp>

// ML depth integration includes
#include "ML/MLInference.h"

// FPS logging includes
#include "util/FPSLogger.h"

// Custom structure for parsing associations.txt
struct RGBDAssociation {
    double rgb_timestamp;
    std::string rgb_file;
    double depth_timestamp;
    std::string depth_file;
};

// Stream operator for parsing associations.txt
inline std::istream& operator>>(std::istream& is, RGBDAssociation& assoc) {
    return is >> assoc.rgb_timestamp >> assoc.rgb_file >> assoc.depth_timestamp >> assoc.depth_file;
}

using namespace HSLAM;

/**
 * Converts HSLAM's undistorted float channels to RGB Mat for ML processing
 */
cv::Mat convertUndistortedToRGB(ImageAndExposure* img) {
    cv::Mat rgb_color = cv::Mat(img->h, img->w, CV_8UC3);
    
    // Optimized conversion using direct pointer access
    unsigned char* rgb_ptr = rgb_color.ptr<unsigned char>(0);
    int total_pixels = img->h * img->w;
    
    for (int i = 0; i < total_pixels; i++) {
        // BGR order for OpenCV Mat
        rgb_ptr[i * 3 + 0] = (unsigned char)std::min(255.0f, std::max(0.0f, img->b_image[i]));
        rgb_ptr[i * 3 + 1] = (unsigned char)std::min(255.0f, std::max(0.0f, img->g_image[i]));
        rgb_ptr[i * 3 + 2] = (unsigned char)std::min(255.0f, std::max(0.0f, img->r_image[i]));
    }
    
    return rgb_color;
}

void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	while(true) pause();
}


int main(int argc, char **argv)
{
	boost::thread exThread = boost::thread(exitThread); // hook crtl+C.

	cxxopts::Options options("HSLAM", "Direct Indirect Feature Fusion SLAM");

	std::string source = "";
	std::string calib = "";
	std::string vocabPath = "";
	std::string vignette = "";
	std::string gammaCalib = "";
	std::string associations = "";

	options.add_options()
        ("f,files", "Input images path - mandatory input", cxxopts::value(source))
		("c,calib", "Camera intrinsic callibration - mandatory input", cxxopts::value(calib))
		("v,vocab", "Path to Vocabulary file - required for loop closure", cxxopts::value(vocabPath))
		("n,vignette", "Path to photmetric calibration Vignette model", cxxopts::value(vignette))
		("g,gamma", "Path to photmetric calibration gamma response Model", cxxopts::value(gammaCalib))
		("a,associations", "Path to associations.txt file for RGB-D data", cxxopts::value(associations))
		("l,loopclosure", "Enable-Disable loop closure", cxxopts::value<bool>()->default_value("false"))
		("r,reverse", "Play a sequence in reverse", cxxopts::value<bool>()->default_value("false"))
		("preload", "Preload all images into memory", cxxopts::value<bool>()->default_value("false"))
		("usesampleoutput", "Replace pangolinViewer with another output wrapper", cxxopts::value<bool>()->default_value("false"))
		("nolog", "Disable logging optimization data", cxxopts::value<bool>()->default_value("false"))
		("nogui", "Disable GUI", cxxopts::value<bool>()->default_value("false"))
		("save", "Save debug images", cxxopts::value<bool>()->default_value("false"))
		("quiet", "Disable message printing", cxxopts::value<bool>()->default_value("true"))
		("nomt", "Turns off multiThreading", cxxopts::value<bool>()->default_value("false"))
		("s,startindex", "Image to start from", cxxopts::value<int>()->default_value("0"))
        ("e,endindex", "Last image to be processed", cxxopts::value<int>()->default_value("100000"))
		("m,mode", "System mode: 0: use precalibrated gamma and vignette -1: photometric mode without calibration - 2: photometric mode with perfect images", cxxopts::value<int>()->default_value("1"))
        ("preset", "Preset configuration}", cxxopts::value<int>()->default_value("0"))
		("speed", "Enforce playback Speed to real-time", cxxopts::value<float>()->default_value("0.0"))
        ("C,colour", "Read colour images", cxxopts::value<bool>()->default_value("false"))
		("S,use16bit", "Read 16 bit images", cxxopts::value<bool>()->default_value("false"))
		("P,outPC", "Output point cloud", cxxopts::value<bool>()->default_value("false"))
		("E,pauseEnd", "Pause at end", cxxopts::value<bool>()->default_value("false"))
		("ml-depth", "Enable ML depth estimation", cxxopts::value<bool>()->default_value("false"))
		("ml-model", "Path to ONNX ML depth model", cxxopts::value<std::string>()->default_value("models/metric3d-vit-small/onnx/model.onnx"))
		("ml-strategy", "ML inference strategy (every_frame|keyframe_only|snapshot_mode)", cxxopts::value<std::string>()->default_value("keyframe_only"))
		("ml-snapshot-interval", "Frames between ML inference in snapshot mode", cxxopts::value<int>()->default_value("5"))
		("ml-benchmark", "Enable ML performance benchmarking", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu", "Enable GPU acceleration for ML inference", cxxopts::value<bool>()->default_value("false"))
		("ml-fp16", "Enable FP16 optimization for GPU inference", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu-device", "GPU device ID for ML inference", cxxopts::value<int>()->default_value("0"))
		("ml-gpu-memory", "GPU memory limit in MB", cxxopts::value<size_t>()->default_value("2048"))
		("ml-model-type", "ML model type: metric3d|depth_anything_v2|midas (default metric3d)", cxxopts::value<std::string>()->default_value("metric3d"))
		("ml-input-width", "Model input width (default 518 for metric3d/dav2)", cxxopts::value<int>()->default_value("518"))
		("ml-input-height", "Model input height (default 518 for metric3d/dav2)", cxxopts::value<int>()->default_value("518"))
		("ml-output-scale", "Multiplier on relative-depth model output (DA V2 only). Default 1.0 = pure relative.", cxxopts::value<float>()->default_value("1.0"))
		("ml-alpha-w", "Phase 0 alpha-prior strength when ML-seeded (default 10000=cross-regime sweet spot; 22500 = mono). Tighter prior reduces iR scatter on jerky motion.", cxxopts::value<float>()->default_value("10000"))
		("ml-idepth-uncertainty", "Phase 1 base idepth uncertainty for ML bounds (default 0.20 = cross-regime optimum per Apr 27 sweep). Smaller = tighter; was 0.05 pre-Apr27.", cxxopts::value<float>()->default_value("0.20"))
		("ml-mean-strategy", "mlMeanDepth computation: 0=arith mean of valid (legacy), 1=median, 2=trimmed mean 5-95%. Default 0.", cxxopts::value<int>()->default_value("0"))
		("ml-inference-mode", "ML inference cadence: 0=every Nth KF (legacy), 1=init_only (lean), 2=disabled. Default 0.", cxxopts::value<int>()->default_value("0"))
		("ml-indirect-filter", "Indirect.Step2 ML depth-ratio filter in feature matchers. true=enabled (legacy), false=disabled. Default true.", cxxopts::value<bool>()->default_value("true"))
		("ml-init", "Enable ML depth for metric scale initialization", cxxopts::value<bool>()->default_value("true"))
		("depth-source", "Depth source: ml|gt|none (default ml). GT requires --associations and uses the same files as ML depth would be computed from.", cxxopts::value<std::string>()->default_value("ml"))
		// Phase toggles for the Phase C config matrix (Phase B/C research). Defaults match current production.
		("p0", "Direct.P0: ML-based metric scale initialization (default: on)", cxxopts::value<bool>()->default_value("true"))
		("p1", "Direct.P1: ML depth bounds in ImmaturePoint tracing (default: on)", cxxopts::value<bool>()->default_value("true"))
		("p2", "Direct.P2: ML energy term in photometric BA (default: off — memory says structurally dead)", cxxopts::value<bool>()->default_value("false"))
		("p3", "Direct.P3: ML depth fusion in CoarseTracker (default: off — marginal benefit)", cxxopts::value<bool>()->default_value("false"))
		("vs", "Direct.VS: virtual stereo in DSO sliding-window BA (default: OFF — futility-tested, no measurable contribution on TUM)", cxxopts::value<bool>()->default_value("false"))
		("vs-weight", "Direct.VS weight multiplier (default matches settings.cpp; typical sweep 1e-5..1e-2)", cxxopts::value<float>()->default_value("-1"))
		("depth-scale", "Divisor applied to 16-bit depth PNG values to get meters (TUM=5000; KITTI projected depth we write at 500 for max ~130m range).", cxxopts::value<float>()->default_value("5000.0"))
		("export-map-ply", "Export marginalized PointHessians to ASCII PLY at end of run (default false)", cxxopts::value<bool>()->default_value("false"))
		("map-ply-out", "Output path for PLY export (default: same directory as result.txt with .ply extension)", cxxopts::value<std::string>()->default_value(""))
		("use-normal-integration", "Sprint 1 master gate: enable downstream consumers of predicted_normal (default false — normals plumbed but not yet read)", cxxopts::value<bool>()->default_value("false"))
		("h,help", "Print usage")
    ;

	auto result = options.parse(argc, argv);

	if (result.count("help"))
    {
		std::cout << options.help() << std::endl;
		exit(0);
    }

	LoopClosure = result["loopclosure"].as<bool>();
	bool reverse = result["reverse"].as<bool>();
	bool preload = result["preload"].as<bool>();
	bool useSampleOutput = result["usesampleoutput"].as<bool>();
	setting_logStuff = !result["nolog"].as<bool>();
	disableAllDisplay = result["nogui"].as<bool>();
	outputPC = result["outPC"].as<bool>();
	debugSaveImages = result["save"].as<bool>();
	setting_debugout_runquiet = result["quiet"].as<bool>();
	multiThreading = !result["nomt"].as<bool>();

	int startIndex = result["startindex"].as<int>();
	int endIndex = result["endindex"].as<int>();
	int preset = result["preset"].as<int>();
	int mode = result["mode"].as<int>();
	float playbackSpeed = result["speed"].as<float>(); // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.

	bool use_colour = result["colour"].as<bool>();
	bool use_16bit = result["use16bit"].as<bool>(); 

	bool pauseEnd = result["pauseEnd"].as<bool>(); 

	// ML depth options (Phase 2 + GPU Integration)
	bool ml_depth_enabled = result["ml-depth"].as<bool>();
	std::string ml_model_path = result["ml-model"].as<std::string>();
	std::string ml_strategy = result["ml-strategy"].as<std::string>();
	int ml_snapshot_interval = result["ml-snapshot-interval"].as<int>();
	bool ml_benchmark_enabled = result["ml-benchmark"].as<bool>();
	std::string ml_model_type_str = result["ml-model-type"].as<std::string>();
	int ml_input_width = result["ml-input-width"].as<int>();
	int ml_input_height = result["ml-input-height"].as<int>();
	float ml_output_scale = result["ml-output-scale"].as<float>();
	setting_alphaWForMLInit = result["ml-alpha-w"].as<float>();
	setting_idepthUncertaintyForMLInit = result["ml-idepth-uncertainty"].as<float>();
	setting_mlMeanDepthStrategy = result["ml-mean-strategy"].as<int>();
	setting_mlInferenceMode = result["ml-inference-mode"].as<int>();
	setting_indirectMatcherUseML = result["ml-indirect-filter"].as<bool>();
	
	// Validate and normalize ML strategy parameters for ablation study
	if (ml_strategy != "keyframe_only" && ml_strategy != "snapshot_mode") {
		printf("WARNING: Invalid ml-strategy '%s', using 'keyframe_only'\n", ml_strategy.c_str());
		ml_strategy = "keyframe_only";
	}
	
	// Validate snapshot interval (repurpose as snapshot_rate for keyframes)
	if (ml_snapshot_interval < 1) {
		printf("WARNING: Invalid snapshot rate %d, using 1\n", ml_snapshot_interval);
		ml_snapshot_interval = 1;
	}
	
	// GPU acceleration options (Phase 3)
	bool ml_gpu_enabled = result["ml-gpu"].as<bool>();
	bool ml_fp16_enabled = result["ml-fp16"].as<bool>();
	int ml_gpu_device = result["ml-gpu-device"].as<int>();
	size_t ml_gpu_memory_mb = result["ml-gpu-memory"].as<size_t>();
	
	// ML initialization control (for A/B testing)
	bool ml_init_enabled = result["ml-init"].as<bool>();

	// Depth PNG scaling (for RGB-D associations loop; 5000 for TUM, ~500 for KITTI)
	float depth_scale = result["depth-scale"].as<float>();
	if (depth_scale <= 0.0f) { printf("ERROR: --depth-scale must be positive.\n"); return 0; }
	printf("[DEPTH_SCALE] divisor=%.1f (maps uint16 PNG -> meters)\n", depth_scale);

	// Map PLY export (Sprint 0d)
	setting_exportMapPly = result["export-map-ply"].as<bool>();
	setting_mapPlyOut = result["map-ply-out"].as<std::string>();
	// When PLY export is requested, enable point collection (allMargPointsHistory is gated on outputPC)
	if (setting_exportMapPly) outputPC = true;

	// Surface Normal Integration (Sprint 1) — master gate
	setting_useNormalIntegration = result["use-normal-integration"].as<bool>();
	printf("[PHASE_CONFIG] use-normal-integration=%s\n", setting_useNormalIntegration ? "on" : "off");

	// Depth source selection (Phase B — research-only validation switch)
	{
		std::string dsrc = result["depth-source"].as<std::string>();
		if (dsrc == "ml")        setting_depthSource = DEPTH_SOURCE_ML;
		else if (dsrc == "gt")   setting_depthSource = DEPTH_SOURCE_GT;
		else if (dsrc == "none") setting_depthSource = DEPTH_SOURCE_NONE;
		else {
			printf("ERROR: invalid --depth-source='%s' (expected ml|gt|none). Using 'ml'.\n", dsrc.c_str());
			setting_depthSource = DEPTH_SOURCE_ML;
		}
		const char* label = (setting_depthSource == DEPTH_SOURCE_ML) ? "ML (Metric3D, production)"
		                   : (setting_depthSource == DEPTH_SOURCE_GT) ? "GT (Kinect/LiDAR from associations.txt)"
		                                                               : "NONE (pure monocular)";
		printf("[DEPTH_SOURCE] %s\n", label);
		if (setting_depthSource == DEPTH_SOURCE_GT && associations.empty()) {
			printf("ERROR: --depth-source=gt requires --associations=<path/to/associations.txt>\n");
			return 0;
		}
	}

	// Phase toggles (Phase C matrix). Each flag overrides the settings.cpp default.
	// Convention: p0/p1/p2/p3/vs = true means enable; settings polarity is flipped per flag.
	setting_useMLForInitialization = result["p0"].as<bool>();
	setting_enableDirectP1Bounds   = result["p1"].as<bool>();
	setting_disableDirectP2BA      = !result["p2"].as<bool>();
	setting_disableDirectP3Tracker = !result["p3"].as<bool>();
	setting_disableDirectVS        = !result["vs"].as<bool>();
	{
		float vsw = result["vs-weight"].as<float>();
		if (vsw >= 0.0f) setting_vsWeight = vsw;  // negative sentinel = keep settings default
	}
	printf("[PHASE_CONFIG] P0=%s P1=%s P2=%s P3=%s VS=%s vsWeight=%.1e\n",
	       setting_useMLForInitialization ? "on":"off",
	       setting_enableDirectP1Bounds   ? "on":"off",
	       !setting_disableDirectP2BA     ? "on":"off",
	       !setting_disableDirectP3Tracker? "on":"off",
	       !setting_disableDirectVS       ? "on":"off",
	       setting_vsWeight);

	if(source.empty() || calib.empty()) { std::cout<< "Path to images or calibration not provided! cannot function without them. exit." << std::endl; return(0);}

	if (debugSaveImages)
	{
		if(42==system("rm -rf images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("mkdir images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("rm -rf images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
		if(42==system("mkdir images_out")) std::cout<<"system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n"<<std::endl;
	}

	switch (mode)
	{
	case 1:
		setting_photometricCalibration = 0;
		setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		break;
	case 2:
		setting_photometricCalibration = 0;
		setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
		setting_minGradHistAdd = 3;
		break;
	default:
		break;
	}
	
	DBoW3::Vocabulary* Vocabpnt;
	if(LoopClosure && !vocabPath.empty())
	{
		Vocabpnt = new DBoW3::Vocabulary();
		Vocabpnt->load(vocabPath.c_str());

		printf("Loading Vocabulary from %s!\n", vocabPath.c_str());
		if (Vocabpnt->empty() || Vocabpnt == NULL)
		{
			printf("Failed to load vocabulary! Exit\n");
			exit(1);
		}
	}
	else
	{
		std::cout << "no vocabulary path provided! disabling loop closure." << std::endl;
		LoopClosure = false; 
	}

	if(preset == 0 || preset == 1)
	{
		printf("OPTIMIZED settings:\n"
				"- %s real-time enforcing\n"
				"- %.0f active points (PHASE1: ML-enhanced SLAM optimization)\n"
				"- 5-7 active frames\n"
				"- 1-6 LM iteration each KF\n"
				"- original image resolution\n", 
				preset==0 ? "no " : "1x", setting_desiredPointDensity);

		playbackSpeed = (preset==0 ? 0 : 1);
		// preload = preset==1;
		
		// CRITICAL: Actually set the optimized values (Phase 1 optimization)
		// Balanced: maintain trajectory quality while improving performance
		setting_desiredImmatureDensity = 1500;  // 33% reduction from 1500
		setting_desiredPointDensity = 2000;     // 25% reduction from 2000
	}
	else if(preset == 2 || preset == 3)
	{
		printf("FAST settings:\n"
				"- %s real-time enforcing\n"
				"- 800 active points\n"
				"- 4-6 active frames\n"
				"- 1-4 LM iteration each KF\n"
				"- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

		playbackSpeed = (preset==2 ? 0 : 5);
		preload = preset==3;
		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations=4;
		setting_minOptIterations=1;

		benchmarkSetting_width = 424;
		benchmarkSetting_height = 320;

		setting_logStuff = false;
	}
	

	ImageFolderReader* reader = new ImageFolderReader(source, calib, gammaCalib, vignette, use_16bit, use_colour);
	reader->setGlobalCalibration();
	set_frame_sz(reader->get_undist_width(), reader->get_undist_height());

	if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}


	int lstart=startIndex;
	int lend = endIndex;
	int linc = 1;
	if(reverse)
	{
		printf("REVERSE!!!!");
		lstart=endIndex-1;
		if(lstart >= reader->getNumImages())
			lstart = reader->getNumImages()-1;
			
		lend = startIndex;
		linc = -1;
	}



	FullSystem* fullSystem = new FullSystem();
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed == 0);
	
	// Apply ML initialization setting from command line
	setting_useMLForInitialization = ml_init_enabled;

	if(LoopClosure)
	{
		fullSystem->setVocab(Vocabpnt);
		printf("Vocabulary Set\n");
	}

	// Initialize ML depth system (Phase 2) if enabled
	if(ml_depth_enabled) {
		printf("Initializing ML depth system (Phase 2)...\n");
		printf("  Model: %s\n", ml_model_path.c_str());
		printf("  Strategy: %s\n", ml_strategy.c_str());
		if(ml_strategy == "snapshot_mode") {
			printf("  Snapshot rate: Every %d keyframes\n", ml_snapshot_interval);
			printf("  Expected ML reduction: %.1f%%\n", 
			       100.0f * (ml_snapshot_interval - 1) / ml_snapshot_interval);
		}
		
		// Create ML configuration using FullSystem's MLConfig
		FullSystem::MLConfig ml_config;
		ml_config.model_path = ml_model_path;
		ml_config.enable_gpu = ml_gpu_enabled;
		ml_config.num_threads = 4;
		
		// GPU-specific configuration
		ml_config.enable_fp16 = ml_fp16_enabled;
		ml_config.gpu_device_id = ml_gpu_device;
		ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
		
		ml_config.input_width = ml_input_width;
		ml_config.input_height = ml_input_height;
		ml_config.model_type = ml_model_type_str;
		ml_config.output_scale = ml_output_scale;
		ml_config.benchmark_enabled = ml_benchmark_enabled;
		
		// Ablation study configuration
		ml_config.inference_strategy = ml_strategy;
		ml_config.snapshot_rate = ml_snapshot_interval;
		
		if(ml_gpu_enabled) {
			printf("  GPU Acceleration: Enabled (Device %d, Memory: %zu MB, FP16: %s)\n",
			       ml_gpu_device, ml_gpu_memory_mb, ml_fp16_enabled ? "Yes" : "No");
		} else {
			printf("  GPU Acceleration: Disabled (CPU mode)\n");
		}
		
		// Initialize MLDepthProcessor (synchronous processing)
		if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
			printf("ERROR: Failed to initialize ML depth processor\n");
			return -1;
		}
		printf("ML Depth Processor initialized successfully\n");
		
		// Perform ML warmup with real frame for effective GPU optimization.
		// Use the requested startIndex frame (not hardcoded frame 0) so --startindex N
		// genuinely skips degenerate-motion preludes; warmup-stored mean depth feeds
		// Phase 0 metric scale anchor.
		printf("GPU warmup: starting with frame index %d (one-time GPU initialization)...\n", startIndex);
		auto warmup_start = std::chrono::high_resolution_clock::now();

		try {
			// Load warmup frame from dataset (respects --startindex)
			ImageAndExposure* first_img = reader->getImage(startIndex);
			if (first_img && first_img->useColour && first_img->r_image != nullptr) {
				// Convert first real frame to RGB using the same pipeline
				cv::Mat first_rgb = convertUndistortedToRGB(first_img);
				
				if (!first_rgb.empty()) {
					printf("GPU warmup: processing first real frame (expected ~6000ms)...\n");
					
					// Process first real frame - this triggers full GPU kernel compilation
					bool warmup_success = fullSystem->performMLWarmup(first_rgb);
					
					auto warmup_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::high_resolution_clock::now() - warmup_start).count();
					
					if (warmup_success) {
						printf("GPU warmup complete in %ldms - subsequent ML inferences ~40ms\n", warmup_duration);
					} else {
						printf("ERROR: Real frame warmup failed - disabling ML depth\n");
						fullSystem->ml_depth_enabled_ = false;
						ml_depth_enabled = false;
					}
				} else {
					printf("ERROR: Could not convert first frame to RGB - disabling ML depth\n");
					fullSystem->ml_depth_enabled_ = false;
					ml_depth_enabled = false;
				}
				delete first_img; // Clean up
			} else {
				printf("ERROR: Could not load first frame - disabling ML depth\n");
				fullSystem->ml_depth_enabled_ = false;
				ml_depth_enabled = false;
			}
		} catch (const std::exception& e) {
			printf("ERROR: Exception during ML warmup: %s - disabling ML depth\n", e.what());
			fullSystem->ml_depth_enabled_ = false;
			ml_depth_enabled = false;
		}
		
		// ML processor is ready immediately (synchronous initialization)
		
		// Extract dataset name from source path for results directory
		std::string dataset_name = "unknown";
		size_t last_slash = source.find_last_of('/');
		if (last_slash != std::string::npos) {
			std::string path_part = source.substr(last_slash + 1);
			if (path_part.find("rgbd_dataset_") == 0) {
				dataset_name = path_part.substr(13); // Remove "rgbd_dataset_" prefix
			} else {
				dataset_name = path_part; // Use the directory name as dataset name
			}
		}

		// Benchmark is now integrated into the service automatically
		if (ml_benchmark_enabled) {
			printf("ML performance benchmarking enabled - metrics will be collected during operation\n");
		}

		// DEPTH_DEBUG: Initialize DepthLogger for ML depth debugging (TEMPORARY - for debugging only)
		#ifdef ENABLE_DEPTH_DEBUG
		printf("DEPTH_DEBUG: Initializing DepthLogger for ML depth debugging...\n");
		HSLAM::DepthLogger::initialize("results", dataset_name);
		HSLAM::DepthLogger::setDebugMode(true); // Enable detailed debug logging
		printf("DEPTH_DEBUG: DepthLogger initialized for dataset: %s\n", dataset_name.c_str());
		#endif
	} else {
		printf("ML depth system disabled\n");
		fullSystem->ml_depth_enabled_ = false;
	}

	IOWrap::PangolinDSOViewer* viewer = 0;
	if(!disableAllDisplay)
    {
        // Check for headless environment (no DISPLAY variable)
        const char* display_env = std::getenv("DISPLAY");
        if (display_env == nullptr || strlen(display_env) == 0) {
            printf("WARNING: No DISPLAY environment variable found. Running in headless mode (GUI disabled).\n");
            disableAllDisplay = true;
        } else {
            // Try to initialize GUI with error handling
            try {
                viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);  // Disable GUI thread (fix GLX error)
                fullSystem->outputWrapper.push_back(viewer);
                printf("GUI viewer initialized successfully\n");
            } catch (const std::exception& e) {
                printf("WARNING: Failed to initialize GUI viewer: %s\n", e.what());
                printf("Continuing in headless mode...\n");
                disableAllDisplay = true;
                viewer = nullptr;
            }
        }
    }



    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());




	// RGB-D pipeline: Use associations file if provided, otherwise fallback to monocular
    std::thread runthread([&]() {
        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset=0;
        int processedFrames = 0;  // Declare at function scope
        std::vector<int> idsToPlay;  // Declare at function scope for statistics

        // Initialize FPSLogger for performance monitoring
        std::string dataset_name = "unknown";
        // Extract dataset name from source path
        size_t last_slash = source.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string path_part = source.substr(last_slash + 1);
            if (path_part.find("rgbd_dataset_") == 0) {
                dataset_name = path_part.substr(13); // Remove "rgbd_dataset_" prefix
            } else {
                dataset_name = path_part; // Use the directory name as dataset name
            }
        }
        if(setting_logStuff) {
            HSLAM::FPSLogger::initialize("results", dataset_name);
            HSLAM::FPSLogger::setDebugMode(false);
        }
        printf("FPSLogger: Initialized for dataset: %s\n", dataset_name.c_str());

        if (!associations.empty()) {
            // RGB-D pipeline using associations.txt
            printf("Using RGB-D pipeline with associations file: %s\n", associations.c_str());
            
            // Initialize DepthLogger for RGB-D pipeline
            HSLAM::DepthLogger::initialize("results", dataset_name);
            HSLAM::DepthLogger::setDebugMode(false); // Disable debug mode by default
            printf("DepthLogger: Initialized for dataset: %s\n", dataset_name.c_str());
            
            // Load associations using tum_benchmark
            typedef tum_benchmark::FileReader<RGBDAssociation> FileReader;
            FileReader reader_assoc(associations);
            
            int frameCount = 0;
            
            for (auto it = reader_assoc.begin(); it != reader_assoc.end(); ++it, ++frameCount) {
                if (frameCount < startIndex) continue;
                if (frameCount >= endIndex) break;
                
                while (Pause) {
                    usleep(5000);
                }
                
                if(!fullSystem->initialized) {
                    gettimeofday(&tv_start, NULL);
                    started = clock();
                    sInitializerOffset = 0;
                }
                
                // Parse association entry
                auto& entry = *it;
                std::string rgb_path = source + "/" + entry.rgb_file;
                std::string depth_path = source + "/" + entry.depth_file;
                
                // Extract timestamp from filename if available, fallback to associations.txt
                double timestamp;
                
                // Try to extract timestamp from RGB filename (TUM RGB-D format: path/timestamp.png)
                std::string rgb_filename = entry.rgb_file;
                
                // Get just the filename part (remove directory path)
                size_t slash_pos = rgb_filename.find_last_of('/');
                if (slash_pos != std::string::npos) {
                    rgb_filename = rgb_filename.substr(slash_pos + 1);
                }
                
                // Extract timestamp from filename
                size_t dot_pos = rgb_filename.find_last_of('.');
                if (dot_pos != std::string::npos) {
                    std::string timestamp_str = rgb_filename.substr(0, dot_pos);
                    try {
                        timestamp = std::stod(timestamp_str);
                        if (!setting_debugout_runquiet) {
                            printf("Extracted timestamp from filename: %.6f\n", timestamp);
                        }
                    } catch (const std::exception& e) {
                        // Fallback to associations.txt timestamp
                        timestamp = entry.rgb_timestamp;
                        if (!setting_debugout_runquiet) {
                            printf("Using associations.txt timestamp: %.6f (filename extraction failed)\n", timestamp);
                        }
                    }
                } else {
                    // Fallback to associations.txt timestamp
                    timestamp = entry.rgb_timestamp;
                    if (!setting_debugout_runquiet) {
                        printf("Using associations.txt timestamp: %.6f (no extension found)\n", timestamp);
                    }
                }
                
                // Load RGB image as color for ML depth processing
                cv::Mat rgb_img_color = cv::imread(rgb_path, cv::IMREAD_COLOR);
                if (rgb_img_color.empty()) {
                    printf("ERROR: Could not load RGB image: %s\n", rgb_path.c_str());
                    continue;
                }
                
                // Create grayscale version for SLAM processing
                cv::Mat rgb_img_raw;
                cv::cvtColor(rgb_img_color, rgb_img_raw, cv::COLOR_BGR2GRAY);
                
                // Convert grayscale to float for SLAM
                cv::Mat rgb_img;
                rgb_img_raw.convertTo(rgb_img, CV_32FC1);
                
                // Convert RGB to proper format for ML
                cv::Mat rgb_for_ml;
                cv::cvtColor(rgb_img_color, rgb_for_ml, cv::COLOR_BGR2RGB);
                
                // Load depth image (16-bit PNG -> CV_32FC1, scaled by 1/5000.0)
                cv::Mat depth_img_raw = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
                if (depth_img_raw.empty()) {
                    printf("ERROR: Could not load depth image: %s\n", depth_path.c_str());
                    continue;
                }
                
                cv::Mat depth_img;
                // Depth-scale divisor: TUM uses 5000, KITTI velodyne-projected we write at ~500.
                // Configurable via --depth-scale CLI flag.
                depth_img_raw.convertTo(depth_img, CV_32FC1, 1.0/depth_scale);
                
                // Debug-only logging for frame processing details
                HSLAM::DepthLogger::logFrameProcessing(processedFrames, rgb_img.cols, rgb_img.rows,
                                                     depth_img.cols, depth_img.rows, timestamp);
                
                // Call TrackRGBD with RGB color for ML and grayscale for SLAM
                fullSystem->TrackRGBD(rgb_for_ml, rgb_img, depth_img, timestamp);
                
                processedFrames++;
                
                if(viewer!=0 && viewer->isDead) break;
                
                if(fullSystem->initFailed || setting_fullResetRequested) {
                    if(processedFrames < 250 || setting_fullResetRequested) {
                        printf("RESETTING!\n");
                        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
                        usleep(20000);
                        if(fullSystem) {
                            delete fullSystem;
                            fullSystem = nullptr;
                        }
                        
                        fullSystem = new FullSystem();
                        fullSystem->setGammaFunction(reader->getPhotometricGamma());
                        fullSystem->linearizeOperation = (playbackSpeed == 0);
                        
                        if(LoopClosure) {
                            fullSystem->setVocab(Vocabpnt);
                            printf("Vocabulary Set\n");
                        }
                        
                        // CRITICAL FIX: Reinitialize ML depth service after reset
                        if(ml_depth_enabled) {
                            printf("Reinitializing ML depth service after SLAM reset...\n");
                            
                            // Recreate ML config (same as original initialization)
                            FullSystem::MLConfig ml_config;
                            ml_config.model_path = ml_model_path;
                            ml_config.enable_gpu = ml_gpu_enabled;
                            ml_config.num_threads = 4;
                            ml_config.enable_fp16 = ml_fp16_enabled;
                            ml_config.gpu_device_id = ml_gpu_device;
                            ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
                            ml_config.input_width = ml_input_width;
                            ml_config.input_height = ml_input_height;
                            ml_config.model_type = ml_model_type_str;
                            ml_config.output_scale = ml_output_scale;
                            ml_config.benchmark_enabled = ml_benchmark_enabled;
                            
                            // Ablation study configuration
                            ml_config.inference_strategy = ml_strategy;
                            ml_config.snapshot_rate = ml_snapshot_interval;
                            
                            // Reinitialize MLDepthProcessor
                            if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
                                printf("WARNING: Failed to reinitialize ML depth processor after reset\n");
                            } else {
                                printf("ML depth processor reinitialized successfully\n");
                            }
                        }
                        
                        fullSystem->outputWrapper = wraps;
                        setting_fullResetRequested=false;
                    }
                }
                
                if(fullSystem->isLost) {
                    printf("LOST!!\n");
                    break;
                }
            }
        } else {
            // Fallback to original monocular pipeline
            printf("Using monocular pipeline (no associations file provided)\n");
            
            std::vector<double> timesToPlayAt;
            for(int i=lstart;i>= 0 && i< reader->getNumImages() && linc*i < linc*lend;i+=linc)
            {
                idsToPlay.push_back(i);
                if(timesToPlayAt.size() == 0)
                {
                    timesToPlayAt.push_back((double)0);
                }
                else
                {
                    double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
                    double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
                    timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
                }
            }

            std::vector<ImageAndExposure*> preloadedImages;
            if(preload)
            {
                printf("LOADING ALL IMAGES!\n");
                for(int ii=0;ii<(int)idsToPlay.size(); ii++)
                {
                    int i = idsToPlay[ii];
                    preloadedImages.push_back(reader->getImage(i));
                }
            }

            for(int ii=0;ii<(int)idsToPlay.size(); ii++)
            {
                while (Pause)
                {
                    usleep(5000);
                }
                    
                if(!fullSystem->initialized)
                {
                    gettimeofday(&tv_start, NULL);
                    started = clock();
                    sInitializerOffset = timesToPlayAt[ii];
                }

                int i = idsToPlay[ii];

                ImageAndExposure* img;
                if(preload)
                    img = preloadedImages[ii];
                else
                    img = reader->getImage(i);

                bool skipFrame=false;
                if(playbackSpeed!=0)
                {
                    struct timeval tv_now; gettimeofday(&tv_now, NULL);
                    double sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));

                    if(sSinceStart < timesToPlayAt[ii])
                        usleep((int)((timesToPlayAt[ii]-sSinceStart)*1000*1000));
                    else if(sSinceStart > timesToPlayAt[ii]+0.5+0.1*(ii%2))
                    {
                        printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                        skipFrame=true;
                    }
                }

                if (!skipFrame) {
                    if(ml_depth_enabled && fullSystem->ml_depth_enabled_) {
                        // Use undistorted RGB if available, otherwise load original
                        cv::Mat rgb_color;
                        
                        // Use DataReader's undistorted RGB channels for all datasets
                        if (img->useColour && img->r_image != nullptr && img->g_image != nullptr && img->b_image != nullptr) {
                            // Universal undistorted RGB processing - trust DataReader architecture
                            rgb_color = convertUndistortedToRGB(img);
                        } else {
                            // Handle case where undistorted color channels are not available
                            printf("Warning: No undistorted color channels available for frame %d\n", i);
                        }
                        
                        // Use ML-enhanced tracking with original ImageAndExposure to preserve both image buffers
                        fullSystem->TrackMonocularWithML(rgb_color, img);
                    } else {
                        // Use standard monocular tracking
                        fullSystem->addActiveFrame(img, i);
                    }
                }

                delete img;
        
                if(viewer!=0)
                    if(viewer->isDead)
                        break;

                if(fullSystem->initFailed || setting_fullResetRequested)
                {
                    if(ii < 250 || setting_fullResetRequested)
                    {
                        printf("RESETTING!\n");
                        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                        for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
                        usleep(20000);
                        if(fullSystem)
                        {
                            delete fullSystem;
                            fullSystem = nullptr;
                        }
                            
                        fullSystem = new FullSystem();
                        fullSystem->setGammaFunction(reader->getPhotometricGamma());
                        fullSystem->linearizeOperation = (playbackSpeed == 0);

                        if(LoopClosure)
                        {
                            fullSystem->setVocab(Vocabpnt);
                            printf("Vocabulary Set\n");
                        }

                        // CRITICAL FIX: Reinitialize ML depth service after reset (second path)
                        if(ml_depth_enabled) {
                            printf("Reinitializing ML depth service after SLAM reset...\n");
                            
                            // Recreate ML config (same as original initialization)
                            FullSystem::MLConfig ml_config;
                            ml_config.model_path = ml_model_path;
                            ml_config.enable_gpu = ml_gpu_enabled;
                            ml_config.num_threads = 4;
                            ml_config.enable_fp16 = ml_fp16_enabled;
                            ml_config.gpu_device_id = ml_gpu_device;
                            ml_config.gpu_memory_limit_mb = ml_gpu_memory_mb;
                            ml_config.input_width = ml_input_width;
                            ml_config.input_height = ml_input_height;
                            ml_config.model_type = ml_model_type_str;
                            ml_config.output_scale = ml_output_scale;
                            ml_config.benchmark_enabled = ml_benchmark_enabled;
                            
                            // Ablation study configuration
                            ml_config.inference_strategy = ml_strategy;
                            ml_config.snapshot_rate = ml_snapshot_interval;
                            
                            // Reinitialize MLDepthProcessor
                            if(!fullSystem->initializeMLDepthProcessor(ml_config)) {
                                printf("WARNING: Failed to reinitialize ML depth processor after reset\n");
                            } else {
                                printf("ML depth processor reinitialized successfully\n");
                            }
                        }

                        fullSystem->outputWrapper = wraps;
                        setting_fullResetRequested=false;
                    }
                }

                if(fullSystem->isLost)
                {
                    printf("LOST!!\n");
                    break;
                }
            }
            processedFrames = idsToPlay.size();  // Set for statistics
        }
		// fullSystem->BAatExit();
		fullSystem->blockUntilMappingIsFinished();
		clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);

        fullSystem->printResult("result.txt");
		if (outputPC) fullSystem->printPC("PC.PCD");

		// [MAP_EXPORT] Sprint 0d: ASCII PLY export of marginalized PointHessians
		if (setting_exportMapPly) {
			// Determine output path: use --map-ply-out if set, else result.ply in CWD
			std::string ply_path = setting_mapPlyOut;
			if (ply_path.empty()) ply_path = "result.ply";
			fullSystem->printMapPly(ply_path);
		}

        // Log keyframe statistics
        fullSystem->printKeyframeStats();

        // [PERF_SUMMARY] — machine-parseable performance line for eval summary
        {
            double avg_ml_ms = (ml_depth_enabled && fullSystem->ml_depth_enabled_)
                             ? fullSystem->ml_metrics_.avg_ml_inference_time_ms : -1.0;
            fullSystem->printPerfSummary(avg_ml_ms);
        }
        
        double MilliSecondsTakenSingle = 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC);
        double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
        
        // Log performance statistics using FPSLogger
        if (!associations.empty()) {
            // RGB-D pipeline statistics
            double avg_ms_per_frame = MilliSecondsTakenMT / (float)processedFrames;
            double fps = 1000.0 / avg_ms_per_frame;
            
            // Log SLAM performance
            HSLAM::FPSLogger::logSlamPerformance(processedFrames, avg_ms_per_frame, fps, "RGB-D");
            
            // Log RGB-D specific statistics using DepthLogger
            HSLAM::DepthLogger::logPerformance(processedFrames, avg_ms_per_frame, fps);
            HSLAM::DepthLogger::finalize();
        } else {
            // Monocular pipeline statistics
            int numFramesProcessed = abs(idsToPlay[0]-idsToPlay.back());
            double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0])-reader->getTimestamp(idsToPlay.back()));
            double avg_ms_per_frame = MilliSecondsTakenMT / (float)numFramesProcessed;
            double fps = 1000.0 / avg_ms_per_frame;
            
            // Determine pipeline type for logging
            std::string pipeline_type = (ml_depth_enabled && fullSystem->ml_depth_enabled_) ? "ML_Enhanced" : "Monocular";
            
            // Log SLAM performance with total keyframes created (not just active sliding window)
            int keyframes = static_cast<int>(fullSystem->allKeyFramesHistory.size());
            HSLAM::FPSLogger::logSlamPerformance(numFramesProcessed, keyframes, avg_ms_per_frame, fps, pipeline_type);
            
            // Log ML performance if enabled
            if (ml_depth_enabled && fullSystem->ml_depth_enabled_) {
                auto ml_metrics = fullSystem->ml_metrics_;
                HSLAM::FPSLogger::logMLPerformance(
                    ml_metrics.ml_keyframes_attempted,
                    ml_metrics.ml_keyframes_successful,
                    ml_metrics.avg_ml_inference_time_ms,
                    ml_metrics.ml_depth_utilization
                );
            }
            
            // Console output for compatibility
            printf("\n======================"
                    "\n%d Frames (%.1f fps)"
                    "\n%.2fms per frame (single core); "
                    "\n%.2fms per frame (multi core); "
                    "\n%.3fx (single core); "
                    "\n%.3fx (multi core); "
                    "\n======================\n\n",
                    numFramesProcessed, numFramesProcessed/numSecondsProcessed,
                    MilliSecondsTakenSingle/numFramesProcessed,
                    MilliSecondsTakenMT / (float)numFramesProcessed,
                    1000 / (MilliSecondsTakenSingle/numSecondsProcessed),
                    1000 / (MilliSecondsTakenMT / numSecondsProcessed));
        }
        
        // Finalize FPSLogger
        if(setting_logStuff) HSLAM::FPSLogger::finalize();
        
        //fullSystem->printFrameLifetimes();
        // time.txt: two-number timing summary, fully redundant with fpsLog.txt and run_log
        // if(setting_logStuff)
        // {
        //     std::ofstream tmlog;
        //     tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
        //     if (!associations.empty()) {
        //         tmlog << MilliSecondsTakenSingle/processedFrames << " " << MilliSecondsTakenMT / (float)processedFrames << "\n";
        //     } else {
        //         tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
        //               << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
        //     }
        //     tmlog.flush();
        //     tmlog.close();
        // }

		if(!pauseEnd){
		for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
		{
			ow->join();
		}}
    });


	if(viewer != 0)
	    viewer->run();

	runthread.join();

	for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
	{
		if(pauseEnd) ow->join();
		delete ow;
	}



	printf("DELETE FULLSYSTEM!\n");
	if (fullSystem)
	{
		delete fullSystem;
		fullSystem = nullptr;
	}

	if(LoopClosure && !vocabPath.empty())
	{
		delete Vocabpnt;
	}

	printf("DELETE READER!\n");
	delete reader;

	printf("EXIT NOW!\n");
	return 0;
}
