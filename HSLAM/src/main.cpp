#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

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
		("ml-strategy", "ML inference strategy (every_frame|keyframe_only|snapshot_mode)", cxxopts::value<std::string>()->default_value("every_frame"))
		("ml-snapshot-interval", "Frames between ML inference in snapshot mode", cxxopts::value<int>()->default_value("5"))
		("ml-benchmark", "Enable ML performance benchmarking", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu", "Enable GPU acceleration for ML inference", cxxopts::value<bool>()->default_value("false"))
		("ml-fp16", "Enable FP16 optimization for GPU inference", cxxopts::value<bool>()->default_value("false"))
		("ml-gpu-device", "GPU device ID for ML inference", cxxopts::value<int>()->default_value("0"))
		("ml-gpu-memory", "GPU memory limit in MB", cxxopts::value<size_t>()->default_value("2048"))
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
	
	// GPU acceleration options (Phase 3)
	bool ml_gpu_enabled = result["ml-gpu"].as<bool>();
	bool ml_fp16_enabled = result["ml-fp16"].as<bool>();
	int ml_gpu_device = result["ml-gpu-device"].as<int>();
	size_t ml_gpu_memory_mb = result["ml-gpu-memory"].as<size_t>();

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
		printf("DEFAULT settings:\n"
				"- %s real-time enforcing\n"
				"- 2000 active points\n"
				"- 5-7 active frames\n"
				"- 1-6 LM iteration each KF\n"
				"- original image resolution\n", preset==0 ? "no " : "1x");

		playbackSpeed = (preset==0 ? 0 : 1);
		// preload = preset==1;
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
			printf("  Snapshot interval: %d frames\n", ml_snapshot_interval);
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
		
		ml_config.input_width = 256;   // Optimized for performance (was 518)
		ml_config.input_height = 256;  // Optimized for performance (was 518)
		ml_config.min_depth = 0.1f;
		ml_config.max_depth = 10.0f;
		ml_config.benchmark_enabled = ml_benchmark_enabled;
		
		if(ml_gpu_enabled) {
			printf("  GPU Acceleration: Enabled (Device %d, Memory: %zu MB, FP16: %s)\n",
			       ml_gpu_device, ml_gpu_memory_mb, ml_fp16_enabled ? "Yes" : "No");
		} else {
			printf("  GPU Acceleration: Disabled (CPU mode)\n");
		}
		
		// Initialize ML depth service (asynchronous)
		if(!fullSystem->initializeMLDepthService(ml_config, ml_strategy, ml_snapshot_interval)) {
			printf("ERROR: Failed to initialize ML depth service\n");
			return -1;
		}
		
		printf("ML Depth Service (Phase 2) initialized successfully\n");
		
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
                viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], true);  // Enable GUI thread
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
        HSLAM::FPSLogger::initialize("results", dataset_name);
        HSLAM::FPSLogger::setDebugMode(false); // Disable debug mode by default
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
                
                // Load RGB image
                cv::Mat rgb_img_raw = cv::imread(rgb_path, cv::IMREAD_GRAYSCALE);
                if (rgb_img_raw.empty()) {
                    printf("ERROR: Could not load RGB image: %s\n", rgb_path.c_str());
                    continue;
                }
                
                // Convert RGB to float
                cv::Mat rgb_img;
                rgb_img_raw.convertTo(rgb_img, CV_32FC1);
                
                // Load depth image (16-bit PNG -> CV_32FC1, scaled by 1/5000.0)
                cv::Mat depth_img_raw = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
                if (depth_img_raw.empty()) {
                    printf("ERROR: Could not load depth image: %s\n", depth_path.c_str());
                    continue;
                }
                
                cv::Mat depth_img;
                depth_img_raw.convertTo(depth_img, CV_32FC1, 1.0/5000.0);
                
                // Debug-only logging for frame processing details
                HSLAM::DepthLogger::logFrameProcessing(processedFrames, rgb_img.cols, rgb_img.rows,
                                                     depth_img.cols, depth_img.rows, timestamp);
                
                // Call new TrackRGBD method
                fullSystem->TrackRGBD(rgb_img, depth_img, timestamp);
                
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
                        // Use ML-enhanced tracking
                        cv::Mat rgb_mat(img->h, img->w, CV_32FC1, img->image);
                        fullSystem->TrackMonocularWithML(rgb_mat, img->timestamp);
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
        
        // Log keyframe statistics
        fullSystem->printKeyframeStats();
        
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
            
            // Log SLAM performance
            HSLAM::FPSLogger::logSlamPerformance(numFramesProcessed, avg_ms_per_frame, fps, pipeline_type);
            
            // Log ML performance if enabled
            if (ml_depth_enabled && fullSystem->ml_depth_enabled_) {
                auto ml_metrics = fullSystem->ml_metrics_;
                HSLAM::FPSLogger::logMLPerformance(
                    ml_metrics.total_frames,
                    ml_metrics.frames_with_ml_depth,
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
        HSLAM::FPSLogger::finalize();
        
        //fullSystem->printFrameLifetimes();
        if(setting_logStuff)
        {
            std::ofstream tmlog;
            tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
            if (!associations.empty()) {
                tmlog << MilliSecondsTakenSingle/processedFrames << " " << MilliSecondsTakenMT / (float)processedFrames << "\n";
            } else {
                tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
                      << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
            }
            tmlog.flush();
            tmlog.close();
        }

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
