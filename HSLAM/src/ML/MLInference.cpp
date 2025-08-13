#include "ML/MLInference.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <fstream>
#include <unistd.h>

namespace HSLAM {
namespace ML {

MLInference::MLInference(const InferenceConfig& config) 
    : config_(config) {
}

MLInference::~MLInference() {
    shutdown();
}

bool MLInference::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_.load()) {
        return true;
    }

#ifdef HAS_ONNXRUNTIME
    try {
        printf("MLInference: Initializing ONNX Runtime...\n");
        
        // Initialize ONNX Runtime environment
        onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HSLAM_ML");
        
        // Configure session options (Sprint 2 Optimization)
        session_options_ = std::make_unique<Ort::SessionOptions>();
        session_options_->SetIntraOpNumThreads(config_.num_threads);
        session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        
        // Additional optimizations for GPU performance
        session_options_->SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        session_options_->EnableMemPattern();
        session_options_->EnableCpuMemArena();
        
        // Configure GPU execution provider if enabled
        if(config_.enable_gpu) {
        #ifdef HAS_CUDA
            try {
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = config_.gpu_device_id;
                cuda_options.arena_extend_strategy = 0;
                cuda_options.gpu_mem_limit = config_.gpu_memory_limit;
                cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
                cuda_options.do_copy_in_default_stream = 1;
                
                session_options_->AppendExecutionProvider_CUDA(cuda_options);
                printf("MLInference: GPU inference enabled (device %d, memory limit: %.1f GB)\n", 
                       config_.gpu_device_id, config_.gpu_memory_limit / (1024.0f * 1024.0f * 1024.0f));
                       
                // Enable FP16 optimization if requested
                if(config_.enable_fp16) {
                    printf("MLInference: FP16 optimization enabled\n");
                }
            } catch(const std::exception& e) {
                printf("WARNING: Failed to initialize GPU provider (%s), falling back to CPU\n", e.what());
                config_.enable_gpu = false;
            }
        #else
            printf("WARNING: GPU requested but CUDA not available, using CPU\n");
            config_.enable_gpu = false;
        #endif
        }
        
        printf("MLInference: Loading model from %s\n", config_.model_path.c_str());
        
        // Check if model file exists
        std::ifstream model_file(config_.model_path);
        if (!model_file.good()) {
            printf("ERROR: Model file not found or not readable: %s\n", config_.model_path.c_str());
            
            // Try to get current working directory
            char cwd[1024];
            if (getcwd(cwd, sizeof(cwd)) != nullptr) {
                printf("  Current working directory: %s\n", cwd);
            }
            
            // Check if parent directory exists
            size_t last_slash = config_.model_path.find_last_of("/\\");
            if (last_slash != std::string::npos) {
                std::string parent_dir = config_.model_path.substr(0, last_slash);
                std::ifstream parent_check(parent_dir);
                printf("  Parent directory (%s) exists: %s\n", parent_dir.c_str(), 
                       parent_check.good() ? "Yes" : "No");
            }
            
            return false;
        }
        model_file.close();
        
        // Load ONNX model with enhanced error handling
        try {
            onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, config_.model_path.c_str(), *session_options_);
            printf("MLInference: Model loaded successfully\n");
        } catch (const Ort::Exception& e) {
            printf("ERROR: Failed to load ONNX model: %s\n", e.what());
            printf("  Model path: %s\n", config_.model_path.c_str());
            printf("  ONNX Error code: %d\n", e.GetOrtErrorCode());
            return false;
        } catch (const std::exception& e) {
            printf("ERROR: Unexpected error loading model: %s\n", e.what());
            return false;
        }
        
        // Create memory info
        memory_info_ = std::make_unique<Ort::MemoryInfo>(
            Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
        
        // Setup input/output names
        setupInputOutputNames();
        
        // Validate model signature
        if (!validateModelSignature()) {
            printf("MLInference: Model signature validation failed\n");
            return false;
        }
        
        initialized_.store(true);
        printf("MLInference: Initialized successfully\n");
        printf("  Model: %s\n", config_.model_path.c_str());
        printf("  Device: %s\n", config_.enable_gpu ? "GPU" : "CPU");
        if(config_.enable_gpu) {
            printf("  GPU Device ID: %d\n", config_.gpu_device_id);
            printf("  GPU Memory Limit: %.1f GB\n", config_.gpu_memory_limit / (1024.0f * 1024.0f * 1024.0f));
            printf("  FP16 Optimization: %s\n", config_.enable_fp16 ? "Enabled" : "Disabled");
        }
        printf("  Threads: %d\n", config_.num_threads);
        printf("  Input size: %dx%d\n", config_.input_width, config_.input_height);
        
        return true;
        
    } catch(const Ort::Exception& e) {
        printf("MLInference: ONNX Runtime error: %s\n", e.what());
        return false;
    } catch(const std::exception& e) {
        printf("MLInference: Initialization error: %s\n", e.what());
        return false;
    }
#else
    printf("MLInference: ONNX Runtime not available - compile with HAS_ONNXRUNTIME\n");
    return false;
#endif
}

MLInference::InferenceResult MLInference::inferDepth(const cv::Mat& input_image) {
    InferenceResult result;
    result.success = false;
    
    if (!initialized_.load()) {
        result.error_message = "MLInference not initialized";
        return result;
    }
    
    if (input_image.empty()) {
        result.error_message = "Input image is empty";
        return result;
    }

#ifdef HAS_ONNXRUNTIME
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        auto start_time = std::chrono::steady_clock::now();
        
        // Preprocess input based on model type
        cv::Mat preprocessed;
        switch(config_.model_type) {
            case METRIC3D_V2:
                preprocessed = preprocessMetric3D(input_image);
                break;
            case DEPTH_ANYTHING:
                preprocessed = preprocessDepthAnything(input_image);
                break;
            case MIDAS_V3:
                preprocessed = preprocessMiDaS(input_image);
                break;
            default:
                result.error_message = "Unknown model type";
                return result;
        }
        
        if(preprocessed.empty()) {
            result.error_message = "Preprocessing failed";
            return result;
        }
        
        // Prepare ONNX input tensor
        std::vector<int64_t> input_shape = {1, 3, preprocessed.rows, preprocessed.cols};
        size_t input_tensor_size = preprocessed.total() * preprocessed.channels();
        std::vector<float> input_tensor_values(input_tensor_size);
        
        // Convert Mat to tensor format (CHW)
        std::vector<cv::Mat> channels;
        cv::split(preprocessed, channels);
        size_t channel_size = preprocessed.rows * preprocessed.cols;
        
        for(int c = 0; c < 3; ++c) {
            std::memcpy(input_tensor_values.data() + c * channel_size, 
                       channels[c].ptr<float>(), channel_size * sizeof(float));
        }
        
        // Use memory pool for GPU optimization (Sprint 2)
        std::vector<Ort::Value> output_tensors;
        if (config_.enable_gpu && memory_pool_ && memory_pool_->initialized_) {
            // Use IoBinding for GPU memory pool
            try {
                auto& io_binding = *memory_pool_->io_binding_;
                
                // Bind input tensor to GPU memory
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    *memory_info_, input_tensor_values.data(), input_tensor_size, 
                    input_shape.data(), input_shape.size());
                
                io_binding.BindInput(input_names_[0], input_tensor);
                io_binding.BindOutput(output_names_[0], *memory_info_);
                
                // Run with IoBinding (GPU optimized)
                onnx_session_->Run(Ort::RunOptions{nullptr}, io_binding);
                output_tensors = io_binding.GetOutputValues();
                
                // Update memory pool statistics
                memory_pool_->reuse_count_++;
                
            } catch (const std::exception& e) {
                // Fallback to regular inference on memory pool failure
                printf("MLInference: Memory pool failed, falling back: %s\n", e.what());
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    *memory_info_, input_tensor_values.data(), input_tensor_size, 
                    input_shape.data(), input_shape.size());
                
                output_tensors = onnx_session_->Run(Ort::RunOptions{nullptr}, 
                                                   input_names_.data(), &input_tensor, 1,
                                                   output_names_.data(), output_names_.size());
            }
        } else {
            // Regular inference (CPU or GPU without memory pool)
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                *memory_info_, input_tensor_values.data(), input_tensor_size, 
                input_shape.data(), input_shape.size());
            
            output_tensors = onnx_session_->Run(Ort::RunOptions{nullptr}, 
                                               input_names_.data(), &input_tensor, 1,
                                               output_names_.data(), output_names_.size());
        }
        
        // Extract output tensor
        if(output_tensors.empty()) {
            result.error_message = "No output from inference";
            return result;
        }
        
        // Debug: Check all output shapes for Metric3D
        if(config_.model_type == METRIC3D_V2) {
            printf("MLInference: Metric3D model has %zu outputs:\n", output_tensors.size());
            for(size_t i = 0; i < output_tensors.size(); i++) {
                auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
                printf("  Output[%zu] shape: [", i);
                for(size_t j = 0; j < shape.size(); j++) {
                    printf("%lld", shape[j]);
                    if(j < shape.size()-1) printf(", ");
                }
                printf("]\n");
            }
        }
        
        // Use appropriate output for Metric3D (typically the depth output is at index 1 or 2)
        size_t depth_output_index = 0;
        if(config_.model_type == METRIC3D_V2 && output_tensors.size() > 1) {
            // Try to find the depth output by examining shapes
            for(size_t i = 0; i < output_tensors.size(); i++) {
                auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
                // Depth output should be 4D: [batch, 1, height, width] or 3D: [1, height, width]
                if((shape.size() == 4 && shape[1] == 1) || 
                   (shape.size() == 3 && shape[0] == 1) ||
                   (shape.size() == 2)) {  // 2D height x width
                    depth_output_index = i;
                    printf("MLInference: Using output[%zu] as depth map\n", i);
                    break;
                }
            }
        }
        
        auto& output_tensor = output_tensors[depth_output_index];
        auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
        auto output_data = output_tensor.GetTensorData<float>();
        
        // Debug: Validate output shape
        if(output_shape.empty()) {
            result.error_message = "Output tensor has empty shape";
            return result;
        }
        
        // Convert output to vector
        size_t output_size = 1;
        for(auto dim : output_shape) output_size *= dim;
        
        if(output_size == 0) {
            result.error_message = "Output tensor has zero size";
            return result;
        }
        
        std::vector<float> output_values(output_data, output_data + output_size);
        
        // Postprocess output based on model type
        int output_height, output_width;
        if(output_shape.size() >= 2) {
            output_height = static_cast<int>(output_shape[output_shape.size()-2]);
            output_width = static_cast<int>(output_shape[output_shape.size()-1]);
        } else {
            result.error_message = "Invalid output shape dimensions";
            return result;
        }
        
        cv::Mat depth_map;
        switch(config_.model_type) {
            case METRIC3D_V2:
                depth_map = postprocessMetric3D(output_values, output_width, output_height);
                break;
            case DEPTH_ANYTHING:
                depth_map = postprocessDepthAnything(output_values, output_width, output_height);
                break;
            case MIDAS_V3:
                depth_map = postprocessMiDaS(output_values, output_width, output_height);
                break;
        }
        
        // Check if depth map is valid before resizing
        if(depth_map.empty()) {
            result.error_message = "Postprocessing returned empty depth map";
            return result;
        }
        
        // Resize to input image size if needed
        if(depth_map.size() != input_image.size()) {
            cv::resize(depth_map, depth_map, input_image.size(), 0, 0, cv::INTER_LINEAR);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        result.inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
        
        result.depth_map = depth_map;
        result.success = true;
        
        return result;
        
    } catch(const Ort::Exception& e) {
        result.error_message = "ONNX inference error: " + std::string(e.what());
        return result;
    } catch(const std::exception& e) {
        result.error_message = "Inference error: " + std::string(e.what());
        return result;
    }
#else
    result.error_message = "ONNX Runtime not available";
    return result;
#endif
}

cv::Mat MLInference::preprocessMetric3D(const cv::Mat& input_image) const {
    cv::Mat processed;
    
    // Handle both grayscale and RGB inputs
    if(input_image.channels() == 1) {
        // Grayscale input - convert to 3-channel RGB (replicate to R=G=B)
        cv::Mat rgb_image;
        cv::cvtColor(input_image, rgb_image, cv::COLOR_GRAY2BGR);
        
        // Convert BGR to RGB and normalize by 255.0 only
        cv::cvtColor(rgb_image, processed, cv::COLOR_BGR2RGB);
        processed.convertTo(processed, CV_32FC3, 1.0/255.0);
    } else if(input_image.channels() == 3) {
        // BGR input - convert to RGB once for ML processing
        cv::Mat rgb_image;
        cv::cvtColor(input_image, rgb_image, cv::COLOR_BGR2RGB);
        
        // Normalize by 255.0 to [0,1] range only (no [-1,1] conversion)
        rgb_image.convertTo(processed, CV_32FC3, 1.0/255.0);
    } else {
        // Invalid number of channels
        printf("ERROR: preprocessMetric3D - Invalid input channels: %d (expected 1 or 3)\n", input_image.channels());
        return cv::Mat();
    }
    
    // SIMPLIFIED APPROACH: Remove center-crop, use multiples-of-14 preserving aspect ratio
    int w = processed.cols;  // Original width (e.g., 640)  
    int h = processed.rows;  // Original height (e.g., 480)
    
    // Resize to nearest multiples of 14, preserving aspect ratio approximately  
    int target_width = int(std::round(w / 14.0f)) * 14;   // 640 → 644
    int target_height = int(std::round(h / 14.0f)) * 14;  // 480 → 476
    
    // Ensure minimum size (at least 14x14)
    target_width = std::max(target_width, 14);
    target_height = std::max(target_height, 14);
    
    // Direct resize preserving aspect ratio - NO CENTER CROP, NO LETTERBOX
    cv::Mat result;
    cv::resize(processed, result, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
    
    printf("MLInference: preprocessMetric3D - Simplified: %dx%d → %dx%d (multiples of 14, AR preserved)\n", 
           w, h, target_width, target_height);
    
    return result;
}

cv::Mat MLInference::preprocessDepthAnything(const cv::Mat& input_image) const {
    // Basic preprocessing for DepthAnything (placeholder)
    cv::Mat processed;
    if(input_image.type() != CV_32FC3) {
        input_image.convertTo(processed, CV_32FC3, 1.0/255.0);
    } else {
        processed = input_image.clone();
    }
    
    cv::resize(processed, processed, cv::Size(config_.input_width, config_.input_height), 
               0, 0, cv::INTER_LINEAR);
    
    return processed;
}

cv::Mat MLInference::preprocessMiDaS(const cv::Mat& input_image) const {
    // Basic preprocessing for MiDaS (placeholder)
    cv::Mat processed;
    if(input_image.type() != CV_32FC3) {
        input_image.convertTo(processed, CV_32FC3, 1.0/255.0);
    } else {
        processed = input_image.clone();
    }
    
    cv::resize(processed, processed, cv::Size(config_.input_width, config_.input_height), 
               0, 0, cv::INTER_LINEAR);
    
    return processed;
}

cv::Mat MLInference::postprocessMetric3D(const std::vector<float>& raw_output, int width, int height) const {
    // Validate input dimensions
    size_t expected_size = static_cast<size_t>(width * height);
    if(raw_output.size() != expected_size) {
        printf("MLInference: Dimension mismatch - expected %zu elements (%dx%d), got %zu\n", 
               expected_size, width, height, raw_output.size());
        return cv::Mat();
    }
    
    // Create depth map from raw output
    cv::Mat depth_map(height, width, CV_32FC1);
    std::memcpy(depth_map.ptr<float>(), raw_output.data(), raw_output.size() * sizeof(float));
    
    // Add postprocess sanity checks (Fix #4)
    int total_pixels = depth_map.rows * depth_map.cols;
    cv::Mat finite_mask;
    cv::compare(depth_map, depth_map, finite_mask, cv::CMP_EQ); // NaN check
    int finite_count = cv::countNonZero(finite_mask);
    float finite_ratio = (float)finite_count / total_pixels;
    
    double min_depth, max_depth;
    cv::minMaxLoc(depth_map, &min_depth, &max_depth);
    
    // Relaxed validation: Allow wider depth range for real-world scenes
    // finite_ratio > 0.5 and max_depth > 0.5 (some valid depth)
    if (finite_ratio < 0.5 || max_depth < 0.5) {
        printf("ML depth validation failed: finite_ratio=%.2f, range=[%.2f, %.2f], marking as invalid\n", 
               finite_ratio, min_depth, max_depth);
        // Return empty result to skip depth fusion
        return cv::Mat();
    }
    
    printf("ML depth validation passed: finite_ratio=%.2f, range=[%.2f, %.2f]\n", 
           finite_ratio, min_depth, max_depth);
    
    // Validate depth range
    double min_val, max_val;
    cv::minMaxLoc(depth_map, &min_val, &max_val);
    
    // Apply gentle clipping to outliers (preserve most of the depth range)
    cv::Mat clipped_depth;
    double clip_max = std::min(max_val * 1.1, 50.0);  // Allow 10% overhead, cap at 50m
    double clip_min = std::max(min_val * 0.9, 0.01);  // Allow 10% underhead, floor at 1cm
    
    cv::threshold(depth_map, clipped_depth, clip_max, clip_max, cv::THRESH_TRUNC);
    cv::threshold(clipped_depth, clipped_depth, clip_min, 0, cv::THRESH_TOZERO);
    
    // Apply final clipping to standard depth range for SLAM
    cv::threshold(clipped_depth, clipped_depth, 10.0, 10.0, cv::THRESH_TRUNC);  // Cap at 10m
    cv::threshold(clipped_depth, clipped_depth, 0.1, 0, cv::THRESH_TOZERO);     // Floor at 0.1m
    
    // Return depth map at inference resolution
    // The calling code will handle resizing to match the original image size if needed
    return clipped_depth;
}

cv::Mat MLInference::postprocessDepthAnything(const std::vector<float>& raw_output, int width, int height) const {
    // Basic postprocessing for DepthAnything (placeholder)
    cv::Mat depth_map(height, width, CV_32FC1);
    std::memcpy(depth_map.ptr<float>(), raw_output.data(), raw_output.size() * sizeof(float));
    return depth_map;
}

cv::Mat MLInference::postprocessMiDaS(const std::vector<float>& raw_output, int width, int height) const {
    // Basic postprocessing for MiDaS (placeholder)
    cv::Mat depth_map(height, width, CV_32FC1);
    std::memcpy(depth_map.ptr<float>(), raw_output.data(), raw_output.size() * sizeof(float));
    return depth_map;
}

bool MLInference::validateModelSignature() const {
#ifdef HAS_ONNXRUNTIME
    try {
        // Check input count
        size_t input_count = onnx_session_->GetInputCount();
        if(input_count == 0) {
            printf("MLInference: Model has no inputs\n");
            return false;
        }
        
        // Check output count
        size_t output_count = onnx_session_->GetOutputCount();
        if(output_count == 0) {
            printf("MLInference: Model has no outputs\n");
            return false;
        }
        
        printf("MLInference: Model validation - %zu inputs, %zu outputs\n", input_count, output_count);
        return true;
        
    } catch(const std::exception& e) {
        printf("MLInference: Model validation error: %s\n", e.what());
        return false;
    }
#else
    return false;
#endif
}

void MLInference::setupInputOutputNames() {
#ifdef HAS_ONNXRUNTIME
    try {
        // Get input names
        size_t input_count = onnx_session_->GetInputCount();
        input_name_strings_.clear();
        input_names_.clear();
        
        for(size_t i = 0; i < input_count; ++i) {
            auto input_name = onnx_session_->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
            input_name_strings_.push_back(input_name.get());
        }
        
        for(const auto& name : input_name_strings_) {
            input_names_.push_back(name.c_str());
        }
        
        // Get output names
        size_t output_count = onnx_session_->GetOutputCount();
        output_name_strings_.clear();
        output_names_.clear();
        
        for(size_t i = 0; i < output_count; ++i) {
            auto output_name = onnx_session_->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
            output_name_strings_.push_back(output_name.get());
        }
        
        for(const auto& name : output_name_strings_) {
            output_names_.push_back(name.c_str());
        }
        
        printf("MLInference: Setup %zu input names, %zu output names\n", 
               input_names_.size(), output_names_.size());
        
    } catch(const std::exception& e) {
        printf("MLInference: Error setting up input/output names: %s\n", e.what());
    }
#endif
}

void MLInference::shutdown() {
    // Stop warm-up thread if running
    if (warm_up_thread_.joinable()) {
        warm_up_thread_.join();
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return;
    }

#ifdef HAS_ONNXRUNTIME
    // Clean up memory pool
    memory_pool_.reset();
    
    onnx_session_.reset();
    session_options_.reset();
    memory_info_.reset();
    onnx_env_.reset();
    
    input_name_strings_.clear();
    output_name_strings_.clear();
    input_names_.clear();
    output_names_.clear();
#endif

    initialized_.store(false);
    warm_up_complete_.store(false);
    printf("MLInference: Shutdown complete\n");
}

// GPU Warm-up Implementation (Sprint 2 Optimization)

bool MLInference::startBackgroundWarmup(bool async) {
    if (!initialized_.load()) {
        printf("MLInference: Cannot start warm-up - not initialized\n");
        return false;
    }
    
    if (warm_up_complete_.load()) {
        printf("MLInference: Warm-up already complete\n");
        return true;
    }
    
    // Check if GPU is enabled
    if (!config_.enable_gpu) {
        // No warm-up needed for CPU inference
        warm_up_complete_.store(true);
        return true;
    }
    
    printf("MLInference: Starting GPU warm-up (async=%s)...\n", async ? "true" : "false");
    
    // Create dummy input for warm-up
    {
        std::lock_guard<std::mutex> lock(warm_up_mutex_);
        dummy_input_ = cv::Mat(config_.input_height, config_.input_width, CV_8UC3);
        cv::randu(dummy_input_, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    }
    
    if (async) {
        // Start warm-up in background thread
        warm_up_thread_ = std::thread([this]() {
            auto start = std::chrono::steady_clock::now();
            performWarmupInference();
            auto end = std::chrono::steady_clock::now();
            
            double warmup_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            printf("MLInference: GPU warm-up completed in %.2f ms\n", warmup_time_ms);
            
            warm_up_complete_.store(true);
            warm_up_cv_.notify_all();
        });
        
        // Detach thread to run independently
        warm_up_thread_.detach();
        return true;
    } else {
        // Synchronous warm-up
        auto start = std::chrono::steady_clock::now();
        performWarmupInference();
        auto end = std::chrono::steady_clock::now();
        
        double warmup_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
        printf("MLInference: GPU warm-up completed in %.2f ms\n", warmup_time_ms);
        
        warm_up_complete_.store(true);
        return true;
    }
}

void MLInference::performWarmupInference() {
#ifdef HAS_ONNXRUNTIME
    try {
        printf("MLInference: Executing warm-up inference...\n");
        
        // Initialize memory pool if GPU enabled
        if (config_.enable_gpu && !memory_pool_) {
            memory_pool_ = std::make_unique<GPUMemoryPool>();
            
            // Create IO binding for GPU memory management
            if (onnx_session_) {
                try {
                    memory_pool_->io_binding_ = std::make_unique<Ort::IoBinding>(*onnx_session_);
                    memory_pool_->allocated_bytes_ = config_.input_width * config_.input_height * 3 * sizeof(float);
                    memory_pool_->initialized_ = true;
                    printf("MLInference: Memory pool initialized (%zu bytes)\n", memory_pool_->allocated_bytes_);
                } catch (const std::exception& e) {
                    printf("MLInference: Memory pool initialization failed: %s\n", e.what());
                    memory_pool_.reset();
                }
            }
        }
        
        // Run 2-3 dummy inferences to fully warm up GPU
        for (int i = 0; i < 3; ++i) {
            auto result = inferDepth(dummy_input_);
            if (!result.success) {
                printf("MLInference: Warm-up inference %d failed: %s\n", 
                       i+1, result.error_message.c_str());
            } else {
                printf("MLInference: Warm-up inference %d completed in %.2f ms\n", 
                       i+1, result.inference_time_ms);
            }
        }
        
        // Clear dummy input
        {
            std::lock_guard<std::mutex> lock(warm_up_mutex_);
            dummy_input_.release();
        }
        
        printf("MLInference: GPU warm-up successful\n");
    } catch (const std::exception& e) {
        printf("MLInference: Warm-up failed: %s\n", e.what());
    }
#else
    printf("MLInference: ONNX Runtime not available for warm-up\n");
#endif
}

bool MLInference::waitForWarmup(int timeout_ms) {
    if (warm_up_complete_.load()) {
        return true;
    }
    
    if (!config_.enable_gpu) {
        // No warm-up needed for CPU
        return true;
    }
    
    std::unique_lock<std::mutex> lock(warm_up_mutex_);
    return warm_up_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                [this] { return warm_up_complete_.load(); });
}

MLInference::MemoryPoolStats MLInference::getMemoryPoolStats() const {
    MemoryPoolStats stats;
    
    if (memory_pool_) {
        stats.allocated_bytes = memory_pool_->allocated_bytes_;
        stats.reuse_count = memory_pool_->reuse_count_;
        stats.pool_initialized = memory_pool_->initialized_;
        
        if (memory_pool_->reuse_count_ > 0) {
            // Calculate efficiency as ratio of reuses to total allocations
            stats.memory_efficiency = static_cast<float>(memory_pool_->reuse_count_) / 
                                     (memory_pool_->reuse_count_ + 1);
        }
    }
    
    return stats;
}

bool MLInference::validateMetric3DModel() const {
#ifdef HAS_ONNXRUNTIME
    if (!initialized_.load()) {
        printf("MLInference: Cannot validate - not initialized\n");
        return false;
    }
    
    try {
        // Check model inputs
        size_t input_count = onnx_session_->GetInputCount();
        if (input_count != 1) {
            printf("MLInference: Metric3D model should have 1 input, found %zu\n", input_count);
            return false;
        }
        
        // Check input shape
        auto input_type_info = onnx_session_->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_shape = input_tensor_info.GetShape();
        
        if (input_shape.size() != 4) {
            printf("MLInference: Expected 4D input tensor, got %zuD\n", input_shape.size());
            return false;
        }
        
        // Validate input dimensions (batch, channels, height, width)
        // Handle dynamic shapes (-1) - only validate channels
        if (input_shape[1] != 3) {
            printf("MLInference: Channel count mismatch. Expected 3 channels, got %ld\n", input_shape[1]);
            return false;
        }
        
        // Check if spatial dimensions are dynamic (-1) or match expected size
        bool height_ok = (input_shape[2] == -1) || (input_shape[2] == config_.input_height);
        bool width_ok = (input_shape[3] == -1) || (input_shape[3] == config_.input_width);
        
        if (!height_ok || !width_ok) {
            printf("MLInference: Spatial dimensions mismatch. Expected [*,3,%d,%d] or [*,3,-1,-1], got [%ld,%ld,%ld,%ld]\n",
                   config_.input_height, config_.input_width,
                   input_shape[0], input_shape[1], input_shape[2], input_shape[3]);
            return false;
        }
        
        // Check model outputs
        size_t output_count = onnx_session_->GetOutputCount();
        if (output_count < 1) {
            printf("MLInference: Metric3D model should have at least 1 output, found %zu\n", output_count);
            return false;
        }
        
        printf("MLInference: Metric3D model validation successful\n");
        printf("  Input shape: [%ld,%ld,%ld,%ld]\n", 
               input_shape[0], input_shape[1], input_shape[2], input_shape[3]);
        printf("  Output count: %zu\n", output_count);
        
        return true;
        
    } catch (const std::exception& e) {
        printf("MLInference: Model validation error: %s\n", e.what());
        return false;
    }
#else
    printf("MLInference: ONNX Runtime not available for validation\n");
    return false;
#endif
}

float MLInference::benchmarkInferencePerformance(const cv::Mat& test_image, const std::string& results_dir) const {
#ifdef HAS_ONNXRUNTIME
    if (!initialized_.load()) {
        printf("MLInference: Cannot benchmark - not initialized\n");
        return -1.0f;
    }
    
    if (!config_.benchmark_enabled) {
        printf("MLInference: Benchmarking disabled in configuration\n");
        return -1.0f;
    }
    
    const int num_runs = 10;
    std::vector<float> inference_times;
    inference_times.reserve(num_runs);
    
    // Use provided test image or create a dummy image
    cv::Mat benchmark_image;
    if (!test_image.empty()) {
        benchmark_image = test_image.clone();
        printf("MLInference: Running benchmark with provided test image (%dx%d)\n", 
               benchmark_image.cols, benchmark_image.rows);
    } else {
        // Create a realistic dummy image for benchmarking
        benchmark_image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
        printf("MLInference: Running benchmark with dummy image (640x480)\n");
    }
    
    printf("MLInference: Running performance benchmark...\n");
    
    // Variables for visualization
    cv::Mat first_depth_result;
    bool visualization_saved = false;
    
    for (int i = 0; i < num_runs; ++i) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Run inference
        auto result = const_cast<MLInference*>(this)->inferDepth(benchmark_image);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        float inference_time_ms = duration.count() / 1000.0f;
        
        if (result.success) {
            inference_times.push_back(result.inference_time_ms);
            
            // Save first successful result for visualization
            if (i == 0 && !result.depth_map.empty()) {
                first_depth_result = result.depth_map.clone();
            }
        } else {
            printf("MLInference: Benchmark run %d failed: %s\n", i + 1, result.error_message.c_str());
        }
    }
    
    // Save visualization images to disk instead of displaying (avoids OpenGL conflicts)
    if (!first_depth_result.empty() && !visualization_saved) {
        try {
            // Find min/max values for normalization
            double min_depth, max_depth;
            cv::minMaxLoc(first_depth_result, &min_depth, &max_depth);
            
            printf("MLInference: Depth range: %.3f - %.3f meters\n", min_depth, max_depth);
            
            // Determine save directory
            std::string save_dir = results_dir.empty() ? "." : results_dir;
            
            // Create file paths
            std::string input_path = save_dir + "/ml_benchmark_input.png";
            std::string depth_path = save_dir + "/ml_benchmark_depth.png";
            std::string depth_raw_path = save_dir + "/ml_benchmark_depth_raw.png";
            
            // Save input image
            cv::Mat input_display;
            if (benchmark_image.channels() == 1) {
                cv::cvtColor(benchmark_image, input_display, cv::COLOR_GRAY2BGR);
            } else {
                input_display = benchmark_image.clone();
            }
            cv::imwrite(input_path, input_display);
            
            // Normalize depth map for visualization (0-255 range)
            cv::Mat depth_normalized;
            first_depth_result.convertTo(depth_normalized, CV_8UC1, 255.0 / (max_depth - min_depth), -min_depth * 255.0 / (max_depth - min_depth));
            
            // Apply colormap for better visualization
            cv::Mat depth_display;
            cv::applyColorMap(depth_normalized, depth_display, cv::COLORMAP_PLASMA);
            
            // Add depth information text
            std::string depth_info = cv::format("Depth Range: %.2f - %.2f meters", min_depth, max_depth);
            cv::putText(depth_display, depth_info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
            
            // Save depth visualization
            cv::imwrite(depth_path, depth_display);
            cv::imwrite(depth_raw_path, depth_normalized);
            
            printf("MLInference: Visualization images saved to %s:\n", save_dir.c_str());
            printf("  - ml_benchmark_input.png (input image)\n");
            printf("  - ml_benchmark_depth.png (colorized depth)\n");
            printf("  - ml_benchmark_depth_raw.png (raw depth grayscale)\n");
            
            visualization_saved = true;
            
        } catch (const cv::Exception& e) {
            printf("MLInference: Visualization error: %s\n", e.what());
        }
    }
    
    if (inference_times.empty()) {
        printf("MLInference: All benchmark runs failed\n");
        return -1.0f;
    }
    
    // Calculate statistics
    float avg_time = 0.0f;
    for (float time : inference_times) {
        avg_time += time;
    }
    avg_time /= inference_times.size();
    
    float min_time = *std::min_element(inference_times.begin(), inference_times.end());
    float max_time = *std::max_element(inference_times.begin(), inference_times.end());
    
    printf("MLInference: Benchmark complete\n");
    printf("  Successful runs: %zu/%d\n", inference_times.size(), num_runs);
    printf("  Average inference time: %.2f ms\n", avg_time);
    printf("  Min/Max time: %.2f/%.2f ms\n", min_time, max_time);
    printf("  Estimated FPS impact: %.1f fps\n", 1000.0f / avg_time);
    
    if (avg_time > 50.0f) {
        printf("WARNING: ML inference time (%.1fms) may impact real-time performance target (20+ fps)\n", avg_time);
    }
    
    return avg_time;
#else
    printf("MLInference: Benchmarking not available - ONNX Runtime not compiled\n");
    return -1.0f;
#endif
}

} // namespace ML
} // namespace HSLAM 