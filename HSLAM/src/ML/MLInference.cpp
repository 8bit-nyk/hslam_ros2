#include "ML/MLInference.h"
#include "util/settings.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <numeric>  // PHASE 2: For std::accumulate

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
            } catch(const std::exception& e) {
                printf("WARNING: Failed to initialize GPU provider (%s), falling back to CPU\n", e.what());
                config_.enable_gpu = false;
            }
        #else
            printf("WARNING: GPU requested but CUDA not available, using CPU\n");
            config_.enable_gpu = false;
        #endif
        }
        
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
        printf("MLInference: Initialized (%s:%d | %.1fGB | FP16:%s | %dx%d)\n",
               config_.enable_gpu ? "GPU" : "CPU",
               config_.gpu_device_id,
               config_.gpu_memory_limit / (1024.0f * 1024.0f * 1024.0f),
               config_.enable_fp16 ? "on" : "off",
               config_.input_width, config_.input_height);
        
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
    // === DEBUG_ML_PHASE1: MLInference entry point ===
    // printf("DEBUG_ML_PHASE1: MLInference::inferDepth() entry\n");
    // printf("DEBUG_ML_PHASE1: Input validation - size: %dx%d, channels: %d, type: %d\n",
    //        input_image.cols, input_image.rows, input_image.channels(), input_image.type());
    // printf("DEBUG_ML_PHASE1: Initialized state: %s\n", initialized_.load() ? "true" : "false");
    // printf("DEBUG_ML_PHASE1: Session pointer: %p\n", onnx_session_.get());
    // === END DEBUG_ML_PHASE1 ===
    
    InferenceResult result;
    result.success = false;
    
    if (!initialized_.load()) {
        // printf("DEBUG_ML_PHASE1: ERROR - MLInference not initialized!\n");
        result.error_message = "MLInference not initialized";
        return result;
    }
    
    if (input_image.empty()) {
        // printf("DEBUG_ML_PHASE1: ERROR - Input image is empty!\n");
        result.error_message = "Input image is empty";
        return result;
    }
    
    // printf("DEBUG_ML_PHASE1: Basic validation passed, proceeding with preprocessing...\n");
    
    // Additional validation checks
    if (input_image.cols <= 0 || input_image.rows <= 0) {
        result.error_message = "Input image has invalid dimensions";
        return result;
    }
    
    if (input_image.channels() != 3) {
        result.error_message = "Input image must have 3 channels (RGB)";
        return result;
    }
    
    if (input_image.type() != CV_8UC3) {
        result.error_message = "Input image must be CV_8UC3 format";
        return result;
    }
    
    if (!input_image.data) {
        result.error_message = "Input image data pointer is null";
        return result;
    }
    
    // DEBUG: MLInference.inferDepth - Input RGB: %dx%d channels=%d type=%d\n", 
    //        input_image.cols, input_image.rows, input_image.channels(), input_image.type());

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
        
        // printf("DEBUG: MLInference - Checking memory pool: enable_gpu=%s, memory_pool_=%s, initialized_=%s\n",
        //        config_.enable_gpu ? "true" : "false",
        //        memory_pool_ ? "present" : "null",
        //        (memory_pool_ && memory_pool_->initialized_) ? "true" : "false");
        
        if (config_.enable_gpu && memory_pool_ && memory_pool_->initialized_) {
            // printf("DEBUG: MLInference - Using IoBinding GPU path\n");
            // Use IoBinding for GPU memory pool
            try {
                auto& io_binding = *memory_pool_->io_binding_;
                
                // CRITICAL FIX: Clear previous bindings to prevent state corruption
                io_binding.ClearBoundInputs();
                io_binding.ClearBoundOutputs();
                
                // Bind input tensor to GPU memory
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    *memory_info_, input_tensor_values.data(), input_tensor_size, 
                    input_shape.data(), input_shape.size());
                
                io_binding.BindInput(input_names_[0], input_tensor);
                // [ML_OUTPUT_TENSORS] Sprint 0c fix: bind ALL outputs, not just output[0]
                for (size_t i = 0; i < output_names_.size(); ++i) {
                    io_binding.BindOutput(output_names_[i], *memory_info_);
                }

                // printf("DEBUG: MLInference - IoBinding configured, running session...\n");

                // Run with IoBinding (GPU optimized)
                onnx_session_->Run(Ort::RunOptions{nullptr}, io_binding);
                output_tensors = io_binding.GetOutputValues();

                // [ML_OUTPUT_TENSORS] One-shot diagnostic: verify multi-output binding is active
                {
                    static bool printed = false;
                    if (!printed) {
                        printed = true;
                        printf("[ML_OUTPUT_TENSORS] GPU IoBinding: output_tensors.size()=%zu\n", output_tensors.size());
                        for (size_t i = 0; i < output_tensors.size(); ++i) {
                            auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
                            printf("[ML_OUTPUT_TENSORS]   output[%zu] shape: [", i);
                            for (size_t j = 0; j < shape.size(); ++j) {
                                printf("%lld", (long long)shape[j]);
                                if (j < shape.size() - 1) printf(", ");
                            }
                            printf("]\n");
                        }
                    }
                }
                
                // printf("DEBUG: MLInference - Session run completed successfully\n");
                
                // Update memory pool statistics
                memory_pool_->reuse_count_++;
                
            } catch (const std::exception& e) {
                // Fallback to regular inference on IoBinding failure
                printf("MLInference: IoBinding failed, falling back to regular inference: %s\n", e.what());
                
                try {
                    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                        *memory_info_, input_tensor_values.data(), input_tensor_size, 
                        input_shape.data(), input_shape.size());
                    
                    // printf("DEBUG: MLInference - Fallback: Running session with regular inference...\n");
                    
                    output_tensors = onnx_session_->Run(Ort::RunOptions{nullptr}, 
                                                       input_names_.data(), &input_tensor, 1,
                                                       output_names_.data(), output_names_.size());
                                                       
                    // printf("DEBUG: MLInference - Fallback completed successfully\n");
                    
                } catch (const std::exception& fallback_e) {
                    result.error_message = "Both IoBinding and regular inference failed: " + std::string(fallback_e.what());
                    return result;
                }
            }
        } else {
            // Regular inference (CPU or GPU without memory pool)
            // printf("DEBUG: MLInference - Using regular inference path\n");
            
            try {
                // CRITICAL FIX: Validate session state before execution
                if (!onnx_session_) {
                    result.error_message = "ONNX session is null";
                    return result;
                }
                
                Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                    *memory_info_, input_tensor_values.data(), input_tensor_size, 
                    input_shape.data(), input_shape.size());
                
                // printf("DEBUG: MLInference - About to run session (tensor size: %zu)...\n", input_tensor_size);
                
                // CRITICAL FIX: Use safer RunOptions with explicit configuration
                Ort::RunOptions run_options;
                run_options.SetRunLogVerbosityLevel(0);  // Disable verbose logging
                run_options.SetRunTag("ml_inference");   // Add run tag for debugging
                
                output_tensors = onnx_session_->Run(run_options, 
                                                   input_names_.data(), &input_tensor, 1,
                                                   output_names_.data(), output_names_.size());
                                                   
                // printf("DEBUG: MLInference - Session run completed successfully\n");
                
            } catch (const Ort::Exception& ort_e) {
                result.error_message = "ONNX Runtime exception: " + std::string(ort_e.what());
                printf("ERROR: MLInference - ONNX Runtime exception: %s\n", ort_e.what());
                return result;
            } catch (const std::exception& e) {
                result.error_message = "Exception during regular inference: " + std::string(e.what());
                printf("ERROR: MLInference - Exception during inference: %s\n", e.what());
                return result;
            }
        }
        
        // [ML_OUTPUT_TENSORS] One-shot diagnostic (common path): verify output tensor count
        {
            static bool printed_common = false;
            if (!printed_common && !output_tensors.empty()) {
                printed_common = true;
                printf("[ML_OUTPUT_TENSORS] output_tensors.size()=%zu (path: %s)\n",
                       output_tensors.size(),
                       (config_.enable_gpu && memory_pool_ && memory_pool_->initialized_) ? "GPU-IoBinding" : "regular");
                for (size_t i = 0; i < output_tensors.size(); ++i) {
                    auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
                    printf("[ML_OUTPUT_TENSORS]   output[%zu] shape: [", i);
                    for (size_t j = 0; j < shape.size(); ++j) {
                        printf("%lld", (long long)shape[j]);
                        if (j < shape.size() - 1) printf(", ");
                    }
                    printf("]\n");
                }
            }
        }

        // Extract output tensor
        if(output_tensors.empty()) {
            result.error_message = "No output from inference";
            return result;
        }

        // [Sprint 1] Name-based output lookup (replaces heuristic by-shape selection).
        // Uses output_name_strings_ populated in setupInputOutputNames().
        // Expected names for Metric3D-v2: "predicted_depth", "predicted_normal", "normal_confidence".
        size_t depth_output_index = 0;
        size_t normal_output_index = SIZE_MAX;
        size_t kappa_output_index = SIZE_MAX;

        if(config_.model_type == METRIC3D_V2 && !output_name_strings_.empty()) {
            for(size_t i = 0; i < output_name_strings_.size() && i < output_tensors.size(); ++i) {
                const std::string& name = output_name_strings_[i];
                if(name == "predicted_depth")   depth_output_index  = i;
                if(name == "predicted_normal")  normal_output_index = i;
                if(name == "normal_confidence") kappa_output_index  = i;
            }
        } else if(config_.model_type == METRIC3D_V2 && output_tensors.size() > 1) {
            // Fallback: heuristic by-shape (pre-Sprint-1 behavior, no output_name_strings_)
            for(size_t i = 0; i < output_tensors.size(); i++) {
                auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
                if((shape.size() == 4 && shape[1] == 1) ||
                   (shape.size() == 3 && shape[0] == 1) ||
                   (shape.size() == 2)) {
                    depth_output_index = i;
                    break;
                }
            }
        }

        auto& output_tensor = output_tensors[depth_output_index];
        auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
        auto output_data = output_tensor.GetTensorData<float>();

        // PHASE 2: Extract uncertainty/confidence from kappa output if available
        std::vector<float> uncertainty_values;
        int uncertainty_height = 0, uncertainty_width = 0;
        bool has_uncertainty = false;

        if(config_.model_type == METRIC3D_V2 && kappa_output_index != SIZE_MAX) {
            auto& uncertainty_tensor = output_tensors[kappa_output_index];
            auto uncertainty_shape = uncertainty_tensor.GetTensorTypeAndShapeInfo().GetShape();
            auto uncertainty_data = uncertainty_tensor.GetTensorData<float>();

            if(uncertainty_shape.size() >= 2) {
                uncertainty_height = static_cast<int>(uncertainty_shape[uncertainty_shape.size()-2]);
                uncertainty_width = static_cast<int>(uncertainty_shape[uncertainty_shape.size()-1]);

                size_t uncertainty_size = 1;
                for(auto dim : uncertainty_shape) uncertainty_size *= dim;

                if(uncertainty_size > 0) {
                    uncertainty_values = std::vector<float>(uncertainty_data, uncertainty_data + uncertainty_size);
                    has_uncertainty = true;
                }
            }
        } else if(config_.model_type == METRIC3D_V2 && kappa_output_index == SIZE_MAX && output_tensors.size() > 2) {
            // Legacy fallback: output[2] was kappa before name-based lookup
            auto& uncertainty_tensor = output_tensors[2];
            auto uncertainty_shape = uncertainty_tensor.GetTensorTypeAndShapeInfo().GetShape();
            auto uncertainty_data = uncertainty_tensor.GetTensorData<float>();

            if(uncertainty_shape.size() >= 2) {
                uncertainty_height = static_cast<int>(uncertainty_shape[uncertainty_shape.size()-2]);
                uncertainty_width = static_cast<int>(uncertainty_shape[uncertainty_shape.size()-1]);
                size_t uncertainty_size = 1;
                for(auto dim : uncertainty_shape) uncertainty_size *= dim;
                if(uncertainty_size > 0) {
                    uncertainty_values = std::vector<float>(uncertainty_data, uncertainty_data + uncertainty_size);
                    has_uncertainty = true;
                }
            }
        }
        // Note: Model has only %zu outputs, no uncertainty available
        
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
        
        cv::Mat depth_map, confidence_map;
        
        switch(config_.model_type) {
            case METRIC3D_V2:
                depth_map = postprocessMetric3D(output_values, output_width, output_height, input_image.cols, input_image.rows);
                
                // PHASE 2: Process uncertainty into confidence map
                if(has_uncertainty) {
                    confidence_map = processUncertaintyToConfidence(uncertainty_values, uncertainty_width, uncertainty_height,
                                                                  input_image.cols, input_image.rows);
                    // [CONFIDENCE_STATS] One-shot: confirm confidence_map is non-trivially distributed
                    {
                        static bool conf_printed = false;
                        if (!conf_printed && !confidence_map.empty()) {
                            conf_printed = true;
                            double conf_min, conf_max;
                            cv::Scalar conf_mean, conf_std;
                            cv::minMaxLoc(confidence_map, &conf_min, &conf_max);
                            cv::meanStdDev(confidence_map, conf_mean, conf_std);
                            printf("[CONFIDENCE_STATS] Size: %dx%d, min=%.4f, max=%.4f, mean=%.4f, std=%.4f\n",
                                   confidence_map.cols, confidence_map.rows,
                                   (float)conf_min, (float)conf_max, (float)conf_mean[0], (float)conf_std[0]);
                        }
                    }
                } else {
                    // PHASE2_DEBUG: Create default confidence map
                    confidence_map = cv::Mat::ones(input_image.rows, input_image.cols, CV_32F);
                    // printf("PHASE2_DEBUG: Created default confidence map (all 1.0)\n");
                }
                break;
            case DEPTH_ANYTHING:
                depth_map = postprocessDepthAnything(output_values, output_width, output_height);
                // Resize from model output resolution back to input image resolution.
                if (!depth_map.empty() &&
                    (depth_map.cols != input_image.cols || depth_map.rows != input_image.rows)) {
                    cv::resize(depth_map, depth_map,
                               cv::Size(input_image.cols, input_image.rows),
                               0, 0, cv::INTER_LINEAR);
                }
                confidence_map = cv::Mat::ones(input_image.rows, input_image.cols, CV_32F);
                break;
            case MIDAS_V3:
                depth_map = postprocessMiDaS(output_values, output_width, output_height);
                // PHASE2_DEBUG: No uncertainty for MiDaS
                confidence_map = cv::Mat::ones(input_image.rows, input_image.cols, CV_32F);
                // printf("PHASE2_DEBUG: MiDaS - Created default confidence map\n");
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
        
        // PHASE 2: Resize confidence map if needed
        if(!confidence_map.empty() && confidence_map.size() != input_image.size()) {
            cv::resize(confidence_map, confidence_map, input_image.size(), 0, 0, cv::INTER_LINEAR);
        }
        
        auto end_time = std::chrono::steady_clock::now();
        result.inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
        
        result.depth_map = depth_map;
        result.confidence_map = confidence_map;  // PHASE 2: Include confidence map

        // [Sprint 1] Extract normal map from predicted_normal output if available.
        // postprocessNormalMap mirrors postprocessMetric3D: strip padding, resize, renormalize.
        if(config_.model_type == METRIC3D_V2 && normal_output_index != SIZE_MAX) {
            result.normal_map = postprocessNormalMap(output_tensors[normal_output_index],
                                                     input_image.cols, input_image.rows);
        }

        // [NORMAL_KAPPA] Per-frame κ stats from normal_confidence output (Sprint 1).
        if(has_uncertainty && !uncertainty_values.empty()) {
            static bool kappa_header_printed = false;
            if(!kappa_header_printed) {
                kappa_header_printed = true;
                printf("[NORMAL_KAPPA] First inference — per-frame κ stats follow\n");
            }
            float kmin = *std::min_element(uncertainty_values.begin(), uncertainty_values.end());
            float kmax = *std::max_element(uncertainty_values.begin(), uncertainty_values.end());
            float kmean = std::accumulate(uncertainty_values.begin(), uncertainty_values.end(), 0.0f)
                          / static_cast<float>(uncertainty_values.size());
            float kvar = 0.0f;
            for(float v : uncertainty_values) kvar += (v - kmean) * (v - kmean);
            kvar /= static_cast<float>(uncertainty_values.size());
            float kstd = std::sqrt(kvar);
            // p90
            std::vector<float> kappa_sorted(uncertainty_values);
            std::nth_element(kappa_sorted.begin(),
                             kappa_sorted.begin() + static_cast<ptrdiff_t>(kappa_sorted.size() * 9 / 10),
                             kappa_sorted.end());
            float kp90 = kappa_sorted[kappa_sorted.size() * 9 / 10];
            printf("[NORMAL_KAPPA] min=%.3f max=%.3f mean=%.3f std=%.3f p90=%.3f\n",
                   kmin, kmax, kmean, kstd, kp90);
        }

        // [NORMAL_STATS] Per-frame ‖n‖₂ distribution — verifies post-resize unit-norm preservation (Sprint 1).
        if(!result.normal_map.empty()) {
            double norm_min = 1e9, norm_max = 0.0;
            double norm_sum = 0.0, norm_sum_sq = 0.0;
            int total_pixels = result.normal_map.rows * result.normal_map.cols;
            int in_band = 0;
            for(int y = 0; y < result.normal_map.rows; y++) {
                const cv::Vec3f* row = result.normal_map.ptr<cv::Vec3f>(y);
                for(int x = 0; x < result.normal_map.cols; x++) {
                    float nx = row[x][0], ny = row[x][1], nz = row[x][2];
                    double norm = std::sqrt((double)nx*nx + (double)ny*ny + (double)nz*nz);
                    norm_min = std::min(norm_min, norm);
                    norm_max = std::max(norm_max, norm);
                    norm_sum += norm;
                    norm_sum_sq += norm * norm;
                    if(norm >= 0.95 && norm <= 1.05) in_band++;
                }
            }
            double norm_mean = norm_sum / total_pixels;
            double norm_var = norm_sum_sq / total_pixels - norm_mean * norm_mean;
            double norm_std = std::sqrt(std::max(0.0, norm_var));
            float frac_in_band = (float)in_band / (float)total_pixels;
            printf("[NORMAL_STATS] size=%dx%d norm_min=%.4f norm_max=%.4f mean=%.4f std=%.5f frac_in_band(0.95-1.05)=%.4f\n",
                   result.normal_map.cols, result.normal_map.rows,
                   (float)norm_min, (float)norm_max, (float)norm_mean, (float)norm_std, frac_in_band);
        }

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
        cv::cvtColor(input_image, processed, cv::COLOR_GRAY2BGR);
    } else if(input_image.channels() == 3) {
        processed = input_image.clone();
    } else {
        printf("ERROR: preprocessMetric3D - Invalid input channels: %d (expected 1 or 3)\n", input_image.channels());
        return cv::Mat();
    }
    
    // Convert BGR to RGB (model expects RGB)
    cv::cvtColor(processed, processed, cv::COLOR_BGR2RGB);
    
    // Calculate scale to preserve aspect ratio (keep_aspect_ratio: true)
    float scale = std::min(
        (float)config_.input_height / processed.rows,
        (float)config_.input_width / processed.cols
    );
    
    // DEBUG: Preprocessing - Input %dx%d, scale=%.3f for target %dx%d\n", 
    //        processed.cols, processed.rows, scale, config_.input_width, config_.input_height);
    
    // Resize preserving aspect ratio
    cv::Mat resized;
    int new_width = (int)(processed.cols * scale);
    int new_height = (int)(processed.rows * scale);
    cv::resize(processed, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
    
    // DEBUG: After aspect-ratio resize: %dx%d\n", resized.cols, resized.rows);
    
    // CRITICAL FIX: Validate configuration state before padding calculation
    if (config_.input_height <= 0 || config_.input_width <= 0) {
        printf("ERROR: Invalid config dimensions: %dx%d\n", config_.input_width, config_.input_height);
        throw std::runtime_error("Invalid configuration dimensions");
    }
    
    if (new_height > config_.input_height || new_width > config_.input_width) {
        printf("ERROR: Resized image larger than target: %dx%d > %dx%d\n", 
               new_width, new_height, config_.input_width, config_.input_height);
        throw std::runtime_error("Resized image larger than target");
    }
    
    // printf("DEBUG: Computing padding for target %dx%d from resized %dx%d\n",
    //        config_.input_width, config_.input_height, new_width, new_height);
    
    // Calculate padding to reach target size
    int pad_top = (config_.input_height - new_height) / 2;
    int pad_bottom = config_.input_height - new_height - pad_top;
    int pad_left = (config_.input_width - new_width) / 2;
    int pad_right = config_.input_width - new_width - pad_left;
    
    // printf("DEBUG: Computed padding: top=%d, bottom=%d, left=%d, right=%d\n",
    //        pad_top, pad_bottom, pad_left, pad_right);
    
    // Validate padding values
    if (pad_top < 0 || pad_bottom < 0 || pad_left < 0 || pad_right < 0) {
        printf("ERROR: Negative padding values: top=%d, bottom=%d, left=%d, right=%d\n",
               pad_top, pad_bottom, pad_left, pad_right);
        throw std::runtime_error("Negative padding values");
    }
    
    // Store padding info for postprocessing
    current_padding_.top = pad_top;
    current_padding_.bottom = pad_bottom;
    current_padding_.left = pad_left;
    current_padding_.right = pad_right;
    
    // DEBUG: Padding - top:%d, bottom:%d, left:%d, right:%d\n", 
    //        pad_top, pad_bottom, pad_left, pad_right);
    
    // Pad with ImageNet mean values (RGB: 123.675, 116.28, 103.53)
    cv::Mat padded;
    cv::copyMakeBorder(resized, padded, pad_top, pad_bottom, pad_left, pad_right,
                       cv::BORDER_CONSTANT, cv::Scalar(123.675, 116.28, 103.53));
    
    // DEBUG: After padding: %dx%d\n", padded.cols, padded.rows);
    
    // Convert to float32 WITHOUT normalization (keep [0,255] range)
    cv::Mat result;
    padded.convertTo(result, CV_32FC3); // No 1.0/255.0 scaling!
    
    // Debug: Check final values
    cv::Scalar mean, stddev;
    cv::meanStdDev(result, mean, stddev);
    // printf("DEBUG: Final values - mean=[%.1f,%.1f,%.1f], std=[%.1f,%.1f,%.1f]\n",
    //        mean[0], mean[1], mean[2], stddev[0], stddev[1], stddev[2]);
    
    // Reduce verbosity - preprocessing message commented out after initial testing
    // printf("MLInference: preprocessMetric3D - RGB %dx%d → Model input %dx%d (aspect ratio preserved)\n", 
    //        input_image.cols, input_image.rows, config_.input_width, config_.input_height);
    
    return result;
}

cv::Mat MLInference::preprocessDepthAnything(const cv::Mat& input_image) const {
    // Depth Anything V2 preprocessing per HuggingFace `preprocessor_config.json`:
    //   1. BGR -> RGB
    //   2. Resize to (input_width, input_height) -- square, no aspect-ratio padding
    //      (do_pad=false; we use direct square resize for runtime simplicity)
    //   3. Rescale to [0, 1]
    //   4. Normalize with ImageNet mean / std
    //   5. NCHW float32 (handled later in inferDepth via cv::dnn::blobFromImage shape)
    //
    // No padding info recorded -- direct resize means postprocess just resizes back.
    cv::Mat rgb;
    cv::cvtColor(input_image, rgb, cv::COLOR_BGR2RGB);

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(config_.input_width, config_.input_height),
               0, 0, cv::INTER_CUBIC);

    cv::Mat f32;
    resized.convertTo(f32, CV_32FC3, 1.0 / 255.0);

    // ImageNet mean/std normalization (channel-wise on RGB)
    const float mean[3] = {0.485f, 0.456f, 0.406f};
    const float std_[3] = {0.229f, 0.224f, 0.225f};
    std::vector<cv::Mat> chans(3);
    cv::split(f32, chans);
    for (int c = 0; c < 3; c++) {
        chans[c] = (chans[c] - mean[c]) / std_[c];
    }
    cv::merge(chans, f32);

    // Reset padding info (no padding for DA V2)
    current_padding_.top = -1;
    current_padding_.bottom = -1;
    current_padding_.left = -1;
    current_padding_.right = -1;

    return f32;
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

cv::Mat MLInference::postprocessMetric3D(const std::vector<float>& raw_output, int width, int height, int target_width, int target_height) const {
    // Debug: Model output analysis
    // DEBUG: Model output - dimensions %dx%d, total elements: %zu\n", width, height, raw_output.size());
    
    // Validate input dimensions
    size_t expected_size = static_cast<size_t>(width * height);
    if(raw_output.size() != expected_size) {
        printf("ERROR: Dimension mismatch - expected %zu elements (%dx%d), got %zu\n", 
               expected_size, width, height, raw_output.size());
        return cv::Mat();
    }
    
    // Create depth map from raw output
    cv::Mat depth_map(height, width, CV_32FC1);
    std::memcpy(depth_map.ptr<float>(), raw_output.data(), raw_output.size() * sizeof(float));
    
    // Remove padding if it was applied during preprocessing
    cv::Mat unpadded_depth;
    if (current_padding_.top >= 0) {
        int unpadded_height = height - current_padding_.top - current_padding_.bottom;
        int unpadded_width = width - current_padding_.left - current_padding_.right;
        
        // printf("DEBUG: Removing padding - original %dx%d, unpadded %dx%d\n", 
        //        width, height, unpadded_width, unpadded_height);
        
        if (unpadded_height > 0 && unpadded_width > 0 && 
            current_padding_.top + unpadded_height <= height &&
            current_padding_.left + unpadded_width <= width) {
            
            cv::Rect crop_region(current_padding_.left, current_padding_.top, 
                                 unpadded_width, unpadded_height);
            unpadded_depth = depth_map(crop_region).clone();
            // DEBUG: Padding removed - depth map now %dx%d\n", 
            //        unpadded_depth.cols, unpadded_depth.rows);
        } else {
            printf("WARNING: Invalid padding info, using full depth map\n");
            unpadded_depth = depth_map;
        }
    } else {
        printf("DEBUG: No padding to remove\n");
        unpadded_depth = depth_map;
    }
    
    // Resize to target resolution if needed
    cv::Mat final_depth;
    if (target_width > 0 && target_height > 0 && 
        (target_width != unpadded_depth.cols || target_height != unpadded_depth.rows)) {
        cv::resize(unpadded_depth, final_depth, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        // printf("ML depth postprocess: %dx%d → %dx%d (after padding removal)\n", 
        //        unpadded_depth.cols, unpadded_depth.rows, target_width, target_height);
    } else {
        final_depth = unpadded_depth;
    }
    
    // Enhanced depth validation and statistics
    double min_depth, max_depth;
    cv::minMaxLoc(final_depth, &min_depth, &max_depth);
    
    cv::Scalar mean_depth = cv::mean(final_depth);
    // TEMP_DEBUG_REPETITIVE: Commented out for cleaner testing output
    // printf("ML depth postprocess: range=[%.2f, %.2f], mean=%.2f\n", min_depth, max_depth, mean_depth[0]);
    
    // Quality validation
    if (std::abs(min_depth - max_depth) < 1e-6) {
        printf("ERROR: All depth values are nearly identical (%.6f) - processing may have failed\n", min_depth);
    }
    
    // Return depth map at target resolution
    return final_depth;
}

cv::Mat MLInference::postprocessDepthAnything(const std::vector<float>& raw_output, int width, int height) const {
    // Depth Anything V2 outputs INVERSE RELATIVE depth (no metric scale).
    // Larger value = closer pixel; the units are arbitrary.
    //
    // Conversion strategy (applied per-frame, no per-dataset tuning):
    //   - depth_relative = 1.0 / max(raw, eps)
    //   - Multiply by depth_scale config (default 1.0 = pure relative)
    //
    // Caller (CoarseInitializer / Phase 0) interprets `mlMeanDepth` as the metric
    // anchor. With DA V2 in default config (depth_scale=1.0), `mlMeanDepth` is in
    // arbitrary units -> Phase 0 metric init produces a non-metric trajectory whose
    // absolute scale is wrong by a constant factor; Sim(3) post-hoc alignment
    // recovers comparable ATE for evaluation.
    //
    // For an empirical "approximately metric" approach, calibrate depth_scale on
    // a held-out reference scene (e.g. TUM mean GT depth ~1.27m, DA V2 mean
    // inverse output ~6.5 -> 1/6.5*x = 1.27 -> x ~= 8.25). Pass via CLI.
    size_t expected_size = static_cast<size_t>(width * height);
    if (raw_output.size() != expected_size) {
        printf("ERROR: DA V2 dimension mismatch - expected %zu (%dx%d), got %zu\n",
               expected_size, width, height, raw_output.size());
        return cv::Mat();
    }

    cv::Mat inv_depth(height, width, CV_32FC1);
    std::memcpy(inv_depth.ptr<float>(), raw_output.data(),
                raw_output.size() * sizeof(float));

    // Convert inverse -> depth (relative units), apply optional scale factor
    cv::Mat depth_map(height, width, CV_32FC1);
    const float eps = 1e-3f;
    const float scale = config_.depth_scale > 0 ? config_.depth_scale : 1.0f;
    for (int y = 0; y < height; y++) {
        const float* in_row = inv_depth.ptr<float>(y);
        float* out_row = depth_map.ptr<float>(y);
        for (int x = 0; x < width; x++) {
            float v = in_row[x];
            out_row[x] = (v > eps) ? scale / v : 0.0f;
        }
    }

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
        
        // Model validation passed (%zu inputs, %zu outputs)
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
        
        // Input/output names configured (%zu in, %zu out)
        
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

// PHASE 2: Convert normal uncertainty to confidence
cv::Mat MLInference::processUncertaintyToConfidence(const std::vector<float>& uncertainty_output, int width, int height, int target_width, int target_height) const {
    // PHASE2_DEBUG: Log function entry
    // printf("PHASE2_DEBUG: processUncertaintyToConfidence called - Input: %dx%d -> Target: %dx%d\n", 
    //        width, height, target_width, target_height);
    
    if(uncertainty_output.empty()) {
        // PHASE2_DEBUG: Empty input
        // printf("PHASE2_DEBUG: Empty uncertainty output - returning empty confidence map\n");
        return cv::Mat();
    }
    
    // Validate input dimensions
    size_t expected_size = static_cast<size_t>(width * height);
    if(uncertainty_output.size() != expected_size) {
        // PHASE2_DEBUG: Size mismatch
        // printf("PHASE2_DEBUG: Size mismatch - expected %zu, got %zu\n", expected_size, uncertainty_output.size());
        return cv::Mat();
    }
    
    // Create uncertainty map from raw output
    cv::Mat uncertainty_map(height, width, CV_32FC1);
    std::memcpy(uncertainty_map.ptr<float>(), uncertainty_output.data(), uncertainty_output.size() * sizeof(float));
    
    // Step 0 RESOLVED (Mar 3, 2026): Case B confirmed. Raw range [0.7, 26.6] = uncertainty.
    // Sigmoid 1/(1+val/5) is correct. Uncomment below to re-verify if model changes.
    // cv::Scalar unc_mean, unc_std;
    // cv::meanStdDev(uncertainty_map, unc_mean, unc_std);
    // double unc_min, unc_max;
    // cv::minMaxLoc(uncertainty_map, &unc_min, &unc_max);
    // printf("[UNCERTAINTY_RAW] Size: %dx%d, Range: [%.3f, %.3f], Mean: %.3f, Std: %.3f\n",
    //        width, height, unc_min, unc_max, unc_mean[0], unc_std[0]);
    
    // Convert κ (AngMF concentration) to confidence [0, 1].
    // Sprint 3 — A.2: AngMF expected-angle formula replaces the mis-oriented sigmoid.
    //   AngMF path (setting_useAngmfConfidence=true, default):
    //     E_angle(κ) = 2κ/(κ²+1) + π·exp(-κπ)/(1+exp(-κπ))  ∈ [0, π/2]
    //     confidence  = 1 - E_angle(κ)/π                       ∈ [0.5, 1.0]
    //     High κ → low E_angle → high confidence (correctly oriented).
    //   Legacy sigmoid path (setting_useAngmfConfidence=false, ablation only):
    //     confidence = 1/(1 + κ/5)  ∈ (0, 1)  — provably mis-oriented (Sprint 0a).
    cv::Mat confidence_map = cv::Mat::zeros(height, width, CV_32FC1);

    if (setting_useAngmfConfidence) {
        // Sprint 3: AngMF closed-form expected angular error → confidence
        const float pi_f = static_cast<float>(M_PI);
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                const float kappa = std::max(uncertainty_map.at<float>(y, x), 1e-6f);
                const float term1 = 2.0f * kappa / (kappa * kappa + 1.0f);
                const float e_neg_kpi = std::exp(-kappa * pi_f);
                const float term2 = pi_f * e_neg_kpi / (1.0f + e_neg_kpi);
                const float e_angle = term1 + term2;       // expected angular error ∈ [0, π/2]
                confidence_map.at<float>(y, x) = 1.0f - e_angle / pi_f;  // ∈ [0.5, 1.0]
            }
        }
    } else {
        // Legacy sigmoid — retained for ablation reproducibility (Sprint 0a confirmed mis-oriented).
        const float uncertainty_scale = 5.0f;
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                const float uncertainty = uncertainty_map.at<float>(y, x);
                confidence_map.at<float>(y, x) = 1.0f / (1.0f + uncertainty / uncertainty_scale);
            }
        }
    }
    
    // Step 0 RESOLVED: Confidence output [0.16, 0.87] with mean ~0.44. Correct.
    // cv::Scalar conf_mean, conf_std;
    // cv::meanStdDev(confidence_map, conf_mean, conf_std);
    // double conf_min, conf_max;
    // cv::minMaxLoc(confidence_map, &conf_min, &conf_max);
    // printf("[UNCERTAINTY_CONF] Range: [%.3f, %.3f], Mean: %.3f, Std: %.3f\n",
    //        conf_min, conf_max, conf_mean[0], conf_std[0]);
    
    // Remove padding if it was applied (similar to depth processing)
    cv::Mat unpadded_confidence;
    if (current_padding_.top >= 0) {
        int unpadded_height = height - current_padding_.top - current_padding_.bottom;
        int unpadded_width = width - current_padding_.left - current_padding_.right;
        
        if(unpadded_height > 0 && unpadded_width > 0) {
            cv::Rect roi(current_padding_.left, current_padding_.top, unpadded_width, unpadded_height);
            unpadded_confidence = confidence_map(roi).clone();
            // PHASE2_DEBUG: Log unpadding
            // printf("PHASE2_DEBUG: Unpadded confidence %dx%d -> %dx%d\n", width, height, unpadded_width, unpadded_height);
        } else {
            unpadded_confidence = confidence_map;
        }
    } else {
        unpadded_confidence = confidence_map;
    }
    
    // Resize to target dimensions if specified
    if(target_width > 0 && target_height > 0 && 
       (unpadded_confidence.cols != target_width || unpadded_confidence.rows != target_height)) {
        cv::Mat resized_confidence;
        cv::resize(unpadded_confidence, resized_confidence, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        // PHASE2_DEBUG: Log resizing
        // printf("PHASE2_DEBUG: Resized confidence %dx%d -> %dx%d\n", 
        //        unpadded_confidence.cols, unpadded_confidence.rows, target_width, target_height);
        return resized_confidence;
    }
    
    return unpadded_confidence;
}

// [Sprint 1] Postprocess the predicted_normal output tensor.
// Mirrors postprocessMetric3D: strip aspect-ratio padding, resize to target_width × target_height,
// then per-pixel renormalize (defensive — Sprint 0b showed naïve cv::resize is correct to 100%
// on TUM but renormalization is ~3 LoC and costs nothing).
cv::Mat MLInference::postprocessNormalMap(const Ort::Value& tensor, int target_width, int target_height) const {
#ifdef HAS_ONNXRUNTIME
    auto shape = tensor.GetTensorTypeAndShapeInfo().GetShape();
    // Expected shape: [1, 3, H, W] (NCHW)
    if(shape.size() < 3) {
        printf("[NORMAL_STATS] ERROR: unexpected normal tensor rank %zu\n", shape.size());
        return cv::Mat();
    }

    int tensor_h = static_cast<int>(shape[shape.size()-2]);
    int tensor_w = static_cast<int>(shape[shape.size()-1]);
    const float* data = tensor.GetTensorData<float>();
    size_t spatial = static_cast<size_t>(tensor_h * tensor_w);

    // Deinterleave CHW (3, H, W) → HWC CV_32FC3
    cv::Mat normal_chw(tensor_h, tensor_w * 3, CV_32F);  // scratch storage
    cv::Mat ch0(tensor_h, tensor_w, CV_32F, const_cast<float*>(data));
    cv::Mat ch1(tensor_h, tensor_w, CV_32F, const_cast<float*>(data + spatial));
    cv::Mat ch2(tensor_h, tensor_w, CV_32F, const_cast<float*>(data + 2 * spatial));
    std::vector<cv::Mat> channels = {ch0.clone(), ch1.clone(), ch2.clone()};
    cv::Mat normal_hwc;
    cv::merge(channels, normal_hwc);  // CV_32FC3, shape [H, W, 3]

    // Strip aspect-ratio padding (same crop region as postprocessMetric3D)
    cv::Mat unpadded;
    if(current_padding_.top >= 0) {
        int unpadded_h = tensor_h - current_padding_.top - current_padding_.bottom;
        int unpadded_w = tensor_w - current_padding_.left - current_padding_.right;
        if(unpadded_h > 0 && unpadded_w > 0 &&
           current_padding_.top + unpadded_h <= tensor_h &&
           current_padding_.left + unpadded_w <= tensor_w) {
            cv::Rect roi(current_padding_.left, current_padding_.top, unpadded_w, unpadded_h);
            unpadded = normal_hwc(roi).clone();
        } else {
            unpadded = normal_hwc;
        }
    } else {
        unpadded = normal_hwc;
    }

    // Resize to target resolution (per-channel linear — Sprint 0b verified this preserves ‖n‖₂ at 100%)
    cv::Mat resized;
    if(target_width > 0 && target_height > 0 &&
       (unpadded.cols != target_width || unpadded.rows != target_height)) {
        cv::resize(unpadded, resized, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
    } else {
        resized = unpadded;
    }

    // Per-pixel renormalization (defensive, ~3 LoC — Sprint 0b says not strictly needed but recommended)
    for(int y = 0; y < resized.rows; y++) {
        cv::Vec3f* row = resized.ptr<cv::Vec3f>(y);
        for(int x = 0; x < resized.cols; x++) {
            float nx = row[x][0], ny = row[x][1], nz = row[x][2];
            float norm = std::sqrt(nx*nx + ny*ny + nz*nz);
            if(norm > 1e-6f) {
                row[x][0] = nx / norm;
                row[x][1] = ny / norm;
                row[x][2] = nz / norm;
            }
        }
    }

    return resized;
#else
    (void)tensor; (void)target_width; (void)target_height;
    return cv::Mat();
#endif
}

} // namespace ML
} // namespace HSLAM