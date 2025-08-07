# **HSLAM Deep Monocular Depth Estimation Integration Plan**

## **📋 Executive Summary**

This comprehensive plan integrates deep monocular depth estimation (MDE) models, specifically Metric3D, into the HSLAM framework as an **independent depth source** that operates alongside the traditional monocular SLAM pipeline. The implementation is **completely independent** from the existing TUM ground truth depth integration and operates on standard HSLAM inputs (RGB images + timestamps) without requiring association files or synchronized depth data.

The plan adopts an **iterative CI/CD architecture** with **timestamp-based synchronization** to maintain real-time performance (20+ fps) and core system integrity while enabling systematic testing of different inference strategies and device configurations.

---

## **🎯 Strategic Objectives**

### **Primary Goals**
1. **Replace Ground Truth Depth**: Transition from TUM GT depth maps to ML-estimated depth
2. **Independent Operation**: No dependency on associations.txt or synchronized depth data
3. **Real-time Performance**: Maintain 20+ fps SLAM processing with ML depth enhancement
4. **Strategy Optimization**: Test and compare inference strategies (every_frame, snapshot, keyframes)
5. **Device Flexibility**: Support both CPU-only and CPU/GPU hybrid configurations
6. **Systematic Testing**: Iterative development with clear success criteria for each phase

### **Technical Requirements**
- **Input**: Standard HSLAM RGB images + timestamps (no associations file needed)
- **Synchronization**: Timestamp-based frame correlation with configurable tolerance
- **Performance**: 20+ fps SLAM with 60-70% ML depth utilization target
- **Fallback**: Graceful degradation to monocular SLAM when ML depth unavailable
- **Logging**: Simple FPS and inference timing logs similar to existing DepthLogger

---

## **🏗️ Architectural Overview**

### **System Independence**
```
Traditional HSLAM Pipeline:
RGB Images + Timestamps → HSLAM → Trajectory + Map

Enhanced ML Pipeline:
RGB Images + Timestamps → HSLAM + ML Depth Service → Enhanced Trajectory + Map
                     ↓
            Timestamp Synchronization
                     ↓
              ML Depth Estimation
```

### **Core Components**
1. **MLInference**: ONNX-based Metric3D inference engine
2. **MLDepthService**: Asynchronous depth estimation with strategy management
3. **TimestampSynchronizer**: Frame correlation without association files
4. **StrategyTestFramework**: Performance comparison of inference strategies
5. **FPSLogger**: Simple performance logging following DepthLogger pattern

---

## **Phase 1: ML Inference Infrastructure**

### **WP-1.1: MLInference Core Engine**
**Objective**: ONNX-based Metric3D inference with CPU/GPU support

```cpp
/**
 * @brief Core ML inference engine for depth estimation
 * 
 * Provides ONNX-based inference for Metric3D and other MDE models
 * with support for both CPU and GPU execution.
 */
class MLInference {
public:
    enum ModelType {
        METRIC3D_V2,
        DEPTH_ANYTHING,
        MIDAS_V3
    };
    
    struct InferenceConfig {
        std::string model_path = "models/metric3d_depth.onnx";
        ModelType model_type = METRIC3D_V2;
        bool enable_gpu = false;
        bool enable_fp16 = false;
        int num_threads = 4;
        
        // Input preprocessing parameters
        int input_width = 616;
        int input_height = 518;
        bool normalize_input = true;
        
        // Output processing parameters
        float depth_scale = 1.0f;
        float min_depth = 0.1f;
        float max_depth = 10.0f;
    };
    
private:
    std::unique_ptr<Ort::Session> onnx_session_;
    std::unique_ptr<Ort::Env> onnx_env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    
    InferenceConfig config_;
    bool initialized_ = false;
    
    // Model-specific preprocessing
    cv::Mat preprocessMetric3D(const cv::Mat& input_image);
    cv::Mat preprocessDepthAnything(const cv::Mat& input_image);
    cv::Mat preprocessMiDaS(const cv::Mat& input_image);
    
    // Model-specific postprocessing
    cv::Mat postprocessMetric3D(const std::vector<float>& raw_output, int width, int height);
    cv::Mat postprocessDepthAnything(const std::vector<float>& raw_output, int width, int height);
    cv::Mat postprocessMiDaS(const std::vector<float>& raw_output, int width, int height);
    
public:
    explicit MLInference(const InferenceConfig& config = InferenceConfig{}) 
        : config_(config) {}
    
    /**
     * @brief Initialize ML inference engine
     * @return Initialization success status
     */
    bool initialize() {
        try {
            // Initialize ONNX Runtime environment
            onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HSLAM_ML");
            
            // Configure session options
            session_options_ = std::make_unique<Ort::SessionOptions>();
            session_options_->SetIntraOpNumThreads(config_.num_threads);
            session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
            
            // Configure GPU if requested and available
            if(config_.enable_gpu) {
#ifdef USE_CUDA
                OrtCUDAProviderOptions cuda_options;
                cuda_options.device_id = 0;
                cuda_options.arena_extend_strategy = 0;
                cuda_options.gpu_mem_limit = 2ULL * 1024 * 1024 * 1024;  // 2GB limit
                cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearch::EXHAUSTIVE;
                cuda_options.do_copy_in_default_stream = 1;
                
                session_options_->AppendExecutionProvider_CUDA(cuda_options);
                printf("GPU inference enabled with CUDA provider\n");
#else
                printf("WARNING: GPU requested but CUDA not available, using CPU\n");
                config_.enable_gpu = false;
#endif
            }
            
            // Load ONNX model
            onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, config_.model_path.c_str(), *session_options_);
            
            // Validate model inputs/outputs
            validateModelSignature();
            
            initialized_ = true;
            printf("MLInference initialized successfully\n");
            printf("  Model: %s\n", config_.model_path.c_str());
            printf("  Device: %s\n", config_.enable_gpu ? "GPU" : "CPU");
            printf("  Threads: %d\n", config_.num_threads);
            
            return true;
            
        } catch(const Ort::Exception& e) {
            printf("ONNX Runtime error: %s\n", e.what());
            return false;
        } catch(const std::exception& e) {
            printf("MLInference initialization error: %s\n", e.what());
            return false;
        }
    }
    
    /**
     * @brief Perform depth inference on input image
     * @param input_image Input RGB image (CV_8UC3 or CV_32FC3)
     * @return Depth map (CV_32FC1) or empty Mat on failure
     */
    cv::Mat inferDepth(const cv::Mat& input_image) {
        if(!initialized_) {
            printf("ERROR: MLInference not initialized\n");
            return cv::Mat();
        }
        
        try {
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
                    printf("ERROR: Unknown model type\n");
                    return cv::Mat();
            }
            
            if(preprocessed.empty()) {
                printf("ERROR: Preprocessing failed\n");
                return cv::Mat();
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
            
            // Create input tensor
            auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
                memory_info, input_tensor_values.data(), input_tensor_size, 
                input_shape.data(), input_shape.size());
            
            // Get input/output names
            auto input_names = getInputNames();
            auto output_names = getOutputNames();
            
            // Run inference
            auto output_tensors = onnx_session_->Run(Ort::RunOptions{nullptr}, 
                                                    input_names.data(), &input_tensor, 1,
                                                    output_names.data(), output_names.size());
            
            // Extract output tensor
            if(output_tensors.empty()) {
                printf("ERROR: No output from inference\n");
                return cv::Mat();
            }
            
            auto& output_tensor = output_tensors[0];
            auto output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
            auto output_data = output_tensor.GetTensorData<float>();
            
            // Convert output to vector
            size_t output_size = 1;
            for(auto dim : output_shape) output_size *= dim;
            std::vector<float> output_values(output_data, output_data + output_size);
            
            // Postprocess output based on model type
            cv::Mat depth_map;
            int output_height = static_cast<int>(output_shape[output_shape.size()-2]);
            int output_width = static_cast<int>(output_shape[output_shape.size()-1]);
            
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
            
            // Resize to input image size if needed
            if(depth_map.size() != input_image.size()) {
                cv::resize(depth_map, depth_map, input_image.size(), 0, 0, cv::INTER_LINEAR);
            }
            
            return depth_map;
            
        } catch(const Ort::Exception& e) {
            printf("ONNX inference error: %s\n", e.what());
            return cv::Mat();
        } catch(const std::exception& e) {
            printf("Inference error: %s\n", e.what());
            return cv::Mat();
        }
    }
    
    bool isInitialized() const { return initialized_; }
    const InferenceConfig& getConfig() const { return config_; }
    
private:
    void validateModelSignature();
    std::vector<const char*> getInputNames();
    std::vector<const char*> getOutputNames();
};
```

### **WP-1.2: Metric3D Preprocessing Implementation**
**Objective**: Implement Metric3D-specific preprocessing pipeline

```cpp
cv::Mat MLInference::preprocessMetric3D(const cv::Mat& input_image) {
    cv::Mat processed;
    
    // Convert input to float if needed
    if(input_image.type() != CV_32FC3) {
        input_image.convertTo(processed, CV_32FC3, 1.0/255.0);
    } else {
        processed = input_image.clone();
    }
    
    // Resize to model input size
    cv::resize(processed, processed, cv::Size(config_.input_width, config_.input_height), 
               0, 0, cv::INTER_LINEAR);
    
    // Metric3D normalization: [-1, 1] range
    processed = processed * 2.0 - 1.0;
    
    return processed;
}

cv::Mat MLInference::postprocessMetric3D(const std::vector<float>& raw_output, int width, int height) {
    // Create depth map from raw output
    cv::Mat depth_map(height, width, CV_32FC1);
    std::memcpy(depth_map.ptr<float>(), raw_output.data(), raw_output.size() * sizeof(float));
    
    // Apply depth scaling and clamping
    depth_map *= config_.depth_scale;
    cv::threshold(depth_map, depth_map, config_.max_depth, config_.max_depth, cv::THRESH_TRUNC);
    cv::threshold(depth_map, depth_map, config_.min_depth, 0, cv::THRESH_TOZERO);
    
    return depth_map;
}
```

---

## **Phase 2: Asynchronous ML Depth Service**

### **WP-2.1: MLDepthService Architecture**
**Objective**: Asynchronous depth estimation with multiple inference strategies

```cpp
/**
 * @brief Asynchronous ML depth estimation service
 * 
 * Manages ML depth inference with different strategies and provides
 * timestamp-based synchronization for SLAM integration.
 */
class MLDepthService {
public:
    enum InferenceStrategy {
        EVERY_FRAME,        // Run inference on every frame
        SNAPSHOT_N_FRAMES,  // Run inference every N frames
        KEYFRAMES_ONLY      // Run inference only on keyframes
    };
    
    struct FrameRequest {
        cv::Mat rgb_image;
        double timestamp;
        int frame_id;
        bool is_keyframe = false;
    };
    
    struct DepthResult {
        cv::Mat depth_map;
        double timestamp;
        int frame_id;
        bool valid = false;
        float inference_time_ms = 0.0f;
        float confidence = 0.0f;
    };
    
private:
    // Core components
    std::unique_ptr<MLInference> ml_inference_;
    InferenceStrategy current_strategy_;
    
    // Asynchronous processing
    std::queue<FrameRequest> request_queue_;
    std::map<double, DepthResult> result_cache_;  // Keyed by timestamp
    std::thread processing_thread_;
    std::mutex queue_mutex_;
    std::mutex cache_mutex_;
    std::condition_variable queue_condition_;
    std::atomic<bool> processing_active_{false};
    
    // Strategy parameters
    int snapshot_interval_ = 5;
    int frame_counter_ = 0;
    
    // Cache management
    static constexpr size_t MAX_CACHE_SIZE = 100;
    static constexpr double CACHE_TIMEOUT_SECONDS = 5.0;
    
    // Performance tracking
    std::atomic<int> successful_inferences_{0};
    std::atomic<int> failed_inferences_{0};
    std::atomic<float> avg_inference_time_ms_{0.0f};
    
public:
    explicit MLDepthService(const MLInference::InferenceConfig& config, 
                           InferenceStrategy strategy = EVERY_FRAME)
        : current_strategy_(strategy), snapshot_interval_(5) {
        
        // Initialize ML inference engine
        ml_inference_ = std::make_unique<MLInference>(config);
        if(!ml_inference_->initialize()) {
            throw std::runtime_error("Failed to initialize ML inference engine");
        }
    }
    
    ~MLDepthService() {
        stop();
    }
    
    /**
     * @brief Start asynchronous processing
     * @return Start success status
     */
    bool start() {
        if(processing_active_.load()) {
            return true;  // Already running
        }
        
        processing_active_.store(true);
        processing_thread_ = std::thread(&MLDepthService::processingLoop, this);
        
        printf("MLDepthService started with strategy: %s\n", getStrategyName(current_strategy_).c_str());
        return true;
    }
    
    /**
     * @brief Stop asynchronous processing
     */
    void stop() {
        processing_active_.store(false);
        queue_condition_.notify_all();
        
        if(processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        printf("MLDepthService stopped\n");
    }
    
    /**
     * @brief Submit frame for depth estimation
     * @param request Frame request with RGB image and metadata
     * @return Submission success status
     */
    bool submitFrame(const FrameRequest& request) {
        if(!processing_active_.load()) {
            return false;
        }
        
        // Apply strategy filtering
        bool should_process = false;
        
        switch(current_strategy_) {
            case EVERY_FRAME:
                should_process = true;
                break;
                
            case SNAPSHOT_N_FRAMES:
                should_process = (frame_counter_ % snapshot_interval_ == 0);
                break;
                
            case KEYFRAMES_ONLY:
                should_process = request.is_keyframe;
                break;
        }
        
        frame_counter_++;
        
        if(!should_process) {
            return true;  // Frame filtered by strategy, but not an error
        }
        
        // Add to processing queue
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(request);
        }
        queue_condition_.notify_one();
        
        return true;
    }
    
    /**
     * @brief Get depth result for timestamp with tolerance
     * @param target_timestamp Target timestamp to match
     * @param tolerance_ms Acceptable time difference in milliseconds
     * @return Depth result if available within tolerance
     */
    std::optional<DepthResult> getDepthResult(double target_timestamp, double tolerance_ms = 50.0) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        
        double tolerance_seconds = tolerance_ms / 1000.0;
        double best_time_diff = std::numeric_limits<double>::max();
        auto best_result = result_cache_.end();
        
        // Find closest timestamp within tolerance
        for(auto it = result_cache_.begin(); it != result_cache_.end(); ++it) {
            double time_diff = std::abs(it->first - target_timestamp);
            if(time_diff <= tolerance_seconds && time_diff < best_time_diff) {
                best_time_diff = time_diff;
                best_result = it;
            }
        }
        
        if(best_result != result_cache_.end()) {
            DepthResult result = best_result->second;
            result_cache_.erase(best_result);  // Remove used result
            return result;
        }
        
        return std::nullopt;
    }
    
    /**
     * @brief Get performance statistics
     */
    struct PerformanceStats {
        int successful_inferences = 0;
        int failed_inferences = 0;
        float avg_inference_time_ms = 0.0f;
        float success_rate = 0.0f;
        size_t queue_size = 0;
        size_t cache_size = 0;
    };
    
    PerformanceStats getPerformanceStats() const {
        PerformanceStats stats;
        stats.successful_inferences = successful_inferences_.load();
        stats.failed_inferences = failed_inferences_.load();
        stats.avg_inference_time_ms = avg_inference_time_ms_.load();
        
        int total = stats.successful_inferences + stats.failed_inferences;
        stats.success_rate = total > 0 ? (float)stats.successful_inferences / total : 0.0f;
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stats.queue_size = request_queue_.size();
        }
        
        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            stats.cache_size = result_cache_.size();
        }
        
        return stats;
    }
    
    void setStrategy(InferenceStrategy strategy, int snapshot_interval = 5) {
        current_strategy_ = strategy;
        snapshot_interval_ = snapshot_interval;
        frame_counter_ = 0;  // Reset counter
        
        printf("MLDepthService strategy changed to: %s\n", getStrategyName(strategy).c_str());
        if(strategy == SNAPSHOT_N_FRAMES) {
            printf("  Snapshot interval: %d frames\n", snapshot_interval);
        }
    }
    
    InferenceStrategy getCurrentStrategy() const { return current_strategy_; }
    
private:
    /**
     * @brief Main processing loop (runs in separate thread)
     */
    void processingLoop() {
        printf("MLDepthService processing thread started\n");
        
        while(processing_active_.load()) {
            FrameRequest request;
            bool has_request = false;
            
            // Get next request from queue
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_condition_.wait(lock, [this] { 
                    return !request_queue_.empty() || !processing_active_.load(); 
                });
                
                if(!request_queue_.empty()) {
                    request = request_queue_.front();
                    request_queue_.pop();
                    has_request = true;
                }
            }
            
            if(!has_request || !processing_active_.load()) {
                continue;
            }
            
            // Process frame
            auto start_time = std::chrono::steady_clock::now();
            cv::Mat depth_map = ml_inference_->inferDepth(request.rgb_image);
            auto end_time = std::chrono::steady_clock::now();
            
            float inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
            
            // Create result
            DepthResult result;
            result.timestamp = request.timestamp;
            result.frame_id = request.frame_id;
            result.inference_time_ms = inference_time_ms;
            result.valid = !depth_map.empty();
            
            if(result.valid) {
                result.depth_map = depth_map;
                result.confidence = calculateDepthConfidence(depth_map);
                successful_inferences_.fetch_add(1);
            } else {
                failed_inferences_.fetch_add(1);
            }
            
            // Update average inference time
            updateAverageInferenceTime(inference_time_ms);
            
            // Store result in cache
            {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                result_cache_[request.timestamp] = result;
                
                // Clean up old results
                cleanupCache();
            }
        }
        
        printf("MLDepthService processing thread stopped\n");
    }
    
    void updateAverageInferenceTime(float new_time_ms) {
        // Simple exponential moving average
        float current_avg = avg_inference_time_ms_.load();
        float alpha = 0.1f;  // Smoothing factor
        float new_avg = (1.0f - alpha) * current_avg + alpha * new_time_ms;
        avg_inference_time_ms_.store(new_avg);
    }
    
    float calculateDepthConfidence(const cv::Mat& depth_map) {
        // Simple confidence metric based on valid depth percentage
        cv::Mat valid_mask;
        cv::threshold(depth_map, valid_mask, 0.1f, 1.0f, cv::THRESH_BINARY);
        valid_mask.convertTo(valid_mask, CV_8UC1, 255.0);
        
        int valid_pixels = cv::countNonZero(valid_mask);
        int total_pixels = depth_map.rows * depth_map.cols;
        
        return (float)valid_pixels / total_pixels;
    }
    
    void cleanupCache() {
        // Remove results older than timeout
        double current_time = getCurrentTimestamp();
        
        auto it = result_cache_.begin();
        while(it != result_cache_.end()) {
            if(current_time - it->first > CACHE_TIMEOUT_SECONDS) {
                it = result_cache_.erase(it);
            } else {
                ++it;
            }
        }
        
        // Limit cache size
        while(result_cache_.size() > MAX_CACHE_SIZE) {
            result_cache_.erase(result_cache_.begin());
        }
    }
    
    double getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        return static_cast<double>(time_t);
    }
    
    std::string getStrategyName(InferenceStrategy strategy) {
        switch(strategy) {
            case EVERY_FRAME: return "every_frame";
            case SNAPSHOT_N_FRAMES: return "snapshot";
            case KEYFRAMES_ONLY: return "keyframes_only";
            default: return "unknown";
        }
    }
};
```

---

## **Phase 3: SLAM System Integration**

### **WP-3.1: FullSystem ML Depth Interface**
**Objective**: Integrate ML depth service into HSLAM's FullSystem

```cpp
// Enhanced FullSystem class with ML depth support
class FullSystem {
private:
    // Existing HSLAM members...
    
    // ML depth integration
    std::unique_ptr<ML::MLDepthService> ml_depth_service_;
    bool ml_depth_enabled_ = false;
    cv::Mat current_ml_depth_image_;
    mutable std::mutex ml_depth_mutex_;
    
    // ML performance tracking
    struct MLMetrics {
        size_t total_frames = 0;
        size_t frames_with_ml_depth = 0;
        float ml_depth_utilization = 0.0f;
        float avg_ml_inference_time_ms = 0.0f;
    };
    MLMetrics ml_metrics_;
    
public:
    /**
     * @brief Initialize ML depth system
     * @param config ML inference configuration
     * @param strategy Initial inference strategy
     * @return Initialization success status
     */
    bool initializeMLDepth(const ML::MLInference::InferenceConfig& config,
                          ML::MLDepthService::InferenceStrategy strategy = ML::MLDepthService::EVERY_FRAME) {
        try {
            ml_depth_service_ = std::make_unique<ML::MLDepthService>(config, strategy);
            
            if(!ml_depth_service_->start()) {
                printf("Failed to start ML depth service\n");
                return false;
            }
            
            ml_depth_enabled_ = true;
            printf("ML depth system initialized successfully\n");
            return true;
            
        } catch(const std::exception& e) {
            printf("ML depth initialization error: %s\n", e.what());
            ml_depth_enabled_ = false;
            return false;
        }
    }
    
    /**
     * @brief Enhanced monocular tracking with ML depth
     * @param rgb_image Input RGB image
     * @param timestamp Frame timestamp
     */
    void TrackMonocularWithML(const cv::Mat& rgb_image, double timestamp) {
        ml_metrics_.total_frames++;
        
        // Submit frame for ML depth estimation
        if(ml_depth_enabled_ && ml_depth_service_) {
            ML::MLDepthService::FrameRequest request;
            request.rgb_image = rgb_image.clone();
            request.timestamp = timestamp;
            request.frame_id = static_cast<int>(ml_metrics_.total_frames);
            request.is_keyframe = shouldCreateKeyframe();  // Use existing HSLAM keyframe logic
            
            ml_depth_service_->submitFrame(request);
        }
        
        // Try to get ML depth result for current or recent frame
        cv::Mat ml_depth_image;
        bool ml_depth_available = false;
        
        if(ml_depth_enabled_ && ml_depth_service_) {
            auto depth_result = ml_depth_service_->getDepthResult(timestamp, 50.0);  // 50ms tolerance
            if(depth_result && depth_result->valid) {
                ml_depth_image = depth_result->depth_map;
                ml_depth_available = true;
                ml_metrics_.frames_with_ml_depth++;
                
                // Update ML depth image for other components
                {
                    std::lock_guard<std::mutex> lock(ml_depth_mutex_);
                    current_ml_depth_image_ = ml_depth_image.clone();
                }
            }
        }
        
        // Update ML utilization rate
        ml_metrics_.ml_depth_utilization = (float)ml_metrics_.frames_with_ml_depth / ml_metrics_.total_frames;
        
        // Process frame with or without ML depth
        if(ml_depth_available) {
            // Enhanced tracking with ML depth
            TrackFrameWithMLDepth(rgb_image, ml_depth_image, timestamp);
        } else {
            // Standard monocular tracking
            ImageAndExposure* img = createImageAndExposure(rgb_image, timestamp);
            addActiveFrame(img, static_cast<int>(ml_metrics_.total_frames));
            delete img;
        }
    }
    
    /**
     * @brief Process frame with ML depth enhancement
     * @param rgb_image RGB input image
     * @param depth_image ML-estimated depth image
     * @param timestamp Frame timestamp
     */
    void TrackFrameWithMLDepth(const cv::Mat& rgb_image, const cv::Mat& depth_image, double timestamp) {
        // Convert RGB to HSLAM format
        ImageAndExposure* img = createImageAndExposure(rgb_image, timestamp);
        
        // Store ML depth for point creation and tracking
        {
            std::lock_guard<std::mutex> lock(ml_depth_mutex_);
            current_ml_depth_image_ = depth_image.clone();
        }
        
        // Use enhanced tracking with ML depth integration
        addActiveFrameWithMLDepth(img, static_cast<int>(ml_metrics_.total_frames), depth_image);
        
        delete img;
    }
    
    /**
     * @brief Enhanced active frame processing with ML depth
     * @param img HSLAM image structure
     * @param frame_id Frame identifier
     * @param ml_depth ML depth image
     */
    void addActiveFrameWithMLDepth(ImageAndExposure* img, int frame_id, const cv::Mat& ml_depth) {
        // Create frame hierarchy with ML depth enhancement
        FrameHessian* fh = new FrameHessian();
        fh->ab_exposure = img->exposure_time;
        fh->makeImages(img->image, &Hcalib);
        
        // Enhanced point creation using ML depth
        if(!initialized) {
            // Initialize with ML depth assistance
            initializeWithMLDepth(fh, ml_depth);
        } else {
            // Track with ML depth enhancement
            trackFrameWithMLDepth(fh, ml_depth);
        }
        
        // Continue with existing HSLAM pipeline
        if(initialized) {
            // ... existing tracking and mapping code ...
        }
    }
    
    /**
     * @brief Get current ML depth image (thread-safe)
     * @return Current ML depth image or empty Mat
     */
    cv::Mat getCurrentMLDepth() const {
        std::lock_guard<std::mutex> lock(ml_depth_mutex_);
        return current_ml_depth_image_.clone();
    }
    
    /**
     * @brief Get ML performance metrics
     * @return Current ML performance statistics
     */
    MLMetrics getMLMetrics() const {
        MLMetrics metrics = ml_metrics_;
        
        if(ml_depth_service_) {
            auto perf_stats = ml_depth_service_->getPerformanceStats();
            metrics.avg_ml_inference_time_ms = perf_stats.avg_inference_time_ms;
        }
        
        return metrics;
    }
    
    bool isMLDepthEnabled() const { return ml_depth_enabled_; }
    void setMLDepthEnabled(bool enabled) { ml_depth_enabled_ = enabled; }
    
private:
    /**
     * @brief Create ImageAndExposure from OpenCV Mat
     * @param rgb_image Input RGB image
     * @param timestamp Frame timestamp
     * @return HSLAM ImageAndExposure structure
     */
    ImageAndExposure* createImageAndExposure(const cv::Mat& rgb_image, double timestamp) {
        // Convert RGB to grayscale if needed
        cv::Mat gray_image;
        if(rgb_image.channels() == 3) {
            cv::cvtColor(rgb_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = rgb_image;
        }
        
        // Convert to float
        cv::Mat float_image;
        gray_image.convertTo(float_image, CV_32FC1);
        
        // Create HSLAM image structure
        ImageAndExposure* img = new ImageAndExposure();
        img->timestamp = timestamp;
        img->exposure_time = 1.0f;  // Default exposure
        
        // Allocate and copy image data
        img->w = float_image.cols;
        img->h = float_image.rows;
        img->image = new float[img->w * img->h];
        std::memcpy(img->image, float_image.ptr<float>(), img->w * img->h * sizeof(float));
        
        return img;
    }
    
    void initializeWithMLDepth(FrameHessian* fh, const cv::Mat& ml_depth);
    void trackFrameWithMLDepth(FrameHessian* fh, const cv::Mat& ml_depth);
    bool shouldCreateKeyframe();  // Use existing HSLAM keyframe detection
};
```

### **WP-3.2: Enhanced Point Creation with ML Depth**
**Objective**: Integrate ML depth into HSLAM's point creation pipeline

```cpp
/**
 * @brief Enhanced makeNewTraces with ML depth integration
 * 
 * Integrates ML depth estimates into HSLAM's point creation process
 * while preserving the original algorithm as fallback.
 */
void FullSystem::makeNewTracesWithMLDepth(FrameHessian* newFrame, const cv::Mat& ml_depth, float* map) {
    pixelSelector->allowFast = true;
    int numPointsTotal = pixelSelector->makeMaps(newFrame, map, setting_desiredImmatureDensity);
    
    newFrame->pointHessians.reserve(numPointsTotal * 1.2f);
    newFrame->pointHessiansMarginalized.reserve(numPointsTotal * 1.2f);
    newFrame->pointHessiansOut.reserve(numPointsTotal * 1.2f);
    
    int ml_depth_integrated_points = 0;
    int total_points_created = 0;
    
    for(int y = patternPadding + 1; y < hG[0] - patternPadding - 2; y++) {
        for(int x = patternPadding + 1; x < wG[0] - patternPadding - 2; x++) {
            int i = x + y * wG[0];
            if(map[i] == 0) continue;
            
            // Create ImmaturePoint with potential ML depth integration
            ImmaturePoint* impt = new ImmaturePoint(x, y, newFrame, map[i], &Hcalib);
            
            // Try to integrate ML depth if available
            bool ml_depth_used = false;
            if(!ml_depth.empty() && y < ml_depth.rows && x < ml_depth.cols) {
                float depth_value = ml_depth.at<float>(y, x);
                
                // Validate ML depth
                if(depth_value > 0.1f && depth_value < 10.0f && !std::isnan(depth_value)) {
                    // Convert depth to inverse depth
                    float idepth = 1.0f / depth_value;
                    
                    // Set ML depth with uncertainty bounds
                    float uncertainty = 0.08f + 0.1f * depth_value;  // Uncertainty increases with distance
                    impt->idepth_min = idepth - uncertainty;
                    impt->idepth_max = idepth + uncertainty;
                    impt->idepth_GT = idepth;  // Use ML depth as "ground truth" estimate
                    
                    ml_depth_integrated_points++;
                    ml_depth_used = true;
                }
            }
            
            if(!ml_depth_used) {
                // Use original HSLAM depth initialization
                float idepth_min = 0;
                float idepth_max = NAN;
                float prior = 1;
                
                if(setting_globalMapInitialization) {
                    // ... existing global map initialization code ...
                }
                
                impt->idepth_min = idepth_min;
                impt->idepth_max = idepth_max;
                impt->idepth_GT = NAN;
            }
            
            // Continue with existing point processing
            if(!std::isfinite(impt->energyTH)) {
                delete impt;
                continue;
            }
            
            newFrame->pointHessians.push_back(impt);
            total_points_created++;
        }
    }
    
    // Log ML depth integration statistics
    if(ml_depth_integrated_points > 0) {
        float integration_rate = (float)ml_depth_integrated_points / total_points_created;
        printf("Point Creation: %d total, %d with ML depth (%.1f%%)\n",
               total_points_created, ml_depth_integrated_points, integration_rate * 100.0f);
    }
    
    pixelSelector->allowFast = false;
    
    // Set flag for statistics
    newFrame->statistics_goodResOnThis = total_points_created;
}
```

---

## **🚀 Iterative CI/CD Implementation Strategy**

### **Phase 1: Minimal Viable Pipeline (First Goal)**
**Objective**: Get HSLAM and Metric3D running concurrently in the simplest setup
- **Milestone 1.1**: Basic MLInference class with CPU-only Metric3D
- **Milestone 1.2**: Simple TrackMonocularWithML method (every_frame strategy only)
- **Milestone 1.3**: Command-line integration (--ml-depth flag)
- **Milestone 1.4**: Basic FPS logging system
- **Success Criteria**: System builds, runs, and processes frames without crashing

### **Phase 2: Core Functionality (Second Goal)**
**Objective**: Complete the core ML depth integration with all strategies
- **Milestone 2.1**: Implement all three inference strategies (snapshot, keyframes)
- **Milestone 2.2**: Timestamp-based synchronization system
- **Milestone 2.3**: Strategy testing framework
- **Success Criteria**: All strategies work correctly and can be compared

### **Phase 3: Device Optimization (Third Goal)**
**Objective**: Add GPU support and optimize performance
- **Milestone 3.1**: GPU device configuration and ONNX GPU providers
- **Milestone 3.2**: CPU vs GPU performance comparison
- **Milestone 3.3**: Memory management and thread safety improvements
- **Success Criteria**: GPU inference works and shows performance benefits

### **Phase 4: Production Readiness (Final Goal)**
**Objective**: Robust, configurable, and well-documented system
- **Milestone 4.1**: Comprehensive error handling and fallback mechanisms
- **Milestone 4.2**: Enhanced configuration options and deployment scripts
- **Milestone 4.3**: Documentation and usage examples
- **Success Criteria**: Production-ready system with complete documentation

---

This comprehensive implementation plan provides a robust foundation for integrating deep monocular depth estimation into HSLAM while maintaining complete independence from ground truth depth systems and ensuring real-time performance through iterative development and testing strategies. 