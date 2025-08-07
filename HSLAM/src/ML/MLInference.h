#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

#ifdef HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace HSLAM {
namespace ML {

/**
 * @brief Core ML inference engine for depth estimation
 * 
 * Provides ONNX-based inference for Metric3D and other MDE models
 * with support for CPU execution (MVP version).
 */
class MLInference {
public:
    enum ModelType {
        METRIC3D_V2,
        DEPTH_ANYTHING,
        MIDAS_V3
    };
    
    struct InferenceConfig {
        std::string model_path;
        ModelType model_type;
        bool enable_gpu;
        int num_threads;
        
        // GPU-specific parameters
        bool enable_fp16;           // FP16 optimization for GPU
        int gpu_device_id;          // GPU device selection
        size_t gpu_memory_limit;    // GPU memory limit in bytes
        
        // Input preprocessing parameters
        int input_width;
        int input_height;
        bool normalize_input;
        
        // Output processing parameters
        float depth_scale;
        float min_depth;
        float max_depth;
        bool benchmark_enabled;  // Enable performance benchmarking
        
        // Constructor with default values
        InferenceConfig() 
            : model_path("models/metric3d-vit-small/onnx/model.onnx")
            , model_type(METRIC3D_V2)
            , enable_gpu(false)
            , num_threads(4)
            , enable_fp16(false)
            , gpu_device_id(0)
            , gpu_memory_limit(2ULL * 1024 * 1024 * 1024)  // 2GB default
            , input_width(256)   // Optimized for performance (was 518)
            , input_height(256)  // Optimized for performance (was 518)
            , normalize_input(true)
            , depth_scale(1.0f)
            , min_depth(0.1f)
            , max_depth(10.0f)
            , benchmark_enabled(false)
        {}
    };
    
    struct InferenceResult {
        cv::Mat depth_map;
        bool success = false;
        float inference_time_ms = 0.0f;
        std::string error_message;
    };
    
private:
#ifdef HAS_ONNXRUNTIME
    std::unique_ptr<Ort::Session> onnx_session_;
    std::unique_ptr<Ort::Env> onnx_env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;
    
    std::vector<std::string> input_name_strings_;
    std::vector<std::string> output_name_strings_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
#endif
    
    InferenceConfig config_;
    std::atomic<bool> initialized_{false};
    mutable std::mutex mutex_;
    
    // Model-specific preprocessing
    cv::Mat preprocessMetric3D(const cv::Mat& input_image) const;
    cv::Mat preprocessDepthAnything(const cv::Mat& input_image) const;
    cv::Mat preprocessMiDaS(const cv::Mat& input_image) const;
    
    // Model-specific postprocessing
    cv::Mat postprocessMetric3D(const std::vector<float>& raw_output, int width, int height) const;
    cv::Mat postprocessDepthAnything(const std::vector<float>& raw_output, int width, int height) const;
    cv::Mat postprocessMiDaS(const std::vector<float>& raw_output, int width, int height) const;
    
    // Helper methods
    bool validateModelSignature() const;
    void setupInputOutputNames();
    
public:
    explicit MLInference(const InferenceConfig& config = InferenceConfig());
    virtual ~MLInference();
    
    // Disable copy constructor and assignment operator
    MLInference(const MLInference&) = delete;
    MLInference& operator=(const MLInference&) = delete;
    
    /**
     * @brief Initialize ML inference engine
     * @return Initialization success status
     */
    bool initialize();
    
    /**
     * @brief Perform depth inference on input image
     * @param input_image Input RGB image (CV_8UC3 or CV_32FC3)
     * @return Inference result with depth map or error information
     */
    InferenceResult inferDepth(const cv::Mat& input_image);
    
    /**
     * @brief Check if the inference engine is initialized
     * @return True if initialized, false otherwise
     */
    bool isInitialized() const { return initialized_.load(); }
    
    /**
     * @brief Get current configuration
     * @return Current inference configuration
     */
    const InferenceConfig& getConfig() const { return config_; }
    
    /**
     * @brief Shutdown the inference engine
     */
    void shutdown();
    
    /**
     * @brief Validate the loaded Metric3D model
     * @return True if model is valid and compatible
     */
    bool validateMetric3DModel() const;
    
    /**
     * @brief Benchmark inference performance
     * @param test_image Optional test image for realistic benchmarking
     * @param results_dir Optional directory to save visualization images
     * @return Average inference time in milliseconds
     */
    float benchmarkInferencePerformance(const cv::Mat& test_image = cv::Mat(), const std::string& results_dir = "") const;
};

} // namespace ML
} // namespace HSLAM 