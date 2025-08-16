#pragma once

#include "MLInference.h"
#include <opencv2/opencv.hpp>
#include <memory>
#include <optional>

namespace HSLAM {
namespace ML {

/**
 * @brief Simplified synchronous ML depth processor
 * 
 * Replaces the complex asynchronous MLDepthService with direct synchronous processing.
 * Designed for keyframe-only processing where 37ms inference time is acceptable.
 * 
 * Key benefits:
 * - No background threading complexity
 * - No dual caching systems  
 * - Direct error propagation
 * - Simplified debugging
 * - Predictable timing
 */
class MLDepthProcessor {
public:
    struct ProcessingResult {
        cv::Mat depth_map;
        bool success = false;
        float inference_time_ms = 0.0f;
        float confidence = 1.0f;  // Default high confidence for successful processing
        std::string error_message;
        
        ProcessingResult() = default;
        
        ProcessingResult(const cv::Mat& depth, float time_ms, float conf = 1.0f)
            : depth_map(depth.clone()), success(!depth.empty()), 
              inference_time_ms(time_ms), confidence(conf) {}
    };

private:
    std::unique_ptr<MLInference> ml_inference_;
    MLInference::InferenceConfig config_;
    
    // Simple statistics (no complex performance tracking)
    mutable std::atomic<int> total_processed_{0};
    mutable std::atomic<int> successful_processed_{0};
    mutable std::atomic<float> total_inference_time_{0.0f};

public:
    /**
     * @brief Initialize ML depth processor with configuration
     * @param config ML inference configuration (reuses MLInference config)
     */
    explicit MLDepthProcessor(const MLInference::InferenceConfig& config = MLInference::InferenceConfig());
    
    /**
     * @brief Initialize the ML inference engine
     * @return True if initialization successful
     */
    bool initialize();
    
    /**
     * @brief Process keyframe for ML depth estimation (synchronous)
     * @param rgb_image Input RGB image (CV_8UC3 format expected)
     * @return Optional depth map result, empty if processing failed
     */
    std::optional<cv::Mat> processKeyframe(const cv::Mat& rgb_image);
    
    /**
     * @brief Process keyframe with detailed result information
     * @param rgb_image Input RGB image
     * @return Detailed processing result with timing and error info
     */
    ProcessingResult processKeyframeDetailed(const cv::Mat& rgb_image);
    
    /**
     * @brief Check if processor is initialized and ready
     * @return True if ready for processing
     */
    bool isReady() const;
    
    /**
     * @brief Get simple processing statistics
     * @return Basic performance metrics
     */
    struct SimpleStats {
        int total_processed = 0;
        int successful_processed = 0;
        float success_rate = 0.0f;
        float avg_inference_time_ms = 0.0f;
    };
    SimpleStats getStats() const;
    
    /**
     * @brief Reset processing statistics
     */
    void resetStats();
    
    /**
     * @brief Get current configuration
     * @return ML inference configuration
     */
    const MLInference::InferenceConfig& getConfig() const { return config_; }
    
    /**
     * @brief Shutdown processor and cleanup resources
     */
    void shutdown();
    
    // Disable copy constructor and assignment (same as MLInference)
    MLDepthProcessor(const MLDepthProcessor&) = delete;
    MLDepthProcessor& operator=(const MLDepthProcessor&) = delete;
};

} // namespace ML
} // namespace HSLAM