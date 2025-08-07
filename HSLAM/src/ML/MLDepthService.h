#pragma once

#include "MLInference.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <optional>
#include <chrono>

namespace HSLAM {
namespace ML {

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
        KEYFRAME_ONLY,      // Run inference only on keyframes  
        SNAPSHOT_MODE       // Run inference every N frames
    };
    
    struct FrameRequest {
        cv::Mat rgb_image;
        double timestamp;
        int frame_id;
        bool is_keyframe = false;
        
        FrameRequest() = default;
        
        FrameRequest(const cv::Mat& img, double ts, int id, bool keyframe = false)
            : rgb_image(img.clone()), timestamp(ts), frame_id(id), is_keyframe(keyframe) {}
    };
    
    struct DepthResult {
        cv::Mat depth_map;
        double timestamp;
        int frame_id;
        bool valid = false;
        float inference_time_ms = 0.0f;
        float confidence = 0.0f;
        std::string error_message;
        
        DepthResult() = default;
        
        DepthResult(const cv::Mat& depth, double ts, int id, float time_ms, float conf = 0.0f)
            : depth_map(depth.clone()), timestamp(ts), frame_id(id), valid(!depth.empty()),
              inference_time_ms(time_ms), confidence(conf) {}
    };
    
    struct PerformanceStats {
        int successful_inferences = 0;
        int failed_inferences = 0;
        float avg_inference_time_ms = 0.0f;
        float success_rate = 0.0f;
        size_t queue_size = 0;
        size_t cache_size = 0;
        InferenceStrategy current_strategy;
        int frames_processed = 0;
        int frames_skipped = 0;
    };

private:
    // Core components
    std::unique_ptr<MLInference> ml_inference_;
    InferenceStrategy current_strategy_;
    
    // Asynchronous processing
    std::queue<FrameRequest> request_queue_;
    std::map<double, DepthResult> result_cache_;  // Keyed by timestamp
    std::thread processing_thread_;
    mutable std::mutex queue_mutex_;
    mutable std::mutex cache_mutex_;
    std::condition_variable queue_condition_;
    std::atomic<bool> processing_active_{false};
    std::atomic<bool> shutdown_requested_{false};
    
    // Strategy parameters
    int snapshot_interval_ = 5;
    int frame_counter_ = 0;
    
    // Cache management
    static constexpr size_t MAX_CACHE_SIZE = 100;
    static constexpr double CACHE_TIMEOUT_SECONDS = 5.0;
    
    // Performance tracking
    std::atomic<int> successful_inferences_{0};
    std::atomic<int> failed_inferences_{0};
    std::atomic<int> frames_processed_{0};
    std::atomic<int> frames_skipped_{0};
    std::atomic<float> avg_inference_time_ms_{0.0f};

public:
    explicit MLDepthService(const MLInference::InferenceConfig& config, 
                           InferenceStrategy strategy = EVERY_FRAME,
                           int snapshot_interval = 5);
    
    virtual ~MLDepthService();
    
    // Disable copy constructor and assignment operator
    MLDepthService(const MLDepthService&) = delete;
    MLDepthService& operator=(const MLDepthService&) = delete;
    
    /**
     * @brief Start asynchronous processing
     * @return Start success status
     */
    bool start();
    
    /**
     * @brief Stop asynchronous processing
     */
    void stop();
    
    /**
     * @brief Submit frame for depth estimation
     * @param request Frame request with RGB image and metadata
     * @return Submission success status
     */
    bool submitFrame(const FrameRequest& request);
    
    /**
     * @brief Get depth result for timestamp with tolerance
     * @param target_timestamp Target timestamp to match
     * @param tolerance_ms Acceptable time difference in milliseconds
     * @return Depth result if available within tolerance
     */
    std::optional<DepthResult> getDepthResult(double target_timestamp, double tolerance_ms = 50.0);
    
    /**
     * @brief Get performance statistics
     * @return Current performance metrics
     */
    PerformanceStats getPerformanceStats() const;
    
    /**
     * @brief Change inference strategy
     * @param strategy New inference strategy
     * @param snapshot_interval Interval for snapshot mode (ignored for other strategies)
     */
    void setStrategy(InferenceStrategy strategy, int snapshot_interval = 5);
    
    /**
     * @brief Get current inference strategy
     * @return Current strategy
     */
    InferenceStrategy getCurrentStrategy() const { return current_strategy_; }
    
    /**
     * @brief Get current snapshot interval
     * @return Snapshot interval for SNAPSHOT_MODE strategy
     */
    int getSnapshotInterval() const { return snapshot_interval_; }
    
    /**
     * @brief Check if service is active
     * @return True if processing is active
     */
    bool isActive() const { return processing_active_.load(); }
    
    /**
     * @brief Clear result cache
     */
    void clearCache();

private:
    /**
     * @brief Main processing loop (runs in separate thread)
     */
    void processingLoop();
    
    /**
     * @brief Check if frame should be processed based on current strategy
     * @param frame_id Frame identifier
     * @param is_keyframe Whether the frame is a keyframe
     * @return True if frame should be processed
     */
    bool shouldProcessFrame(int frame_id, bool is_keyframe);
    
    /**
     * @brief Update average inference time using exponential moving average
     * @param new_time_ms New inference time measurement
     */
    void updateAverageInferenceTime(float new_time_ms);
    
    /**
     * @brief Calculate depth confidence metric
     * @param depth_map Input depth map
     * @return Confidence score [0.0, 1.0]
     */
    float calculateDepthConfidence(const cv::Mat& depth_map) const;
    
    /**
     * @brief Clean up old results from cache
     */
    void cleanupCache();
    
    /**
     * @brief Get current timestamp
     * @return Current system timestamp
     */
    double getCurrentTimestamp() const;
    
    /**
     * @brief Get strategy name as string
     * @param strategy Strategy enum value
     * @return Strategy name string
     */
    static std::string getStrategyName(InferenceStrategy strategy);
};

} // namespace ML
} // namespace HSLAM 