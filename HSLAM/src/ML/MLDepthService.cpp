#include "ML/MLDepthService.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>

namespace HSLAM {
namespace ML {

MLDepthService::MLDepthService(const MLInference::InferenceConfig& config, 
                               InferenceStrategy strategy,
                               int snapshot_interval)
    : current_strategy_(strategy), snapshot_interval_(snapshot_interval) {
    
    try {
        // Initialize ML inference engine
        ml_inference_ = std::make_unique<MLInference>(config);
        if (!ml_inference_->initialize()) {
            throw std::runtime_error("Failed to initialize ML inference engine");
        }
        
        printf("MLDepthService: Initialized with strategy %s\n", 
               getStrategyName(strategy).c_str());
        if (strategy == SNAPSHOT_MODE) {
            printf("MLDepthService: Snapshot interval set to %d frames\n", snapshot_interval);
        }
        
    } catch (const std::exception& e) {
        printf("MLDepthService initialization failed: %s\n", e.what());
        throw;
    }
}

MLDepthService::~MLDepthService() {
    stop();
}

bool MLDepthService::start() {
    if (processing_active_.load()) {
        printf("MLDepthService: Already running\n");
        return true;
    }
    
    if (!ml_inference_ || !ml_inference_->isInitialized()) {
        printf("MLDepthService: Cannot start - ML inference not initialized\n");
        return false;
    }
    
    // Start GPU warm-up in background (Sprint 2 Optimization)
    if (ml_inference_->getConfig().enable_gpu) {
        printf("MLDepthService: Starting background GPU warm-up...\n");
        ml_inference_->startBackgroundWarmup(true);
    }
    
    shutdown_requested_.store(false);
    processing_active_.store(true);
    processing_thread_ = std::thread(&MLDepthService::processingLoop, this);
    
    printf("MLDepthService: Started with strategy %s\n", 
           getStrategyName(current_strategy_).c_str());
    return true;
}

void MLDepthService::stop() {
    if (!processing_active_.load()) {
        return;
    }
    
    printf("MLDepthService: Stopping...\n");
    
    // Signal shutdown and wake up processing thread
    shutdown_requested_.store(true);
    processing_active_.store(false);
    queue_condition_.notify_all();
    
    // Wait for processing thread to finish
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // Clear queues and caches
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::queue<FrameRequest> empty_queue;
        request_queue_.swap(empty_queue);
    }
    
    clearCache();
    
    printf("MLDepthService: Stopped\n");
}

bool MLDepthService::submitFrame(const FrameRequest& request) {
    if (!processing_active_.load()) {
        return false;
    }
    
    // Validate input
    if (request.rgb_image.empty()) {
        printf("MLDepthService: Rejecting empty image\n");
        return false;
    }
    
    // Apply strategy filtering
    frame_counter_++;
    bool should_process = shouldProcessFrame(request.frame_id, request.is_keyframe);
    
    if (!should_process) {
        frames_skipped_.fetch_add(1);
        return true;  // Frame filtered by strategy, but not an error
    }
    
    // Add to processing queue
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // Prevent queue overflow
        if (request_queue_.size() >= MAX_CACHE_SIZE) {
            printf("MLDepthService: Queue overflow, dropping oldest request\n");
            request_queue_.pop();
        }
        
        request_queue_.push(request);
    }
    
    queue_condition_.notify_one();
    return true;
}

std::optional<MLDepthService::DepthResult> MLDepthService::getDepthResult(double target_timestamp, double tolerance_ms) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    double tolerance_seconds = tolerance_ms / 1000.0;
    double best_time_diff = std::numeric_limits<double>::max();
    auto best_result = result_cache_.end();
    
    // Find closest timestamp within tolerance
    for (auto it = result_cache_.begin(); it != result_cache_.end(); ++it) {
        double time_diff = std::abs(it->first - target_timestamp);
        if (time_diff <= tolerance_seconds && time_diff < best_time_diff) {
            best_time_diff = time_diff;
            best_result = it;
        }
    }
    
    if (best_result != result_cache_.end()) {
        DepthResult result = best_result->second;
        result_cache_.erase(best_result);  // Remove used result
        
        // Debug output for successful retrieval
        if (result.valid) {
            printf("MLDepthService: Retrieved depth result (%.1fms ago, %.1fms inference)\n", 
                   best_time_diff * 1000.0, result.inference_time_ms);
        }
        
        return result;
    }
    
    // Debug: If no result found, show cache status
    if (!result_cache_.empty()) {
        printf("MLDepthService: No result for timestamp %.6f (tolerance: %.1fms)\n", target_timestamp, tolerance_ms);
        printf("MLDepthService: Cache contains %zu results with timestamps:\n", result_cache_.size());
        int count = 0;
        for (const auto& entry : result_cache_) {
            if (count < 3) {  // Show first 3 timestamps
                printf("  %.6f (diff: %.1fms)\n", entry.first, std::abs(entry.first - target_timestamp) * 1000.0);
                count++;
            }
        }
    }
    
    return std::nullopt;
}

MLDepthService::PerformanceStats MLDepthService::getPerformanceStats() const {
    PerformanceStats stats;
    stats.successful_inferences = successful_inferences_.load();
    stats.failed_inferences = failed_inferences_.load();
    stats.avg_inference_time_ms = avg_inference_time_ms_.load();
    stats.frames_processed = frames_processed_.load();
    stats.frames_skipped = frames_skipped_.load();
    stats.current_strategy = current_strategy_;
    
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

void MLDepthService::setStrategy(InferenceStrategy strategy, int snapshot_interval) {
    current_strategy_ = strategy;
    snapshot_interval_ = snapshot_interval;
    frame_counter_ = 0;  // Reset counter
    
    printf("MLDepthService: Strategy changed to %s\n", getStrategyName(strategy).c_str());
    if (strategy == SNAPSHOT_MODE) {
        printf("MLDepthService: Snapshot interval set to %d frames\n", snapshot_interval);
    }
}

void MLDepthService::clearCache() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    result_cache_.clear();
}

void MLDepthService::processingLoop() {
    printf("MLDepthService: Processing thread started\n");
    
    while (processing_active_.load() && !shutdown_requested_.load()) {
        FrameRequest request;
        bool has_request = false;
        
        // Get next request from queue
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this] { 
                return !request_queue_.empty() || !processing_active_.load() || shutdown_requested_.load(); 
            });
            
            if (!request_queue_.empty()) {
                request = request_queue_.front();
                request_queue_.pop();
                has_request = true;
            }
        }
        
        if (!has_request || !processing_active_.load() || shutdown_requested_.load()) {
            continue;
        }
        
        // Process frame
        frames_processed_.fetch_add(1);
        auto start_time = std::chrono::steady_clock::now();
        
        auto inference_result = ml_inference_->inferDepth(request.rgb_image);
        
        auto end_time = std::chrono::steady_clock::now();
        float inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
        
        // Performance warning for slow inference
        if (inference_time_ms > 100.0f) {
            printf("MLDepthService: WARNING - Slow inference detected: %.1fms (target: <100ms)\n", inference_time_ms);
        }
        
        // Create result
        DepthResult result;
        result.timestamp = request.timestamp;
        result.frame_id = request.frame_id;
        result.inference_time_ms = inference_time_ms;
        
        if (inference_result.success && !inference_result.depth_map.empty()) {
            result.depth_map = inference_result.depth_map;
            result.valid = true;
            result.confidence = calculateDepthConfidence(inference_result.depth_map);
            successful_inferences_.fetch_add(1);
            
            printf("MLDepthService: Processed frame %d (%.1fms, conf=%.2f)\n", 
                   request.frame_id, inference_time_ms, result.confidence);
        } else {
            result.valid = false;
            result.error_message = inference_result.error_message;
            failed_inferences_.fetch_add(1);
            
            printf("MLDepthService: Failed to process frame %d: %s\n", 
                   request.frame_id, result.error_message.c_str());
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
    
    printf("MLDepthService: Processing thread stopped\n");
}

bool MLDepthService::shouldProcessFrame(int frame_id, bool is_keyframe) {
    bool should_process = false;
    
    switch (current_strategy_) {
        case EVERY_FRAME:
            should_process = true;
            break;
            
        case KEYFRAME_ONLY:
            should_process = is_keyframe;
            if (is_keyframe) {
                printf("MLDepthService: Processing keyframe %d\n", frame_id);
            }
            break;
            
        case SNAPSHOT_MODE:
            should_process = (frame_counter_ % snapshot_interval_ == 0);
            if (should_process) {
                printf("MLDepthService: Processing snapshot frame %d (counter: %d, interval: %d)\n", 
                       frame_id, frame_counter_, snapshot_interval_);
            }
            break;
            
        default:
            printf("MLDepthService: Unknown strategy, defaulting to EVERY_FRAME\n");
            should_process = true;
            break;
    }
    
    if (!should_process) {
        printf("MLDepthService: Skipping frame %d (strategy: %s, keyframe: %s)\n", 
               frame_id, getStrategyName(current_strategy_).c_str(), 
               is_keyframe ? "true" : "false");
    }
    
    return should_process;
}

void MLDepthService::updateAverageInferenceTime(float new_time_ms) {
    // Simple exponential moving average
    float current_avg = avg_inference_time_ms_.load();
    float alpha = 0.1f;  // Smoothing factor
    float new_avg = (current_avg == 0.0f) ? new_time_ms : 
                    (1.0f - alpha) * current_avg + alpha * new_time_ms;
    avg_inference_time_ms_.store(new_avg);
}

float MLDepthService::calculateDepthConfidence(const cv::Mat& depth_map) const {
    if (depth_map.empty()) {
        return 0.0f;
    }
    
    // Calculate confidence based on valid depth percentage
    cv::Mat valid_mask;
    cv::threshold(depth_map, valid_mask, 0.1f, 1.0f, cv::THRESH_BINARY);
    
    if (valid_mask.type() != CV_8UC1) {
        valid_mask.convertTo(valid_mask, CV_8UC1, 255.0);
    }
    
    int valid_pixels = cv::countNonZero(valid_mask);
    int total_pixels = depth_map.rows * depth_map.cols;
    
    if (total_pixels == 0) {
        return 0.0f;
    }
    
    return (float)valid_pixels / total_pixels;
}

void MLDepthService::cleanupCache() {
    // Remove results older than timeout
    double current_time = getCurrentTimestamp();
    
    auto it = result_cache_.begin();
    while (it != result_cache_.end()) {
        if (current_time - it->first > CACHE_TIMEOUT_SECONDS) {
            it = result_cache_.erase(it);
        } else {
            ++it;
        }
    }
    
    // Limit cache size
    while (result_cache_.size() > MAX_CACHE_SIZE) {
        result_cache_.erase(result_cache_.begin());
    }
}

double MLDepthService::getCurrentTimestamp() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

std::string MLDepthService::getStrategyName(InferenceStrategy strategy) {
    switch (strategy) {
        case EVERY_FRAME: return "every_frame";
        case KEYFRAME_ONLY: return "keyframe_only";
        case SNAPSHOT_MODE: return "snapshot_mode";
        default: return "unknown";
    }
}

} // namespace ML
} // namespace HSLAM 