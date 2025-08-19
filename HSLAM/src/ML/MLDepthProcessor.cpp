#include "MLDepthProcessor.h"
#include <iostream>
#include <chrono>

namespace HSLAM {
namespace ML {

MLDepthProcessor::MLDepthProcessor(const MLInference::InferenceConfig& config) 
    : config_(config) {
    // Create the ML inference engine with provided configuration
    ml_inference_ = std::make_unique<MLInference>(config_);
}

bool MLDepthProcessor::initialize() {
    if (!ml_inference_) {
        printf("ERROR: MLDepthProcessor - ML inference not created\n");
        return false;
    }
    
    printf("MLDepthProcessor: Initializing simplified synchronous processor...\n");
    
    // Initialize the underlying ML inference engine
    bool success = ml_inference_->initialize();
    if (success) {
        printf("MLDepthProcessor: Initialization successful\n");
        printf("  - Model: %s\n", config_.model_path.c_str());
        printf("  - Input size: %dx%d\n", config_.input_width, config_.input_height);
        printf("  - GPU enabled: %s\n", config_.enable_gpu ? "Yes" : "No");
        printf("  - Processing mode: Synchronous (no background threads)\n");
        
    } else {
        printf("ERROR: MLDepthProcessor - Failed to initialize ML inference\n");
    }
    
    return success;
}

std::optional<cv::Mat> MLDepthProcessor::processKeyframe(const cv::Mat& rgb_image) {
    if (!isReady()) {
        printf("WARNING: MLDepthProcessor - Not initialized, cannot process keyframe\n");
        return std::nullopt;
    }
    
    if (rgb_image.empty()) {
        printf("WARNING: MLDepthProcessor - Empty input image\n");
        return std::nullopt;
    }
    
    // Perform synchronous ML inference (37ms typical)
    auto start_time = std::chrono::high_resolution_clock::now();
    
    MLInference::InferenceResult result = ml_inference_->inferDepth(rgb_image);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    float inference_time = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    
    // Update simple statistics
    total_processed_.fetch_add(1);
    
    // For atomic<float>, use load-add-store pattern for C++17 compatibility
    float current_time = total_inference_time_.load();
    while (!total_inference_time_.compare_exchange_weak(current_time, current_time + inference_time)) {
        // Loop until successful update
    }
    
    if (result.success && !result.depth_map.empty()) {
        successful_processed_.fetch_add(1);
        
        // Debug output for successful processing
        double min_depth, max_depth;
        cv::minMaxLoc(result.depth_map, &min_depth, &max_depth);
        
        printf("MLDepthProcessor: Keyframe processed successfully (%.1fms, depth range: %.2f-%.2fm)\n", 
               inference_time, min_depth, max_depth);
        
        return result.depth_map;
    } else {
        printf("WARNING: MLDepthProcessor - Processing failed: %s\n", 
               result.error_message.empty() ? "Unknown error" : result.error_message.c_str());
        return std::nullopt;
    }
}

MLDepthProcessor::ProcessingResult MLDepthProcessor::processKeyframeDetailed(const cv::Mat& rgb_image) {
    ProcessingResult detailed_result;
    
    if (!isReady()) {
        detailed_result.error_message = "Processor not initialized";
        return detailed_result;
    }
    
    if (rgb_image.empty()) {
        detailed_result.error_message = "Empty input image";
        return detailed_result;
    }
    
    // === DEBUG_ML_PHASE1: MLDepthProcessor pre-inference ===
    // printf("DEBUG_ML_PHASE1: MLDepthProcessor::processKeyframeDetailed() entry\n");
    // printf("DEBUG_ML_PHASE1: Input image - size: %dx%d, channels: %d, type: %d\n",
    //        rgb_image.cols, rgb_image.rows, rgb_image.channels(), rgb_image.type());
    // printf("DEBUG_ML_PHASE1: ml_inference_ pointer: %p\n", ml_inference_.get());
    // === END DEBUG_ML_PHASE1 ===
    
    // Perform synchronous ML inference with detailed timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // printf("DEBUG_ML_PHASE1: About to call ml_inference_->inferDepth()...\n");
    MLInference::InferenceResult result = ml_inference_->inferDepth(rgb_image);
    // printf("DEBUG_ML_PHASE1: ml_inference_->inferDepth() returned successfully!\n");
    
    auto end_time = std::chrono::high_resolution_clock::now();
    detailed_result.inference_time_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    
    // Update statistics
    total_processed_.fetch_add(1);
    
    // For atomic<float>, use load-add-store pattern for C++17 compatibility
    float current_time = total_inference_time_.load();
    while (!total_inference_time_.compare_exchange_weak(current_time, current_time + detailed_result.inference_time_ms)) {
        // Loop until successful update
    }
    
    if (result.success && !result.depth_map.empty()) {
        successful_processed_.fetch_add(1);
        
        detailed_result.depth_map = result.depth_map;
        detailed_result.success = true;
        detailed_result.confidence = 1.0f;  // High confidence for successful Metric3D processing
        
        // Add depth quality information
        double min_depth, max_depth;
        cv::minMaxLoc(result.depth_map, &min_depth, &max_depth);
        
        printf("MLDepthProcessor: Detailed processing successful (%.1fms, range: %.2f-%.2fm)\n", 
               detailed_result.inference_time_ms, min_depth, max_depth);
    } else {
        detailed_result.success = false;
        detailed_result.confidence = 0.0f;
        detailed_result.error_message = result.error_message.empty() ? "ML inference failed" : result.error_message;
        
        printf("WARNING: MLDepthProcessor - Detailed processing failed: %s\n", 
               detailed_result.error_message.c_str());
    }
    
    return detailed_result;
}

bool MLDepthProcessor::isReady() const {
    return ml_inference_ && ml_inference_->isInitialized();
}


MLDepthProcessor::SimpleStats MLDepthProcessor::getStats() const {
    SimpleStats stats;
    stats.total_processed = total_processed_.load();
    stats.successful_processed = successful_processed_.load();
    
    if (stats.total_processed > 0) {
        stats.success_rate = (float)stats.successful_processed / stats.total_processed * 100.0f;
        stats.avg_inference_time_ms = total_inference_time_.load() / stats.total_processed;
    }
    
    return stats;
}

void MLDepthProcessor::resetStats() {
    total_processed_.store(0);
    successful_processed_.store(0);
    total_inference_time_.store(0.0f);
}

void MLDepthProcessor::shutdown() {
    printf("MLDepthProcessor: Shutting down...\n");
    
    // Print final statistics
    auto stats = getStats();
    if (stats.total_processed > 0) {
        printf("MLDepthProcessor: Final statistics:\n");
        printf("  - Total processed: %d keyframes\n", stats.total_processed);
        printf("  - Success rate: %.1f%%\n", stats.success_rate);
        printf("  - Average inference time: %.1fms\n", stats.avg_inference_time_ms);
    }
    
    if (ml_inference_) {
        ml_inference_->shutdown();
        ml_inference_.reset();
    }
    
    printf("MLDepthProcessor: Shutdown complete\n");
}

} // namespace ML
} // namespace HSLAM