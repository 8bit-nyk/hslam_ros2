#include "util/FPSLogger.h"
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace HSLAM {

// Static member initialization
std::ofstream FPSLogger::log_file;
std::string FPSLogger::log_file_path;
bool FPSLogger::debug_mode = false;
std::chrono::steady_clock::time_point FPSLogger::start_time;

void FPSLogger::initialize(const std::string& results_dir, const std::string& dataset_name) {
    // Create logs directory if it doesn't exist (same as other HSLAM logs)
    std::string logs_dir = "logs";
    
    struct stat st = {0};
    if (stat(logs_dir.c_str(), &st) == -1) {
        int result = mkdir(logs_dir.c_str(), 0700);
        if (result != 0) {
            std::cerr << "WARNING: Could not create logs directory: " << logs_dir << std::endl;
        }
    }
    
    // Generate log file path
    log_file_path = logs_dir + "/fpsLog.txt";
    
    // Open log file
    log_file.open(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "WARNING: Could not open FPS log file: " << log_file_path << std::endl;
        return;
    }
    
    start_time = std::chrono::steady_clock::now();
    
    // Write header
    log_file << "HSLAM FPS and ML Performance Log" << std::endl;
    log_file << "Dataset: " << dataset_name << std::endl;
    log_file << "Started: " << getTimestamp() << std::endl;
    log_file << "========================================" << std::endl;
    log_file << std::endl;
    
    std::cout << "FPSLogger: Initialized with log file: " << log_file_path << std::endl;
}

void FPSLogger::logSlamPerformance(int processed_frames, double avg_ms_per_frame, 
                                  double fps, const std::string& component) {
    // Recompute if caller passed invalid metrics (e.g. INF / NAN → fps 0.0)
    if (!std::isfinite(avg_ms_per_frame) || avg_ms_per_frame <= 0.0 ||
        !std::isfinite(fps)              || fps              <= 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (processed_frames > 0 && elapsed_ms > 0.0)
        {
            avg_ms_per_frame = elapsed_ms / processed_frames;
            fps              = 1000.0 / avg_ms_per_frame;
        }
    }
    std::string timestamp = getTimestamp();
    
    // Console output (essential statistics)
    printf("SLAM Performance [%s]: %d frames, %.2fms/frame, %.1f fps\n",
           component.c_str(), processed_frames, avg_ms_per_frame, fps);
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] " << component << " SLAM Performance:" << std::endl;
        log_file << "  Processed Frames: " << processed_frames << std::endl;
        log_file << "  Average Time per Frame: " << std::fixed << std::setprecision(2) 
                 << avg_ms_per_frame << " ms" << std::endl;
        log_file << "  Frames per Second: " << std::fixed << std::setprecision(1) 
                 << fps << " fps" << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void FPSLogger::logSlamPerformance(int processed_frames, int keyframes,
                                  double avg_ms_per_frame, double fps, 
                                  const std::string& component) {
    // Recompute if invalid values supplied
    if (!std::isfinite(avg_ms_per_frame) || avg_ms_per_frame <= 0.0 ||
        !std::isfinite(fps)              || fps              <= 0.0)
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (processed_frames > 0 && elapsed_ms > 0.0)
        {
            avg_ms_per_frame = elapsed_ms / processed_frames;
            fps              = 1000.0 / avg_ms_per_frame;
        }
    }
    std::string timestamp = getTimestamp();
    
    // Console output with keyframe information
    printf("SLAM Performance [%s]: %d frames, %d keyframes, %.2fms/frame, %.1f fps\n",
           component.c_str(), processed_frames, keyframes, avg_ms_per_frame, fps);
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] " << component << " SLAM Performance:" << std::endl;
        log_file << "  Processed Frames: " << processed_frames << std::endl;
        log_file << "  Keyframes Created: " << keyframes << std::endl;
        log_file << "  Average Time per Frame: " << std::fixed << std::setprecision(2) 
                 << avg_ms_per_frame << " ms" << std::endl;
        log_file << "  Frames per Second: " << std::fixed << std::setprecision(1) 
                 << fps << " fps" << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void FPSLogger::logMLPerformance(int total_keyframes, int successful_keyframes,
                                float avg_inference_time_ms, float ml_utilization) {
    std::string timestamp = getTimestamp();
    std::string utilization_str = formatRate(ml_utilization);
    float success_rate = (total_keyframes > 0) ? 
        (100.0f * successful_keyframes / total_keyframes) : 0.0f;
    
    // Console output (essential statistics)
    printf("ML Performance: %d/%d keyframes with ML depth (%.1f%%), %.2fms avg, %s utilization\n",
           successful_keyframes, total_keyframes, success_rate, 
           avg_inference_time_ms, utilization_str.c_str());
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] ML Keyframe Performance:" << std::endl;
        log_file << "  Total Keyframes: " << total_keyframes << std::endl;
        log_file << "  Successful Keyframes: " << successful_keyframes << std::endl;
        log_file << "  Success Rate: " << std::fixed << std::setprecision(1) 
                 << success_rate << "%" << std::endl;
        log_file << "  Average Inference Time: " << std::fixed << std::setprecision(2) 
                 << avg_inference_time_ms << " ms" << std::endl;
        log_file << "  ML Depth Utilization: " << utilization_str << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void FPSLogger::logFrameTiming(int frame_id, float slam_time_ms, 
                              float ml_time_ms, float total_time_ms) {
    if (!debug_mode) return;
    
    std::string timestamp = getTimestamp();
    
    // Console output (debug only)
    if (ml_time_ms > 0) {
        printf("Frame %d: SLAM=%.1fms, ML=%.1fms, Total=%.1fms\n",
               frame_id, slam_time_ms, ml_time_ms, total_time_ms);
    } else {
        printf("Frame %d: SLAM=%.1fms, Total=%.1fms\n",
               frame_id, slam_time_ms, total_time_ms);
    }
    
    // File logging (debug details)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] Frame " << frame_id << " Timing:" << std::endl;
        log_file << "  SLAM Processing: " << std::fixed << std::setprecision(2) 
                 << slam_time_ms << " ms" << std::endl;
        if (ml_time_ms > 0) {
            log_file << "  ML Inference: " << std::fixed << std::setprecision(2) 
                     << ml_time_ms << " ms" << std::endl;
        }
        log_file << "  Total Frame Time: " << std::fixed << std::setprecision(2) 
                 << total_time_ms << " ms" << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void FPSLogger::debugLog(const std::string& message, const std::string& component) {
    if (!debug_mode) return;
    
    std::string timestamp = getTimestamp();
    std::string prefix = component.empty() ? "" : "[" + component + "] ";
    
    // Console output (debug only)
    printf("DEBUG %s%s\n", prefix.c_str(), message.c_str());
    
    // File logging (debug details)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] DEBUG " << prefix << message << std::endl;
        log_file.flush();
    }
}

void FPSLogger::logMLInitialization(const std::string& model_path, 
                                   float initialization_time_ms, bool success) {
    if (!debug_mode) return;
    
    std::string message = "ML system initialization: " + model_path + 
                         " (" + std::to_string(initialization_time_ms) + "ms) - " +
                         (success ? "SUCCESS" : "FAILED");
    
    debugLog(message, "MLInit");
}

void FPSLogger::logPerformanceComparison(double monocular_fps, double ml_enhanced_fps,
                                        float performance_impact) {
    if (!debug_mode) return;
    
    std::string message = "Performance comparison: Monocular=" + 
                         std::to_string(monocular_fps) + "fps, ML-Enhanced=" +
                         std::to_string(ml_enhanced_fps) + "fps, Impact=" +
                         std::to_string(performance_impact) + "%";
    
    debugLog(message, "Performance");
}

void FPSLogger::logInitializationPerformance(
    double total_time_ms, double ml_time_ms, double tracking_time_ms,
    const std::string& scale_method, float scale_factor, 
    int points_initialized, float ml_confidence) {
    
    std::string timestamp = getTimestamp();
    
    // Console output (brief summary)
    printf("\nInitialization Complete: %s scale (%.1fms total, %d points)\n",
           scale_method.c_str(), total_time_ms, points_initialized);
    
    // File logging (detailed, human-readable format)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] Initialization Performance:" << std::endl;
        log_file << "  Total Time: " << std::fixed << std::setprecision(1) 
                 << total_time_ms << " ms" << std::endl;
        
        if (ml_time_ms > 0) {
            log_file << "  ML Depth Processing: " << std::fixed << std::setprecision(1)
                     << ml_time_ms << " ms (" 
                     << std::setprecision(1) << (ml_time_ms/total_time_ms)*100 
                     << "%)" << std::endl;
            log_file << "  ML Confidence: " << std::fixed << std::setprecision(2)
                     << ml_confidence << std::endl;
        }
        
        log_file << "  Tracking/Triangulation: " << std::fixed << std::setprecision(1)
                 << tracking_time_ms << " ms" << std::endl;
        log_file << "  Scale Method: " << scale_method << std::endl;
        log_file << "  Scale Factor: " << std::fixed << std::setprecision(4) 
                 << scale_factor << std::endl;
        log_file << "  Points Initialized: " << points_initialized << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void FPSLogger::finalize() {
    if (log_file.is_open()) {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        
        log_file << "==========================================" << std::endl;
        log_file << "Log Session Duration: " << duration.count() << " seconds" << std::endl;
        log_file << "Ended: " << getTimestamp() << std::endl;
        log_file << "==========================================" << std::endl;
        
        log_file.close();
        std::cout << "FPSLogger: Finalized log file: " << log_file_path << std::endl;
    }
}

void FPSLogger::setDebugMode(bool enabled) {
    debug_mode = enabled;
    std::cout << "FPSLogger: Debug mode " << (enabled ? "enabled" : "disabled") << std::endl;
}

std::string FPSLogger::getLogFilePath() {
    return log_file_path;
}

std::string FPSLogger::getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    return ss.str();
}

std::string FPSLogger::formatRate(float rate) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << (rate * 100.0f) << "%";
    return ss.str();
}

} // namespace HSLAM 