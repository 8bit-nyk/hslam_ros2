#include "util/DepthLogger.h"
#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "util/settings.h"  // Add this include for setting_debugout_runquiet

namespace HSLAM {

// Static member initialization
std::ofstream DepthLogger::log_file;
std::string DepthLogger::log_file_path;
bool DepthLogger::debug_mode = false;
std::chrono::steady_clock::time_point DepthLogger::start_time;

void DepthLogger::initialize(const std::string& results_dir, const std::string& dataset_name) {
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
    log_file_path = logs_dir + "/depthLog.txt";
    
    // Open log file
    log_file.open(log_file_path);
    if (!log_file.is_open()) {
        std::cerr << "WARNING: Could not open depth log file: " << log_file_path << std::endl;
        return;
    }
    
    start_time = std::chrono::steady_clock::now();
    
    // Write header
    log_file << "HSLAM Depth Integration Log" << std::endl;
    log_file << "Dataset: " << dataset_name << std::endl;
    log_file << "Started: " << getTimestamp() << std::endl;
    log_file << "========================================" << std::endl;
    log_file << std::endl;
    
    std::cout << "DepthLogger: Initialized with log file: " << log_file_path << std::endl;
}

void DepthLogger::logIntegrationStats(int external_pixels, int fused_pixels, 
                                     int total_pixels, float integration_rate,
                                     const std::string& component) {
    std::string timestamp = getTimestamp();
    std::string rate_str = formatRate(integration_rate);
    
    // Console output (essential statistics)
    if (!setting_debugout_runquiet) {
        printf("Depth Integration [%s]: External=%d, Fused=%d, Total=%d, Rate=%s\n",
               component.c_str(), external_pixels, fused_pixels, total_pixels, rate_str.c_str());
    }
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] " << component << " Integration Stats:" << std::endl;
        log_file << "  External Depth Pixels: " << external_pixels << std::endl;
        log_file << "  Fused Pixels: " << fused_pixels << std::endl;
        log_file << "  Total Valid Pixels: " << total_pixels << std::endl;
        log_file << "  Integration Rate: " << rate_str << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void DepthLogger::logPointCreation(int total_points, int depth_integrated_points) {
    std::string timestamp = getTimestamp();
    float integration_percentage = (total_points > 0) ? 
        (100.0f * depth_integrated_points / total_points) : 0.0f;
    
    // Console output (essential statistics)
    if (!setting_debugout_runquiet) {
        printf("Points Created: %d total, %d with depth integration (%.1f%%)\n",
               total_points, depth_integrated_points, integration_percentage);
    }
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] Point Creation Stats:" << std::endl;
        log_file << "  Total Points: " << total_points << std::endl;
        log_file << "  Depth Integrated Points: " << depth_integrated_points << std::endl;
        log_file << "  Integration Percentage: " << std::fixed << std::setprecision(1) 
                 << integration_percentage << "%" << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void DepthLogger::logPerformance(int processed_frames, double avg_ms_per_frame, double fps) {
    std::string timestamp = getTimestamp();
    
    // Console output (essential statistics)
    if (!setting_debugout_runquiet) {
        printf("RGB-D Performance: %d frames, %.2fms/frame, %.1f fps\n",
               processed_frames, avg_ms_per_frame, fps);
    }
    
    // File logging (detailed)
    if (log_file.is_open()) {
        log_file << "[" << timestamp << "] Performance Summary:" << std::endl;
        log_file << "  Processed Frames: " << processed_frames << std::endl;
        log_file << "  Average Time per Frame: " << std::fixed << std::setprecision(2) 
                 << avg_ms_per_frame << " ms" << std::endl;
        log_file << "  Frames per Second: " << std::fixed << std::setprecision(1) 
                 << fps << " fps" << std::endl;
        log_file << std::endl;
        log_file.flush();
    }
}

void DepthLogger::debugLog(const std::string& message, const std::string& component) {
    if (!debug_mode || setting_debugout_runquiet) return;
    
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

void DepthLogger::logDepthPoint(int point_id, int u, int v, float depth, 
                               float idepth, float idepth_min, float idepth_max) {
    if (!debug_mode) return;
    
    std::string message = "Depth-integrated point " + std::to_string(point_id) + 
                         ": (u=" + std::to_string(u) + ", v=" + std::to_string(v) + 
                         ") depth=" + std::to_string(depth) + "m, idepth=" + 
                         std::to_string(idepth) + ", range=[" + 
                         std::to_string(idepth_min) + ", " + std::to_string(idepth_max) + "]";
    
    debugLog(message, "PointCreation");
}

void DepthLogger::logFrameProcessing(int frame_id, int rgb_cols, int rgb_rows,
                                   int depth_cols, int depth_rows, double timestamp) {
    if (!debug_mode) return;
    
    std::string message = "Frame " + std::to_string(frame_id) + ": RGB " + 
                         std::to_string(rgb_cols) + "x" + std::to_string(rgb_rows) + 
                         ", Depth " + std::to_string(depth_cols) + "x" + std::to_string(depth_rows) + 
                         ", timestamp=" + std::to_string(timestamp);
    
    debugLog(message, "FrameProcessing");
}

void DepthLogger::logExternalDepthDetails(int new_pixels, int fused_pixels) {
    if (!debug_mode) return;
    
    std::string message = "External depth integration: " + std::to_string(new_pixels) + 
                         " new pixels, " + std::to_string(fused_pixels) + " fused pixels";
    
    debugLog(message, "ExternalDepth");
}

void DepthLogger::finalize() {
    if (log_file.is_open()) {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        
        log_file << "==========================================" << std::endl;
        log_file << "Log Session Duration: " << duration.count() << " seconds" << std::endl;
        log_file << "Ended: " << getTimestamp() << std::endl;
        log_file << "==========================================" << std::endl;
        
        log_file.close();
        std::cout << "DepthLogger: Finalized log file: " << log_file_path << std::endl;
    }
}

void DepthLogger::setDebugMode(bool enabled) {
    debug_mode = enabled;
    if (!setting_debugout_runquiet) {
        std::cout << "DepthLogger: Debug mode " << (enabled ? "enabled" : "disabled") << std::endl;
    }
}

std::string DepthLogger::getLogFilePath() {
    return log_file_path;
}

std::string DepthLogger::getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    return ss.str();
}

std::string DepthLogger::formatRate(float rate) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << (rate * 100.0f) << "%";
    return ss.str();
}

} // namespace HSLAM 