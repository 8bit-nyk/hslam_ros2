#ifndef FPSLOGGER_H
#define FPSLOGGER_H

#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <vector>

namespace HSLAM {

/**
 * @brief Centralized FPS and ML inference timing logging utility
 * 
 * Provides debug-only logging for repetitive console output while preserving
 * essential performance statistics in a dedicated FPS log file.
 */
class FPSLogger {
public:
    /**
     * @brief Initialize FPS logger
     * 
     * @param results_dir Base results directory
     * @param dataset_name Dataset name for log file naming
     */
    static void initialize(const std::string& results_dir, const std::string& dataset_name);
    
    /**
     * @brief Log essential SLAM performance statistics
     * 
     * @param processed_frames Number of processed frames
     * @param avg_ms_per_frame Average milliseconds per frame
     * @param fps Frames per second
     * @param component Component name (e.g., "SLAM", "ML_Enhanced")
     */
    static void logSlamPerformance(int processed_frames, double avg_ms_per_frame, 
                                  double fps, const std::string& component);
    
    /**
     * @brief Log ML inference performance statistics
     * 
     * @param total_inferences Total ML inferences performed
     * @param successful_inferences Number of successful inferences
     * @param avg_inference_time_ms Average ML inference time in milliseconds
     * @param ml_utilization ML depth utilization rate (0.0-1.0)
     */
    static void logMLPerformance(int total_inferences, int successful_inferences,
                                float avg_inference_time_ms, float ml_utilization);
    
    /**
     * @brief Log frame processing timing details
     * 
     * @param frame_id Frame identifier
     * @param slam_time_ms SLAM processing time in milliseconds
     * @param ml_time_ms ML inference time in milliseconds (0 if not used)
     * @param total_time_ms Total frame processing time in milliseconds
     */
    static void logFrameTiming(int frame_id, float slam_time_ms, 
                              float ml_time_ms, float total_time_ms);
    
    /**
     * @brief Debug-only logging for repetitive console output
     * 
     * @param message Debug message to log
     * @param component Component name
     */
    static void debugLog(const std::string& message, const std::string& component = "");
    
    /**
     * @brief Log ML system initialization details (debug only)
     * 
     * @param model_path Path to ML model
     * @param initialization_time_ms Time taken to initialize ML system
     * @param success Whether initialization was successful
     */
    static void logMLInitialization(const std::string& model_path, 
                                   float initialization_time_ms, bool success);
    
    /**
     * @brief Log performance comparison (debug only)
     * 
     * @param monocular_fps Standard monocular FPS
     * @param ml_enhanced_fps ML-enhanced FPS
     * @param performance_impact Performance impact percentage
     */
    static void logPerformanceComparison(double monocular_fps, double ml_enhanced_fps,
                                        float performance_impact);
    
    /**
     * @brief Close logger and finalize log file
     */
    static void finalize();
    
    /**
     * @brief Set debug mode
     * 
     * @param enabled True to enable debug logging
     */
    static void setDebugMode(bool enabled);
    
    /**
     * @brief Get current log file path
     * 
     * @return Log file path
     */
    static std::string getLogFilePath();

private:
    static std::ofstream log_file;
    static std::string log_file_path;
    static bool debug_mode;
    static std::chrono::steady_clock::time_point start_time;
    
    /**
     * @brief Get current timestamp for logging
     * 
     * @return Timestamp string
     */
    static std::string getTimestamp();
    
    /**
     * @brief Format utilization rate for display
     * 
     * @param rate Utilization rate (0.0-1.0)
     * @return Formatted percentage string
     */
    static std::string formatRate(float rate);
};

} // namespace HSLAM

#endif // FPSLOGGER_H 