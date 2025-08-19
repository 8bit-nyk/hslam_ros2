#ifndef DEPTHLOGGER_H
#define DEPTHLOGGER_H

#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <vector>

namespace HSLAM {

/**
 * @brief Centralized depth integration logging utility
 * 
 * Provides debug-only logging for repetitive console output while preserving
 * essential statistics in a dedicated depth log file.
 */
class DepthLogger {
public:
    /**
     * @brief Initialize depth logger
     * 
     * @param results_dir Base results directory
     * @param dataset_name Dataset name for log file naming
     */
    static void initialize(const std::string& results_dir, const std::string& dataset_name);
    
    /**
     * @brief Log essential depth integration statistics
     * 
     * @param external_pixels Number of pixels using external depth
     * @param fused_pixels Number of pixels fused with map points
     * @param total_pixels Total valid pixels
     * @param integration_rate Integration rate percentage
     * @param component Component name (e.g., "CoarseTracker", "FullSystem")
     */
    static void logIntegrationStats(int external_pixels, int fused_pixels, 
                                   int total_pixels, float integration_rate,
                                   const std::string& component);
    
    /**
     * @brief Log point creation statistics
     * 
     * @param total_points Total points created
     * @param depth_integrated_points Points with depth integration
     */
    static void logPointCreation(int total_points, int depth_integrated_points);
    
    /**
     * @brief Log performance statistics
     * 
     * @param processed_frames Number of processed frames
     * @param avg_ms_per_frame Average milliseconds per frame
     * @param fps Frames per second
     */
    static void logPerformance(int processed_frames, double avg_ms_per_frame, double fps);
    
    /**
     * @brief Debug-only logging for repetitive console output
     * 
     * @param message Debug message to log
     * @param component Component name
     */
    static void debugLog(const std::string& message, const std::string& component = "");
    
    /**
     * @brief Log depth-integrated point details (debug only)
     * 
     * @param point_id Point identifier
     * @param u Horizontal coordinate
     * @param v Vertical coordinate
     * @param depth Depth value
     * @param idepth Inverse depth
     * @param idepth_min Minimum inverse depth
     * @param idepth_max Maximum inverse depth
     */
    static void logDepthPoint(int point_id, int u, int v, float depth, 
                             float idepth, float idepth_min, float idepth_max);
    
    /**
     * @brief Log frame processing details (debug only)
     * 
     * @param frame_id Frame identifier
     * @param rgb_cols RGB image width
     * @param rgb_rows RGB image height
     * @param depth_cols Depth image width
     * @param depth_rows Depth image height
     * @param timestamp Frame timestamp
     */
    static void logFrameProcessing(int frame_id, int rgb_cols, int rgb_rows,
                                 int depth_cols, int depth_rows, double timestamp);
    
    /**
     * @brief Log external depth integration details (debug only)
     * 
     * @param new_pixels Number of new pixels from external depth
     * @param fused_pixels Number of fused pixels
     */
    static void logExternalDepthDetails(int new_pixels, int fused_pixels);
    
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
    
    /**
     * @brief Log ML vs HSLAM depth comparison data
     * 
     * @param pixel_x X coordinate
     * @param pixel_y Y coordinate  
     * @param ml_depth ML depth value
     * @param hslam_depth HSLAM depth value
     * @param absolute_error Absolute error between depths
     * @param relative_error Relative error percentage
     */
    static void logMLvsHSLAMComparison(float pixel_x, float pixel_y,
                                      float ml_depth, float hslam_depth,
                                      float absolute_error, float relative_error);

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
     * @brief Format integration rate for display
     * 
     * @param rate Integration rate (0.0-1.0)
     * @return Formatted percentage string
     */
    static std::string formatRate(float rate);
};

} // namespace HSLAM

#endif // DEPTHLOGGER_H 