/**
* This file is part of DSO, written by Jakob Engel.
* It has been modified by Georges Younes, Daniel Asmar, John Zelek, and Yan Song Hu
*
* Copyright 2024 University of Waterloo and American University of Beirut.
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

 
#include "util/NumType.h"
#include "vector"
#include <math.h>
#include "util/settings.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "IOWrapper/Output3DWrapper.h"
#include <opencv2/opencv.hpp>




namespace HSLAM
{
struct CalibHessian;
struct FrameHessian;
struct PointFrameResidual;

class CoarseTracker {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	CoarseTracker(int w, int h);
	~CoarseTracker();

	bool trackNewestCoarse(
			FrameHessian* newFrameHessian,
			SE3 &lastToNew_out, AffLight &aff_g2l_out,
			int coarsestLvl, Vec5 minResForAbort,
			IOWrap::Output3DWrapper* wrap=0);

	void setCoarseTrackingRef(
			std::vector<FrameHessian*> frameHessians);

	void makeK(
			CalibHessian* HCalib);

	// =================== DEPTH INTEGRATION INTERFACE ===================
	/**
	 * @brief Set external depth image for depth integration
	 * 
	 * @param depth_image External depth image (CV_32FC1 format, values in meters)
	 */
	void setExternalDepthImage(const cv::Mat& depth_image);
	
	/**
	 * @brief Clear external depth image and disable depth integration
	 */
	void clearExternalDepthImage();
	
	/**
	 * @brief Enhanced depth map creation with external depth integration
	 * 
	 * @param frameHessians Active frame hessians for depth map creation
	 */
	void makeCoarseDepthL0Enhanced(std::vector<FrameHessian*> frameHessians);
	
	/**
	 * @brief Statistics structure for depth integration monitoring
	 */
	struct DepthIntegrationStats {
		int pixels_from_map_points;      // Pixels using existing map points
		int pixels_from_external_depth;  // Pixels using external depth
		int pixels_fused;                // Pixels using both sources (fused)
		int total_valid_pixels;          // Total valid pixels in depth map
		float integration_rate;          // Percentage of pixels using external depth
		
		DepthIntegrationStats() : pixels_from_map_points(0), pixels_from_external_depth(0), 
		                         pixels_fused(0), total_valid_pixels(0), integration_rate(0.0f) {}
	};
	
	/**
	 * @brief Get statistics from last depth integration
	 * 
	 * @return DepthIntegrationStats Statistics from last makeCoarseDepthL0Enhanced call
	 */
	DepthIntegrationStats getLastIntegrationStats() const;
	
	/**
	 * @brief Check if external depth image is available
	 * 
	 * @return bool True if external depth image is set and valid
	 */
	bool hasExternalDepth() const;

	bool debugPrint, debugPlot;

	Mat33f K[PYR_LEVELS];
	Mat33f Ki[PYR_LEVELS];
	float fx[PYR_LEVELS];
	float fy[PYR_LEVELS];
	float fxi[PYR_LEVELS];
	float fyi[PYR_LEVELS];
	float cx[PYR_LEVELS];
	float cy[PYR_LEVELS];
	float cxi[PYR_LEVELS];
	float cyi[PYR_LEVELS];
	int w[PYR_LEVELS];
	int h[PYR_LEVELS];

    void debugPlotIDepthMap(float* minID, float* maxID, std::vector<IOWrap::Output3DWrapper*> &wraps);
    void debugPlotIDepthMapFloat(std::vector<IOWrap::Output3DWrapper*> &wraps);

	FrameHessian* lastRef;
	AffLight lastRef_aff_g2l;
	FrameHessian* newFrame;
	int refFrameID;

	// act as pure ouptut
	Vec5 lastResiduals;
	Vec3 lastFlowIndicators;
	double firstCoarseRMSE;
private:


	void makeCoarseDepthL0(std::vector<FrameHessian*> frameHessians);
	void makeCoarseDepthL0Original(std::vector<FrameHessian*> frameHessians);
	float* idepth[PYR_LEVELS];
	float* weightSums[PYR_LEVELS];
	float* weightSums_bak[PYR_LEVELS];


	Vec6 calcResAndGS(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH);
	Vec6 calcRes(int lvl, const SE3 &refToNew, AffLight aff_g2l, float cutoffTH);
	void calcGSSSE(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l);
	void calcGS(int lvl, Mat88 &H_out, Vec8 &b_out, const SE3 &refToNew, AffLight aff_g2l);

	// pc buffers
	float* pc_u[PYR_LEVELS];
	float* pc_v[PYR_LEVELS];
	float* pc_idepth[PYR_LEVELS];
	float* pc_color[PYR_LEVELS];
	int pc_n[PYR_LEVELS];

	// warped buffers
	float* buf_warped_idepth;
	float* buf_warped_u;
	float* buf_warped_v;
	float* buf_warped_dx;
	float* buf_warped_dy;
	float* buf_warped_residual;
	float* buf_warped_weight;
	float* buf_warped_refColor;
	int buf_warped_n;


    std::vector<float*> ptrToDelete;


	Accumulator9 acc;
	
	// =================== DEPTH INTEGRATION INFRASTRUCTURE ===================
	/**
	 * @brief External depth image storage
	 * 
	 * Thread-safe storage for external depth information.
	 * Format: CV_32FC1, values in meters, same dimensions as tracking images.
	 */
	cv::Mat external_depth_image;
	
	/**
	 * @brief Flag indicating external depth availability
	 */
	bool has_external_depth;
	
	/**
	 * @brief Statistics from last depth integration
	 */
	DepthIntegrationStats last_stats;
	
	// =================== DEPTH PROCESSING UTILITIES ===================
	/**
	 * @brief Integrate external depth into level 0 depth map
	 * 
	 * Core method for incorporating external depth information into the
	 * semi-dense depth map creation process.
	 */
	void integrateExternalDepthL0();
	
	/**
	 * @brief Fuse depth sources at specified pyramid level
	 * 
	 * @param level Pyramid level for depth fusion
	 */
	void fuseDepthSources(int level);
	
	/**
	 * @brief Validate external depth value
	 * 
	 * @param depth Depth value to validate (in meters)
	 * @param u Horizontal pixel coordinate
	 * @param v Vertical pixel coordinate
	 * @return bool True if depth value is valid for integration
	 */
	bool validateExternalDepth(float depth, int u, int v);
	
	/**
	 * @brief Calculate confidence weight for depth value
	 * 
	 * @param depth Depth value (in meters)
	 * @param u Horizontal pixel coordinate
	 * @param v Vertical pixel coordinate
	 * @param is_external True if depth is from external source, false if from map points
	 * @return float Confidence weight for depth integration
	 */
	float calculateConfidenceWeight(float depth, int u, int v, bool is_external);
};


class CoarseDistanceMap {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	CoarseDistanceMap(int w, int h);
	~CoarseDistanceMap();

	void makeDistanceMap(
			std::vector<FrameHessian*> frameHessians,
			FrameHessian* frame);

	void makeInlierVotes(
			std::vector<FrameHessian*> frameHessians);

	void makeK( CalibHessian* HCalib);


	float* fwdWarpedIDDistFinal;

	Mat33f K[PYR_LEVELS];
	Mat33f Ki[PYR_LEVELS];
	float fx[PYR_LEVELS];
	float fy[PYR_LEVELS];
	float fxi[PYR_LEVELS];
	float fyi[PYR_LEVELS];
	float cx[PYR_LEVELS];
	float cy[PYR_LEVELS];
	float cxi[PYR_LEVELS];
	float cyi[PYR_LEVELS];
	int w[PYR_LEVELS];
	int h[PYR_LEVELS];

	void addIntoDistFinal(int u, int v);


private:

	PointFrameResidual** coarseProjectionGrid;
	int* coarseProjectionGridNum;
	Eigen::Vector2i* bfsList1;
	Eigen::Vector2i* bfsList2;

	void growDistBFS(int bfsNum);
};

}

