/**
 * This file is part of GSO
 *
 * Copyright (C) 2024-2025 Yan Song Hu, University of Waterloo.
 * 
 * GSO is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GSO is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Photo-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 * 
 * This work is modified from the original work of Photo-SLAM by Longwei Li, Hui Cheng, Huajian Huang, and Sai-Kit Yeung
 * This work is a derivative work of gaussian-splatting by Inria and the Max Planck Institut for Informatik (MPII)
 */



#pragma once

#include <memory>

#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

// gd -dev -17june2026
// #include "thirdparty/Sophus/sophus/se3.hpp"
#include "sophus/se3.hpp"
// gd -dev -17june2026

#include "types.h"
#include "camera.h"
#include "general_utils.h"
#include "graphics_utils.h"
#include "tensor_utils.h"



class GaussianKeyframe
{
public:
    GaussianKeyframe() {}

    GaussianKeyframe(std::size_t fid, int creation_iter = 0)
        : fid_(fid), creation_iter_(creation_iter) {}

    void setPose(
        const double qw,
        const double qx,
        const double qy,
        const double qz,
        const double tx,
        const double ty,
        const double tz);
    
    void setPose(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d& t);

    Sophus::SE3d getPose();
    Sophus::SE3f getPosef();

    void setCameraParams(const Camera& camera);

    void computeTransformTensors();

    Eigen::Matrix4f getWorld2View2(
        const Eigen::Vector3f& trans = {0.0f, 0.0f, 0.0f},
        float scale = 1.0f);

    torch::Tensor getProjectionMatrix(
        float znear,
        float zfar,
        float fovX,
        float fovY,
        torch::DeviceType device_type = torch::kCUDA);

    int getCurrentGausPyramidLevel();

public:
    std::size_t fid_;
    int creation_iter_;
    int remaining_times_of_use_ = 0;

    bool set_camera_ = false;

    camera_id_t camera_id_;
    int camera_model_id_ = 0;

    std::string img_filename_;
    cv::Mat img_undist_;
    torch::Tensor original_image_; ///< image
    int image_width_;              ///< image
    int image_height_;             ///< image

    int num_gaus_pyramid_sub_levels_;
    std::vector<int> gaus_pyramid_times_of_use_;
    std::vector<std::size_t> gaus_pyramid_width_;            ///< gaus_pyramid image
    std::vector<std::size_t> gaus_pyramid_height_;           ///< gaus_pyramid image
    std::vector<torch::Tensor> gaus_pyramid_original_image_; ///< gaus_pyramid image

    std::vector<float> intr_; ///< intrinsics

    float FoVx_; ///< intrinsics
    float FoVy_; ///< intrinsics

    bool set_pose_ = false;
    bool set_projection_matrix_ = false;

    // Transform of the keyframe
    Eigen::Quaterniond R_quaternion_;  ///< extrinsics
    Eigen::Vector3d t_;                ///< extrinsics
    Sophus::SE3d Tcw_;                 ///< extrinsics

    float zfar_ = 100.0f;
    float znear_ = 0.01f;

    // Translation and Scale of the Global Frame
    Eigen::Vector3f g_trans_ = {0.0f, 0.0f, 0.0f};
    float g_scale_ = 1.0f;

    // Tensor forms of the camera transformations
    torch::Tensor world_view_transform_;    ///< transform tensors
    torch::Tensor projection_matrix_;       ///< transform tensors
    torch::Tensor full_proj_transform_;     ///< transform tensors
    torch::Tensor camera_center_;           ///< transform tensors
};
