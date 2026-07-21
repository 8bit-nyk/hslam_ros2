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

#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <tuple>
#include <filesystem>

#include "types.h"
#include "camera.h"
#include "point3d.h"
#include "gaussian_parameters.h"
#include "gaussian_model.h"
#include "gaussian_keyframe.h"



class GaussianScene
{
public:
    GaussianScene(
        GaussianModelParams& args,
        int load_iteration = 0,
        bool shuffle = true,
        std::vector<float> resolution_scales = {1.0f});

public:
    void addCamera(Camera& camera);
    Camera& getCamera(camera_id_t cameraId);

    void addKeyframe(std::shared_ptr<GaussianKeyframe> new_kf, bool* shuffled);
    std::shared_ptr<GaussianKeyframe> getKeyframe(std::size_t fid);
    std::map<std::size_t, std::shared_ptr<GaussianKeyframe>>& keyframes();
    std::map<std::size_t, std::shared_ptr<GaussianKeyframe>> getAllKeyframes();

    void cachePoint3D(point3D_id_t point3D_id, Point3D& point3d);
    Point3D& getPoint3D(point3D_id_t point3DId);
    void clearCachedPoint3D();

    std::tuple<Eigen::Vector3f, float> getNerfppNorm();

public:
    float cameras_extent_; ///< scene_info.nerf_normalization["radius"]

    int loaded_iter_;

    std::map<camera_id_t, Camera> cameras_;
    std::map<std::size_t, std::shared_ptr<GaussianKeyframe>> keyframes_;
    std::map<std::size_t, std::shared_ptr<GaussianKeyframe>> keyframes_deleted_;
    std::map<point3D_id_t, Point3D> cached_point_cloud_;

protected:
    std::mutex mutex_kfs_;
};
