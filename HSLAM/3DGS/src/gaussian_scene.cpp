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



#include "3DGS/include/gaussian_scene.h"



/**
 * @brief Construct a new Gaussian Scene:: Gaussian Scene object
 * 
 * The Gaussian Scene class organizes the inputs to the Gaussian Model
 * 
 * @param args 
 * @param load_iteration 
 * @param shuffle 
 * @param resolution_scales 
 */
GaussianScene::GaussianScene(
    GaussianModelParams& args,
    int load_iteration,
    bool shuffle,
    std::vector<float> resolution_scales)
{
    if (load_iteration)
    {
        this->loaded_iter_ = load_iteration;
        std::cout << "Loading trained model at iteration " << load_iteration << std::endl;
    }
}


void GaussianScene::addCamera(Camera& camera)
{
    this->cameras_.emplace(camera.camera_id_, camera);
}

Camera& GaussianScene::getCamera(camera_id_t cameraId)
{
    return this->cameras_[cameraId];
}


void GaussianScene::addKeyframe(std::shared_ptr<GaussianKeyframe> new_kf, bool* shuffled)
{
    std::unique_lock<std::mutex> lock_kfs(this->mutex_kfs_);
    this->keyframes_.emplace(new_kf->fid_, new_kf);
    *shuffled = false;
}

std::shared_ptr<GaussianKeyframe>
GaussianScene::getKeyframe(std::size_t fid)
{
    std::unique_lock<std::mutex> lock_kfs(this->mutex_kfs_);
    if (this->keyframes_.find(fid) != this->keyframes_.end())
        return this->keyframes_[fid];
    else
        return nullptr;
}

std::map<std::size_t, std::shared_ptr<GaussianKeyframe>>&
GaussianScene::keyframes()
{
    return this->keyframes_;
}

std::map<std::size_t, std::shared_ptr<GaussianKeyframe>>
GaussianScene::getAllKeyframes()
{
    std::unique_lock<std::mutex> lock_kfs(this->mutex_kfs_);
    return this->keyframes_;
}


void GaussianScene::cachePoint3D(point3D_id_t point3D_id, Point3D& point3d)
{
    this->cached_point_cloud_[point3D_id] = point3d;
}

Point3D& GaussianScene::getPoint3D(point3D_id_t point3DId)
{
    if (this->cached_point_cloud_.find(point3DId) == this->cached_point_cloud_.end())
        std::cout << "GaussianScene::getPoint3D(" << point3DId << ") invalid point Id, creating new point." << std::endl;

    return this->cached_point_cloud_[point3DId];
}

void GaussianScene::clearCachedPoint3D()
{
    this->cached_point_cloud_.clear();
}


/**
 * @brief 
 * 
 * Used to help in calculating the camera extents
 * 
 * @return std::tuple<Eigen::Vector3f, float> 
 */
std::tuple<Eigen::Vector3f, float>
GaussianScene::getNerfppNorm()
{
    std::vector<Eigen::Matrix<float, 3, 1>> cam_centers;
    auto kfs = this->getAllKeyframes();
    std::size_t n_cams = kfs.size();
    cam_centers.reserve(n_cams);
    for (auto& kfit : kfs) {
        auto pkf = kfit.second;
        auto W2C = pkf->getWorld2View2();
        auto C2W = W2C.inverse();
        auto cam_center = C2W.block<3, 1>(0, 3);
        cam_centers.emplace_back(cam_center);
    }

    // get_center_and_diag(cam_centers)
    Eigen::Vector3f avg_cam_center;
    avg_cam_center.setZero();
    for (const auto& cam_center : cam_centers) {
        avg_cam_center.x() += cam_center.x();
        avg_cam_center.y() += cam_center.y();
        avg_cam_center.z() += cam_center.z();
    }
    avg_cam_center.x() /= n_cams;
    avg_cam_center.y() /= n_cams;
    avg_cam_center.z() /= n_cams;

    float max_dist = 0.0f; // diagonal
    for (std::size_t cam_idx = 0; cam_idx < n_cams; ++cam_idx) {
        float dist = (cam_centers[cam_idx] - avg_cam_center).norm();
        if (dist > max_dist)
            max_dist = dist;
    }

    float radius = max_dist * 1.1;

    Eigen::Vector3f translate = -avg_cam_center;

    return std::make_tuple(translate, radius);
}