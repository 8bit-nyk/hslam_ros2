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

#include <Eigen/Core>



class BasicPointCloud
{
public:
    BasicPointCloud() {}
    BasicPointCloud(std::size_t num_points) {
        this->points_.resize(num_points);
        this->colors_.resize(num_points);
        this->normals_.resize(num_points);
    }

public:
    std::vector<Eigen::Vector3f> points_;
    std::vector<Eigen::Vector3f> colors_;
    std::vector<Eigen::Vector3f> normals_;
};
