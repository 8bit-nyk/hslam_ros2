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

#include <Eigen/Core>



class Point3D
{
public:
    Point3D()
        : xyz_(0.0, 0.0, 0.0),
          color_(0.0f, 0.0f, 0.0f),
          color256_(0, 0, 0),
          error_(-1.0)
    {}

public:
    Eigen::Vector3d xyz_;

    Eigen::Matrix<uint8_t, 3, 1> color256_; // not needed if we get color_ directly
    Eigen::Matrix<float, 3, 1> color_;

    double error_;
};
