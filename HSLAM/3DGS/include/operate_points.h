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

#include <torch/torch.h>

#include <memory>

#include "rasterize_points.h"

void transformPoints(
    torch::Tensor& points,
    torch::Tensor& transformmatrix);

void scaleAndTransformThenMarkVisiblePoints(
    torch::Tensor& points,
    torch::Tensor& rots,
    torch::Tensor& point_not_transformed_mask,
    torch::Tensor& point_unstable_mask,
    torch::Tensor& transformmatrix,
    torch::Tensor& viewmatrix,
    torch::Tensor& projmatrix,
    int& num_transformed,
    const float scale = 1.0f);
