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



namespace GEN_UTIL
{

inline torch::Tensor inverse_sigmoid(const torch::Tensor &x)
{
    return torch::log(x / (1 - x));
}

inline torch::Tensor build_rotation(torch::Tensor &r)
{
    auto r0 = r.index({torch::indexing::Slice(), 0});
    auto r1 = r.index({torch::indexing::Slice(), 1});
    auto r2 = r.index({torch::indexing::Slice(), 2});
    auto r3 = r.index({torch::indexing::Slice(), 3});
    auto norm = torch::sqrt(r0 * r0 + r1 * r1 + r2 * r2 + r3 * r3);

    auto q = r / norm.unsqueeze(/*dim=*/1);
    r = q.index({torch::indexing::Slice(), 0});
    auto x = q.index({torch::indexing::Slice(), 1});
    auto y = q.index({torch::indexing::Slice(), 2});
    auto z = q.index({torch::indexing::Slice(), 3});

    auto R = torch::zeros({q.size(0), 3, 3}, torch::TensorOptions().device(torch::kCUDA));
    R.select(1, 0).select(1, 0).copy_(1 - 2 * (y * y + z * z));
    R.select(1, 0).select(1, 1).copy_(2 * (x * y - r * z));
    R.select(1, 0).select(1, 2).copy_(2 * (x * z + r * y));
    R.select(1, 1).select(1, 0).copy_(2 * (x * y + r * z));
    R.select(1, 1).select(1, 1).copy_(1 - 2 * (x * x + z * z));
    R.select(1, 1).select(1, 2).copy_(2 * (y * z - r * x));
    R.select(1, 2).select(1, 0).copy_(2 * (x * z - r * y));
    R.select(1, 2).select(1, 1).copy_(2 * (y * z + r * x));
    R.select(1, 2).select(1, 2).copy_(1 - 2 * (x * x + y * y));
    return R;
}

/**
 * @brief A functor that implements an exponential learning rate decay function.
 * 
 * @details Modified from Plenoxels
 *  Continuous learning rate decay function. Adapted from JaxNeRF
 *  The returned rate is lr_init when step=0 and lr_final when step=max_steps, and
 *  is log-linearly interpolated elsewhere (equivalent to exponential decay).
 *  If lr_delay_steps>0 then the learning rate will be scaled by some smooth
 *  function of lr_delay_mult, such that the initial learning rate is
 *  lr_init*lr_delay_mult at the beginning of optimization but will be eased back
 *  to the normal learning rate when steps>lr_delay_steps.
 *  :param conf: config subtree 'lr' or similar
 *  :param max_steps: int, the number of steps during optimization.
 *  :return HoF which takes step as input
 * 
 * @param lr_init The initial learning rate.
 * @param lr_final The final learning rate.
 * @param lr_delay_steps The number of steps to delay the decay.
 * @param lr_delay_mult The delay multiplier.
 * @param max_steps The maximum number of steps.
 *
 * @return The learning rate at the given step.
 */
struct Expon_lr_func {
    float lr_init;
    float lr_final;
    float lr_delay_steps;
    float lr_delay_mult;
    int64_t max_steps;
    Expon_lr_func(float lr_init = 0.f, float lr_final = 1.f, float lr_delay_mult = 1.f, int64_t max_steps = 1000000, float lr_delay_steps = 0.f)
        : lr_init(lr_init),
          lr_final(lr_final),
          lr_delay_mult(lr_delay_mult),
          max_steps(max_steps),
          lr_delay_steps(lr_delay_steps) {}

    float operator()(int64_t step) const {
        if (step < 0 || (lr_init == 0.0 && lr_final == 0.0))
            return 0.0;

        float delay_rate;
        if (lr_delay_steps > 0. && step != 0) {
            delay_rate = lr_delay_mult + (1.0f - lr_delay_mult) * std::sin(M_PI_2f32 * std::clamp(static_cast<float>(step) / lr_delay_steps, 0.0f, 1.0f));
        } else {
            delay_rate = 1.0f;
        }
        float t = std::clamp(static_cast<float>(step) / static_cast<float>(max_steps), 0.0f, 1.0f);
        float log_lerp = std::exp(std::log(lr_init) * (1.f - t) + std::log(lr_final) * t);
        return delay_rate * log_lerp;
    }
};

}
