/*
 * Copyright (C) 2023, Inria
 * GRAPHDECO research group, https://team.inria.fr/graphdeco
 * All rights reserved.
 *
 * This software is free for non-commercial, research and evaluation use 
 * under the terms of the LICENSE.md file.
 *
 * For inquiries contact  george.drettakis@inria.fr
 * 
 * This file is Derivative Works of Gaussian Splatting,
 * created by Yan Song Hu in 2024,
 * as part of GSO.
 */



#include "3DGS/include/gaussian_trainer.h"



GaussianTrainer::GaussianTrainer()
{}

void GaussianTrainer::trainingReport(
    int iteration,
    int num_iterations,
    torch::Tensor& Ll1,
    torch::Tensor& loss,
    float ema_loss_for_log,
    std::function<torch::Tensor(torch::Tensor&, torch::Tensor&)> l1_loss,
    int64_t elapsed_time,
    GaussianModel& gaussians,
    GaussianScene& scene,
    GaussianPipelineParams& pipe,
    torch::Tensor& background)
{
    std::cout << std::fixed << std::setprecision(8)
              << "Training iteration " << iteration << "/" << num_iterations
              << ", time elapsed:" << elapsed_time / 1000.0 << "s"
              << ", ema_loss:" << ema_loss_for_log
              << ", num_points:" << gaussians.xyz_.size(0)
              << std::endl;
}