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



#include "3DGS/include/gaussian_rasterizer.h"



// ----- GaussianRasterizerFunction -----
/**
 * @brief Calls the C++/CUDA rasterizer
 *
 * @param ctx
 * @param means3D
 * @param means2D
 * @param sh
 * @param colors_precomp
 * @param opacities
 * @param scales
 * @param rotations
 * @param cov3Ds_precomp
 * @param raster_settings
 * @return torch::autograd::tensor_list
 */
torch::autograd::tensor_list
GaussianRasterizerFunction::forward(
    torch::autograd::AutogradContext *ctx,
    torch::Tensor means3D,
    torch::Tensor means2D,
    torch::Tensor means2D_densify,
    torch::Tensor sh,
    torch::Tensor colors_precomp,
    torch::Tensor opacities,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3Ds_precomp,
    GaussianRasterizationSettings raster_settings)
{
    // Invoke C++/CUDA rasterizer
    auto [num_rendered, color, radii, geomBuffer, binningBuffer, imgBuffer] =
        RasterizeGaussiansCUDA(
            raster_settings.bg_,
            means3D,
            colors_precomp,
            opacities,
            scales,
            rotations,
            raster_settings.scale_modifier_,
            cov3Ds_precomp,
            raster_settings.viewmatrix_,
            raster_settings.projmatrix_,
            raster_settings.tanfovx_,
            raster_settings.tanfovy_,
            raster_settings.image_height_,
            raster_settings.image_width_,
            sh,
            raster_settings.sh_degree_,
            raster_settings.campos_,
            raster_settings.prefiltered_);

    // Keep relevant tensors for backward
    ctx->saved_data["num_rendered"] = num_rendered;
    ctx->saved_data["scale_modifier"] = raster_settings.scale_modifier_;
    ctx->saved_data["tanfovx"] = raster_settings.tanfovx_;
    ctx->saved_data["tanfovy"] = raster_settings.tanfovy_;
    ctx->saved_data["sh_degree"] = raster_settings.sh_degree_;
    ctx->save_for_backward({raster_settings.bg_,
                            raster_settings.viewmatrix_,
                            raster_settings.projmatrix_,
                            raster_settings.campos_,
                            colors_precomp,
                            means3D,
                            scales,
                            rotations,
                            cov3Ds_precomp,
                            radii,
                            sh,
                            geomBuffer,
                            binningBuffer,
                            imgBuffer});

    return {color, radii};
}

/**
 * @brief Calls the C++/CUDA backwards rasterizer
 *
 * @param ctx
 * @param grad_outputs
 * @return torch::autograd::tensor_list
 */
torch::autograd::tensor_list
GaussianRasterizerFunction::backward(
    torch::autograd::AutogradContext *ctx,
    torch::autograd::tensor_list grad_outputs)
{
    // Restore necessary values from context (obtained from forward)
    auto saved = ctx->get_saved_variables();
    auto bg = saved[0];
    auto viewmatrix = saved[1];
    auto projmatrix = saved[2];
    auto campos = saved[3];
    auto colors_precomp = saved[4];
    auto means3D = saved[5];
    auto scales = saved[6];
    auto rotations = saved[7];
    auto cov3Ds_precomp = saved[8];
    auto radii = saved[9];
    auto sh = saved[10];
    auto geomBuffer = saved[11];
    auto binningBuffer = saved[12];
    auto imgBuffer = saved[13];

    auto grad_out_color = grad_outputs[0];

    // Compute gradients for relevant tensors by invoking backward method
    auto [grad_means2D, grad_means2D_densify, grad_colors_precomp, grad_opacities, grad_means3D, grad_cov3Ds_precomp, grad_sh, grad_scales, grad_rotations] =
    RasterizeGaussiansBackwardCUDA(
        bg,
        means3D,
        radii,
        colors_precomp,
        scales,
        rotations,
        static_cast<float>(ctx->saved_data["scale_modifier"].toDouble()),
        cov3Ds_precomp,
        viewmatrix,
        projmatrix,
        static_cast<float>(ctx->saved_data["tanfovx"].toDouble()),
        static_cast<float>(ctx->saved_data["tanfovy"].toDouble()),
        grad_out_color,
        sh,
        ctx->saved_data["sh_degree"].toInt(),
        campos,
        geomBuffer,
        ctx->saved_data["num_rendered"].toInt(),
        binningBuffer,
        imgBuffer
    );

    return {
        grad_means3D,
        grad_means2D,
        grad_means2D_densify,
        grad_sh,
        grad_colors_precomp,
        grad_opacities,
        grad_scales,
        grad_rotations,
        grad_cov3Ds_precomp,
        torch::Tensor()
    };
}

// ----- GaussianRasterizer -----
/**
 * @brief Marks Gaussians visable
 *
 * @param positions
 * @return torch::Tensor
 */
torch::Tensor
GaussianRasterizer::markVisibleGaussians(
    torch::Tensor &positions)
{
    // Mark visible points (based on frustum culling for camera) with a boolean
    torch::NoGradGuard no_grad;
    auto raster_settings = this->raster_settings_;
    return markVisible(positions, raster_settings.viewmatrix_, raster_settings.projmatrix_);
}

/**
 * @brief Applies the forward function of GaussianRasterizerFunction
 *
 * Note that this is part of GaussianRasterizer while
 * the GaussianRasterizerFunction has its own forward
 *
 * @param means3D
 * @param means2D
 * @param opacities
 * @param has_shs
 * @param has_colors_precomp
 * @param has_scales
 * @param has_rotations
 * @param has_cov3D_precomp
 * @param shs
 * @param colors_precomp
 * @param scales
 * @param rotations
 * @param cov3D_precomp
 * @return std::tuple<torch::Tensor, torch::Tensor>
 */
std::tuple<torch::Tensor, torch::Tensor>
GaussianRasterizer::forward(
    torch::Tensor means3D,
    torch::Tensor means2D,
    torch::Tensor means2D_densify,
    torch::Tensor opacities,
    bool has_shs,
    bool has_colors_precomp,
    bool has_scales,
    bool has_rotations,
    bool has_cov3D_precomp,
    torch::Tensor shs,
    torch::Tensor colors_precomp,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3D_precomp)

{
    auto raster_settings = this->raster_settings_;

    if ((!has_shs/*shs is None*/ && !has_colors_precomp/*colors_precomp is None*/)
        || (has_shs/*shs is not None*/ && has_colors_precomp/*colors_precomp is not None*/))
        throw std::runtime_error("Please provide excatly one of either SHs or precomputed colors!");
    
    if (((!has_scales/*scales is None*/ || !has_rotations/*rotations is None*/) && !has_cov3D_precomp/*cov3D_precomp is None*/)
        || ((has_scales/*scales is not None*/ || has_rotations/*rotations is not None*/) && has_cov3D_precomp/*cov3D_precomp is not None*/))
        throw std::runtime_error("Please provide exactly one of either scale/rotation pair or precomputed 3D covariance!");

    // Check if tensors are undefined, and if so, initialize them
    torch::TensorOptions options;
    if (!has_shs)
        shs = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_colors_precomp)
        colors_precomp = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_scales)
        scales = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_rotations)
        rotations = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_cov3D_precomp)
        cov3D_precomp = torch::tensor({}, options.device(torch::kCUDA));

    // This will call the apply() fuction of GaussianRasterizerFunction
    auto result = rasterizeGaussians(
        means3D,
        means2D,
        means2D_densify,
        shs,
        colors_precomp,
        opacities,
        scales,
        rotations,
        cov3D_precomp,
        raster_settings);

    return std::make_tuple(result[0] /*color*/, result[1] /*radii*/);
}

torch::Tensor GaussianRasterizer::visible_filter(
    torch::Tensor means3D,
    bool has_scales,
    bool has_rotations,
    bool has_cov3D_precomp,
    torch::Tensor scales,
    torch::Tensor rotations,
    torch::Tensor cov3D_precomp)

{
    auto raster_settings = this->raster_settings_;

    // Check if tensors are undefined, and if so, initialize them
    torch::TensorOptions options;
    if (!has_scales)
        scales = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_rotations)
        rotations = torch::tensor({}, options.device(torch::kCUDA));
    if (!has_cov3D_precomp)
        cov3D_precomp = torch::tensor({}, options.device(torch::kCUDA));

    torch::NoGradGuard no_grad;
    // This will call the apply() fuction of GaussianRasterizerFunction
    auto radii = RasterizeGaussiansfilterCUDA(
        means3D,
        scales,
        rotations,
        raster_settings.scale_modifier_,
        cov3D_precomp,
        raster_settings.viewmatrix_,
        raster_settings.projmatrix_,
        raster_settings.tanfovx_,
        raster_settings.tanfovy_,
        raster_settings.image_height_,
        raster_settings.image_width_,
        raster_settings.prefiltered_
        );

    return radii;
}