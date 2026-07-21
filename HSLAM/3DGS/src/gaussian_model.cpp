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



#include "3DGS/include/gaussian_model.h"



/**
 * @brief Construct a new Gaussian Model:: Gaussian Model object
 * 
 * The Gaussian Model stores the tensors and provides utility functions for 3DGS
 * 
 * Also contains the ADAM optimizer
 * 
 * @param sh_degree 
 */
GaussianModel::GaussianModel(const int sh_degree)
    : active_sh_degree_(0), spatial_lr_scale_(0.0)
{
    this->max_sh_degree_ = sh_degree;

    // Device
    if (torch::cuda::is_available())
        this->device_type_ = torch::kCUDA;
    else
        this->device_type_ = torch::kCPU;

    GAUSSIAN_MODEL_INIT_TENSORS(this->device_type_)
}

GaussianModel::GaussianModel(const GaussianModelParams &model_params)
    : active_sh_degree_(0), spatial_lr_scale_(0.0)
{
    this->max_sh_degree_ = model_params.sh_degree_;

    // Device
    if (model_params.data_device_ == "cuda")
        this->device_type_ = torch::kCUDA;
    else
        this->device_type_ = torch::kCPU;

    GAUSSIAN_MODEL_INIT_TENSORS(this->device_type_)
}


torch::Tensor GaussianModel::getScalingActivation()
{
    return torch::exp(this->scaling_);
}

torch::Tensor GaussianModel::getRotationActivation()
{
    return torch::nn::functional::normalize(this->rotation_);
}

torch::Tensor GaussianModel::getXYZ()
{
    return this->xyz_;
}

torch::Tensor GaussianModel::getFeatures()
{
    return torch::cat({this->features_dc_.clone(), this->features_rest_.clone()}, /*dim=*/1);
}

torch::Tensor GaussianModel::getOpacityActivation()
{
    return torch::sigmoid(this->opacity_);
}

torch::Tensor GaussianModel::getCovarianceActivation(int scaling_modifier)
{
    // build_rotation
    auto r = this->rotation_;
    auto R = GEN_UTIL::build_rotation(r);

    // build_scaling_rotation(scaling_modifier * scaling(Activation), rotation(_))
    auto s = scaling_modifier * this->getScalingActivation();
    auto L = torch::zeros({s.size(0), 3, 3}, torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    L.select(1, 0).select(1, 0).copy_(s.index({torch::indexing::Slice(), 0}));
    L.select(1, 1).select(1, 1).copy_(s.index({torch::indexing::Slice(), 1}));
    L.select(1, 2).select(1, 2).copy_(s.index({torch::indexing::Slice(), 2}));
    L = R.matmul(L); // L = R @ L

    // build_covariance_from_scaling_rotation
    auto actual_covariance = L.matmul(L.transpose(1, 2));
    // strip_symmetric
    // strip_lowerdiag
    auto symm_uncertainty = torch::zeros({actual_covariance.size(0), 6}, torch::TensorOptions().dtype(torch::kFloat).device(device_type_));

    symm_uncertainty.select(1, 0).copy_(actual_covariance.index({torch::indexing::Slice(), 0, 0}));
    symm_uncertainty.select(1, 1).copy_(actual_covariance.index({torch::indexing::Slice(), 0, 1}));
    symm_uncertainty.select(1, 2).copy_(actual_covariance.index({torch::indexing::Slice(), 0, 2}));
    symm_uncertainty.select(1, 3).copy_(actual_covariance.index({torch::indexing::Slice(), 1, 1}));
    symm_uncertainty.select(1, 4).copy_(actual_covariance.index({torch::indexing::Slice(), 1, 2}));
    symm_uncertainty.select(1, 5).copy_(actual_covariance.index({torch::indexing::Slice(), 2, 2}));

    return symm_uncertainty;
}

void GaussianModel::oneUpShDegree()
{
    if (this->active_sh_degree_ < this->max_sh_degree_)
        this->active_sh_degree_ += 1;
}

void GaussianModel::setShDegree(const int sh)
{
    this->active_sh_degree_ = (sh > this->max_sh_degree_ ? this->max_sh_degree_ : sh);
}


void GaussianModel::createFromPcd(
    std::map<point3D_id_t, Point3D> pcd,
    const float spatial_lr_scale)
{
    this->spatial_lr_scale_ = spatial_lr_scale;
    int num_points = static_cast<int>(pcd.size());
    torch::Tensor fused_point_cloud = torch::zeros(
        {num_points, 3},
        torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    torch::Tensor color = torch::zeros(
        {num_points, 3},
        torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    auto pcd_it = pcd.begin();
    for (int point_idx = 0; point_idx < num_points; ++point_idx) {
        auto& point = (*pcd_it).second;
        fused_point_cloud.index({point_idx, 0}) = point.xyz_(0);
        fused_point_cloud.index({point_idx, 1}) = point.xyz_(1);
        fused_point_cloud.index({point_idx, 2}) = point.xyz_(2);
        color.index({point_idx, 0}) = point.color_(0);
        color.index({point_idx, 1}) = point.color_(1);
        color.index({point_idx, 2}) = point.color_(2);
        ++pcd_it;
    }

    torch::Tensor fused_color = sh_utils::RGB2SH(color);
    auto temp = this->max_sh_degree_ + 1;
    torch::Tensor features = torch::zeros(
        {fused_color.size(0), 3, temp * temp},
        torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(0, 3),
         0}) = fused_color;
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(3, features.size(1)),
         torch::indexing::Slice(1, features.size(2))}) = 0.0f;

    torch::Tensor point_cloud_copy = fused_point_cloud.clone();
    torch::Tensor dist2 = torch::clamp_min(distCUDA2(point_cloud_copy), 0.0000001);
    torch::Tensor scales = torch::log(torch::sqrt(dist2));
    auto scales_ndimension = scales.ndimension();
    scales = scales.unsqueeze(scales_ndimension).repeat({1, 3});
    torch::Tensor rots = torch::zeros({fused_point_cloud.size(0), 4}, torch::TensorOptions().device(device_type_));
    rots.index({torch::indexing::Slice(), 0}) = 1;

    torch::Tensor opacities = GEN_UTIL::inverse_sigmoid(
        0.1f * torch::ones(
                   {fused_point_cloud.size(0), 1},
                   torch::TensorOptions().dtype(torch::kFloat).device(device_type_)));

    this->exist_since_iter_ = torch::zeros(
        {fused_point_cloud.size(0)},
        torch::TensorOptions().dtype(torch::kInt32).device(device_type_));

    this->xyz_ = fused_point_cloud.requires_grad_();
    this->features_dc_ = features.index({torch::indexing::Slice(),
                                         torch::indexing::Slice(),
                                         torch::indexing::Slice(0, 1)})
                             .transpose(1, 2)
                             .contiguous()
                             .requires_grad_();
    this->features_rest_ = features.index({torch::indexing::Slice(),
                                           torch::indexing::Slice(),
                                           torch::indexing::Slice(1, features.size(2))})
                               .transpose(1, 2)
                               .contiguous()
                               .requires_grad_();
    this->scaling_ = scales.requires_grad_();
    this->rotation_ = rots.requires_grad_();
    this->opacity_ = opacities.requires_grad_();

    GAUSSIAN_MODEL_TENSORS_TO_VEC

    this->max_radii2D_ = torch::zeros({this->getXYZ().size(0)}, torch::TensorOptions().device(device_type_));
}

void GaussianModel::increasePcdOpacFilter(std::vector<float> points, std::vector<float> colors, torch::Tensor cumlative_opacities, float visibility_threshold, const int iteration) 
{
    // auto keep_mask = torch::ones_like(new_types);
    auto keep_mask = cumlative_opacities < visibility_threshold;
    // auto keep_mask = new_types == 0;

    int num_new_points = static_cast<int>(points.size() / 3);
    torch::Tensor new_point_cloud = torch::from_blob(
        points.data(), {num_new_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);
    torch::Tensor new_colors = torch::from_blob(
        colors.data(), {num_new_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    new_point_cloud = new_point_cloud.index({keep_mask});
    new_colors = new_colors.index({keep_mask});
    cumlative_opacities = cumlative_opacities.index({keep_mask});

    float low_opacity = 0.3;

    // Clone the very low opacity gaussians.
    torch::Tensor clone_indicies = (cumlative_opacities < low_opacity);
    torch::Tensor cumlative_opacities_extend = cumlative_opacities.index({clone_indicies});
    torch::Tensor new_colors_extend = new_colors.index({clone_indicies});
    torch::Tensor point_cloud_extend = new_point_cloud.index({clone_indicies});
    auto clone_gauss_noise = torch::randn_like(point_cloud_extend) * 0.1;
    point_cloud_extend = point_cloud_extend + clone_gauss_noise;
    new_point_cloud = torch::cat({new_point_cloud, point_cloud_extend}, 0);
    new_colors = torch::cat({new_colors, new_colors_extend}, 0);
    cumlative_opacities = torch::cat({cumlative_opacities, cumlative_opacities_extend}, 0);

    // Initialize novel gaussians to be opaque
    auto new_opacities = 0.1 * torch::ones_like(cumlative_opacities);
    new_opacities = torch::where(cumlative_opacities < low_opacity, 0.9, new_opacities).unsqueeze(1);

    // Add new points to the model
    if (torch::sum(keep_mask).item<int>() >= 30) {
        torch::NoGradGuard no_grad;
        this->increasePcd(new_point_cloud, new_colors, new_opacities, iteration);
    }
}

void GaussianModel::increasePcd(std::vector<float> points, std::vector<float> colors, const int iteration)
{
    assert(points.size() == colors.size());
    assert(points.size() % 3 == 0);
    auto num_new_points = static_cast<int>(points.size() / 3);
    if (num_new_points == 0)
        return;

    torch::Tensor new_point_cloud = torch::from_blob(
        points.data(), {num_new_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);
    torch::Tensor new_colors = torch::from_blob(
        colors.data(), {num_new_points, 3},
        torch::TensorOptions().dtype(torch::kFloat32)).to(device_type_);

    if (sparse_points_xyz_.size(0) == 0) {
        sparse_points_xyz_ = new_point_cloud;
        sparse_points_color_ = new_colors;
    }
    else {
        sparse_points_xyz_ = torch::cat({sparse_points_xyz_, new_point_cloud}, /*dim=*/0);
        sparse_points_color_ = torch::cat({sparse_points_color_, new_colors}, /*dim=*/0);
    }

    torch::Tensor new_fused_colors = sh_utils::RGB2SH(new_colors);
    auto temp = this->max_sh_degree_ + 1;
    torch::Tensor features = torch::zeros(
        {new_fused_colors.size(0), 3, temp * temp},
        torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(0, 3),
         0}) = new_fused_colors;
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(3, features.size(1)),
         torch::indexing::Slice(1, features.size(2))}) = 0.0f;

    torch::Tensor dist2 = torch::clamp_min(
        distCUDA2(new_point_cloud.clone()), 0.0000001);
    torch::Tensor scales = torch::log(torch::sqrt(dist2));
    auto scales_ndimension = scales.ndimension();
    scales = scales.unsqueeze(scales_ndimension).repeat({1, 3});
    torch::Tensor rots = torch::zeros(
        {new_point_cloud.size(0), 4},
         torch::TensorOptions().device(device_type_));
    rots.index({torch::indexing::Slice(), 0}) = 1;
    torch::Tensor opacities = GEN_UTIL::inverse_sigmoid(
        0.1f * torch::ones(
                   {new_point_cloud.size(0), 1},
                   torch::TensorOptions().dtype(torch::kFloat).device(device_type_)));

    torch::Tensor new_exist_since_iter = torch::full(
        {new_point_cloud.size(0)},
        iteration,
        torch::TensorOptions().dtype(torch::kInt32).device(device_type_));

    auto new_xyz = new_point_cloud;
    auto new_features_dc = features.index({torch::indexing::Slice(),
                                                    torch::indexing::Slice(),
                                                    torch::indexing::Slice(0, 1)})
                                        .transpose(1, 2)
                                        .contiguous();
    auto new_features_rest = features.index({torch::indexing::Slice(),
                                                      torch::indexing::Slice(),
                                                      torch::indexing::Slice(1, features.size(2))})
                                          .transpose(1, 2)
                                          .contiguous();
    auto new_opacities = opacities;
    auto new_scaling = scales;
    auto new_rotation = rots;

    densificationPostfix(
        new_xyz,
        new_features_dc,
        new_features_rest,
        new_opacities,
        new_scaling,
        new_rotation,
        new_exist_since_iter
    );

    c10::cuda::CUDACachingAllocator::emptyCache();
}

void GaussianModel::increasePcd(torch::Tensor& new_point_cloud, torch::Tensor& new_colors, torch::Tensor& new_opacities, const int iteration)
{
    auto num_new_points = new_point_cloud.size(0);
    if (num_new_points == 0)
        return;

    if (sparse_points_xyz_.size(0) == 0) {
        sparse_points_xyz_ = new_point_cloud;
        sparse_points_color_ = new_colors;
    }
    else {
        sparse_points_xyz_ = torch::cat({sparse_points_xyz_, new_point_cloud}, /*dim=*/0);
        sparse_points_color_ = torch::cat({sparse_points_color_, new_colors}, /*dim=*/0);
    }

    torch::Tensor new_fused_colors = sh_utils::RGB2SH(new_colors);
    auto temp = this->max_sh_degree_ + 1;
    torch::Tensor features = torch::zeros(
        {new_fused_colors.size(0), 3, temp * temp},
        torch::TensorOptions().dtype(torch::kFloat).device(device_type_));
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(0, 3),
         0}) = new_fused_colors;
    features.index(
        {torch::indexing::Slice(),
         torch::indexing::Slice(3, features.size(1)),
         torch::indexing::Slice(1, features.size(2))}) = 0.0f;

    torch::Tensor dist2 = torch::clamp_min(
        distCUDA2(new_point_cloud.clone()), 0.0000001);
    torch::Tensor scales = torch::log(torch::sqrt(dist2));
    auto scales_ndimension = scales.ndimension();
    scales = scales.unsqueeze(scales_ndimension).repeat({1, 3});
    torch::Tensor rots = torch::zeros(
        {new_point_cloud.size(0), 4},
         torch::TensorOptions().device(device_type_));
    rots.index({torch::indexing::Slice(), 0}) = 1;

    torch::Tensor new_exist_since_iter = torch::full(
        {new_point_cloud.size(0)},
        iteration,
        torch::TensorOptions().dtype(torch::kInt32).device(device_type_));

    auto new_xyz = new_point_cloud;
    auto new_features_dc = features.index({torch::indexing::Slice(),
                                                    torch::indexing::Slice(),
                                                    torch::indexing::Slice(0, 1)})
                                        .transpose(1, 2)
                                        .contiguous();
    auto new_features_rest = features.index({torch::indexing::Slice(),
                                                      torch::indexing::Slice(),
                                                      torch::indexing::Slice(1, features.size(2))})
                                          .transpose(1, 2)
                                          .contiguous();
    new_opacities = GEN_UTIL::inverse_sigmoid(0.1f * new_opacities);
    auto new_scaling = scales;
    auto new_rotation = rots;

    densificationPostfix(
        new_xyz,
        new_features_dc,
        new_features_rest,
        new_opacities,
        new_scaling,
        new_rotation,
        new_exist_since_iter
    );

    c10::cuda::CUDACachingAllocator::emptyCache();
}


void GaussianModel::trainingSetup(const GaussianOptimizationParams& training_args)
{
    setPercentDense(training_args.percent_dense_);
    this->xyz_gradient_accum_ = torch::zeros({this->getXYZ().size(0), 1}, torch::TensorOptions().device(device_type_));
    this->denom_ = torch::zeros({this->getXYZ().size(0), 1}, torch::TensorOptions().device(device_type_));

    torch::optim::AdamOptions adam_options;
    adam_options.set_lr(0.0); adam_options.eps() = 1e-15;

    this->optimizer_.reset(new torch::optim::Adam(Tensor_vec_xyz_, adam_options));
    optimizer_->param_groups()[0].options().set_lr(training_args.position_lr_init_ * this->spatial_lr_scale_);
    optimizer_->add_param_group(Tensor_vec_feature_dc_);
    optimizer_->param_groups()[1].options().set_lr(training_args.feature_lr_);
    optimizer_->add_param_group(Tensor_vec_feature_rest_);
    optimizer_->param_groups()[2].options().set_lr(training_args.feature_lr_ / 20.0);
    optimizer_->add_param_group(Tensor_vec_opacity_);
    optimizer_->param_groups()[3].options().set_lr(training_args.opacity_lr_);
    optimizer_->add_param_group(Tensor_vec_scaling_);
    optimizer_->param_groups()[4].options().set_lr(training_args.scaling_lr_);
    optimizer_->add_param_group(Tensor_vec_rotation_);
    optimizer_->param_groups()[5].options().set_lr(training_args.rotation_lr_);

    this->xyz_scheduler_args_ = GEN_UTIL::Expon_lr_func(training_args.position_lr_init_ * this->spatial_lr_scale_,
                                                        training_args.position_lr_final_ * this->spatial_lr_scale_,
                                                        training_args.position_lr_delay_mult_,
                                                        training_args.position_lr_max_steps_);
}


// ==================================
// param_groups[0] = xyz_
// param_groups[1] = feature_dc_
// param_groups[2] = feature_rest_
// param_groups[3] = opacity_
// param_groups[4] = scaling_
// param_groups[5] = rotation_
// ==================================
float GaussianModel::updateLearningRate(int step)
{
    float lr = xyz_scheduler_args_(step);
    optimizer_->param_groups()[0].options().set_lr(lr);
    return lr;
}

void GaussianModel::setPositionLearningRate(float position_lr)
{
    optimizer_->param_groups()[0].options().set_lr(position_lr * this->spatial_lr_scale_);
}

void GaussianModel::setFeatureLearningRate(float feature_lr)
{
    optimizer_->param_groups()[1].options().set_lr(feature_lr);
    optimizer_->param_groups()[2].options().set_lr(feature_lr / 20.0);
}

void GaussianModel::setOpacityLearningRate(float opacity_lr)
{
    optimizer_->param_groups()[3].options().set_lr(opacity_lr);
}

void GaussianModel::setScalingLearningRate(float scaling_lr)
{
    optimizer_->param_groups()[4].options().set_lr(scaling_lr);
}

void GaussianModel::setRotationLearningRate(float rot_lr)
{
    optimizer_->param_groups()[5].options().set_lr(rot_lr);
}


void GaussianModel::resetOpacity()
{
    torch::Tensor opacities_new = GEN_UTIL::inverse_sigmoid(
        torch::min(
            this->getOpacityActivation(),
            torch::ones_like(this->getOpacityActivation() * 0.01)));
   
    torch::Tensor optimizable_tensors = this->replaceTensorToOptimizer(opacities_new, 3); // "opacity"
    
    this->opacity_ = optimizable_tensors;
    this->Tensor_vec_opacity_ = {this->opacity_};
}


torch::Tensor GaussianModel::replaceTensorToOptimizer(torch::Tensor& tensor, int tensor_idx)
{
    auto& param = this->optimizer_->param_groups()[tensor_idx].params()[0];
    auto& state = optimizer_->state();
    auto key = c10::guts::to_string(param.unsafeGetTensorImpl());
    auto& stored_state = static_cast<torch::optim::AdamParamState&>(*state[key]);
    auto new_state = std::make_unique<torch::optim::AdamParamState>();
    new_state->step(stored_state.step());
    new_state->exp_avg(torch::zeros_like(tensor));
    new_state->exp_avg_sq(torch::zeros_like(tensor));

    state.erase(key);
    param = tensor.requires_grad_();
    key = c10::guts::to_string(param.unsafeGetTensorImpl());
    state[key] = std::move(new_state);

    auto optimizable_tensors = param;
    return optimizable_tensors;
}


void GaussianModel::prunePoints(torch::Tensor& mask)
{
    auto valid_points_mask = ~mask;

    // _prune_optimizer
    std::vector<torch::Tensor> optimizable_tensors(6);
    auto& param_groups = this->optimizer_->param_groups();
    auto& state = this->optimizer_->state();
    for (int group_idx = 0; group_idx < 6; ++group_idx) {
        auto& param = param_groups[group_idx].params()[0];
        auto key = c10::guts::to_string(param.unsafeGetTensorImpl());
        if (state.find(key) != state.end()) {
            auto& stored_state = static_cast<torch::optim::AdamParamState&>(*state[key]);
            auto new_state = std::make_unique<torch::optim::AdamParamState>();
            new_state->step(stored_state.step());
            new_state->exp_avg(stored_state.exp_avg().index({valid_points_mask}).clone());
            new_state->exp_avg_sq(stored_state.exp_avg_sq().index({valid_points_mask}).clone());
            // new_state->max_exp_avg_sq(stored_state.max_exp_avg_sq().clone()); // needed only when options.amsgrad(true), which is false by default

            state.erase(key);
            param = param.index({valid_points_mask}).requires_grad_();
            key = c10::guts::to_string(param.unsafeGetTensorImpl());
            state[key] = std::move(new_state);
            optimizable_tensors[group_idx] = param;
        }
        else {
            param = param.index({valid_points_mask}).requires_grad_();
            optimizable_tensors[group_idx] = param;
        }
    }


    // ==================================
    // param_groups[0] = xyz_
    // param_groups[1] = feature_dc_
    // param_groups[2] = feature_rest_
    // param_groups[3] = opacity_
    // param_groups[4] = scaling_
    // param_groups[5] = rotation_
    // ==================================
    this->xyz_ = optimizable_tensors[0];
    this->features_dc_ = optimizable_tensors[1];
    this->features_rest_ = optimizable_tensors[2];
    this->opacity_ = optimizable_tensors[3];
    this->scaling_ = optimizable_tensors[4];
    this->rotation_ = optimizable_tensors[5];

    GAUSSIAN_MODEL_TENSORS_TO_VEC

    this->exist_since_iter_ = this->exist_since_iter_.index({valid_points_mask});

    this->xyz_gradient_accum_ = this->xyz_gradient_accum_.index({valid_points_mask});
    this->denom_ = this->denom_.index({valid_points_mask});
    this->max_radii2D_ = this->max_radii2D_.index({valid_points_mask});
}


void GaussianModel::densificationPostfix(
    torch::Tensor& new_xyz,
    torch::Tensor& new_features_dc,
    torch::Tensor& new_features_rest,
    torch::Tensor& new_opacities,
    torch::Tensor& new_scaling,
    torch::Tensor& new_rotation,
    torch::Tensor& new_exist_since_iter)
{
    // cat_tensors_to_optimizer
    std::vector<torch::Tensor> optimizable_tensors(6);
    std::vector<torch::Tensor> tensors_dict = {
        new_xyz,
        new_features_dc,
        new_features_rest,
        new_opacities,
        new_scaling,
        new_rotation
    };
    auto& param_groups = this->optimizer_->param_groups();
    auto& state = this->optimizer_->state();
    for (int group_idx = 0; group_idx < 6; ++group_idx) {
        auto& group = param_groups[group_idx];
        assert(group.params().size() == 1);
        auto& extension_tensor = tensors_dict[group_idx];
        auto& param = group.params()[0];
        auto key = c10::guts::to_string(param.unsafeGetTensorImpl());
        if (state.find(key) != state.end()) {
            auto& stored_state = static_cast<torch::optim::AdamParamState&>(*state[key]);
            auto new_state = std::make_unique<torch::optim::AdamParamState>();
            new_state->step(stored_state.step());
            new_state->exp_avg(torch::cat({stored_state.exp_avg().clone(), torch::zeros_like(extension_tensor)}, /*dim=*/0));
            new_state->exp_avg_sq(torch::cat({stored_state.exp_avg_sq().clone(), torch::zeros_like(extension_tensor)}, /*dim=*/0));
            // new_state->max_exp_avg_sq(stored_state.max_exp_avg_sq().clone());  // needed only when options.amsgrad(true), which is false by default

            state.erase(key);
            param = torch::cat({param, extension_tensor}, /*dim=*/0).requires_grad_();
            key = c10::guts::to_string(param.unsafeGetTensorImpl());
            state[key] = std::move(new_state);

            optimizable_tensors[group_idx] = param;
        }
        else {
            param = torch::cat({param, extension_tensor}, /*dim=*/0).requires_grad_();
            optimizable_tensors[group_idx] = param;
        }
    }


    // ==================================
    // param_groups[0] = xyz_
    // param_groups[1] = feature_dc_
    // param_groups[2] = feature_rest_
    // param_groups[3] = opacity_
    // param_groups[4] = scaling_
    // param_groups[5] = rotation_
    // ==================================
    this->xyz_ = optimizable_tensors[0];
    this->features_dc_ = optimizable_tensors[1];
    this->features_rest_ = optimizable_tensors[2];
    this->opacity_ = optimizable_tensors[3];
    this->scaling_ = optimizable_tensors[4];
    this->rotation_ = optimizable_tensors[5];

    GAUSSIAN_MODEL_TENSORS_TO_VEC

    this->exist_since_iter_ = torch::cat({this->exist_since_iter_, new_exist_since_iter}, /*dim=*/0);

    this->xyz_gradient_accum_ = torch::zeros({this->getXYZ().size(0), 1}, torch::TensorOptions().device(device_type_));
    this->denom_ = torch::zeros({this->getXYZ().size(0), 1}, torch::TensorOptions().device(device_type_));
    this->max_radii2D_ = torch::zeros({this->getXYZ().size(0)}, torch::TensorOptions().device(device_type_));
}


void GaussianModel::densifyAndSplit(
    torch::Tensor& grads,
    float grad_threshold,
    float scene_extent,
    int N)
{
    int n_init_points = this->getXYZ().size(0);

    // Extract points that satisfy the gradient condition
    torch::Tensor padded_grad = torch::zeros({n_init_points}, torch::TensorOptions().device(device_type_));
    padded_grad.slice(/*dim=*/0L, /*start=*/0, /*end=*/grads.size(0)).copy_(grads.squeeze());
    torch::Tensor selected_pts_mask = torch::where(padded_grad >= grad_threshold, true, false);
    selected_pts_mask = torch::logical_and(
        selected_pts_mask,
        std::get<0>(torch::max(this->getScalingActivation(), /*dim=*/1)) > percentDense() * scene_extent
    );

    torch::Tensor stds = this->getScalingActivation().index({selected_pts_mask}).repeat({N, 1});
    torch::Tensor means = torch::zeros({stds.size(0), 3}, torch::TensorOptions().device(device_type_));
    torch::Tensor samples = at::normal(means, stds);
    torch::Tensor r_masked = this->rotation_.index({selected_pts_mask});
    torch::Tensor rots = GEN_UTIL::build_rotation(r_masked).repeat({N, 1, 1});

    torch::Tensor new_xyz = torch::bmm(rots, samples.unsqueeze(-1)).squeeze(-1) + this->getXYZ().index({selected_pts_mask}).repeat({N, 1});
    torch::Tensor new_scaling = torch::log(this->getScalingActivation().index({selected_pts_mask}).repeat({N, 1}) / (0.8 * N)); // scaling_inverse_activation
    torch::Tensor new_rotation = this->rotation_.index({selected_pts_mask}).repeat({N, 1});
    torch::Tensor new_features_dc = this->features_dc_.index({selected_pts_mask}).repeat({N, 1, 1});
    torch::Tensor new_features_rest = this->features_rest_.index({selected_pts_mask}).repeat({N, 1, 1});
    torch::Tensor new_opacity = this->opacity_.index({selected_pts_mask}).repeat({N, 1});

    torch::Tensor new_exist_since_iter = this->exist_since_iter_.index({selected_pts_mask}).repeat({N});

    this->densificationPostfix(
        new_xyz,
        new_features_dc,
        new_features_rest,
        new_opacity,
        new_scaling,
        new_rotation,
        new_exist_since_iter
    );

    torch::Tensor prune_filter = torch::cat({
        selected_pts_mask,
        torch::zeros({(N * selected_pts_mask.sum()).item<int>()}, torch::TensorOptions().device(device_type_).dtype(torch::kBool))
    });
    this->prunePoints(prune_filter);
}

void GaussianModel::densifyAndClone(
    torch::Tensor& grads,
    float grad_threshold,
    float scene_extent)
{
    // Extract points that satisfy the gradient condition
    torch::Tensor selected_pts_mask = torch::where(torch::frobenius_norm(grads, /*dim=*/-1) >= grad_threshold, true, false);
    selected_pts_mask = torch::logical_and(
        selected_pts_mask,
        std::get<0>(torch::max(this->getScalingActivation(), /*dim=*/1)) <= percentDense() * scene_extent
    );

    torch::Tensor new_xyz = this->xyz_.index({selected_pts_mask});
    torch::Tensor new_features_dc = this->features_dc_.index({selected_pts_mask});
    torch::Tensor new_features_rest = this->features_rest_.index({selected_pts_mask});
    torch::Tensor new_opacities = this->opacity_.index({selected_pts_mask});
    torch::Tensor new_scaling = this->scaling_.index({selected_pts_mask});
    torch::Tensor new_rotation = this->rotation_.index({selected_pts_mask});

    torch::Tensor new_exist_since_iter = this->exist_since_iter_.index({selected_pts_mask});

    this->densificationPostfix(
        new_xyz,
        new_features_dc,
        new_features_rest,
        new_opacities,
        new_scaling,
        new_rotation,
        new_exist_since_iter
    );
}

void GaussianModel::densify(float max_grad, float extent) 
{
    torch::Tensor grads = this->xyz_gradient_accum_ / this->denom_;
    grads.index_put_({grads.isnan()}, 0.0f);
    this->densifyAndClone(grads, max_grad, extent);
    this->densifyAndSplit(grads, max_grad, extent);
};

void GaussianModel::prune(float min_opacity, float extent, int max_screen_size) 
{
    torch::Tensor prune_mask = (this->getOpacityActivation() < min_opacity).squeeze();
    if (max_screen_size) {
        auto big_points_vs = this->max_radii2D_ > max_screen_size;
        auto big_points_ws = std::get<0>(this->getScalingActivation().max(/*dim=*/1)) > 0.1f * extent;
        prune_mask = torch::logical_or(torch::logical_or(prune_mask, big_points_vs), big_points_ws);
    }
    this->prunePoints(prune_mask);
    c10::cuda::CUDACachingAllocator::emptyCache(); // torch.cuda.empty_cache()
};

void GaussianModel::addDensificationStats(
    torch::Tensor& viewspace_point_tensor,
    torch::Tensor& update_filter)
{
    this->xyz_gradient_accum_.index_put_(
        {update_filter},
        torch::frobenius_norm(viewspace_point_tensor.grad().index({update_filter, torch::indexing::Slice(0, 2)}),
                              /*dim=*/-1,
                              /*keepdim=*/true),
        /*accumulate=*/true);

    this->denom_.index_put_(
        {update_filter},
        this->denom_.index({update_filter}) + 1);
}


float GaussianModel::percentDense()
{
    std::unique_lock<std::mutex> lock_model(model_mutex_settings_);
    return percent_dense_;
}

void GaussianModel::setPercentDense(const float percent_dense)
{
    std::unique_lock<std::mutex> lock_model(model_mutex_settings_);
    percent_dense_ = percent_dense;
}