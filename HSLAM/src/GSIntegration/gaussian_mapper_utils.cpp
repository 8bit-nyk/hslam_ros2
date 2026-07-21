// gd -dev -16june2026
#include "GSIntegration/gaussian_mapper.h"

namespace HSLAM
{

    void GaussianMapper::readConfigFromFile(std::filesystem::path cfg_path)
    {
        cv::FileStorage settings_file(cfg_path.string().c_str(), cv::FileStorage::READ);
        if (!settings_file.isOpened())
        {
            std::cerr << "[Gaussian Mapper]Failed to open settings file at: " << cfg_path << std::endl;
            exit(-1);
        }

        std::cout << "[Gaussian Mapper]Reading parameters from " << cfg_path << std::endl;
        std::unique_lock<std::mutex> lock(mutex_settings_);

        record_ =
            (settings_file["Record.record"].operator int()) != 0;
        savePLY_ =
            (settings_file["Record.savePLY"].operator int()) != 0;
        // Model parameters
        model_params_.sh_degree_ =
            settings_file["Model.sh_degree"].operator int();
        model_params_.resolution_ =
            settings_file["Model.resolution"].operator float();
        model_params_.white_background_ =
            (settings_file["Model.white_background"].operator int()) != 0;
        model_params_.eval_ =
            (settings_file["Model.eval"].operator int()) != 0;

        // Pipeline Parameters
        z_near_ =
            settings_file["Camera.z_near"].operator float();
        z_far_ =
            settings_file["Camera.z_far"].operator float();

        monocular_inactive_geo_densify_max_pixel_dist_ =
            settings_file["Monocular.inactive_geo_densify_max_pixel_dist"].operator float();

        inactive_geo_densify_ =
            (settings_file["Mapper.inactive_geo_densify"].operator int()) != 0;
        max_depth_cached_ =
            settings_file["Mapper.depth_cache"].operator int();
        min_num_initial_map_kfs_ =
            static_cast<unsigned long>(settings_file["Mapper.min_num_initial_map_kfs"].operator int());
        new_keyframe_times_of_use_ =
            settings_file["Mapper.new_keyframe_times_of_use"].operator int();
        local_BA_increased_times_of_use_ =
            settings_file["Mapper.local_BA_increased_times_of_use"].operator int();
        min_post_slam_iterations_ =
            settings_file["Mapper.min_post_slam_iterations"].operator int(); // gd -dev -23june2026 


        cull_keyframes_ =
            (settings_file["Mapper.cull_keyframes"].operator int()) != 0;
        cull_iterations_ =
            settings_file["Mapper.cull_iterations"].operator int();
        max_keyframes_ =
            settings_file["Mapper.max_keyframes"].operator int();
        long_term_keyframes_iteration_ =
            settings_file["Mapper.long_term_keyframes_iteration"].operator int();
        large_rot_th_ =
            settings_file["Mapper.large_rotation_threshold"].operator float();
        large_trans_th_ =
            settings_file["Mapper.large_translation_threshold"].operator float();
        stable_num_iter_existence_ =
            settings_file["Mapper.stable_num_iter_existence"].operator int();

        pipe_params_.convert_SHs_ =
            (settings_file["Pipeline.convert_SHs"].operator int()) != 0;
        pipe_params_.compute_cov3D_ =
            (settings_file["Pipeline.compute_cov3D"].operator int()) != 0;

        do_gaus_pyramid_training_ =
            (settings_file["GausPyramid.do"].operator int()) != 0;
        num_gaus_pyramid_sub_levels_ =
            settings_file["GausPyramid.num_sub_levels"].operator int();
        int sub_level_times_of_use =
            settings_file["GausPyramid.sub_level_times_of_use"].operator int();
        kf_gaus_pyramid_times_of_use_.resize(num_gaus_pyramid_sub_levels_);
        kf_gaus_pyramid_factors_.resize(num_gaus_pyramid_sub_levels_);
        for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
        {
            kf_gaus_pyramid_times_of_use_[l] = sub_level_times_of_use;
            kf_gaus_pyramid_factors_[l] = std::pow(0.5f, num_gaus_pyramid_sub_levels_ - l);
        }

        keyframe_record_interval_ =
            settings_file["Record.keyframe_record_interval"].operator int();
        all_keyframes_record_interval_ =
            settings_file["Record.all_keyframes_record_interval"].operator int();
        record_rendered_image_ =
            (settings_file["Record.record_rendered_image"].operator int()) != 0;
        record_ground_truth_image_ =
            (settings_file["Record.record_ground_truth_image"].operator int()) != 0;
        record_loss_image_ =
            (settings_file["Record.record_loss_image"].operator int()) != 0;
        training_report_interval_ =
            settings_file["Record.training_report_interval"].operator int();
        record_loop_ply_ =
            (settings_file["Record.record_loop_ply"].operator int()) != 0;
        record_render_frames_ =
            (settings_file["Record.record_viewer_ply"].operator int()) != 0;

        // Optimization Parameters
        opt_params_.iterations_ =
            settings_file["Optimization.max_num_iterations"].operator int();
        opt_params_.position_lr_init_ =
            settings_file["Optimization.position_lr_init"].operator float();
        opt_params_.position_lr_final_ =
            settings_file["Optimization.position_lr_final"].operator float();
        opt_params_.position_lr_delay_mult_ =
            settings_file["Optimization.position_lr_delay_mult"].operator float();
        opt_params_.position_lr_max_steps_ =
            settings_file["Optimization.position_lr_max_steps"].operator int();
        opt_params_.feature_lr_ =
            settings_file["Optimization.feature_lr"].operator float();
        opt_params_.opacity_lr_ =
            settings_file["Optimization.opacity_lr"].operator float();
        opt_params_.scaling_lr_ =
            settings_file["Optimization.scaling_lr"].operator float();
        opt_params_.rotation_lr_ =
            settings_file["Optimization.rotation_lr"].operator float();

        opt_params_.percent_dense_ =
            settings_file["Optimization.percent_dense"].operator float();
        opt_params_.lambda_dssim_ =
            settings_file["Optimization.lambda_dssim"].operator float();
        opt_params_.lambda_isoreg_ =
            settings_file["Optimization.lambda_isoreg"].operator float();
        opt_params_.visibility_threshold_ =
            settings_file["Optimization.visibility_threshold"].operator float();
        opt_params_.densification_interval_ =
            settings_file["Optimization.densification_interval"].operator int();
        opt_params_.pruning_interval_ =
            settings_file["Optimization.pruning_interval"].operator int();
        opt_params_.opacity_reset_interval_ =
            settings_file["Optimization.opacity_reset_interval"].operator int();
        opt_params_.densify_from_iter_ =
            settings_file["Optimization.densify_from_iter_"].operator int();
        opt_params_.densify_until_iter_ =
            settings_file["Optimization.densify_until_iter"].operator int();
        opt_params_.densify_grad_threshold_ =
            settings_file["Optimization.densify_grad_threshold"].operator float();

        prune_big_point_after_iter_ =
            settings_file["Optimization.prune_big_point_after_iter"].operator int();
        densify_min_opacity_ =
            settings_file["Optimization.densify_min_opacity"].operator float();

        // Viewer Parameters
        rendered_image_viewer_scale_ =
            settings_file["GaussianViewer.image_scale"].operator float();
        rendered_image_viewer_scale_main_ =
            settings_file["GaussianViewer.image_scale_main"].operator float();
    }

    bool GaussianMapper::isStopped()
    {
        std::unique_lock<std::mutex> lock_status(mutex_status_);
        return this->stopped_;
    }

    void GaussianMapper::signalStop(const bool going_to_stop)
    {
        std::unique_lock<std::mutex> lock_status(mutex_status_);
        this->stopped_ = going_to_stop;
    }

    void GaussianMapper::generateKfidRandomShuffle()
    {
        if (scene_->keyframes().empty())
            return;

        std::size_t nkfs = scene_->keyframes().size();
        kfid_shuffle_.resize(nkfs);
        std::iota(kfid_shuffle_.begin(), kfid_shuffle_.end(), 0);
        std::mt19937 g(rd_());
        std::shuffle(kfid_shuffle_.begin(), kfid_shuffle_.end(), g);

        kfid_shuffled_ = true;
    }

    std::shared_ptr<GaussianKeyframe>
    GaussianMapper::useOneRandomSlidingWindowKeyframe()
    {
        if (scene_->keyframes().empty())
            return nullptr;

        if (!kfid_shuffled_)
            generateKfidRandomShuffle();

        std::shared_ptr<GaussianKeyframe> viewpoint_cam = nullptr;

        if (kfid_shuffled_)
        {
            int start_shuffle_idx = kfid_shuffle_idx_;
            do
            {
                int random_cam_idx;
                // Next shuffled idx
                ++kfid_shuffle_idx_;
                if (kfid_shuffle_idx_ >= kfid_shuffle_.size())
                    kfid_shuffle_idx_ = 0;
                // Add 1 time of use to all kfs if they are all unavalible
                if (kfid_shuffle_idx_ == start_shuffle_idx)
                    for (auto &kfit : scene_->keyframes())
                        increaseKeyframeTimesOfUse(kfit.second, 1);
                // Get viewpoint kf
                random_cam_idx = kfid_shuffle_[kfid_shuffle_idx_];
                auto random_cam_it = scene_->keyframes().begin();
                for (int cam_idx = 0; cam_idx < random_cam_idx; ++cam_idx)
                    ++random_cam_it;
                viewpoint_cam = (*random_cam_it).second;
            } while (viewpoint_cam->remaining_times_of_use_ <= 0);
        }

        // Count used times
        auto viewpoint_fid = viewpoint_cam->fid_;
        if (kfs_used_times_.find(viewpoint_fid) == kfs_used_times_.end())
            kfs_used_times_[viewpoint_fid] = 1;
        else
            ++kfs_used_times_[viewpoint_fid];

        // Handle times of use
        --(viewpoint_cam->remaining_times_of_use_);

        return viewpoint_cam;
    }

    std::shared_ptr<GaussianKeyframe>
    GaussianMapper::useOneRandomKeyframe()
    {
        if (scene_->keyframes().empty())
            return nullptr;

        // Get randomly
        int nkfs = static_cast<int>(scene_->keyframes().size());
        int random_cam_idx = std::rand() / ((RAND_MAX + 1u) / nkfs);
        auto random_cam_it = scene_->keyframes().begin();
        for (int cam_idx = 0; cam_idx < random_cam_idx; ++cam_idx)
            ++random_cam_it;
        std::shared_ptr<GaussianKeyframe> viewpoint_cam = (*random_cam_it).second;

        // Count used times
        auto viewpoint_fid = viewpoint_cam->fid_;
        if (kfs_used_times_.find(viewpoint_fid) == kfs_used_times_.end())
            kfs_used_times_[viewpoint_fid] = 1;
        else
            ++kfs_used_times_[viewpoint_fid];

        return viewpoint_cam;
    }

    void GaussianMapper::increaseKeyframeTimesOfUse(
        std::shared_ptr<GaussianKeyframe> pkf,
        int times)
    {
        pkf->remaining_times_of_use_ += times;
    }

    int GaussianMapper::getIteration()
    {
        std::unique_lock<std::mutex> lock_status(mutex_status_);
        return iteration_;
    }
    void GaussianMapper::increaseIteration(const int inc)
    {
        std::unique_lock<std::mutex> lock_status(mutex_status_);
        iteration_ += inc;
    }

    float GaussianMapper::positionLearningRateInit()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.position_lr_init_;
    }
    float GaussianMapper::featureLearningRate()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.feature_lr_;
    }
    float GaussianMapper::opacityLearningRate()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.opacity_lr_;
    }
    float GaussianMapper::scalingLearningRate()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.scaling_lr_;
    }
    float GaussianMapper::rotationLearningRate()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.rotation_lr_;
    }
    float GaussianMapper::percentDense()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.percent_dense_;
    }
    float GaussianMapper::lambdaDssim()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.lambda_dssim_;
    }
    float GaussianMapper::lambdaIsoReg()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.lambda_isoreg_;
    }
    float GaussianMapper::visibilityThreshold()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.visibility_threshold_;
    }
    int GaussianMapper::opacityResetInterval()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.opacity_reset_interval_;
    }
    float GaussianMapper::densifyGradThreshold()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.densify_grad_threshold_;
    }
    int GaussianMapper::densifyInterval()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.densification_interval_;
    }
    int GaussianMapper::pruneInterval()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return opt_params_.pruning_interval_;
    }
    int GaussianMapper::newKeyframeTimesOfUse()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return new_keyframe_times_of_use_;
    }
    int GaussianMapper::stableNumIterExistence()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return stable_num_iter_existence_;
    }
    int GaussianMapper::recordRenderFrames()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return record_render_frames_;
    }
    std::filesystem::path GaussianMapper::getResultDir()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return result_dir_;
    }
    bool GaussianMapper::isKeepingTraining()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return keep_training_;
    }
    void GaussianMapper::setKeepTrainingTrue()
    {
        keep_training_ = true;
    }
    bool GaussianMapper::isdoingGausPyramidTraining()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        return do_gaus_pyramid_training_;
    }

    void GaussianMapper::setPositionLearningRateInit(const float lr)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.position_lr_init_ = lr;
    }
    void GaussianMapper::setFeatureLearningRate(const float lr)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.feature_lr_ = lr;
    }
    void GaussianMapper::setOpacityLearningRate(const float lr)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.opacity_lr_ = lr;
    }
    void GaussianMapper::setScalingLearningRate(const float lr)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.scaling_lr_ = lr;
    }
    void GaussianMapper::setRotationLearningRate(const float lr)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.rotation_lr_ = lr;
    }
    void GaussianMapper::setPercentDense(const float percent_dense)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.percent_dense_ = percent_dense;
        gaussians_->setPercentDense(percent_dense);
    }
    void GaussianMapper::setLambdaDssim(const float lambda_dssim)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.lambda_dssim_ = lambda_dssim;
    }
    void GaussianMapper::setLambdaIsoReg(const float lambda_isoreg)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.lambda_isoreg_ = lambda_isoreg;
    }
    void GaussianMapper::setVisbilityThreshold(const float visibility_threshold)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.visibility_threshold_ = visibility_threshold;
    }
    void GaussianMapper::setOpacityResetInterval(const int interval)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.opacity_reset_interval_ = interval;
    }
    void GaussianMapper::setDensifyGradThreshold(const float th)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.densify_grad_threshold_ = th;
    }
    void GaussianMapper::setDensifyInterval(const int interval)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.densification_interval_ = interval;
    }
    void GaussianMapper::setPruneInterval(const int interval)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.pruning_interval_ = interval;
    }
    void GaussianMapper::setNewKeyframeTimesOfUse(const int times)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        new_keyframe_times_of_use_ = times;
    }
    void GaussianMapper::setStableNumIterExistence(const int niter)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        stable_num_iter_existence_ = niter;
    }
    void GaussianMapper::setKeepTraining(const bool keep)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        keep_training_ = keep;
    }
    void GaussianMapper::setDoGausPyramidTraining(const bool gaus_pyramid)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        do_gaus_pyramid_training_ = gaus_pyramid;
    }
    void GaussianMapper::setDoInactiveGeoDensify(const bool inactive_geo_densify)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        inactive_geo_densify_ = inactive_geo_densify;
    }

    VariableParameters GaussianMapper::getVaribleParameters()
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        VariableParameters params;
        params.position_lr_init = opt_params_.position_lr_init_;
        params.feature_lr = opt_params_.feature_lr_;
        params.opacity_lr = opt_params_.opacity_lr_;
        params.scaling_lr = opt_params_.scaling_lr_;
        params.rotation_lr = opt_params_.rotation_lr_;
        params.percent_dense = opt_params_.percent_dense_;
        params.lambda_dssim = opt_params_.lambda_dssim_;
        params.lambda_isoreg = opt_params_.lambda_isoreg_;
        params.opacity_reset_interval = opt_params_.opacity_reset_interval_;
        params.densify_grad_th = opt_params_.densify_grad_threshold_;
        params.densify_interval = opt_params_.densification_interval_;
        params.new_kf_times_of_use = new_keyframe_times_of_use_;
        params.stable_num_iter_existence = stable_num_iter_existence_;
        params.keep_training = keep_training_;
        params.do_gaus_pyramid_training = do_gaus_pyramid_training_;
        params.do_inactive_geo_densify = inactive_geo_densify_;
        return params;
    }

    void GaussianMapper::setVaribleParameters(const VariableParameters &params)
    {
        std::unique_lock<std::mutex> lock(mutex_settings_);
        opt_params_.position_lr_init_ = params.position_lr_init;
        opt_params_.feature_lr_ = params.feature_lr;
        opt_params_.opacity_lr_ = params.opacity_lr;
        opt_params_.scaling_lr_ = params.scaling_lr;
        opt_params_.rotation_lr_ = params.rotation_lr;
        opt_params_.percent_dense_ = params.percent_dense;
        gaussians_->setPercentDense(params.percent_dense);
        opt_params_.lambda_dssim_ = params.lambda_dssim;
        opt_params_.lambda_isoreg_ = params.lambda_isoreg;
        opt_params_.opacity_reset_interval_ = params.opacity_reset_interval;
        opt_params_.densification_interval_ = params.densify_interval;
        new_keyframe_times_of_use_ = params.new_kf_times_of_use;
        stable_num_iter_existence_ = params.stable_num_iter_existence;
        keep_training_ = params.keep_training;
        do_gaus_pyramid_training_ = params.do_gaus_pyramid_training;
        inactive_geo_densify_ = params.do_inactive_geo_densify;
    }

} // namespace HSLAM

// gd -dev -16june2026
