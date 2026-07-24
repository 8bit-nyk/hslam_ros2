// gd -dev -16june2026
#include "GSIntegration/gaussian_mapper.h"

#include "FullSystem/HessianBlocks.h"
#include "util/globalCalib.h"
#include "util/globalFuncs.h"

namespace HSLAM
{
    // constructor
    GaussianMapper::GaussianMapper(
        std::shared_ptr<HSLAM::FullSystem> pSLAM,
        std::filesystem::path gaussian_config_file_path,
        std::filesystem::path result_dir,
        int seed,
        torch::DeviceType device_type) : pSLAM_(pSLAM),
                                         initial_mapped_(false),
                                         interrupt_training_(false),
                                         stopped_(false),
                                         iteration_(0),
                                         ema_loss_for_log_(0.0f),
                                         SLAM_ended_(false),
                                         min_num_initial_map_kfs_(15UL),
                                         large_rot_th_(1e-1f),
                                         large_trans_th_(1e-2f),
                                         training_report_interval_(0),
                                         add_keyframes_(0)
    {
        // =========================== Set up paths and configs =========================
        std::srand(seed);
        torch::manual_seed(seed);

        if (device_type == torch::kCUDA && torch::cuda::is_available())
        {
            std::cout << "[Gaussian Mapper]CUDA available! Training on GPU." << std::endl;
            device_type_ = torch::kCUDA;
            model_params_.data_device_ = "cuda";
        }
        else
        {
            std::cout << "[Gaussian Mapper]Training on CPU." << std::endl;
            device_type_ = torch::kCPU;
            model_params_.data_device_ = "cpu";
        }

        result_dir_ = result_dir;
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)
        config_file_path_ = gaussian_config_file_path;
        readConfigFromFile(gaussian_config_file_path);

        std::vector<float> bg_color;
        if (model_params_.white_background_)
            bg_color = {1.0f, 1.0f, 1.0f};
        else
            bg_color = {0.0f, 0.0f, 0.0f};
        background_ = torch::tensor(bg_color,
                                    torch::TensorOptions().dtype(torch::kFloat32).device(device_type_));

        override_color_ = torch::empty(0, torch::TensorOptions().device(device_type_));

        // =========================== Set up Gaussian Mapper =========================
        gaussians_ = std::make_shared<GaussianModel>(model_params_);
        scene_ = std::make_shared<GaussianScene>(model_params_);

        if (!pSLAM)
        {
            return;
        }

        this->sensor_type_ = MONOCULAR;

        // Cameras — H-SLAM exposes image size via the global wG[0]/hG[0], not a Global_Calib struct
        cv::Size SLAM_im_size = cv::Size(wG[0], hG[0]);
        UndistortParams undistort_params(
            SLAM_im_size);

        // Set up calibration
        HSLAM::CalibHessian *calib = pSLAM->getCalibPtr();

        Camera camera;
        camera.camera_id_ = 0;

        camera.setModelId(Camera::CameraModelType::PINHOLE);
        float SLAM_fx = calib->fxl();
        float SLAM_fy = calib->fyl();
        float SLAM_cx = calib->cxl();
        float SLAM_cy = calib->cyl();

        cv::Mat K = (cv::Mat_<float>(3, 3)
                         << SLAM_fx,
                     0.f, SLAM_cx,
                     0.f, SLAM_fy, SLAM_cy,
                     0.f, 0.f, 1.f);

        camera.width_ = undistort_params.old_size_.width;
        float x_ratio = static_cast<float>(camera.width_) / undistort_params.old_size_.width;
        camera.height_ = undistort_params.old_size_.height;
        float y_ratio = static_cast<float>(camera.height_) / undistort_params.old_size_.height;

        camera.num_gaus_pyramid_sub_levels_ = num_gaus_pyramid_sub_levels_;
        camera.gaus_pyramid_width_.resize(num_gaus_pyramid_sub_levels_);
        camera.gaus_pyramid_height_.resize(num_gaus_pyramid_sub_levels_);
        for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
        {
            camera.gaus_pyramid_width_[l] = camera.width_ * this->kf_gaus_pyramid_factors_[l];
            camera.gaus_pyramid_height_[l] = camera.height_ * this->kf_gaus_pyramid_factors_[l];
        }

        camera.params_[0] = SLAM_fx * x_ratio;
        camera.params_[1] = SLAM_fy * y_ratio;
        camera.params_[2] = SLAM_cx * x_ratio;
        camera.params_[3] = SLAM_cy * y_ratio;

        cv::Mat K_new = (cv::Mat_<float>(3, 3)
                             << camera.params_[0],
                         0.f, camera.params_[2],
                         0.f, camera.params_[1], camera.params_[3],
                         0.f, 0.f, 1.f);

        if (this->sensor_type_ == MONOCULAR)
            undistort_params.dist_coeff_.copyTo(camera.dist_coeff_);

        camera.initUndistortRectifyMapAndMask(K, SLAM_im_size, K_new, true);

        undistort_mask_[camera.camera_id_] =
            tensor_utils::cvMat2TorchTensor_Float32(
                camera.undistort_mask, device_type_);

        cv::Mat viewer_sub_undistort_mask;
        int viewer_image_height_ = camera.height_ * rendered_image_viewer_scale_;
        int viewer_image_width_ = camera.width_ * rendered_image_viewer_scale_;
        cv::resize(camera.undistort_mask, viewer_sub_undistort_mask,
                   cv::Size(viewer_image_width_, viewer_image_height_));
        viewer_sub_undistort_mask_[camera.camera_id_] =
            tensor_utils::cvMat2TorchTensor_Float32(
                viewer_sub_undistort_mask, device_type_);

        cv::Mat viewer_main_undistort_mask;
        int viewer_image_height_main_ = camera.height_ * rendered_image_viewer_scale_main_;
        int viewer_image_width_main_ = camera.width_ * rendered_image_viewer_scale_main_;
        cv::resize(camera.undistort_mask, viewer_main_undistort_mask,
                   cv::Size(viewer_image_width_main_, viewer_image_height_main_));
        viewer_main_undistort_mask_[camera.camera_id_] =
            tensor_utils::cvMat2TorchTensor_Float32(
                viewer_main_undistort_mask, device_type_);

        if (!viewer_camera_id_set_)
        {
            viewer_camera_id_ = camera.camera_id_;
            viewer_camera_id_set_ = true;
        }
        this->scene_->addCamera(camera);
    }

    // mapping block
    void GaussianMapper::run()
    {
        // First loop: Initial gaussian mapping
        while (!isStopped())
        {
            if (hasMetInitialMappingConditions())
            {
                printf("Starting 3DGS!\n");
                HSLAM::CalibHessian *calib = pSLAM_->getCalibPtr();
                Sophus::SE3d firstPose = pSLAM_->firstPose;

                pSLAM_->clearMappingOperation();

                boost::unique_lock<boost::mutex> lock_mapping(pSLAM_->mapMutex);

                // Get keyframes
                std::vector<HSLAM::FrameHessian *> vpKFs = pSLAM_->GetAllKeyFrames();

                for (HSLAM::FrameHessian *pKF : vpKFs)
                {
                    std::shared_ptr<GaussianKeyframe> new_kf = std::make_shared<GaussianKeyframe>(pKF->frameID, getIteration());
                    new_kf->zfar_ = z_far_;
                    new_kf->znear_ = z_near_;

                    // Get Pose — Tcw, thread-safe accessor (H-SLAM's camToWorld is private)
                    Sophus::SE3d pose = pKF->shell->getPoseInverse();
                    new_kf->setPose(
                        pose.unit_quaternion(),
                        pose.translation());
                    cv::Mat imgRGB_undistorted;

                    try
                    {
                        Camera &camera = scene_->cameras_.at(0);
                        new_kf->setCameraParams(camera);

                        // Image — H-SLAM's dI_c is a raw Eigen::Vector3f* in [0,255] RGB,
                        // not a cv::Mat already in [0,1] like MGSO. Wrap + normalize.
                        if (!pKF->colourValid)
                            continue;
                        cv::Mat color_raw(hG[0], wG[0], CV_32FC3, (void *)pKF->dI_c);
                        color_raw.convertTo(imgRGB_undistorted, CV_32FC3, 1.0 / 255.0);

                        new_kf->original_image_ =
                            tensor_utils::cvMat2TorchTensor_Float32(imgRGB_undistorted, device_type_);
                        new_kf->img_filename_ = std::to_string(pKF->frameID); // H-SLAM has no image_name
                        new_kf->gaus_pyramid_height_ = camera.gaus_pyramid_height_;
                        new_kf->gaus_pyramid_width_ = camera.gaus_pyramid_width_;
                        new_kf->gaus_pyramid_times_of_use_ = kf_gaus_pyramid_times_of_use_;
                    }
                    catch (std::out_of_range)
                    {
                        throw std::runtime_error("[GaussianMapper::run]KeyFrame Camera not found!");
                    }

                    new_kf->computeTransformTensors();
                    scene_->addKeyframe(new_kf, &kfid_shuffled_);
                    new_kf->img_undist_ = imgRGB_undistorted;

                    increaseKeyframeTimesOfUse(new_kf, newKeyframeTimesOfUse());

                    // Add points to the scene — H-SLAM has only pointHessians per KF
                    // (no immatureHessians / HessiansStatic), filtered to DIRECT-only
                    // (Phase 2 policy: HYBRID/ORB points deferred to Phase 4 ablation)
                    for (HSLAM::PointHessian *pMP : pKF->pointHessians)
                    {
                        if (!pMP->Mp.expired())
                            continue;
                        pMP->setRetrived(true);
                        Point3D point3D;
                        Eigen::Vector3d pos = pMP->getWorldPosition(
                            1 / calib->fxl(), 1 / calib->fyl(),
                            -calib->cxl() / calib->fxl(), -calib->cyl() / calib->fyl(),
                            pKF->shell->getPose()); // Twc — getWorldPosition expects camera-to-world
                        point3D.xyz_(0) = pos.x();
                        point3D.xyz_(1) = pos.y();
                        point3D.xyz_(2) = pos.z();
                        Eigen::Vector3f color = pMP->getColourRGBfloat();
                        point3D.color_(0) = color(0);
                        point3D.color_(1) = color(1);
                        point3D.color_(2) = color(2);
                        scene_->cachePoint3D(pMP->point_id, point3D); // gd -dev -17june2026
                    }
                }

                // Prepare multi resolution images for training
                for (auto &kfit : scene_->keyframes())
                {
                    auto pkf = kfit.second;
#ifdef HAVE_OPENCV_CUDAWARPING
                    if (device_type_ == torch::kCUDA)
                    {
                        cv::cuda::GpuMat img_gpu;
                        img_gpu.upload(pkf->img_undist_);
                        pkf->gaus_pyramid_original_image_.resize(num_gaus_pyramid_sub_levels_);
                        for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
                        {
                            cv::cuda::GpuMat img_resized;
                            cv::cuda::resize(img_gpu, img_resized,
                                             cv::Size(pkf->gaus_pyramid_width_[l], pkf->gaus_pyramid_height_[l]));
                            pkf->gaus_pyramid_original_image_[l] =
                                tensor_utils::cvGpuMat2TorchTensor_Float32(img_resized);
                        }
                    }
                    else
#endif
                    {
                        pkf->gaus_pyramid_original_image_.resize(num_gaus_pyramid_sub_levels_);
                        for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
                        {
                            cv::Mat img_resized;
                            cv::resize(pkf->img_undist_, img_resized,
                                       cv::Size(pkf->gaus_pyramid_width_[l], pkf->gaus_pyramid_height_[l]));
                            pkf->gaus_pyramid_original_image_[l] =
                                tensor_utils::cvMat2TorchTensor_Float32(img_resized, device_type_);
                        }
                    }
                }

                {
                    std::unique_lock<std::mutex> lock_render(mutex_render_);
                    scene_->cameras_extent_ = std::get<1>(scene_->getNerfppNorm());
                    gaussians_->createFromPcd(scene_->cached_point_cloud_, scene_->cameras_extent_);
                    std::unique_lock<std::mutex> lock(mutex_settings_);
                    gaussians_->trainingSetup(opt_params_);
                }

                trainForOneIteration();

                initial_mapped_ = true;
                break;
            }
            else if (pSLAM_->isShutDown())
            {
                break;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        printf("Starting incremental mapping!\n");

        // Second loop: Incremental gaussian mapping
        int SLAM_stop_iter = 0;
        int loops = 0;
        while (!isStopped())
        {
            if (hasMetIncrementalMappingConditions())
            {
                combineMappingOperations();
                if (cull_keyframes_ && (loops % cull_iterations_ == 0))
                    cullKeyframes();
                if (loops % long_term_keyframes_iteration_ == 0)
                    add_keyframes_++;
                loops++;
            }

            trainForOneIteration();

            if (pSLAM_->isShutDown())
            {
                SLAM_stop_iter = getIteration();
                SLAM_ended_ = true;
            }

            if (SLAM_ended_ || getIteration() >= opt_params_.iterations_)
                break;
        }

        // Third loop: Tail gaussian optimization
        int densify_interval = densifyInterval();
        int n_delay_iters = densify_interval * 0.8;
        while (getIteration() - SLAM_stop_iter <= n_delay_iters || getIteration() % densify_interval <= n_delay_iters || getIteration() - SLAM_stop_iter < min_post_slam_iterations_ || isKeepingTraining()) // gd -dev -23june2026; done for debugging
        {
            trainForOneIteration();
            densify_interval = densifyInterval();
            n_delay_iters = densify_interval * 0.8;
        }

        c10::cuda::CUDACachingAllocator::emptyCache();

        if (record_)
        {
            renderAndRecordAllKeyframes("_shutdown");
            writeKeyframeUsedTimes(result_dir_ / "used_times", "final");
        }
        if (savePLY_)
        {
            savePly(result_dir_ / (std::to_string(getIteration()) + "_shutdown") / "ply");
        }

        signalStop();
    }

    bool GaussianMapper::hasMetInitialMappingConditions()
    {
        if (!pSLAM_->isShutDown() &&
            pSLAM_->getKeyframeCount() >= static_cast<int>(min_num_initial_map_kfs_) &&
            pSLAM_->hasMappingOperation())
            return true;

        bool conditions_met = false;
        return conditions_met;
    }

    bool GaussianMapper::hasMetIncrementalMappingConditions()
    {
        if (!pSLAM_->isShutDown() &&
            pSLAM_->hasMappingOperation())
            return true;

        bool conditions_met = false;
        return conditions_met;
    }

    void GaussianMapper::cullKeyframes()
    {
        std::unordered_set<unsigned long> kfids =
            pSLAM_->GetCurrentKeyFrameIds();
        std::size_t nkfs = scene_->keyframes().size();

        if (nkfs > (max_keyframes_ + add_keyframes_))
        {
            int num_to_delete = nkfs - (max_keyframes_ + add_keyframes_);

            int to_delete = 0;
            int max_rand_runs = 64;

            while (to_delete < num_to_delete)
            {
                if (max_rand_runs <= 0)
                    break;

                auto it = scene_->keyframes().begin();
                std::advance(it, rand() % nkfs);
                unsigned long kfid = it->first;

                if (kfids.find(kfid) != kfids.end())
                {
                    max_rand_runs--;
                    continue;
                }
                if (kfid % long_term_keyframes_iteration_ == 0)
                {
                    max_rand_runs--;
                    continue;
                }

                to_delete++;
                scene_->keyframes_deleted_.emplace(kfid, scene_->keyframes_[kfid]);
                scene_->keyframes().erase(kfid);
            }
        }
    }

    void GaussianMapper::combineMappingOperations()
    {
        while (pSLAM_->hasMappingOperation())
        {
            GSI::MappingOperation opr =
                pSLAM_->getAndPopMappingOperation();

            switch (opr.meOperationType)
            {
            case GSI::MappingOperation::LocalMappingBA:
            {
                auto &associated_kfs = opr.associatedKeyFrames();

                for (auto &kf : associated_kfs)
                {
                    auto kfid = std::get<0>(kf);
                    std::shared_ptr<GaussianKeyframe> pkf = scene_->getKeyframe(kfid);
                    if (pkf)
                    {
                        auto &pose = std::get<2>(kf);
                        pkf->setPose(
                            pose.unit_quaternion(),
                            pose.translation());
                        pkf->computeTransformTensors();

                        increaseKeyframeTimesOfUse(pkf, local_BA_increased_times_of_use_);
                    }
                    else
                    {
                        handleNewKeyframe(kf);
                    }
                }

                // --- 7-vector tuple unpack + DIRECT-only filter ---
                auto &associated_points = opr.associatedMapPoints();
                auto &raw_points = std::get<0>(associated_points);
                auto &raw_colors = std::get<1>(associated_points);
                auto &raw_u = std::get<2>(associated_points);
                auto &raw_v = std::get<3>(associated_points);
                auto &raw_ids = std::get<4>(associated_points);
                auto &raw_types = std::get<5>(associated_points);
                // std::get<6>(associated_points) = mp_std_dev — unused in Phase 2

                int num_raw = static_cast<int>(raw_types.size());

                std::vector<float> points, colors;
                std::vector<float> screenspace_u, screenspace_v;
                std::vector<int> hostFrameIDs;

                for (int i = 0; i < num_raw; ++i)
                {
                    if (raw_types[i] != static_cast<uint8_t>(GSI::PointType::DIRECT))
                        continue;
                    points.push_back(raw_points[i * 3 + 0]);
                    points.push_back(raw_points[i * 3 + 1]);
                    points.push_back(raw_points[i * 3 + 2]);
                    colors.push_back(raw_colors[i * 3 + 0]);
                    colors.push_back(raw_colors[i * 3 + 1]);
                    colors.push_back(raw_colors[i * 3 + 2]);
                    screenspace_u.push_back(raw_u[i]);
                    screenspace_v.push_back(raw_v[i]);
                    hostFrameIDs.push_back(raw_ids[i]);
                }
                int num_new_points = static_cast<int>(points.size() / 3);

                // H-SLAM-specific guard: a queued operation can legally contain
                // zero DIRECT points (e.g. a keyframe where every new point was
                // classified HYBRID). MGSO never hits this because it never filters
                // its point list, so torch::from_blob() on an empty vector was never
                // exercised there. Skip the tensor/render work entirely in that case.
                if (num_new_points > 0)
                {
                    int image_height = scene_->cameras_.begin()->second.height_;
                    int image_width = scene_->cameras_.begin()->second.width_;

                    torch::Tensor new_screenspace_u = torch::from_blob(
                                                          screenspace_u.data(), {num_new_points, 1},
                                                          torch::TensorOptions().dtype(torch::kFloat32))
                                                          .to(device_type_)
                                                          .squeeze(1)
                                                          .to(torch::TensorOptions().dtype(torch::kLong));
                    torch::Tensor new_screenspace_v = torch::from_blob(
                                                          screenspace_v.data(), {num_new_points, 1},
                                                          torch::TensorOptions().dtype(torch::kFloat32))
                                                          .to(device_type_)
                                                          .squeeze(1)
                                                          .to(torch::TensorOptions().dtype(torch::kLong));
                    torch::Tensor new_hostFrameIDs = torch::from_blob(
                                                         hostFrameIDs.data(), {num_new_points, 1},
                                                         torch::TensorOptions().dtype(torch::kInt32))
                                                         .to(device_type_)
                                                         .squeeze(1)
                                                         .to(torch::TensorOptions().dtype(torch::kLong));

                    auto unique_keyframes_ret = at::_unique(new_hostFrameIDs, true, true);
                    torch::Tensor unique_keyframes_ids = std::get<0>(unique_keyframes_ret);
                    torch::Tensor unique_keyframes_invind = std::get<1>(unique_keyframes_ret);
                    int num_unique_keyframes = unique_keyframes_ids.size(0);
                    torch::Tensor keyframe_renders = torch::zeros({num_unique_keyframes, image_height, image_width}, torch::TensorOptions().dtype(torch::kFloat32).device(device_type_));
                    for (auto &kf : associated_kfs)
                    {
                        int kfid = std::get<0>(kf);
                        auto is_present = torch::any(unique_keyframes_ids == kfid);
                        if (!(is_present.item<bool>()))
                        {
                            continue;
                        }
                        torch::Tensor unique_keyframes_ind = torch::nonzero(unique_keyframes_ids == kfid);

                        std::shared_ptr<GaussianKeyframe> pkf = scene_->getKeyframe(kfid);

                        auto empty_color = torch::ones_like(gaussians_->xyz_);
                        auto render_pkg = GaussianRenderer::render(
                            pkf,
                            image_height,
                            image_width,
                            gaussians_,
                            pipe_params_,
                            background_,
                            empty_color,
                            1.0f,
                            true);
                        torch::Tensor rendered_image = std::get<0>(render_pkg);
                        keyframe_renders.index_put_({unique_keyframes_ind}, rendered_image.index({0}));
                    }

                    // auto batch_indices = torch::arange(num_new_points, torch::TensorOptions().device(device_type_));
                    // auto indices = torch::stack({batch_indices, new_screenspace_v, new_screenspace_u}, 1);
                    // auto cumlative_opacities = torch::index_select(keyframe_renders, 0, unique_keyframes_invind).index({batch_indices, new_screenspace_v, new_screenspace_u});
                    // gd -dev -17june2027: was index_select two-step lookup that materialized a [N_points,H,W]
                    // tensor (~11 GB @ 1200x672) → OutOfMemory error. Direct 3D advanced-indexing yields [N_points],
                    auto cumlative_opacities = keyframe_renders.index({unique_keyframes_invind, new_screenspace_v, new_screenspace_u});
                    // gd -dev -17june2027 
                    if (initial_mapped_ && points.size() >= 30)
                    {
                        std::unique_lock<std::mutex> lock_render(mutex_render_);
                        gaussians_->increasePcdOpacFilter(points, colors, cumlative_opacities, visibilityThreshold(), getIteration());
                    }
                }
            }
            break;
            default:
            {
                throw std::runtime_error("MappingOperation type not supported!");
            }
            break;
            }
        }
    }

    void GaussianMapper::handleNewKeyframe(std::tuple<unsigned long, unsigned long, Sophus::SE3d, cv::Mat, std::string> &kf)
    {
        std::shared_ptr<GaussianKeyframe> pkf =
            std::make_shared<GaussianKeyframe>(std::get<0>(kf), getIteration());
        pkf->zfar_ = z_far_;
        pkf->znear_ = z_near_;
        auto &pose = std::get<2>(kf);
        pkf->setPose(
            pose.unit_quaternion(),
            pose.translation());
        cv::Mat imgRGB_undistorted;

        try
        {
            Camera &camera = scene_->cameras_.at(std::get<1>(kf));
            pkf->setCameraParams(camera);

            // Image already arrives [0,1] float from MappingOperation::addKeyFrame
            // (Phase 1) — no dI_c conversion needed here, only undistortion.
            cv::Mat imgRGB = std::get<3>(kf);
            camera.undistortImage(imgRGB, imgRGB_undistorted);

            pkf->original_image_ =
                tensor_utils::cvMat2TorchTensor_Float32(imgRGB_undistorted, device_type_);
            pkf->img_filename_ = std::get<4>(kf);
            pkf->gaus_pyramid_height_ = camera.gaus_pyramid_height_;
            pkf->gaus_pyramid_width_ = camera.gaus_pyramid_width_;
            pkf->gaus_pyramid_times_of_use_ = kf_gaus_pyramid_times_of_use_;
        }
        catch (std::out_of_range)
        {
            throw std::runtime_error("[GaussianMapper::combineMappingOperations]KeyFrame Camera not found!");
        }

        pkf->computeTransformTensors();
        scene_->addKeyframe(pkf, &kfid_shuffled_);

        increaseKeyframeTimesOfUse(pkf, newKeyframeTimesOfUse());

        pkf->img_undist_ = imgRGB_undistorted;

#ifdef HAVE_OPENCV_CUDAWARPING
        if (device_type_ == torch::kCUDA)
        {
            cv::cuda::GpuMat img_gpu;
            img_gpu.upload(pkf->img_undist_);
            pkf->gaus_pyramid_original_image_.resize(num_gaus_pyramid_sub_levels_);
            for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
            {
                cv::cuda::GpuMat img_resized;
                cv::cuda::resize(img_gpu, img_resized,
                                 cv::Size(pkf->gaus_pyramid_width_[l], pkf->gaus_pyramid_height_[l]));
                pkf->gaus_pyramid_original_image_[l] =
                    tensor_utils::cvGpuMat2TorchTensor_Float32(img_resized);
            }
        }
        else
#endif
        {
            pkf->gaus_pyramid_original_image_.resize(num_gaus_pyramid_sub_levels_);
            for (int l = 0; l < num_gaus_pyramid_sub_levels_; ++l)
            {
                cv::Mat img_resized;
                cv::resize(pkf->img_undist_, img_resized,
                           cv::Size(pkf->gaus_pyramid_width_[l], pkf->gaus_pyramid_height_[l]));
                pkf->gaus_pyramid_original_image_[l] =
                    tensor_utils::cvMat2TorchTensor_Float32(img_resized, device_type_);
            }
        }
    }

    void GaussianMapper::savePly(std::filesystem::path result_dir)
    {
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)
        saveModelParams(result_dir);

        std::filesystem::path ply_dir = result_dir / "point_cloud";
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(ply_dir)

        ply_dir = ply_dir / ("iteration_" + std::to_string(getIteration()));
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(ply_dir)

        gaussians_->savePly(ply_dir / "point_cloud.ply");
        gaussians_->saveSparsePointsPly(result_dir / "input.ply");
    }

    /**
     * Performs one full training iteration:
     * 1. Pick a random keyframe
     * 2. Calculate the loss
     * 3. Calculate the backwards of the loss
     * 4. Do Gaussian utility steps
     * 5. Do optimization
     */
    void GaussianMapper::trainForOneIteration()
    {
        increaseIteration(1);
        auto iter_start_timing = std::chrono::steady_clock::now();

        // Pick a random Camera
        std::shared_ptr<GaussianKeyframe> viewpoint_cam = useOneRandomSlidingWindowKeyframe();
        if (!viewpoint_cam)
        {
            increaseIteration(-1);
            return;
        }

        writeKeyframeUsedTimes(result_dir_ / "used_times");

        int training_level = num_gaus_pyramid_sub_levels_;
        int image_height, image_width;
        torch::Tensor gt_image, mask;

        if (isdoingGausPyramidTraining())
            training_level = viewpoint_cam->getCurrentGausPyramidLevel();

        if (training_level == num_gaus_pyramid_sub_levels_)
        {
            image_height = viewpoint_cam->image_height_;
            image_width = viewpoint_cam->image_width_;
            gt_image = viewpoint_cam->original_image_.cuda();
            mask = undistort_mask_[viewpoint_cam->camera_id_];
        }
        else
        {
            image_height = viewpoint_cam->gaus_pyramid_height_[training_level];
            image_width = viewpoint_cam->gaus_pyramid_width_[training_level];
            gt_image = viewpoint_cam->gaus_pyramid_original_image_[training_level].cuda();
            mask = scene_->cameras_.at(viewpoint_cam->camera_id_).gaus_pyramid_undistort_mask_[training_level];
        }

        // Mutex lock for usage of the gaussian model
        std::unique_lock<std::mutex> lock_render(mutex_render_);

        // Every 1000 its we increase the levels of SH up to a maximum degree
        if (getIteration() % 1000 == 0 && default_sh_ < model_params_.sh_degree_)
            default_sh_ += 1;
        gaussians_->setShDegree(default_sh_);

        // Update learning rate
        if (pSLAM_)
        {
            int used_times = kfs_used_times_[viewpoint_cam->fid_];
            int step = (used_times <= opt_params_.position_lr_max_steps_ ? used_times : opt_params_.position_lr_max_steps_);
            float position_lr = gaussians_->updateLearningRate(step);
            setPositionLearningRateInit(position_lr);
        }
        else
        {
            gaussians_->updateLearningRate(getIteration());
        }

        gaussians_->setFeatureLearningRate(featureLearningRate());
        gaussians_->setOpacityLearningRate(opacityLearningRate());
        gaussians_->setScalingLearningRate(scalingLearningRate());
        gaussians_->setRotationLearningRate(rotationLearningRate());

        // Render
        auto render_pkg = GaussianRenderer::render(
            viewpoint_cam,
            image_height,
            image_width,
            gaussians_,
            pipe_params_,
            background_,
            override_color_);
        auto rendered_image = std::get<0>(render_pkg);
        auto viewspace_point_tensor = std::get<1>(render_pkg);
        auto visibility_filter = std::get<2>(render_pkg);
        auto radii = std::get<3>(render_pkg);

        // Get rid of black edges caused by undistortion
        torch::Tensor masked_image = rendered_image * mask;

        // Loss
        auto Ll1 = loss_utils::l1_loss(masked_image, gt_image);
        float lambda_dssim = lambdaDssim();
        float lambda_isoreg = lambdaIsoReg();

        auto loss = (1.0 - lambda_dssim) * Ll1 + lambda_dssim * (1.0 - loss_utils::ssim(masked_image, gt_image, device_type_));

        if (lambda_isoreg > 0.0001)
        {
            torch::Tensor epsilon = torch::ones(1, torch::TensorOptions().device(gaussians_->device_type_));
            torch::Tensor ratios = torch::amin(gaussians_->scaling_, 1) / torch::amax(gaussians_->scaling_, 1);
            torch::Tensor clamped_ratios = torch::clamp_min(ratios, epsilon);
            torch::Tensor LIsoReg = torch::mean(clamped_ratios - epsilon);

            loss = loss + lambda_isoreg * LIsoReg;
        }

        loss.backward();

        torch::cuda::synchronize();

        {
            torch::NoGradGuard no_grad;
            ema_loss_for_log_ = 0.4f * loss.item().toFloat() + 0.6 * ema_loss_for_log_;

            if (keyframe_record_interval_ &&
                getIteration() % keyframe_record_interval_ == 0)
                recordKeyframeRendered(masked_image, gt_image, viewpoint_cam->fid_, result_dir_, result_dir_, result_dir_);

            // Densification
            if (getIteration() < opt_params_.densify_until_iter_)
            {
                // Keep track of max radii in image-space for pruning
                gaussians_->max_radii2D_.index_put_(
                    {visibility_filter},
                    torch::max(gaussians_->max_radii2D_.index({visibility_filter}),
                               radii.index({visibility_filter})));
                auto viewspace_point_tensor_densify = std::get<4>(render_pkg);
                gaussians_->addDensificationStats(viewspace_point_tensor_densify, visibility_filter);

                if ((getIteration() > opt_params_.densify_from_iter_) &&
                    (getIteration() % densifyInterval() == 0))
                {
                    gaussians_->densify(
                        densifyGradThreshold(),
                        scene_->cameras_extent_);
                }
                if ((getIteration() > opt_params_.densify_from_iter_) &&
                    (getIteration() % pruneInterval() == 0))
                {
                    int max_screen_size = (getIteration() > prune_big_point_after_iter_) ? 20 : 0;

                    gaussians_->prune(
                        densify_min_opacity_,
                        scene_->cameras_extent_,
                        max_screen_size);
                }

                if (opacityResetInterval() && (getIteration() % opacityResetInterval() == 0 || (model_params_.white_background_ && getIteration() == opt_params_.densify_from_iter_)))
                    gaussians_->resetOpacity();
            }

            auto iter_end_timing = std::chrono::steady_clock::now();
            auto iter_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 iter_end_timing - iter_start_timing)
                                 .count();

            // Log and save
            if (training_report_interval_ && (getIteration() % training_report_interval_ == 0))
                GaussianTrainer::trainingReport(
                    getIteration(),
                    opt_params_.iterations_,
                    Ll1,
                    loss,
                    ema_loss_for_log_,
                    loss_utils::l1_loss,
                    iter_time,
                    *gaussians_,
                    *scene_,
                    pipe_params_,
                    background_);
            if ((all_keyframes_record_interval_ && getIteration() % all_keyframes_record_interval_ == 0))
            {
                renderAndRecordAllKeyframes();
                savePly(result_dir_ / std::to_string(getIteration()) / "ply");
            }

            // Optimizer step
            if (getIteration() < opt_params_.iterations_)
            {
                gaussians_->optimizer_->step();
                gaussians_->optimizer_->zero_grad(true);
            }
        }
    }

} // namespace HSLAM
// gd -dev -16june2026