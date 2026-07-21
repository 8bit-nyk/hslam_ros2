// gd -dev -16june2026
#include "GSIntegration/gaussian_mapper.h"

namespace HSLAM
{
    cv::Mat GaussianMapper::renderFromPose(
        const Sophus::SE3f &Tcw,
        const int width,
        const int height,
        const bool main_vision)
    {
        if (!initial_mapped_ || getIteration() <= 0)
            return cv::Mat(height, width, CV_32FC3, cv::Vec3f(0.0f, 0.0f, 0.0f));
        std::shared_ptr<GaussianKeyframe> pkf = std::make_shared<GaussianKeyframe>();
        pkf->zfar_ = z_far_;
        pkf->znear_ = z_near_;
        // Pose
        pkf->setPose(
            Tcw.unit_quaternion().cast<double>(),
            Tcw.translation().cast<double>());
        try
        {
            // Camera
            Camera &camera = scene_->cameras_.at(viewer_camera_id_);
            pkf->setCameraParams(camera);
            // Transformations
            pkf->computeTransformTensors();
        }
        catch (std::out_of_range)
        {
            throw std::runtime_error("[GaussianMapper::renderFromPose]KeyFrame Camera not found!");
        }

        std::tuple<at::Tensor, at::Tensor, at::Tensor, at::Tensor, at::Tensor> render_pkg;
        {
            std::unique_lock<std::mutex> lock_render(mutex_render_);
            // Render
            render_pkg = GaussianRenderer::render(
                pkf,
                height,
                width,
                gaussians_,
                pipe_params_,
                background_,
                override_color_);
        }

        // Result
        torch::Tensor masked_image;
        if (main_vision)
            masked_image = std::get<0>(render_pkg) * viewer_main_undistort_mask_[pkf->camera_id_];
        else
            masked_image = std::get<0>(render_pkg) * viewer_sub_undistort_mask_[pkf->camera_id_];
        return tensor_utils::torchTensor2CvMat_Float32(masked_image);
    }

    void GaussianMapper::recordKeyframeRendered(
        torch::Tensor &rendered,
        torch::Tensor &ground_truth,
        unsigned long kfid,
        std::filesystem::path result_img_dir,
        std::filesystem::path result_gt_dir,
        std::filesystem::path result_loss_dir,
        std::string name_suffix)
    {
        if (record_rendered_image_)
        {
            auto image_cv = tensor_utils::torchTensor2CvMat_Float32(rendered);
            cv::cvtColor(image_cv, image_cv, cv::COLOR_RGB2BGR);
            image_cv.convertTo(image_cv, CV_8UC3, 255.0f);
            cv::imwrite(result_img_dir / (std::to_string(getIteration()) + "_" + std::to_string(kfid) + name_suffix + ".jpg"), image_cv);
        }

        if (record_ground_truth_image_)
        {
            auto gt_image_cv = tensor_utils::torchTensor2CvMat_Float32(ground_truth);
            cv::cvtColor(gt_image_cv, gt_image_cv, cv::COLOR_RGB2BGR);
            gt_image_cv.convertTo(gt_image_cv, CV_8UC3, 255.0f);
            cv::imwrite(result_gt_dir / (std::to_string(getIteration()) + "_" + std::to_string(kfid) + name_suffix + "_gt.jpg"), gt_image_cv);
        }

        if (record_loss_image_)
        {
            torch::Tensor loss_tensor = torch::abs(rendered - ground_truth);
            auto loss_image_cv = tensor_utils::torchTensor2CvMat_Float32(loss_tensor);
            cv::cvtColor(loss_image_cv, loss_image_cv, cv::COLOR_RGB2BGR);
            loss_image_cv.convertTo(loss_image_cv, CV_8UC3, 255.0f);
            cv::imwrite(result_loss_dir / (std::to_string(getIteration()) + "_" + std::to_string(kfid) + name_suffix + "_loss.jpg"), loss_image_cv);
        }
    }

    void GaussianMapper::renderAndRecordKeyframe(
        std::shared_ptr<GaussianKeyframe> pkf,
        float &dssim,
        float &psnr,
        float &psnr_gs,
        double &render_time,
        std::filesystem::path result_img_dir,
        std::filesystem::path result_gt_dir,
        std::filesystem::path result_loss_dir,
        std::string name_suffix)
    {
        auto start_timing = std::chrono::steady_clock::now();
        auto render_pkg = GaussianRenderer::render(
            pkf,
            pkf->image_height_,
            pkf->image_width_,
            gaussians_,
            pipe_params_,
            background_,
            override_color_);
        auto rendered_image = std::get<0>(render_pkg);
        torch::Tensor masked_image = rendered_image * undistort_mask_[pkf->camera_id_];
        torch::cuda::synchronize();
        auto end_timing = std::chrono::steady_clock::now();
        auto render_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_timing - start_timing).count();
        render_time = 1e-6 * render_time_ns;
        auto gt_image = pkf->original_image_;

        dssim = loss_utils::ssim(masked_image, gt_image, device_type_).item().toFloat();
        psnr_gs = loss_utils::psnr_gaussian_splatting(masked_image, gt_image).item().toFloat();

        recordKeyframeRendered(masked_image, gt_image, pkf->fid_, result_img_dir, result_gt_dir, result_loss_dir, name_suffix);
    }

    void GaussianMapper::renderAndRecordAllKeyframes(
        std::string name_suffix)
    {
        std::filesystem::path result_dir = result_dir_ / (std::to_string(getIteration()) + name_suffix);
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)

        std::filesystem::path image_dir = result_dir / "image";
        if (record_rendered_image_)
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_dir);

        std::filesystem::path image_gt_dir = result_dir / "image_gt";
        if (record_ground_truth_image_)
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_gt_dir);

        std::filesystem::path image_loss_dir = result_dir / "image_loss";
        if (record_loss_image_)
        {
            CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(image_loss_dir);
        }

        std::filesystem::path render_time_path = result_dir / "render_time.txt";
        std::ofstream out_time(render_time_path);
        out_time << "##[Gaussian Mapper]Render time statistics: keyframe id, time(milliseconds)" << std::endl;

        std::filesystem::path dssim_path = result_dir / "dssim.txt";
        std::ofstream out_dssim(dssim_path);
        out_dssim << "##[Gaussian Mapper]keyframe id, dssim" << std::endl;

        std::filesystem::path psnr_gs_path = result_dir / "psnr.txt";
        std::ofstream out_psnr_gs(psnr_gs_path);
        out_psnr_gs << "##[Gaussian Mapper]keyframe id, psnr" << std::endl;

        float dssim, psnr, psnr_gs;
        double render_time;

        std::size_t nkfs = scene_->keyframes_deleted_.size();
        auto kfit = scene_->keyframes_deleted_.begin();

        for (std::size_t i = 0; i < nkfs; ++i)
        {
            renderAndRecordKeyframe((*kfit).second, dssim, psnr, psnr_gs, render_time, image_dir, image_gt_dir, image_loss_dir);
            out_time << (*kfit).first << " " << std::fixed << std::setprecision(8) << render_time << std::endl;

            out_dssim << (*kfit).first << " " << std::fixed << std::setprecision(10) << dssim << std::endl;
            out_psnr_gs << (*kfit).first << " " << std::fixed << std::setprecision(10) << psnr_gs << std::endl;

            ++kfit;
        }

        nkfs = scene_->keyframes().size();
        kfit = scene_->keyframes().begin();

        for (std::size_t i = 0; i < nkfs; ++i)
        {
            renderAndRecordKeyframe((*kfit).second, dssim, psnr, psnr_gs, render_time, image_dir, image_gt_dir, image_loss_dir);
            out_time << (*kfit).first << " " << std::fixed << std::setprecision(8) << render_time << std::endl;

            out_dssim << (*kfit).first << " " << std::fixed << std::setprecision(10) << dssim << std::endl;
            out_psnr_gs << (*kfit).first << " " << std::fixed << std::setprecision(10) << psnr_gs << std::endl;

            ++kfit;
        }
    }

    void GaussianMapper::saveModelParams(std::filesystem::path result_dir)
    {
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)
        std::filesystem::path result_path = result_dir / "cfg_args";
        std::ofstream out_stream;
        out_stream.open(result_path);
        if (!out_stream.is_open())
            throw std::runtime_error("Cannot open file at " + result_path.string());

        out_stream << "Namespace("
                   << "eval=" << (model_params_.eval_ ? "True" : "False") << ", "
                   << "images=" << "\'" << model_params_.images_ << "\', "
                   << "model_path=" << "\'" << model_params_.model_path_.string() << "\', "
                   << "resolution=" << model_params_.resolution_ << ", "
                   << "sh_degree=" << model_params_.sh_degree_ << ", "
                   << "source_path=" << "\'" << model_params_.source_path_.string() << "\', "
                   << "white_background=" << (model_params_.white_background_ ? "True" : "False") << ", "
                   << ")";

        out_stream.close();
    }

    void GaussianMapper::writeKeyframeUsedTimes(std::filesystem::path result_dir, std::string name_suffix)
    {
        CHECK_DIRECTORY_AND_CREATE_IF_NOT_EXISTS(result_dir)
        std::filesystem::path result_path = result_dir / ("keyframe_used_times" + name_suffix + ".txt");
        std::ofstream out_stream;
        out_stream.open(result_path, std::ios::app);
        if (!out_stream.is_open())
            throw std::runtime_error("Cannot open json at " + result_path.string());

        out_stream << "##[Gaussian Mapper]Iteration " << getIteration() << " keyframe id, used times, remaining times:\n";
        for (const auto &used_times_it : kfs_used_times_)
        {
            if (scene_->keyframes().count(used_times_it.first) > 0)
            {
                out_stream << used_times_it.first << " "
                           << used_times_it.second << " "
                           << scene_->keyframes().at(used_times_it.first)->remaining_times_of_use_
                           << "\n";
            }
            if (scene_->keyframes_deleted_.count(used_times_it.first) > 0)
            {
                out_stream << used_times_it.first << " "
                           << used_times_it.second << " "
                           << scene_->keyframes_deleted_.at(used_times_it.first)->remaining_times_of_use_
                           << "\n";
            }
        }
        out_stream << "##=========================================" << std::endl;

        out_stream.close();
    }

    void GaussianMapper::loadPly(std::filesystem::path ply_path, std::filesystem::path camera_path)
    {
        this->gaussians_->loadPly(ply_path);

        // Camera
        if (!camera_path.empty() && std::filesystem::exists(camera_path))
        {
            cv::FileStorage camera_file(camera_path.string().c_str(), cv::FileStorage::READ);
            if (!camera_file.isOpened())
                throw std::runtime_error("[Gaussian Mapper]Failed to open settings file at: " + camera_path.string());

            Camera camera;
            camera.camera_id_ = 0;
            camera.width_ = camera_file["Camera.w"].operator int();
            camera.height_ = camera_file["Camera.h"].operator int();

            std::string camera_type = camera_file["Camera.type"].string();
            if (camera_type == "Pinhole")
            {
                camera.setModelId(Camera::CameraModelType::PINHOLE);

                float fx = camera_file["Camera.fx"].operator float();
                float fy = camera_file["Camera.fy"].operator float();
                float cx = camera_file["Camera.cx"].operator float();
                float cy = camera_file["Camera.cy"].operator float();

                float k1 = camera_file["Camera.k1"].operator float();
                float k2 = camera_file["Camera.k2"].operator float();
                float p1 = camera_file["Camera.p1"].operator float();
                float p2 = camera_file["Camera.p2"].operator float();
                float k3 = camera_file["Camera.k3"].operator float();

                cv::Mat K = (cv::Mat_<float>(3, 3)
                                 << fx,
                             0.f, cx,
                             0.f, fy, cy,
                             0.f, 0.f, 1.f);

                camera.params_[0] = fx;
                camera.params_[1] = fy;
                camera.params_[2] = cx;
                camera.params_[3] = cy;

                std::vector<float> dist_coeff = {k1, k2, p1, p2, k3};
                camera.dist_coeff_ = cv::Mat(5, 1, CV_32F, dist_coeff.data());
                camera.initUndistortRectifyMapAndMask(K, cv::Size(camera.width_, camera.height_), K, false);

                undistort_mask_[camera.camera_id_] =
                    tensor_utils::cvMat2TorchTensor_Float32(
                        camera.undistort_mask, device_type_);

                cv::Mat viewer_main_undistort_mask;
                int viewer_image_height_main_ = camera.height_ * rendered_image_viewer_scale_main_;
                int viewer_image_width_main_ = camera.width_ * rendered_image_viewer_scale_main_;
                cv::resize(camera.undistort_mask, viewer_main_undistort_mask,
                           cv::Size(viewer_image_width_main_, viewer_image_height_main_));
                viewer_main_undistort_mask_[camera.camera_id_] =
                    tensor_utils::cvMat2TorchTensor_Float32(
                        viewer_main_undistort_mask, device_type_);
            }
            else
            {
                throw std::runtime_error("[Gaussian Mapper]Unsupported camera model: " + camera_path.string());
            }

            if (!viewer_camera_id_set_)
            {
                viewer_camera_id_ = camera.camera_id_;
                viewer_camera_id_set_ = true;
            }
            this->scene_->addCamera(camera);
        }

        // Ready
        this->initial_mapped_ = true;
        increaseIteration();
    }

} // namespace HSLAM
// gd -dev -16june2026