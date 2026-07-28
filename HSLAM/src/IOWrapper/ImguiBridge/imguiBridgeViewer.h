// gd -gui -27july2026
#pragma once

#include <boost/thread.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "util/MinimalImage.h"
#include "util/settings.h"
#include "util/globalCalib.h"
#include "util/FrameShell.h"
#include "FullSystem/HessianBlocks.h"
#include "IOWrapper/Output3DWrapper.h"

namespace HSLAM
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

/**
 * @brief Publishes basic information to the 3DGS imgui viewer.
 *
 * Most of the gui handling is done by the 3DGS imgui viewer itself.
 */
class ImguiBrigeViewer : public Output3DWrapper
{
public:
        bool running;

        cv::Mat liveImage;      ///< CV_8UC3, written by TRACKING, read by the viewer thread
        bool haveLiveImage;

        unsigned int w;
        unsigned int h;

        CalibHessian *HCalib;

        Sophus::SE3f camToWorld;    ///< Twc, as consumed by ImGuiViewer

        ImguiBrigeViewer(int w_, int h_) :
        running(true), haveLiveImage(false), w(w_), h(h_), HCalib(0)
        {
            liveImage = cv::Mat::zeros(h, w, CV_8UC3);
        }

        ~ImguiBrigeViewer()
        {
        }

        void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
        {
        }

        void publishKeyframes(std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
        {
        }

        void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
        {
            this->HCalib = HCalib;

            // gd -gui -27july2026: H-SLAM keeps camToWorld private behind a mutex;
            // getPose() returns Twc, which is what MGSO's raw member held.
            camToWorld = frame->getPose().cast<float>();
        }

        void pushLiveFrame(FrameHessian* image, int nIndmatches) override
        {
            // gd -gui -27july2026: dI_c is a raw Eigen::Vector3f* of RGB in [0,255]
            // (see MappingOperation::addKeyFrame), not a cv::Mat in [0,1] as in MGSO.
            // Wrap it, then convert straight to 8-bit — no rescaling needed.
            if (!image->colourValid || image->dI_c == 0)
                return;

            cv::Mat color_raw(hG[0], wG[0], CV_32FC3, (void*)image->dI_c);
            color_raw.convertTo(liveImage, CV_8UC3);

            haveLiveImage = true;
        }

        void pushDepthImage(MinimalImageB3* image) override
        {
            // can be used to get the raw image with depth overlay.
        }

        bool needPushDepthImage() override
        {
            return false;
        }

        virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
        {
        }

        void join() override
        {
            haveLiveImage = false;
            running = false;
        }

        void reset() override
        {
            liveImage = cv::Mat::zeros(h, w, CV_8UC3);

            haveLiveImage = false;
        }
};

}
}
// gd -gui -27july2026
