/**
 * MappingOperation.hpp
 * Adapted from MGSO (Yan Song Hu) for H-SLAM integration.
 * Packages keyframes and points from H-SLAM's active window
 * into a data transfer object consumed by the GaussianMapper thread.
 */
// gd -dev -8june2026 
#pragma once

#include "Eigen/Core"
#include "sophus/se3.hpp"

#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/globalCalib.h"

#include <vector>
#include <tuple>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <opencv2/opencv.hpp>

namespace HSLAM {
namespace GSI {

enum class PointType : uint8_t {
    DIRECT = 0,   // PointHessian with expired/bad MapPoint link
    HYBRID = 1    // PointHessian with valid MapPoint link (thesis contribution)
};

class MappingOperation
{
public:
    enum OprType {
        LocalMappingBA = 1
    };

private:
    // Private copy-with-locks constructor (thread-safe copy pattern from MGSO)
    MappingOperation(
        const MappingOperation& opr,
        const boost::lock_guard<boost::mutex>&,
        const boost::lock_guard<boost::mutex>&)
        : mvAssociatedKeyFrames(opr.mvAssociatedKeyFrames),
          mvAssociatedMapPoints(opr.mvAssociatedMapPoints),
          meOperationType(opr.meOperationType),
          mfScale(opr.mfScale),
          mT(opr.mT),
          Hcalib(opr.Hcalib)
    {}

public:
    MappingOperation(
        CalibHessian* globalCalib,
        OprType type,
        const float scale = 1.0f,
        const Sophus::SE3d T = Sophus::SE3d(
            Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()),
        const std::size_t nKFs = 0UL,
        const std::size_t nMPs = 0UL)
        : meOperationType(type),
          mfScale(scale),
          mT(T),
          Hcalib(globalCalib)
    {
        mvAssociatedKeyFrames.reserve(nKFs);
        int length = nMPs * 3;
        std::get<0>(mvAssociatedMapPoints).reserve(length);
        std::get<1>(mvAssociatedMapPoints).reserve(length);
    }

    MappingOperation(const MappingOperation& opr)
        : MappingOperation(
            opr,
            boost::lock_guard<boost::mutex>(opr.mMutexKeyFrames),
            boost::lock_guard<boost::mutex>(opr.mMutexMapPoints))
    {}

public:
    void reserveKeyFrames(const std::size_t nKFs) {
        if (nKFs > 0) mvAssociatedKeyFrames.reserve(nKFs);
    }

    void addKeyFrame(FrameHessian* pKF)
    {
        boost::unique_lock<boost::mutex> lock(mMutexKeyFrames);

        // dI_c is Eigen::Vector3f* storing 0-255 RGB values.
        // Wrap as CV_32FC3, then normalize to [0,1] — GS training expects [0,1].
        cv::Mat color_raw(hG[0], wG[0], CV_32FC3, (void*)pKF->dI_c);
        cv::Mat color_img;
        color_raw.convertTo(color_img, CV_32FC3, 1.0 / 255.0);

        mvAssociatedKeyFrames.emplace_back(std::make_tuple(
            (unsigned long)pKF->frameID,
            0UL,
            // pKF->shell->camToWorld.inverse(),  // Tcw (world-to-camera)
            pKF->shell->getPoseInverse(),      // gd -dev -9june2026 
            color_img,
            std::to_string(pKF->frameID)        // synthetic name (no image_name in H-SLAM)
        ));
    }

    std::vector<std::tuple<
        unsigned long,   // keyframe ID
        unsigned long,   // camera ID (always 0)
        Sophus::SE3d,    // Tcw pose
        cv::Mat,         // color image [0,1] float
        std::string      // synthetic name
    >>& associatedKeyFrames() { return mvAssociatedKeyFrames; }

    void reserveMapPoints(const std::size_t nMPs) {
        int length = nMPs * 3;
        if (length > 0) {
            std::get<0>(mvAssociatedMapPoints).reserve(length);
            std::get<1>(mvAssociatedMapPoints).reserve(length);
        }
    }

    // Templated so it works for both PointHessian* and ImmaturePoint*
    // (both now expose getWorldPosition, getColourRGBfloat, getHostFrameID)
    template<typename PointT>
    void addMapPoint(PointT* p, const SE3& camToWorld,
                     bool noise, PointType type, float mp_std_dev)
    {
        boost::unique_lock<boost::mutex> lock(mMutexMapPoints);

        Eigen::Vector3d pt = p->getWorldPosition(
            1.0f / Hcalib->fxl(),
            1.0f / Hcalib->fyl(),
            -Hcalib->cxl() / Hcalib->fxl(),
            -Hcalib->cyl() / Hcalib->fyl(),
            camToWorld);

        if (noise) {
            std::get<0>(mvAssociatedMapPoints).emplace_back(
                static_cast<float>(pt[0] * (1 + float(50-(rand()%100)) / 5000)));
            std::get<0>(mvAssociatedMapPoints).emplace_back(
                static_cast<float>(pt[1] * (1 + float(50-(rand()%100)) / 5000)));
            std::get<0>(mvAssociatedMapPoints).emplace_back(
                static_cast<float>(pt[2] * (1 + float(50-(rand()%100)) / 5000)));
        } else {
            std::get<0>(mvAssociatedMapPoints).emplace_back(static_cast<float>(pt[0]));
            std::get<0>(mvAssociatedMapPoints).emplace_back(static_cast<float>(pt[1]));
            std::get<0>(mvAssociatedMapPoints).emplace_back(static_cast<float>(pt[2]));
        }

        Eigen::Vector3f color = p->getColourRGBfloat();
        std::get<1>(mvAssociatedMapPoints).emplace_back(color[0]);
        std::get<1>(mvAssociatedMapPoints).emplace_back(color[1]);
        std::get<1>(mvAssociatedMapPoints).emplace_back(color[2]);

        std::get<2>(mvAssociatedMapPoints).emplace_back(p->u);
        std::get<3>(mvAssociatedMapPoints).emplace_back(p->v);
        std::get<4>(mvAssociatedMapPoints).emplace_back(p->getHostFrameID());
        std::get<5>(mvAssociatedMapPoints).emplace_back(static_cast<uint8_t>(type));
        std::get<6>(mvAssociatedMapPoints).emplace_back(mp_std_dev);
    }

    std::tuple<
        std::vector<float>,    // xyz positions (stride 3)
        std::vector<float>,    // RGB colors    (stride 3, [0,1])
        std::vector<float>,    // screen-space u
        std::vector<float>,    // screen-space v
        std::vector<int>,      // hostFrameID
        std::vector<uint8_t>,  // point_type (0=DIRECT, 1=HYBRID)
        std::vector<float>     // mp_std_dev (0 for DIRECT points)
    >& associatedMapPoints() { return mvAssociatedMapPoints; }

public:
    OprType meOperationType;
    float mfScale;
    Sophus::SE3d mT;

protected:
    std::tuple<
        std::vector<float>,
        std::vector<float>,
        std::vector<float>,
        std::vector<float>,
        std::vector<int>,
        std::vector<uint8_t>,
        std::vector<float>
    > mvAssociatedMapPoints;

    std::vector<std::tuple<
        unsigned long,
        unsigned long,
        Sophus::SE3d,
        cv::Mat,
        std::string
    >> mvAssociatedKeyFrames;

    mutable boost::mutex mMutexMapPoints;
    mutable boost::mutex mMutexKeyFrames;

    CalibHessian* Hcalib;
};

} // namespace GSI
} // namespace HSLAM
// gd -dev -8june2026 