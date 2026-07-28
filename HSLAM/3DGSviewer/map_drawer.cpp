#include "map_drawer.h"



ImGuiMapDrawer::ImGuiMapDrawer()
{
    newParameterLoader();
}

void ImGuiMapDrawer::newParameterLoader() {
    mKeyFrameSize = 0.1f;
    mKeyFrameLineWidth = 1.0f;
    mGraphLineWidth = 1.0f;
    mPointSize = 2.0f;
    mCameraSize = 0.15f;
    mCameraLineWidth  = 2.0f;
}

void ImGuiMapDrawer::SetCurrentCameraTwc(const Sophus::SE3f &Twc)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mCameraPose = Twc;
}

void ImGuiMapDrawer::SetInitCameraTwc(const Sophus::SE3f &Twc)
{
    std::unique_lock<std::mutex> lock(mMutexCamera);
    mInitCameraPose = Twc;
}

void ImGuiMapDrawer::GetOpenGLCameraMatrix(const bool get_current, Sophus::SE3f &Tcw, glm::mat4 &glmTwc, glm::mat4 &MOw)
{
    Eigen::Matrix4f Twc;
    {
        std::unique_lock<std::mutex> lock(mMutexCamera);
        if (get_current)
        {
            Twc = mCameraPose.matrix();
            Tcw = mCameraPose.inverse();
        }
        else
        {
            Twc = mInitCameraPose.matrix();
            Tcw = mInitCameraPose.inverse();
        }
    }
    glmTwc = trans4x4Eigen2glm(Twc);

    MOw = glm::mat4(1.0f);
    MOw[3][0] = Twc(0, 3);
    MOw[3][1] = Twc(1, 3);
    MOw[3][2] = Twc(2, 3);
}
