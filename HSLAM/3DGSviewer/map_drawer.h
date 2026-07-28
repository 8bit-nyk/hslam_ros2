#pragma once

#include "sophus/se3.hpp"  // gd -gui -27july2026

#include "drawer_utils.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <mutex>



class Settings;

class ImGuiMapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImGuiMapDrawer();

    void newParameterLoader();

    void SetCurrentCameraTwc(const Sophus::SE3f &Twc);
    void GetOpenGLCameraMatrix(const bool get_current, Sophus::SE3f &Tcw, glm::mat4 &glmTwc, glm::mat4 &MOw);

    void SetInitCameraTwc(const Sophus::SE3f &Twc);

    bool mbSetInitCamera = false;

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mInitCameraPose;
    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;
};
