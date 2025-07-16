# HSLAM Depth Estimation Architecture

## 1. Overview

This document outlines the architecture and lifecycle of depth estimation within the HSLAM system. Depth is one of the most critical components of the algorithm, and a clear understanding of its handling is essential for research and development.

HSLAM, being a monocular system, cannot measure depth directly. Instead, it estimates depth through motion. The system represents depth as **inverse depth** (`idepth`), which simplifies handling of points at infinity and often leads to more stable optimization.

The process of estimating, refining, and using depth is continuous and integrated into all major components of the SLAM pipeline.

## 2. Depth Lifecycle

The lifecycle of a 3D point's depth can be broken down into four key stages:

### 2.1. Initialization

-   **Component:** `CoarseInitializer`
-   **File:** `FullSystem/CoarseInitializer.cpp`
-   **Process:** The system requires an initial motion between the first two frames to perform triangulation. This establishes an initial set of 3D map points and their corresponding depths. This step is crucial as it sets the initial scale for the entire map. An incorrect initial scale will cause the entire reconstruction to be scaled incorrectly.

### 2.2. Tracking

-   **Component:** `CoarseTracker`
-   **File:** `FullSystem/CoarseTracker.cpp`
-   **Process:** The direct tracking module uses a semi-dense depth map of a reference keyframe to track the camera's pose.
    1.  A depth map is constructed for the reference keyframe by projecting existing map points into it (`makeCoarseDepthL0`).
    2.  These points, with their known depth, are then projected into the new camera frame.
    3.  The system minimizes the photometric error (difference in pixel intensity) between the reference and the new frame. This optimization process yields the relative pose of the new frame. The accuracy of the depth map directly impacts the tracking quality.

### 2.3. Mapping and Depth Refinement

-   **Component:** `ImmaturePoint` Tracker
-   **File:** `FullSystem/ImmaturePoint.cpp`
-   **Process:** When new, high-gradient pixels are identified in a keyframe, they are initialized as "immature points."
    1.  These points begin with a highly uncertain depth, represented by a wide inverse depth range (`idepth_min` to `idepth_max`).
    2.  The system attempts to track these immature points across subsequent frames by searching along their epipolar line (`trace` function).
    3.  Each successful match provides a new geometric constraint, allowing the system to tighten the inverse depth range, thereby refining the depth estimate.
    4.  Once the uncertainty (`idepth_max` - `idepth_min`) falls below a specific threshold, the point is considered "mature" and is promoted to a full map point.

### 2.4. Global Optimization

-   **Component:** Optimization Backend (g2o)
-   **Files:** `Indirect/Optimizer.h`, `Indirect/MapPoint.cpp`
-   **Process:** Mature map points are included in the global optimization process (Bundle Adjustment).
    1.  The `MapPoint` data structure stores the point's final `idepth` value and its uncertainty.
    2.  The inverse depth of each point is treated as a variable in the global optimization problem (`EdgeProjectinvDepth`).
    3.  The g2o-based optimizer adjusts the poses of all keyframes and the inverse depths of all map points simultaneously to minimize the global reprojection error. This ensures that the entire map and trajectory are geometrically consistent.

## 3. Key Data Structures and Files

-   **Core Logic:**
    -   `FullSystem/CoarseTracker.cpp`: Pose tracking using semi-dense depth maps.
    -   `FullSystem/ImmaturePoint.cpp`: New point depth refinement.
-   **Data Management:**
    -   `Indirect/MapPoint.cpp`: Represents a mature 3D point in the map.
    -   `FullSystem/PointHessian.h`: Represents a point within the direct tracking and optimization framework.
-   **Optimization:**
    -   `Indirect/Optimizer.h`: Defines the g2o edges for bundle adjustment, where `idepth` is an optimization variable.
