# CoarseTracker Depth Fusion Logic Analysis

## Overview

This document provides a detailed analysis of the depth fusion logic in HSLAM's CoarseTracker, explaining why the number of fused pixels is significantly lower than the available external depth pixels.

## Key Statistics from Logs

From the depth integration logs, we observe:
- **External Depth Pixels**: ~248,000 (available from ground truth)
- **Fused Pixels**: ~1,500-2,500 (pixels where external depth fuses with existing map points)
- **Total Valid Pixels**: ~250,000 (pixels with any depth information)

**Ratio**: Only ~1% of external depth pixels are fused, while ~97% are used directly.

## Depth Fusion Architecture

### 1. Two-Stage Integration Process

The depth integration follows a sequential process in `makeCoarseDepthL0Enhanced()`:

```cpp
// Step 2: Integrate existing map points (preserve current functionality)
for(FrameHessian* fh : frameHessians) {
    for(PointHessian* ph : fh->pointHessians) {
        // ... map point integration
        last_stats.pixels_from_map_points++;
    }
}

// Step 3: Integrate external depth if available
if(hasExternalDepth()) {
    integrateExternalDepthL0();
}
```

### 2. Core Fusion Logic in `integrateExternalDepthL0()`

The fusion decision is made per pixel:

```cpp
// Check if map point already exists at this location
bool has_map_point = (weightSums[0][idx] > 0);

if(has_map_point) {
    // Fuse external depth with existing map point
    float existing_idepth = idepth[0][idx] / weightSums[0][idx];
    float existing_weight = weightSums[0][idx];
    
    // Weighted combination of map point and external depth
    float combined_weight = existing_weight + external_weight;
    float combined_idepth = (existing_idepth * existing_weight + 
                           external_idepth * external_weight) / combined_weight;
    
    idepth[0][idx] = combined_idepth * combined_weight;
    weightSums[0][idx] = combined_weight;
    
    fused_count++;  // ← This is the "fused" statistic
} else {
    // Use external depth directly
    idepth[0][idx] = external_idepth * external_weight;
    weightSums[0][idx] = external_weight;
    
    integrated_count++;  // ← This is "pixels_from_external_depth"
}
```

## Why Fused Pixels Are Low

### 1. **Spatial Distribution Mismatch**

**Map Points Distribution:**
- Map points are sparse and concentrated on high-gradient features
- Typically 1,500-2,000 active map points per frame
- Distributed across ~640×480 = 307,200 pixels
- Coverage: ~0.5-0.7% of image pixels

**External Depth Distribution:**
- Dense depth information (every pixel has ground truth)
- 307,200 pixels of external depth data
- Coverage: ~100% of image pixels

**Result:** Only ~1,500-2,500 pixels have both map points AND external depth.

### 2. **Feature-Based vs. Dense Depth**

**Map Points (HSLAM's Approach):**
```cpp
// Map points are created only on high-gradient features
if(ph->lastResiduals[0].first != 0 && ph->lastResiduals[0].second == ResState::IN) {
    // Only active, inlier map points contribute
    int u = r->centerProjectedTo[0] + 0.5f;
    int v = r->centerProjectedTo[1] + 0.5f;
    // ... integration
}
```

**External Depth (Ground Truth):**
```cpp
// Every pixel has depth information
for(int v = 0; v < h[0]; v++) {
    for(int u = 0; u < w[0]; u++) {
        float external_depth = external_depth_image.at<float>(v, u);
        // ... validation and integration
    }
}
```

### 3. **Validation Filters**

External depth pixels are filtered through multiple validation steps:

```cpp
bool CoarseTracker::validateExternalDepth(float depth, int u, int v)
{
    // Range validation (TUM dataset appropriate)
    if(depth < 0.1f || depth > 10.0f) return false;
    
    // Finite validation
    if(!std::isfinite(depth)) return false;
    
    // Image bounds validation
    if(u < 0 || u >= w[0] || v < 0 || v >= h[0]) return false;
    
    return true;
}
```

**Impact:** Invalid depth pixels (NaN, out-of-range, boundary pixels) are excluded from both direct integration and fusion.

### 4. **Confidence Weighting**

The fusion uses weighted combination based on confidence:

```cpp
float CoarseTracker::calculateConfidenceWeight(float depth, int u, int v, bool is_external)
{
    if(is_external) {
        float base_confidence = 0.9f;  // High confidence for ground truth
        float depth_factor = 1.0f / (1.0f + depth * 0.1f);  // Closer depths more reliable
        
        // Get image gradient at this location
        float gradient_factor = 1.0f;
        if(lastRef && lastRef->dIp[0]) {
            Eigen::Vector3f grad = lastRef->dIp[0][u + v * w[0]];
            gradient_factor = std::min(2.0f, std::max(0.5f, grad.norm() * 0.01f));
        }
        
        return base_confidence * depth_factor * gradient_factor;
    }
}
```

**Impact:** Low-confidence pixels may have minimal contribution to fusion.

## Statistical Breakdown

### Typical Frame Analysis (640×480 resolution):

1. **Total Pixels**: 307,200
2. **Valid External Depth**: ~248,000 (80.7%)
   - Invalid: NaN, out-of-range, boundary pixels
3. **Map Points**: ~1,500-2,000 (0.5-0.7%)
   - Sparse feature-based distribution
4. **Overlap (Fusion Candidates)**: ~1,500-2,500 (0.5-0.8%)
   - Pixels with both map points AND valid external depth
5. **Direct External Integration**: ~245,500 (79.9%)
   - Pixels with external depth but no map point
6. **Fused Pixels**: ~1,500-2,500 (0.5-0.8%)
   - Pixels where both sources contribute

## Expected Behavior

The low fusion rate is **expected and correct** because:

1. **HSLAM is Feature-Based**: Map points are created only on high-gradient features, not dense depth
2. **Ground Truth is Dense**: External depth provides information for every pixel
3. **Sparse-Dense Mismatch**: Only ~0.5-0.8% of pixels have both sources
4. **High Integration Rate**: ~97% of valid pixels use external depth (either directly or fused)

## Code References

### Key Methods:

1. **`makeCoarseDepthL0Enhanced()`** (lines 1063-1200)
   - Main integration orchestrator
   - Sequential map point → external depth integration

2. **`integrateExternalDepthL0()`** (lines 1240-1315)
   - Core fusion logic
   - Per-pixel decision: fuse vs. direct integration

3. **`validateExternalDepth()`** (lines 1315-1338)
   - External depth validation filters
   - Range, finite, and boundary checks

4. **`calculateConfidenceWeight()`** (lines 1338-1368)
   - Confidence-based weighting for fusion
   - Depth, gradient, and source-specific factors

### Data Structures:

- **`idepth[0]`**: Level 0 inverse depth map
- **`weightSums[0]`**: Confidence weights for each pixel
- **`external_depth_image`**: Ground truth depth (CV_32FC1)
- **`last_stats`**: Integration statistics tracking

## Conclusion

The low fusion rate is **by design** and reflects the fundamental difference between HSLAM's sparse feature-based approach and dense ground truth depth. The system correctly prioritizes external depth (97% integration rate) while maintaining the ability to fuse with existing map points where available.

The fusion logic is working as intended - the statistics reflect the sparse nature of HSLAM's map points rather than a flaw in the fusion algorithm.
