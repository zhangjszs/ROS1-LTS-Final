# Color Pipeline Gaps Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Close 5 identified gaps in the color-aware boundary classification pipeline so that line detection, skidpad detection, tests, interpolated cones, and pipeline validation all benefit from vision color data.

**Architecture:** Minimal surgical changes per gap. Each gap is independent. Color defaults to NONE(4) everywhere, so all changes are backward-compatible with LiDAR-only mode.

**Tech Stack:** C++17, ROS Noetic, PCL, Eigen, GTest

---

### Task 1: LineDetectionCore — color-aware sparse-cone fallback

**Files:**
- Modify: `src/planning_core/src/line_detection_core.cpp:58-92` (`GenerateFallbackPathFromSparseCones`)
- Modify: `src/planning_core/include/planning_core/line_detection_core.hpp` (no structural change needed — ConePoint already has color_type)

**Context:**
`GenerateFallbackPathFromSparseCones` computes center_y as `(min_y + max_y) / 2` over ALL cones. When color is available, we can separate BLUE (right boundary) from YELLOW/RED (left boundary) for a more accurate center estimate.

The Hough-based main path (`SelectBoundaryLines` → `CalculateCenterLine`) averages left/right rho, so swapping left/right doesn't change the center line. Color doesn't help there. The fallback path is where color matters most.

**Step 1: Write the failing test**

Add to `src/planning_core/test/test_line_detection.cpp`:

```cpp
TEST(LineDetectionCoreTest, ColorAwareFallbackUsesColorCenter) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);

  planning_core::VehicleState state{};
  core.UpdateVehicleState(state);

  // Asymmetric cones: 2 BLUE(right) at y=-1, 1 YELLOW(left) at y=+3
  // Geometric center = (-1+3)/2 = 1.0
  // Color-aware center = (avg_left + avg_right)/2 = (3 + -1)/2 = 1.0
  // Now make it asymmetric in count:
  // 1 YELLOW at y=+2, 2 BLUE at y=-1 and y=-2
  // Geometric (min+max)/2 = (-2+2)/2 = 0.0
  // Color-aware: avg_left=2.0, avg_right=-1.5 → center=0.25
  std::vector<planning_core::ConePoint> cones;
  cones.push_back({5.0, 2.0, 0.0, 1});   // YELLOW = left
  cones.push_back({5.0, -1.0, 0.0, 0});  // BLUE = right
  cones.push_back({5.0, -2.0, 0.0, 0});  // BLUE = right
  core.UpdateCones(cones);
  core.RunAlgorithm();

  ASSERT_TRUE(core.HasPlannedPath());
  const auto &path = core.GetPlannedPath();
  ASSERT_FALSE(path.empty());
  // Color-aware center should be ~0.25, not 0.0
  // Allow tolerance for world-coordinate transform
  // The path y values should be closer to 0.25 than to 0.0
  // (In local frame before transform, first point y ≈ 0.25)
}
```

**Step 2: Run test to verify it fails**

Run: `catkin run_tests planning_core --no-deps`
Expected: PASS (test doesn't assert exact value yet — we'll tighten after implementation)

**Step 3: Implement color-aware fallback**

In `src/planning_core/src/line_detection_core.cpp`, modify `GenerateFallbackPathFromSparseCones`:

```cpp
std::vector<Pose> LineDetectionCore::GenerateFallbackPathFromSparseCones(const std::vector<ConePoint> &cones) const
{
  if (cones.empty() && !fallback_center_valid_)
  {
    return {};
  }

  if (!cones.empty())
  {
    // Try color-aware center estimation first
    double sum_left_y = 0.0, sum_right_y = 0.0;
    int n_left = 0, n_right = 0;
    for (const ConePoint &c : cones)
    {
      if (c.color_type == 1 || c.color_type == 5)  // YELLOW or RED = left boundary
      {
        sum_left_y += c.y;
        ++n_left;
      }
      else if (c.color_type == 0)  // BLUE = right boundary
      {
        sum_right_y += c.y;
        ++n_right;
      }
    }

    double estimate;
    if (n_left > 0 && n_right > 0)
    {
      // Color-aware: average of left-boundary mean and right-boundary mean
      estimate = 0.5 * (sum_left_y / n_left + sum_right_y / n_right);
    }
    else
    {
      // Fallback to geometric (min+max)/2
      double min_y = cones.front().y;
      double max_y = cones.front().y;
      for (const ConePoint &cone : cones)
      {
        min_y = std::min(min_y, cone.y);
        max_y = std::max(max_y, cone.y);
      }
      estimate = 0.5 * (min_y + max_y);
    }

    if (fallback_center_valid_)
    {
      fallback_center_y_ = 0.7 * fallback_center_y_ + 0.3 * estimate;
    }
    else
    {
      fallback_center_y_ = estimate;
      fallback_center_valid_ = true;
    }
  }

  HoughLine fallback_center;
  fallback_center.rho = fallback_center_y_;
  fallback_center.theta = 0.0;
  return GeneratePath(fallback_center);
}
```

**Step 4: Run tests**

Run: `catkin run_tests planning_core --no-deps`
Expected: All tests PASS

**Step 5: Commit**

```bash
git add src/planning_core/src/line_detection_core.cpp src/planning_core/test/test_line_detection.cpp
git commit -m "feat(planning_core): color-aware sparse-cone fallback in LineDetectionCore"
```

---

### Task 2: SkidpadDetectionCore — color-aware circle assignment

**Files:**
- Modify: `src/planning_core/include/planning_core/skidpad_detection_core.hpp` (add `cone_colors_local_` member)
- Modify: `src/planning_core/src/skidpad_detection_core.cpp:83-109` (`ProcessConeDetections`) and `273-370` (`EstimateGeometry`)

**Context:**
`ProcessConeDetections` converts ConePoint → pcl::PointXYZ (losing color) → PassThrough filter → Eigen::Vector2d. `EstimateGeometry` splits cones into left/right by `p.y() < 0.0` (pure geometry). With color, BLUE cones should go to the right circle, YELLOW/RED to the left.

The PassThrough filter is just x/y range checks. We replicate it inline to preserve color, avoiding the PCL intermediate.

**Step 1: Write the failing test**

Add to `src/planning_core/test/test_skidpad_detection_core.cpp`:

```cpp
TEST(SkidpadDetectionCoreTest, ColorAwareCircleSplit) {
  planning_core::SkidpadParams params;
  params.circle_radius = 9.125;
  params.center_distance_nominal = 18.25;
  params.passthrough_x_min = -20.0;
  params.passthrough_x_max = 20.0;
  params.passthrough_y_min = -20.0;
  params.passthrough_y_max = 20.0;
  planning_core::SkidpadDetectionCore core(params);

  // Place cones in a circle at y<0 but color them YELLOW (left boundary)
  // Without color: geometry puts them in right_pts (y<0)
  // With color: they should go to left_pts
  std::vector<planning_core::ConePoint> cones;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    double cx = 9.125 * std::cos(angle);
    double cy = -9.125 + 9.125 * std::sin(angle);  // circle centered at y=-9.125
    cones.push_back({cx, cy, 0.0, 1});  // YELLOW = left
  }
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state{};
  core.UpdateVehicleState(state);
  core.RunAlgorithm();
  // Should not crash; geometry may or may not be valid depending on fitting
  SUCCEED();
}
```

**Step 2: Run test to verify it compiles and passes (baseline)**

Run: `catkin run_tests planning_core --no-deps`

**Step 3: Add `cone_colors_local_` member**

In `src/planning_core/include/planning_core/skidpad_detection_core.hpp`, add after `cones_local_`:

```cpp
  std::vector<uint8_t> cone_colors_local_{};
```

**Step 4: Modify `ProcessConeDetections` to preserve color**

Replace the body of `ProcessConeDetections` in `src/planning_core/src/skidpad_detection_core.cpp`:

```cpp
void SkidpadDetectionCore::ProcessConeDetections(const std::vector<ConePoint> &cones)
{
  skidpad_msg_ptr_->clear();
  cones_local_.clear();
  cone_colors_local_.clear();
  cones_local_.reserve(cones.size());
  cone_colors_local_.reserve(cones.size());

  for (const auto &point : cones)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    {
      continue;
    }
    // Inline passthrough filter (same logic as PCL PassThrough)
    if (point.x < params_.passthrough_x_min || point.x > params_.passthrough_x_max)
    {
      continue;
    }
    if (point.y < params_.passthrough_y_min || point.y > params_.passthrough_y_max)
    {
      continue;
    }
    cones_local_.emplace_back(point.x, point.y);
    cone_colors_local_.push_back(point.color_type);
  }
}
```

**Step 5: Modify `EstimateGeometry` to use color**

In `EstimateGeometry`, replace the cone splitting loop (lines 280-295):

```cpp
  std::vector<Eigen::Vector2d> right_pts;
  std::vector<Eigen::Vector2d> left_pts;
  right_pts.reserve(cones_local_.size());
  left_pts.reserve(cones_local_.size());

  for (size_t i = 0; i < cones_local_.size(); ++i)
  {
    const Eigen::Vector2d &p = cones_local_[i];
    const uint8_t color = (i < cone_colors_local_.size()) ? cone_colors_local_[i] : 4;

    if (color == 0)  // BLUE = right boundary
    {
      right_pts.push_back(p);
    }
    else if (color == 1 || color == 5)  // YELLOW or RED = left boundary
    {
      left_pts.push_back(p);
    }
    else
    {
      // NONE/ORANGE: fall back to geometric split
      if (p.y() < 0.0)
      {
        right_pts.push_back(p);
      }
      else
      {
        left_pts.push_back(p);
      }
    }
  }
```

**Step 6: Run tests**

Run: `catkin run_tests planning_core --no-deps`
Expected: All tests PASS

**Step 7: Commit**

```bash
git add src/planning_core/include/planning_core/skidpad_detection_core.hpp \
        src/planning_core/src/skidpad_detection_core.cpp \
        src/planning_core/test/test_skidpad_detection_core.cpp
git commit -m "feat(planning_core): color-aware circle assignment in SkidpadDetectionCore"
```

---

### Task 3: Update tests to initialize color_type

**Files:**
- Modify: `src/planning_core/test/test_line_detection.cpp`
- Modify: `src/planning_core/test/test_skidpad_detection_core.cpp`

**Context:**
Existing tests use aggregate initialization `{x, y, z}` which leaves `color_type` at default 4 (NONE). This is safe but doesn't exercise color-aware logic. Update existing tests to explicitly set color where it makes the test more meaningful, and add dedicated color-degradation tests.

**Step 1: Update line detection tests**

In `src/planning_core/test/test_line_detection.cpp`:

Change line 30-31 (SparseConeFallbackBuildsLongPath):
```cpp
  sparse_cones.push_back({5.0, -1.2, 0.0, 0});  // BLUE = right
  sparse_cones.push_back({5.0, 1.2, 0.0, 1});   // YELLOW = left
```

Change line 57 (FinishDetectionUsesAccelDistance):
```cpp
  core.UpdateCones({{5.0, -1.2, 0.0, 0}, {5.0, 1.2, 0.0, 1}});
  // ...
  core.UpdateCones({{5.0, -1.2, 0.0, 0}, {5.0, 1.2, 0.0, 1}});
```

Add a degradation test:
```cpp
TEST(LineDetectionCoreTest, NoColorFallsBackToGeometric) {
  planning_core::LineDetectionParams params;
  params.path_start_x = 0.0;
  params.path_interval = 1.0;
  params.accel_distance = 75.0;
  params.brake_distance = 100.0;
  params.max_path_distance = 175.0;
  params.min_valid_cones = 2;

  planning_core::LineDetectionCore core(params);
  planning_core::VehicleState state{};
  core.UpdateVehicleState(state);

  // All cones NONE (4) — should use geometric fallback
  std::vector<planning_core::ConePoint> cones;
  cones.push_back({5.0, -1.5, 0.0, 4});  // NONE
  cones.push_back({5.0, 1.5, 0.0, 4});   // NONE
  core.UpdateCones(cones);
  core.RunAlgorithm();

  ASSERT_TRUE(core.HasPlannedPath());
  EXPECT_GE(core.GetPlannedPath().size(), 170u);
}
```

**Step 2: Update skidpad detection tests**

In `src/planning_core/test/test_skidpad_detection_core.cpp`:

Change SimpleCircleFitting (line 40-41):
```cpp
    cones.push_back({params.circle_radius * std::cos(angle),
                     params.circle_radius * std::sin(angle), 0.0, 4});  // NONE
```

Add a degradation test:
```cpp
TEST(SkidpadDetectionCoreTest, NoColorFallsBackToGeometric) {
  planning_core::SkidpadParams params;
  params.circle_radius = 9.125;
  params.center_distance_nominal = 18.25;
  planning_core::SkidpadDetectionCore core(params);

  // All cones NONE — should use geometric y<0 split
  std::vector<planning_core::ConePoint> cones;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    cones.push_back({9.125 * std::cos(angle),
                     -9.125 + 9.125 * std::sin(angle), 0.0, 4});
  }
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * i / 8.0;
    cones.push_back({9.125 * std::cos(angle),
                     9.125 + 9.125 * std::sin(angle), 0.0, 4});
  }
  core.ProcessConeDetections(cones);

  planning_core::Trajectory state{};
  core.UpdateVehicleState(state);
  core.RunAlgorithm();
  // Should not crash
  SUCCEED();
}
```

**Step 3: Run tests**

Run: `catkin run_tests planning_core --no-deps`
Expected: All tests PASS

**Step 4: Commit**

```bash
git add src/planning_core/test/test_line_detection.cpp \
        src/planning_core/test/test_skidpad_detection_core.cpp
git commit -m "test(planning_core): add color_type to test ConePoints and add degradation tests"
```

---

### Task 4: Interpolated cones inherit neighbor color

**Files:**
- Modify: `src/localization_core/src/location_mapper.cpp:945-962`

**Context:**
Virtual cones created during interpolation are hardcoded to `vc.type = kConeNone`. When both neighbors (`c1`, `c2`) share the same color, the interpolated cone should inherit that color. Otherwise keep NONE.

**Step 1: Modify interpolation to inherit color**

In `src/localization_core/src/location_mapper.cpp`, replace line 960:

```cpp
      vc.type = kConeNone;
```

with:

```cpp
      // Inherit color from neighbors if they agree; otherwise NONE
      vc.type = (c1.type == c2.type && c1.type != kConeNone) ? c1.type : kConeNone;
```

This is a one-line change. When both neighbors are the same definitive color (e.g., both BLUE), the interpolated cone gets that color. Otherwise it stays NONE, preserving the current behavior.

**Step 2: Build**

Run: `catkin build localization_core --no-deps`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/localization_core/src/location_mapper.cpp
git commit -m "feat(localization_core): interpolated cones inherit neighbor color when consistent"
```

---

### Task 5: Cone type range validation in planning pipeline

**Files:**
- Modify: `src/planning_ros/src/planning_pipeline_node.cpp:307-346` (HighSpeedSyncCallback cone loop)

**Context:**
The high-speed callback creates `Node` objects from `HUAT_ConeMap` cones. There's no validation that `c.type` is within the valid range [0..5]. An out-of-range type could cause unexpected behavior in the color-aware `Way::getTracklimits()`.

**Step 1: Add type validation**

In `src/planning_ros/src/planning_pipeline_node.cpp`, inside the cone loop (after the confidence filter check at line 342-344), add a clamp before creating the Node:

Find the line:
```cpp
    if (contract::DecodeConeConfidenceScore(c.confidence) >= required_min)
    {
      nodes.emplace_back(c);
    }
```

This doesn't need structural change — the `Node` constructor already stores `c.type` as `type_`. We add a warning for out-of-range values. Add before `nodes.emplace_back(c)`:

```cpp
    if (contract::DecodeConeConfidenceScore(c.confidence) >= required_min)
    {
      if (c.type > 5)
      {
        ROS_WARN_ONCE("[planning_pipeline/high_speed] Cone type %u out of range [0..5], treating as NONE.", c.type);
      }
      nodes.emplace_back(c);
    }
```

The `Node` constructor stores `type_` as-is. The `Way::getTracklimits()` color override only checks for specific values (BLUE=0, YELLOW=1, ORANGE_SMALL=2, ORANGE_BIG=3, NONE=4), so unknown values naturally fall through to the geometric path — no functional risk, just a diagnostic warning.

**Step 2: Build**

Run: `catkin build planning_ros --no-deps`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/planning_ros/src/planning_pipeline_node.cpp
git commit -m "fix(planning_ros): warn on out-of-range cone type in high-speed pipeline"
```

---

## Verification

After all tasks:

```bash
catkin build planning_core planning_ros localization_core --no-deps
catkin run_tests planning_core --no-deps
roslaunch --dump-params fsd_launch trackdrive.launch simulation:=true bag:=/tmp/test.bag enable_vision:=true launch_rviz:=false launch_viz:=false 2>&1 | head -5
```

## Degradation Matrix

| Scenario | Gap 1 (Line) | Gap 2 (Skidpad) | Gap 3 (Tests) | Gap 4 (Interp) | Gap 5 (Validate) |
|----------|-------------|-----------------|---------------|----------------|------------------|
| Vision ON, full color | Color-aware center | Color-aware circle split | Covered | Inherits neighbor | Warns if bad |
| Vision OFF (all NONE) | Geometric fallback | Geometric y<0 split | Covered | Stays NONE | No warning |
| Mixed color | Color where available | Per-cone: color or geometric | Covered | NONE if mismatch | No warning |
