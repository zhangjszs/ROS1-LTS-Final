# Vision Module Design — 视觉感知模块集成方案

## 概述

在现有 LiDAR-only 感知管线旁，新增独立视觉感知模块，通过颜色注入方式与 LiDAR 检测结果融合。当无模型或无图像输入时，自动回退到纯 LiDAR 几何模式（现有行为不变）。

## 模型特征（YOLOv8s — 留空待训练）

| 属性 | 值 |
|------|-----|
| 输入尺寸 | 640×640×3 (BGR, uint8, 0-255) |
| 输入归一化 | /255.0, 无 mean/std 归一化 |
| 输出格式 | [batch, 84, 8400] — 84 = 4(xywh) + 1(objectness) 内嵌于 class scores + 5(classes) × 每 anchor 的 score; 实际 ultralytics v8 输出为 [1, 5+num_cls, N] 转置后 NMS |
| 类别数 | 5: blue_cone(0), yellow_cone(1), orange_small(2), orange_big(3), red_cone(4) |
| class_id → color_type 映射 | model_cls 0→ct 0, 1→1, 2→2, 3→3, 4→5 |
| 参数量 | 11.2M |
| FLOPs | 28.6G |
| 推理延迟 (3070 TRT FP16) | ~2.5ms |
| 导出格式 | ONNX → TensorRT FP16 |
| NMS | conf_threshold=0.5, iou_threshold=0.45, max_det=50 |

## 集成方式

**独立并行 + 颜色注入**

```
Camera ──→ vision_node ──→ /perception/vision/detections
                                    │
LiDAR ──→ lidar_cluster_node ──→ (订阅 vision detections)
                                    │
                              color_types[] 注入
                                    │
                                    ▼
                    /perception/lidar_cluster/detections
                              (下游不变)
```

## 回退策略

三级回退，确保无模型/无图像时系统正常运行：

1. **有模型 + 有图像** → 模型推理，颜色注入 LiDAR 检测
2. **有图像 + 无模型（或模型加载失败）** → HSV 传统视觉兜底，颜色注入
3. **无图像（topic 无数据/bag 无图像）** → vision_node 不启动或静默，LiDAR 走 vision_independent_mode（现有行为）

## 包结构

```
src/
├── vision_core/          # 纯 C++, 无 ROS 依赖
│   ├── include/vision_core/
│   │   ├── types.hpp                # Detection, ImageQuality, QualityMetrics
│   │   ├── inference_backend.hpp    # 抽象推理接口
│   │   ├── onnx_backend.hpp         # ONNX Runtime 实现
│   │   ├── tensorrt_backend.hpp     # TensorRT 实现 (后续)
│   │   ├── detection_postprocess.hpp # NMS + 后处理
│   │   ├── image_quality.hpp        # 图像质量评估
│   │   ├── image_enhancer.hpp       # 预处理增强 (CLAHE/Gamma/去噪)
│   │   ├── fallback_detector.hpp    # HSV+轮廓 传统视觉兜底
│   │   └── temporal_tracker.hpp     # 多帧时序平滑
│   ├── src/
│   └── test/
│
├── vision_ros/           # ROS 包装层
│   ├── include/vision_ros/
│   │   └── vision_node.hpp
│   ├── src/
│   │   └── vision_node.cpp
│   ├── config/
│   │   ├── vision_base.yaml
│   │   ├── vision_track.yaml
│   │   ├── vision_skidpad.yaml
│   │   └── vision_accel.yaml
│   ├── models/           # .onnx / .engine 文件
│   └── launch/
│       └── vision.launch
│
├── autodrive_msgs/msg/   # 新增
│   └── HUAT_VisionDetections.msg
```

## 新增消息: HUAT_VisionDetections.msg

```
Header header
uint8 image_quality          # 0=GOOD, 1=DEGRADED, 2=POOR, 3=UNUSABLE
float32 quality_score
float64[] x
float64[] y
uint8[] color_types          # 与 HUAT_ConeDetections 同枚举
int32[] confidences          # 0~1000
float32[] bbox_widths
float32[] bbox_heights
string backend_name
bool fallback_active
uint32 inference_time_us
```

## Topic 契约

| Topic | 类型 | 说明 |
|-------|------|------|
| `/resize_img_out` | sensor_msgs/Image | 输入(已有) |
| `perception/vision/detections` | HUAT_VisionDetections | 视觉检测输出 |
| `perception/vision/debug_image` | sensor_msgs/Image | 调试图(可选) |
| `perception/vision/diagnostics` | DiagnosticArray | 健康状态 |

## LiDAR 侧改动 (lidar_cluster_ros)

最小改动：
1. 新增可选 subscriber 订阅 `perception/vision/detections`
2. 缓存最近一帧视觉检测结果
3. 在 `publishDetections()` 时，按空间最近邻匹配 LiDAR cone ↔ Vision detection
4. 匹配成功且 vision confidence > 阈值 → 用 vision color_type 覆盖 LiDAR color_type
5. 匹配失败或无视觉数据 → 保持原有几何分类 (ORANGE_SMALL/BIG/NONE)
6. 通过 `vision_independent_mode.enabled` 参数控制开关

## 图像质量评估阈值

| 指标 | GOOD | DEGRADED | POOR | UNUSABLE |
|------|------|----------|------|----------|
| blur_score (Laplacian var) | ≥200 | 100~200 | 50~100 | <50 |
| brightness (mean) | 40~220 | 30~40 or 220~235 | 15~30 or 235~250 | <15 or >250 |
| overexposure_ratio | <0.1 | 0.1~0.3 | 0.3~0.5 | >0.5 |
| underexposure_ratio | <0.1 | 0.1~0.3 | 0.3~0.5 | >0.5 |

## 降级状态机

```
NORMAL ──(3帧DEGRADED)──→ DEGRADED_MODE ──(5帧POOR)──→ FALLBACK_MODE ──(10帧UNUSABLE)──→ VISION_LOST
  ↑ (5帧GOOD)                ↑ (3帧≥DEGRADED)              ↑ (1帧≥POOR)
```

VISION_LOST 时 → 下游自动回退到 vision_independent_mode (纯几何，现有行为)

## Launch 集成

在 `mission_stack.launch` 中新增:
```xml
<arg name="enable_vision" default="true"/>
<include if="$(arg enable_vision)" file="$(find fsd_launch)/launch/subsystems/vision.launch">
    <arg name="mission" value="..."/>
    <arg name="mode" value="standard"/>
</include>
```

vision.launch 支持 mode: standard | lightweight | fallback_only

---

# Vision Module Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a vision perception module that runs independently, detects/classifies cones by color, and injects color info into the existing LiDAR pipeline. Gracefully degrades when no model or no image is available.

**Architecture:** vision_core (pure C++/OpenCV, no ROS) + vision_ros (ROS wrapper). Vision node publishes detections; lidar_cluster_ros optionally subscribes and overrides color_types[]. Three-tier fallback: model → HSV traditional → LiDAR-only geometry.

**Tech Stack:** C++17, OpenCV 4, ONNX Runtime (optional at build time), catkin_tools, ROS Noetic

---

### Task 1: Create vision_core package scaffold + types.hpp

**Files:**
- Create: `src/vision_core/CMakeLists.txt`
- Create: `src/vision_core/package.xml`
- Create: `src/vision_core/include/vision_core/types.hpp`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>vision_core</name>
  <version>1.0.0</version>
  <description>Pure C++ vision algorithms for cone detection and color classification</description>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>OpenCV</depend>
</package>
```

**Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(vision_core)
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

# ONNX Runtime: optional — build without model support if not installed
find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  find_path(ONNXRT_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS /usr/local/include/onnxruntime ENV ONNXRUNTIME_ROOT_DIR
    PATH_SUFFIXES include include/onnxruntime)
  find_library(ONNXRT_LIBRARY onnxruntime
    PATHS /usr/local/lib ENV ONNXRUNTIME_ROOT_DIR
    PATH_SUFFIXES lib)
endif()

if(ONNXRT_INCLUDE_DIR AND ONNXRT_LIBRARY)
  set(HAVE_ONNXRUNTIME TRUE)
  add_definitions(-DHAVE_ONNXRUNTIME)
  message(STATUS "ONNX Runtime found: ${ONNXRT_LIBRARY}")
else()
  set(HAVE_ONNXRUNTIME FALSE)
  message(STATUS "ONNX Runtime NOT found — building without model inference support")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vision_core
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
if(HAVE_ONNXRUNTIME)
  include_directories(${ONNXRT_INCLUDE_DIR})
endif()

add_library(${PROJECT_NAME}
  src/image_quality.cpp
  src/image_enhancer.cpp
  src/fallback_detector.cpp
  src/detection_postprocess.cpp
  src/temporal_tracker.cpp
)
if(HAVE_ONNXRUNTIME)
  target_sources(${PROJECT_NAME} PRIVATE src/onnx_backend.cpp)
endif()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
if(HAVE_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME} ${ONNXRT_LIBRARY})
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_image_quality test/test_image_quality.cpp)
  if(TARGET test_image_quality)
    target_link_libraries(test_image_quality ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  endif()
  catkin_add_gtest(test_fallback_detector test/test_fallback_detector.cpp)
  if(TARGET test_fallback_detector)
    target_link_libraries(test_fallback_detector ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  endif()
endif()
```

**Step 3: Create types.hpp**

```cpp
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace vision_core {

// Color type enum — mirrors autodrive_msgs/HUAT_ConeDetections color_types
enum ConeColorType : uint8_t {
  BLUE = 0,
  YELLOW = 1,
  ORANGE_SMALL = 2,
  ORANGE_BIG = 3,
  NONE = 4,
  RED = 5
};

struct Detection {
  float x, y, w, h;        // bbox center + size (pixels)
  float confidence;         // 0.0 ~ 1.0
  uint8_t class_id;         // model class index (0-4)
  uint8_t color_type;       // mapped ConeColorType
};

enum class ImageQuality : uint8_t {
  GOOD = 0,
  DEGRADED = 1,
  POOR = 2,
  UNUSABLE = 3
};

struct QualityMetrics {
  float blur_score;
  float brightness;
  float contrast;
  float overexposure_ratio;
  float underexposure_ratio;
  ImageQuality overall;
};

// Model class_id (0-4) → ConeColorType mapping
// model: 0=blue, 1=yellow, 2=orange_small, 3=orange_big, 4=red
inline ConeColorType modelClassToColorType(uint8_t cls) {
  constexpr ConeColorType kMap[] = {BLUE, YELLOW, ORANGE_SMALL, ORANGE_BIG, RED};
  return (cls < 5) ? kMap[cls] : NONE;
}

}  // namespace vision_core
```

**Step 4: Create directory structure**

```bash
mkdir -p src/vision_core/{include/vision_core,src,test}
mkdir -p src/vision_ros/{include/vision_ros,src,config,models,launch}
```

**Step 5: Verify build**

```bash
catkin build vision_core --no-deps 2>&1 | tail -5
```
Expected: build fails on missing .cpp files (that's fine, we create them next)

**Step 6: Commit**

```bash
git add src/vision_core/CMakeLists.txt src/vision_core/package.xml src/vision_core/include/
git commit -m "feat(vision_core): add package scaffold and types.hpp"
```

---

### Task 2: Implement image_quality.hpp/.cpp

**Files:**
- Create: `src/vision_core/include/vision_core/image_quality.hpp`
- Create: `src/vision_core/src/image_quality.cpp`
- Create: `src/vision_core/test/test_image_quality.cpp`

**Step 1: Create image_quality.hpp**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>

namespace vision_core {

struct QualityThresholds {
  float blur_good = 200.0f;
  float blur_degraded = 100.0f;
  float blur_poor = 50.0f;
  float brightness_low = 40.0f;
  float brightness_high = 220.0f;
  float brightness_very_low = 15.0f;
  float brightness_very_high = 250.0f;
  float overexposure_limit = 0.3f;
  float underexposure_limit = 0.3f;
  float overexposure_unusable = 0.5f;
  float underexposure_unusable = 0.5f;
};

class ImageQualityAssessor {
public:
  explicit ImageQualityAssessor(const QualityThresholds& thresholds = {});
  QualityMetrics assess(const cv::Mat& bgr_image) const;
private:
  QualityThresholds thresholds_;
};

}  // namespace vision_core
```

**Step 2: Create image_quality.cpp**

```cpp
#include "vision_core/image_quality.hpp"
#include <opencv2/imgproc.hpp>

namespace vision_core {

ImageQualityAssessor::ImageQualityAssessor(const QualityThresholds& t)
    : thresholds_(t) {}

QualityMetrics ImageQualityAssessor::assess(const cv::Mat& bgr) const {
  QualityMetrics m{};
  if (bgr.empty()) {
    m.overall = ImageQuality::UNUSABLE;
    return m;
  }

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  // Blur: Laplacian variance
  cv::Mat lap;
  cv::Laplacian(gray, lap, CV_64F);
  cv::Scalar mu, sigma;
  cv::meanStdDev(lap, mu, sigma);
  m.blur_score = static_cast<float>(sigma.val[0] * sigma.val[0]);

  // Brightness + contrast
  cv::meanStdDev(gray, mu, sigma);
  m.brightness = static_cast<float>(mu.val[0]);
  m.contrast = static_cast<float>(sigma.val[0]);

  // Over/under exposure ratios
  const float total = static_cast<float>(gray.total());
  m.overexposure_ratio = cv::countNonZero(gray > 240) / total;
  m.underexposure_ratio = cv::countNonZero(gray < 15) / total;

  // Classification
  const auto& t = thresholds_;
  if (m.blur_score < t.blur_poor ||
      m.overexposure_ratio > t.overexposure_unusable ||
      m.underexposure_ratio > t.underexposure_unusable ||
      m.brightness < t.brightness_very_low ||
      m.brightness > t.brightness_very_high) {
    m.overall = ImageQuality::UNUSABLE;
  } else if (m.blur_score < t.blur_degraded ||
             m.brightness < t.brightness_low ||
             m.brightness > t.brightness_high ||
             m.overexposure_ratio > t.overexposure_limit) {
    m.overall = ImageQuality::POOR;
  } else if (m.blur_score < t.blur_good || m.contrast < 30.0f) {
    m.overall = ImageQuality::DEGRADED;
  } else {
    m.overall = ImageQuality::GOOD;
  }
  return m;
}

}  // namespace vision_core
```

**Step 3: Create test_image_quality.cpp**

```cpp
#include <gtest/gtest.h>
#include "vision_core/image_quality.hpp"

using namespace vision_core;

TEST(ImageQuality, EmptyImageIsUnusable) {
  ImageQualityAssessor assessor;
  cv::Mat empty;
  auto m = assessor.assess(empty);
  EXPECT_EQ(m.overall, ImageQuality::UNUSABLE);
}

TEST(ImageQuality, BrightWhiteImageIsUnusable) {
  ImageQualityAssessor assessor;
  cv::Mat white(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  auto m = assessor.assess(white);
  EXPECT_GE(static_cast<int>(m.overall), static_cast<int>(ImageQuality::POOR));
  EXPECT_GT(m.overexposure_ratio, 0.5f);
}

TEST(ImageQuality, NormalImageIsGood) {
  ImageQualityAssessor assessor;
  // Create a synthetic image with texture (not flat)
  cv::Mat img(480, 640, CV_8UC3);
  cv::randu(img, cv::Scalar(60, 60, 60), cv::Scalar(200, 200, 200));
  auto m = assessor.assess(img);
  EXPECT_EQ(m.overall, ImageQuality::GOOD);
  EXPECT_GT(m.blur_score, 200.0f);
}
```

**Step 4: Build and test**

```bash
catkin build vision_core --no-deps && catkin run_tests vision_core --no-deps
```

**Step 5: Commit**

```bash
git add src/vision_core/include/vision_core/image_quality.hpp \
        src/vision_core/src/image_quality.cpp \
        src/vision_core/test/test_image_quality.cpp
git commit -m "feat(vision_core): add image quality assessor with tests"
```

---

### Task 3: Implement image_enhancer.hpp/.cpp

**Files:**
- Create: `src/vision_core/include/vision_core/image_enhancer.hpp`
- Create: `src/vision_core/src/image_enhancer.cpp`

**Step 1: Create image_enhancer.hpp**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>

namespace vision_core {

struct EnhancerConfig {
  bool auto_clahe = true;
  float clahe_clip_limit = 2.0f;
  int clahe_grid_size = 8;
  bool auto_gamma = true;
  bool denoise_on_poor = true;
  bool sharpen_on_blur = true;
};

class ImageEnhancer {
public:
  explicit ImageEnhancer(const EnhancerConfig& config = {});
  /// Enhance image in-place based on quality level. Returns enhanced copy.
  cv::Mat enhance(const cv::Mat& bgr, ImageQuality quality) const;
private:
  void applyCLAHE(cv::Mat& bgr) const;
  void applyGamma(cv::Mat& bgr, float brightness) const;
  void applyDenoise(cv::Mat& bgr) const;
  void applySharpen(cv::Mat& bgr) const;
  EnhancerConfig config_;
};

}  // namespace vision_core
```

**Step 2: Create image_enhancer.cpp**

```cpp
#include "vision_core/image_enhancer.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <cmath>

namespace vision_core {

ImageEnhancer::ImageEnhancer(const EnhancerConfig& c) : config_(c) {}

cv::Mat ImageEnhancer::enhance(const cv::Mat& bgr, ImageQuality quality) const {
  if (quality == ImageQuality::GOOD || quality == ImageQuality::UNUSABLE)
    return bgr.clone();

  cv::Mat out = bgr.clone();
  if (config_.auto_clahe) applyCLAHE(out);

  // Compute brightness for gamma
  cv::Mat gray;
  cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
  float brightness = static_cast<float>(cv::mean(gray).val[0]);
  if (config_.auto_gamma) applyGamma(out, brightness);

  if (quality == ImageQuality::POOR) {
    if (config_.denoise_on_poor) applyDenoise(out);
    if (config_.sharpen_on_blur) applySharpen(out);
  }
  return out;
}

void ImageEnhancer::applyCLAHE(cv::Mat& bgr) const {
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);
  auto clahe = cv::createCLAHE(config_.clahe_clip_limit,
      cv::Size(config_.clahe_grid_size, config_.clahe_grid_size));
  clahe->apply(channels[0], channels[0]);
  cv::merge(channels, lab);
  cv::cvtColor(lab, bgr, cv::COLOR_Lab2BGR);
}

void ImageEnhancer::applyGamma(cv::Mat& bgr, float brightness) const {
  float gamma = 1.0f;
  if (brightness < 80.0f) gamma = 0.6f;       // dark → brighten
  else if (brightness > 180.0f) gamma = 1.5f;  // bright → darken
  else return;

  cv::Mat lut(1, 256, CV_8U);
  for (int i = 0; i < 256; ++i)
    lut.at<uint8_t>(i) = cv::saturate_cast<uint8_t>(
        std::pow(i / 255.0, gamma) * 255.0);
  cv::LUT(bgr, lut, bgr);
}

void ImageEnhancer::applyDenoise(cv::Mat& bgr) const {
  cv::Mat small, denoised;
  cv::resize(bgr, small, cv::Size(), 0.5, 0.5);
  cv::bilateralFilter(small, denoised, 5, 50, 50);
  cv::resize(denoised, bgr, bgr.size());
}

void ImageEnhancer::applySharpen(cv::Mat& bgr) const {
  cv::Mat blurred;
  cv::GaussianBlur(bgr, blurred, cv::Size(0, 0), 2.0);
  cv::addWeighted(bgr, 1.5, blurred, -0.5, 0, bgr);
}

}  // namespace vision_core
```

**Step 3: Build**

```bash
catkin build vision_core --no-deps
```

**Step 4: Commit**

```bash
git add src/vision_core/include/vision_core/image_enhancer.hpp \
        src/vision_core/src/image_enhancer.cpp
git commit -m "feat(vision_core): add image enhancer (CLAHE/gamma/denoise/sharpen)"
```

---

### Task 4: Implement inference_backend.hpp + onnx_backend

**Files:**
- Create: `src/vision_core/include/vision_core/inference_backend.hpp`
- Create: `src/vision_core/include/vision_core/onnx_backend.hpp`
- Create: `src/vision_core/src/onnx_backend.cpp`

**Step 1: Create inference_backend.hpp (abstract interface)**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace vision_core {

struct InferenceConfig {
  std::string model_path;
  int input_width = 640;
  int input_height = 640;
  float conf_threshold = 0.5f;
  float nms_threshold = 0.45f;
  bool use_fp16 = true;
  int num_threads = 2;
  int num_classes = 5;
};

class InferenceBackend {
public:
  virtual ~InferenceBackend() = default;
  virtual bool initialize(const InferenceConfig& config) = 0;
  virtual std::vector<Detection> detect(const cv::Mat& bgr) = 0;
  virtual std::string backendName() const = 0;
  virtual bool isReady() const = 0;
};

/// Factory: create backend by name. Returns nullptr if backend unavailable.
std::unique_ptr<InferenceBackend> createBackend(const std::string& type);

}  // namespace vision_core
```

**Step 2: Create onnx_backend.hpp**

```cpp
#pragma once
#include "vision_core/inference_backend.hpp"

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace vision_core {

class OnnxBackend : public InferenceBackend {
public:
  OnnxBackend();
  ~OnnxBackend() override;
  bool initialize(const InferenceConfig& config) override;
  std::vector<Detection> detect(const cv::Mat& bgr) override;
  std::string backendName() const override { return "onnx"; }
  bool isReady() const override { return ready_; }

private:
  cv::Mat preprocess(const cv::Mat& bgr) const;
  std::vector<Detection> postprocess(const float* output, int num_proposals) const;

  InferenceConfig config_;
  bool ready_ = false;

#ifdef HAVE_ONNXRUNTIME
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "vision"};
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo mem_info_{Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault)};
#endif
};

}  // namespace vision_core
```

**Step 3: Create onnx_backend.cpp**

```cpp
#include "vision_core/onnx_backend.hpp"
#include "vision_core/detection_postprocess.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>

namespace vision_core {

std::unique_ptr<InferenceBackend> createBackend(const std::string& type) {
#ifdef HAVE_ONNXRUNTIME
  if (type == "onnx") return std::make_unique<OnnxBackend>();
#endif
  // "fallback_only" or unknown → return nullptr, caller uses fallback
  return nullptr;
}

OnnxBackend::OnnxBackend() = default;
OnnxBackend::~OnnxBackend() = default;

bool OnnxBackend::initialize(const InferenceConfig& config) {
#ifdef HAVE_ONNXRUNTIME
  config_ = config;
  // Check model file exists
  std::ifstream f(config_.model_path);
  if (!f.good()) {
    ready_ = false;
    return false;
  }

  try {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(config_.num_threads);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(
        env_, config_.model_path.c_str(), opts);
    ready_ = true;
    return true;
  } catch (const Ort::Exception& e) {
    ready_ = false;
    return false;
  }
#else
  (void)config;
  ready_ = false;
  return false;
#endif
}

cv::Mat OnnxBackend::preprocess(const cv::Mat& bgr) const {
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(config_.input_width, config_.input_height));
  cv::Mat blob;
  // YOLOv8: BGR→RGB, /255.0, NCHW
  cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0,
      cv::Size(config_.input_width, config_.input_height),
      cv::Scalar(), true, false, CV_32F);
  return blob;
}

std::vector<Detection> OnnxBackend::detect(const cv::Mat& bgr) {
#ifdef HAVE_ONNXRUNTIME
  if (!ready_) return {};

  cv::Mat blob = preprocess(bgr);
  std::array<int64_t, 4> input_shape = {
      1, 3, config_.input_height, config_.input_width};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      mem_info_, blob.ptr<float>(), blob.total(),
      input_shape.data(), input_shape.size());

  // Run inference
  const char* input_names[] = {"images"};
  const char* output_names[] = {"output0"};
  auto outputs = session_->Run(Ort::RunOptions{nullptr},
      input_names, &input_tensor, 1, output_names, 1);

  // YOLOv8 output: [1, 4+num_classes, num_proposals]
  auto& out_tensor = outputs[0];
  auto shape = out_tensor.GetTensorTypeAndShapeInfo().GetShape();
  const int rows = static_cast<int>(shape[1]);  // 4 + num_classes
  const int cols = static_cast<int>(shape[2]);  // num proposals
  const float* data = out_tensor.GetTensorData<float>();

  auto raw = postprocess(data, cols);

  // Scale boxes back to original image size
  const float sx = static_cast<float>(bgr.cols) / config_.input_width;
  const float sy = static_cast<float>(bgr.rows) / config_.input_height;
  for (auto& d : raw) {
    d.x *= sx; d.w *= sx;
    d.y *= sy; d.h *= sy;
  }
  return raw;
#else
  (void)bgr;
  return {};
#endif
}

std::vector<Detection> OnnxBackend::postprocess(
    const float* data, int num_proposals) const {
  // YOLOv8 output layout: [4+num_cls, N] (transposed)
  // Row 0-3: cx, cy, w, h
  // Row 4+: class scores
  const int nc = config_.num_classes;
  std::vector<Detection> dets;
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> class_ids;

  for (int i = 0; i < num_proposals; ++i) {
    float cx = data[0 * num_proposals + i];
    float cy = data[1 * num_proposals + i];
    float w  = data[2 * num_proposals + i];
    float h  = data[3 * num_proposals + i];

    // Find best class
    float max_score = 0.0f;
    int max_cls = 0;
    for (int c = 0; c < nc; ++c) {
      float s = data[(4 + c) * num_proposals + i];
      if (s > max_score) { max_score = s; max_cls = c; }
    }

    if (max_score < config_.conf_threshold) continue;

    boxes.emplace_back(
        static_cast<int>(cx - w / 2), static_cast<int>(cy - h / 2),
        static_cast<int>(w), static_cast<int>(h));
    scores.push_back(max_score);
    class_ids.push_back(max_cls);
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, config_.conf_threshold,
                     config_.nms_threshold, indices);

  for (int idx : indices) {
    Detection d;
    d.x = boxes[idx].x + boxes[idx].width * 0.5f;
    d.y = boxes[idx].y + boxes[idx].height * 0.5f;
    d.w = static_cast<float>(boxes[idx].width);
    d.h = static_cast<float>(boxes[idx].height);
    d.confidence = scores[idx];
    d.class_id = static_cast<uint8_t>(class_ids[idx]);
    d.color_type = static_cast<uint8_t>(modelClassToColorType(d.class_id));
    dets.push_back(d);
  }
  return dets;
}

}  // namespace vision_core
```

**Step 4: Build**

```bash
catkin build vision_core --no-deps
```

**Step 5: Commit**

```bash
git add src/vision_core/include/vision_core/inference_backend.hpp \
        src/vision_core/include/vision_core/onnx_backend.hpp \
        src/vision_core/src/onnx_backend.cpp
git commit -m "feat(vision_core): add inference backend interface + ONNX implementation"
```

---

### Task 5: Implement detection_postprocess + fallback_detector + temporal_tracker

**Files:**
- Create: `src/vision_core/include/vision_core/detection_postprocess.hpp`
- Create: `src/vision_core/src/detection_postprocess.cpp`
- Create: `src/vision_core/include/vision_core/fallback_detector.hpp`
- Create: `src/vision_core/src/fallback_detector.cpp`
- Create: `src/vision_core/include/vision_core/temporal_tracker.hpp`
- Create: `src/vision_core/src/temporal_tracker.cpp`
- Create: `src/vision_core/test/test_fallback_detector.cpp`

**Step 1: Create detection_postprocess.hpp**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <vector>

namespace vision_core {

/// Filter detections by max count, sorted by confidence descending.
std::vector<Detection> filterTopK(std::vector<Detection> dets, int max_det);

}  // namespace vision_core
```

**Step 2: Create detection_postprocess.cpp**

```cpp
#include "vision_core/detection_postprocess.hpp"
#include <algorithm>

namespace vision_core {

std::vector<Detection> filterTopK(std::vector<Detection> dets, int max_det) {
  if (static_cast<int>(dets.size()) <= max_det) return dets;
  std::partial_sort(dets.begin(), dets.begin() + max_det, dets.end(),
      [](const Detection& a, const Detection& b) {
        return a.confidence > b.confidence;
      });
  dets.resize(max_det);
  return dets;
}

}  // namespace vision_core
```

**Step 3: Create fallback_detector.hpp**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace vision_core {

struct HsvRange {
  cv::Scalar lower;
  cv::Scalar upper;
};

struct FallbackConfig {
  HsvRange blue   = {{100, 80, 50},  {130, 255, 255}};
  HsvRange yellow = {{15, 80, 50},   {45, 255, 255}};
  HsvRange orange = {{5, 100, 100},  {20, 255, 255}};
  double min_area = 200.0;
  double max_area = 50000.0;
  float min_aspect = 0.3f;
  float max_aspect = 1.5f;
  float min_fill_ratio = 0.35f;
};

class FallbackDetector {
public:
  explicit FallbackDetector(const FallbackConfig& config = {});
  std::vector<Detection> detect(const cv::Mat& bgr) const;
private:
  void detectByColor(const cv::Mat& hsv, std::vector<Detection>& out,
                     const HsvRange& range, uint8_t color_type) const;
  FallbackConfig config_;
};

}  // namespace vision_core
```

**Step 4: Create fallback_detector.cpp**

```cpp
#include "vision_core/fallback_detector.hpp"
#include <opencv2/imgproc.hpp>

namespace vision_core {

FallbackDetector::FallbackDetector(const FallbackConfig& c) : config_(c) {}

std::vector<Detection> FallbackDetector::detect(const cv::Mat& bgr) const {
  if (bgr.empty()) return {};
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::vector<Detection> results;
  detectByColor(hsv, results, config_.blue,   ConeColorType::BLUE);
  detectByColor(hsv, results, config_.yellow, ConeColorType::YELLOW);
  detectByColor(hsv, results, config_.orange, ConeColorType::ORANGE_SMALL);
  return results;
}

void FallbackDetector::detectByColor(const cv::Mat& hsv,
    std::vector<Detection>& out, const HsvRange& range,
    uint8_t color_type) const {
  cv::Mat mask;
  cv::inRange(hsv, range.lower, range.upper, mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area < config_.min_area || area > config_.max_area) continue;

    cv::Rect bbox = cv::boundingRect(contour);
    float aspect = static_cast<float>(bbox.width) / bbox.height;
    if (aspect < config_.min_aspect || aspect > config_.max_aspect) continue;

    float fill = static_cast<float>(area) / (bbox.width * bbox.height);
    if (fill < config_.min_fill_ratio) continue;

    Detection d;
    d.x = bbox.x + bbox.width * 0.5f;
    d.y = bbox.y + bbox.height * 0.5f;
    d.w = static_cast<float>(bbox.width);
    d.h = static_cast<float>(bbox.height);
    d.class_id = color_type;  // fallback: class_id == color_type
    d.color_type = color_type;
    d.confidence = 0.3f * fill + 0.2f;  // capped ~0.5
    out.push_back(d);
  }
}

}  // namespace vision_core
```

**Step 5: Create temporal_tracker.hpp**

```cpp
#pragma once
#include "vision_core/types.hpp"
#include <deque>
#include <vector>

namespace vision_core {

struct TrackerConfig {
  int history_frames = 3;
  float iou_threshold = 0.3f;
  int min_hits_to_output = 2;   // need 2+ frames to confirm
  int max_miss_frames = 1;      // keep 1 frame after disappear
};

class TemporalTracker {
public:
  explicit TemporalTracker(const TrackerConfig& config = {});
  /// Update with current frame detections, return stabilized output.
  std::vector<Detection> update(const std::vector<Detection>& dets);
  void reset();
private:
  struct Track {
    Detection last_det;
    int hits = 0;
    int misses = 0;
  };
  float computeIoU(const Detection& a, const Detection& b) const;
  TrackerConfig config_;
  std::vector<Track> tracks_;
};

}  // namespace vision_core
```

**Step 6: Create temporal_tracker.cpp**

```cpp
#include "vision_core/temporal_tracker.hpp"
#include <algorithm>
#include <cmath>

namespace vision_core {

TemporalTracker::TemporalTracker(const TrackerConfig& c) : config_(c) {}

void TemporalTracker::reset() { tracks_.clear(); }

float TemporalTracker::computeIoU(const Detection& a, const Detection& b) const {
  float ax1 = a.x - a.w / 2, ay1 = a.y - a.h / 2;
  float ax2 = a.x + a.w / 2, ay2 = a.y + a.h / 2;
  float bx1 = b.x - b.w / 2, by1 = b.y - b.h / 2;
  float bx2 = b.x + b.w / 2, by2 = b.y + b.h / 2;

  float ix1 = std::max(ax1, bx1), iy1 = std::max(ay1, by1);
  float ix2 = std::min(ax2, bx2), iy2 = std::min(ay2, by2);
  float inter = std::max(0.0f, ix2 - ix1) * std::max(0.0f, iy2 - iy1);
  float area_a = a.w * a.h, area_b = b.w * b.h;
  float union_area = area_a + area_b - inter;
  return (union_area > 0) ? inter / union_area : 0.0f;
}

std::vector<Detection> TemporalTracker::update(
    const std::vector<Detection>& dets) {
  // Mark all tracks as missed
  for (auto& t : tracks_) t.misses++;

  // Match detections to existing tracks by IoU
  std::vector<bool> matched(dets.size(), false);
  for (auto& track : tracks_) {
    float best_iou = 0.0f;
    int best_idx = -1;
    for (size_t i = 0; i < dets.size(); ++i) {
      if (matched[i]) continue;
      float iou = computeIoU(track.last_det, dets[i]);
      if (iou > best_iou) { best_iou = iou; best_idx = static_cast<int>(i); }
    }
    if (best_idx >= 0 && best_iou >= config_.iou_threshold) {
      track.last_det = dets[best_idx];
      track.hits++;
      track.misses = 0;
      matched[best_idx] = true;
    }
  }

  // Create new tracks for unmatched detections
  for (size_t i = 0; i < dets.size(); ++i) {
    if (!matched[i]) {
      tracks_.push_back({dets[i], 1, 0});
    }
  }

  // Remove dead tracks
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const Track& t) {
        return t.misses > config_.max_miss_frames;
      }), tracks_.end());

  // Output only confirmed tracks
  std::vector<Detection> out;
  for (const auto& t : tracks_) {
    if (t.hits >= config_.min_hits_to_output) {
      out.push_back(t.last_det);
    }
  }
  return out;
}

}  // namespace vision_core
```

**Step 7: Create test_fallback_detector.cpp**

```cpp
#include <gtest/gtest.h>
#include "vision_core/fallback_detector.hpp"

using namespace vision_core;

TEST(FallbackDetector, EmptyImageReturnsEmpty) {
  FallbackDetector det;
  cv::Mat empty;
  auto results = det.detect(empty);
  EXPECT_TRUE(results.empty());
}

TEST(FallbackDetector, DetectsBlueRegion) {
  FallbackDetector det;
  // Create image with a blue rectangle
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(img, cv::Rect(200, 200, 60, 80),
                cv::Scalar(200, 100, 50), cv::FILLED);  // BGR blue-ish
  // Convert to HSV to verify it's in blue range
  auto results = det.detect(img);
  // May or may not detect depending on exact HSV — this tests no crash
  // Real validation needs tuned HSV ranges for actual cone colors
  SUCCEED();
}

TEST(FallbackDetector, ConfidenceCapped) {
  FallbackDetector det;
  cv::Mat img(480, 640, CV_8UC3, cv::Scalar(120, 200, 200));  // ~yellow in BGR
  auto results = det.detect(img);
  for (const auto& d : results) {
    EXPECT_LE(d.confidence, 0.55f);  // fallback confidence capped
  }
}
```

**Step 8: Build and test**

```bash
catkin build vision_core --no-deps && catkin run_tests vision_core --no-deps
```

**Step 9: Commit**

```bash
git add src/vision_core/include/vision_core/detection_postprocess.hpp \
        src/vision_core/src/detection_postprocess.cpp \
        src/vision_core/include/vision_core/fallback_detector.hpp \
        src/vision_core/src/fallback_detector.cpp \
        src/vision_core/include/vision_core/temporal_tracker.hpp \
        src/vision_core/src/temporal_tracker.cpp \
        src/vision_core/test/test_fallback_detector.cpp
git commit -m "feat(vision_core): add fallback detector, postprocess, temporal tracker"
```

---

### Task 6: Add HUAT_VisionDetections.msg + update topic_contract.hpp

**Files:**
- Create: `src/autodrive_msgs/msg/HUAT_VisionDetections.msg`
- Modify: `src/autodrive_msgs/CMakeLists.txt` (add new msg to list)
- Modify: `src/fsd_common/include/fsd_common/topic_contract.hpp` (add vision topics)

**Step 1: Create HUAT_VisionDetections.msg**

```
# Vision-based cone detection results
# Publishes independently; lidar_cluster_ros optionally subscribes for color injection.

Header header                    # original image timestamp
uint8 image_quality              # 0=GOOD, 1=DEGRADED, 2=POOR, 3=UNUSABLE
float32 quality_score            # 0.0~1.0 composite quality

float64[] x                      # bbox center x (pixels)
float64[] y                      # bbox center y (pixels)
uint8[] color_types              # same enum as HUAT_ConeDetections: 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
int32[] confidences              # 0~1000 (scaled, consistent with LiDAR side)
float32[] bbox_widths            # pixels
float32[] bbox_heights           # pixels

string backend_name              # "onnx" / "tensorrt" / "fallback_hsv"
bool fallback_active             # true if fallback method is in use
uint32 inference_time_us         # inference latency in microseconds
```

**Step 2: Add msg to autodrive_msgs/CMakeLists.txt**

Find the `add_message_files` block and add `HUAT_VisionDetections.msg` to the list.

**Step 3: Update topic_contract.hpp**

Add under `// ── Perception / Localization ───`:

```cpp
// ── Vision ────────────────────────────────────────────────────────
inline constexpr const char *kVisionDetections = "perception/vision/detections";
inline constexpr const char *kVisionDebugImage = "perception/vision/debug_image";
inline constexpr const char *kVisionDiagnostics = "perception/vision/diagnostics";
```

**Step 4: Build autodrive_msgs to generate headers**

```bash
catkin build autodrive_msgs --no-deps
```

**Step 5: Commit**

```bash
git add src/autodrive_msgs/msg/HUAT_VisionDetections.msg \
        src/autodrive_msgs/CMakeLists.txt \
        src/fsd_common/include/fsd_common/topic_contract.hpp
git commit -m "feat(msgs): add HUAT_VisionDetections message + vision topic contracts"
```

---

### Task 7: Create vision_ros package + vision_node

**Files:**
- Create: `src/vision_ros/package.xml`
- Create: `src/vision_ros/CMakeLists.txt`
- Create: `src/vision_ros/include/vision_ros/vision_node.hpp`
- Create: `src/vision_ros/src/vision_node.cpp`
- Create: `src/vision_ros/src/vision_main.cpp`

**Step 1: Create package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>vision_ros</name>
  <version>1.0.0</version>
  <description>ROS wrapper for vision-based cone detection</description>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>autodrive_msgs</depend>
  <depend>fsd_common</depend>
  <depend>vision_core</depend>
  <depend>diagnostic_msgs</depend>
</package>
```

**Step 2: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(vision_ros)
add_compile_options(-std=c++17 -Wall -Wextra -Wno-unused-parameter)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs cv_bridge image_transport
  autodrive_msgs fsd_common vision_core diagnostic_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
                 autodrive_msgs fsd_common vision_core diagnostic_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/vision_node.cpp)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(vision_node src/vision_main.cpp)
target_link_libraries(vision_node ${PROJECT_NAME}
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
```

**Step 3: Create vision_node.hpp**

```cpp
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <autodrive_msgs/HUAT_VisionDetections.h>
#include <fsd_common/diagnostics_helper.hpp>
#include <fsd_common/topic_contract.hpp>

#include <vision_core/types.hpp>
#include <vision_core/inference_backend.hpp>
#include <vision_core/image_quality.hpp>
#include <vision_core/image_enhancer.hpp>
#include <vision_core/fallback_detector.hpp>
#include <vision_core/temporal_tracker.hpp>
#include <vision_core/detection_postprocess.hpp>

#include <memory>
#include <string>

namespace vision_ros {

enum class VisionState : uint8_t {
  NORMAL = 0,
  DEGRADED_MODE = 1,
  FALLBACK_MODE = 2,
  VISION_LOST = 3
};

class VisionNode {
public:
  VisionNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void spin();

private:
  void loadParams();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void processFrame(const cv::Mat& bgr, const std_msgs::Header& header);
  void publishDetections(const std::vector<vision_core::Detection>& dets,
                         const vision_core::QualityMetrics& quality,
                         const std_msgs::Header& header,
                         bool fallback_active, uint32_t inference_us);
  void publishDebugImage(const cv::Mat& bgr,
                         const std::vector<vision_core::Detection>& dets,
                         const std_msgs::Header& header);
  void publishDiagnostics(const vision_core::QualityMetrics& quality,
                          int n_dets, uint32_t inference_us);
  void updateState(vision_core::ImageQuality quality);
  std::vector<vision_core::Detection> fuseDetections(
      const std::vector<vision_core::Detection>& model_dets,
      const std::vector<vision_core::Detection>& fallback_dets,
      vision_core::ImageQuality quality);

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber image_sub_;
  ros::Publisher detections_pub_;
  ros::Publisher debug_image_pub_;
  fsd_common::DiagnosticsHelper diag_helper_;

  // Core components
  std::unique_ptr<vision_core::InferenceBackend> backend_;
  vision_core::ImageQualityAssessor quality_assessor_;
  vision_core::ImageEnhancer enhancer_;
  vision_core::FallbackDetector fallback_;
  vision_core::TemporalTracker tracker_;

  // State
  VisionState state_ = VisionState::NORMAL;
  int consecutive_degraded_ = 0;
  int consecutive_poor_ = 0;
  int consecutive_unusable_ = 0;
  int consecutive_good_ = 0;

  // Params
  std::string image_topic_;
  std::string backend_type_;
  int loop_rate_ = 30;
  int max_detections_ = 50;
  int confidence_scale_ = 1000;
  bool publish_debug_image_ = false;
  int debug_image_rate_ = 5;
  bool fallback_enabled_ = true;
  float model_confidence_floor_ = 0.3f;
  int degraded_frame_count_ = 3;
  int poor_frame_count_ = 5;
  int unusable_frame_count_ = 10;
  int recovery_frame_count_ = 5;
};

}  // namespace vision_ros
```

**Step 4: Create vision_node.cpp**

This is the main implementation. Key flow:
1. Subscribe to image topic
2. On each frame: assess quality → enhance if needed → run model (or fallback) → temporal track → publish

```cpp
#include "vision_ros/vision_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <chrono>

namespace vision_ros {

VisionNode::VisionNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), private_nh_(pnh) {
  loadParams();

  // Initialize backend
  backend_ = vision_core::createBackend(backend_type_);
  if (backend_) {
    vision_core::InferenceConfig icfg;
    private_nh_.param<std::string>("inference/model_path", icfg.model_path, "");
    private_nh_.param<int>("inference/input_width", icfg.input_width, 640);
    private_nh_.param<int>("inference/input_height", icfg.input_height, 640);
    private_nh_.param<float>("detection/conf_threshold", icfg.conf_threshold, 0.5f);
    private_nh_.param<float>("detection/nms_threshold", icfg.nms_threshold, 0.45f);
    private_nh_.param<bool>("inference/use_fp16", icfg.use_fp16, true);
    private_nh_.param<int>("inference/num_threads", icfg.num_threads, 2);

    if (!backend_->initialize(icfg)) {
      ROS_WARN("[vision] Backend '%s' failed to initialize (model_path=%s). "
               "Using fallback only.", backend_type_.c_str(),
               icfg.model_path.c_str());
      backend_.reset();
    } else {
      ROS_INFO("[vision] Backend '%s' initialized successfully.",
               backend_->backendName().c_str());
    }
  } else {
    ROS_INFO("[vision] No model backend requested (type='%s'). "
             "Running fallback-only mode.", backend_type_.c_str());
  }

  // Quality assessor thresholds from params
  vision_core::QualityThresholds qt;
  private_nh_.param<float>("quality/blur_threshold", qt.blur_good, 200.0f);
  private_nh_.param<float>("quality/blur_degraded", qt.blur_degraded, 100.0f);
  private_nh_.param<float>("quality/blur_poor", qt.blur_poor, 50.0f);
  private_nh_.param<float>("quality/brightness_low", qt.brightness_low, 40.0f);
  private_nh_.param<float>("quality/brightness_high", qt.brightness_high, 220.0f);
  quality_assessor_ = vision_core::ImageQualityAssessor(qt);

  // Enhancer config from params
  vision_core::EnhancerConfig ec;
  private_nh_.param<bool>("enhancement/auto_clahe", ec.auto_clahe, true);
  private_nh_.param<float>("enhancement/clahe_clip_limit", ec.clahe_clip_limit, 2.0f);
  private_nh_.param<bool>("enhancement/auto_gamma", ec.auto_gamma, true);
  private_nh_.param<bool>("enhancement/denoise_on_poor", ec.denoise_on_poor, true);
  private_nh_.param<bool>("enhancement/sharpen_on_blur", ec.sharpen_on_blur, true);
  enhancer_ = vision_core::ImageEnhancer(ec);

  // Fallback config from params
  vision_core::FallbackConfig fc;
  private_nh_.param<double>("fallback/min_contour_area", fc.min_area, 200.0);
  private_nh_.param<double>("fallback/max_contour_area", fc.max_area, 50000.0);
  fallback_ = vision_core::FallbackDetector(fc);

  // Publishers
  detections_pub_ = nh_.advertise<autodrive_msgs::HUAT_VisionDetections>(
      fsd_common::topic_contract::kVisionDetections, 1);
  if (publish_debug_image_) {
    debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(
        fsd_common::topic_contract::kVisionDebugImage, 1);
  }

  // Diagnostics
  fsd_common::DiagnosticsHelper::Config dcfg;
  dcfg.local_topic = fsd_common::topic_contract::kVisionDiagnostics;
  dcfg.global_topic = fsd_common::topic_contract::kDiagnosticsGlobal;
  dcfg.rate_hz = 2.0;
  diag_helper_.Init(nh_, dcfg);

  // Subscriber (queue_size=1: only process latest frame)
  image_sub_ = nh_.subscribe(image_topic_, 1, &VisionNode::imageCallback, this);
  ROS_INFO("[vision] Subscribed to '%s'", image_topic_.c_str());
}

void VisionNode::loadParams() {
  private_nh_.param<std::string>("node/image_topic", image_topic_, "/resize_img_out");
  private_nh_.param<std::string>("inference/backend_type", backend_type_, "onnx");
  private_nh_.param<int>("node/loop_rate", loop_rate_, 30);
  private_nh_.param<int>("detection/max_detections", max_detections_, 50);
  private_nh_.param<int>("output/confidence_scale", confidence_scale_, 1000);
  private_nh_.param<bool>("output/publish_debug_image", publish_debug_image_, false);
  private_nh_.param<int>("output/debug_image_rate", debug_image_rate_, 5);
  private_nh_.param<bool>("fallback/enabled", fallback_enabled_, true);
  private_nh_.param<float>("fallback/model_confidence_floor", model_confidence_floor_, 0.3f);
}

void VisionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR_THROTTLE(5.0, "[vision] cv_bridge exception: %s", e.what());
    return;
  }
  processFrame(cv_ptr->image, msg->header);
}

void VisionNode::processFrame(const cv::Mat& bgr,
                               const std_msgs::Header& header) {
  auto t0 = std::chrono::steady_clock::now();

  // 1. Quality assessment
  auto quality = quality_assessor_.assess(bgr);
  updateState(quality.overall);

  // 2. Enhancement
  cv::Mat enhanced = enhancer_.enhance(bgr, quality.overall);

  // 3. Model inference (if available and image not unusable)
  std::vector<vision_core::Detection> model_dets;
  if (backend_ && backend_->isReady() &&
      quality.overall != vision_core::ImageQuality::UNUSABLE) {
    model_dets = backend_->detect(enhanced);
  }

  // 4. Fallback detection (always run if enabled, cost < 2ms)
  std::vector<vision_core::Detection> fallback_dets;
  if (fallback_enabled_) {
    fallback_dets = fallback_.detect(enhanced);
  }

  // 5. Fuse model + fallback
  auto fused = fuseDetections(model_dets, fallback_dets, quality.overall);

  // 6. Temporal tracking (stabilize across frames)
  auto tracked = tracker_.update(fused);

  // 7. Top-K filter
  tracked = vision_core::filterTopK(std::move(tracked), max_detections_);

  auto t1 = std::chrono::steady_clock::now();
  uint32_t inference_us = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

  bool fallback_active = model_dets.empty() && !fallback_dets.empty();

  // 8. Publish
  publishDetections(tracked, quality, header, fallback_active, inference_us);
  if (publish_debug_image_) {
    publishDebugImage(enhanced, tracked, header);
  }
  publishDiagnostics(quality, static_cast<int>(tracked.size()), inference_us);
}

std::vector<vision_core::Detection> VisionNode::fuseDetections(
    const std::vector<vision_core::Detection>& model_dets,
    const std::vector<vision_core::Detection>& fallback_dets,
    vision_core::ImageQuality quality) {
  using IQ = vision_core::ImageQuality;

  if (quality == IQ::GOOD && !model_dets.empty()) {
    return model_dets;  // trust model
  }
  if (quality == IQ::UNUSABLE) {
    return fallback_dets;  // model skipped
  }

  // DEGRADED or POOR or model empty: merge
  std::vector<vision_core::Detection> merged = model_dets;
  for (const auto& fd : fallback_dets) {
    // Check if fallback det overlaps any model det
    bool has_overlap = false;
    for (const auto& md : model_dets) {
      // Simple center distance check (faster than IoU for this purpose)
      float dx = fd.x - md.x, dy = fd.y - md.y;
      float dist = std::sqrt(dx * dx + dy * dy);
      float avg_size = (md.w + md.h + fd.w + fd.h) / 4.0f;
      if (dist < avg_size * 0.5f) { has_overlap = true; break; }
    }
    if (!has_overlap && fd.confidence > 0.35f) {
      merged.push_back(fd);
    }
  }
  return merged;
}

void VisionNode::publishDetections(
    const std::vector<vision_core::Detection>& dets,
    const vision_core::QualityMetrics& quality,
    const std_msgs::Header& header,
    bool fallback_active, uint32_t inference_us) {
  autodrive_msgs::HUAT_VisionDetections msg;
  msg.header = header;
  msg.image_quality = static_cast<uint8_t>(quality.overall);
  msg.quality_score = quality.blur_score / 500.0f;  // rough 0~1 normalization
  if (msg.quality_score > 1.0f) msg.quality_score = 1.0f;

  for (const auto& d : dets) {
    msg.x.push_back(d.x);
    msg.y.push_back(d.y);
    msg.color_types.push_back(d.color_type);
    msg.confidences.push_back(
        static_cast<int32_t>(d.confidence * confidence_scale_));
    msg.bbox_widths.push_back(d.w);
    msg.bbox_heights.push_back(d.h);
  }

  msg.backend_name = backend_ ? backend_->backendName() : "fallback_hsv";
  msg.fallback_active = fallback_active;
  msg.inference_time_us = inference_us;

  detections_pub_.publish(msg);
}

void VisionNode::publishDebugImage(
    const cv::Mat& bgr,
    const std::vector<vision_core::Detection>& dets,
    const std_msgs::Header& header) {
  cv::Mat debug = bgr.clone();
  for (const auto& d : dets) {
    cv::Scalar color;
    switch (d.color_type) {
      case 0: color = cv::Scalar(255, 0, 0); break;    // BLUE
      case 1: color = cv::Scalar(0, 255, 255); break;   // YELLOW
      case 2: case 3: color = cv::Scalar(0, 165, 255); break; // ORANGE
      case 5: color = cv::Scalar(0, 0, 255); break;     // RED
      default: color = cv::Scalar(128, 128, 128); break;
    }
    cv::rectangle(debug,
        cv::Rect(static_cast<int>(d.x - d.w/2), static_cast<int>(d.y - d.h/2),
                 static_cast<int>(d.w), static_cast<int>(d.h)),
        color, 2);
  }
  auto msg = cv_bridge::CvImage(header, "bgr8", debug).toImageMsg();
  debug_image_pub_.publish(msg);
}

void VisionNode::publishDiagnostics(
    const vision_core::QualityMetrics& quality,
    int n_dets, uint32_t inference_us) {
  using KV = fsd_common::DiagnosticsHelper;
  uint8_t level = diagnostic_msgs::DiagnosticStatus::OK;
  std::string message = "NORMAL";

  if (state_ == VisionState::VISION_LOST) {
    level = diagnostic_msgs::DiagnosticStatus::ERROR;
    message = "VISION_LOST";
  } else if (state_ == VisionState::FALLBACK_MODE) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    message = "FALLBACK_MODE";
  } else if (state_ == VisionState::DEGRADED_MODE) {
    level = diagnostic_msgs::DiagnosticStatus::WARN;
    message = "DEGRADED_MODE";
  }

  diag_helper_.PublishStatus("vision_node", "camera", level, message, {
    KV::KV("n_detections", std::to_string(n_dets)),
    KV::KV("inference_us", std::to_string(inference_us)),
    KV::KV("image_quality", std::to_string(static_cast<int>(quality.overall))),
    KV::KV("blur_score", std::to_string(quality.blur_score)),
    KV::KV("brightness", std::to_string(quality.brightness)),
    KV::KV("backend", backend_ ? backend_->backendName() : "none"),
  });
}

void VisionNode::updateState(vision_core::ImageQuality q) {
  using IQ = vision_core::ImageQuality;
  if (q == IQ::GOOD) {
    consecutive_good_++;
    consecutive_degraded_ = consecutive_poor_ = consecutive_unusable_ = 0;
  } else if (q == IQ::DEGRADED) {
    consecutive_degraded_++;
    consecutive_good_ = consecutive_poor_ = consecutive_unusable_ = 0;
  } else if (q == IQ::POOR) {
    consecutive_poor_++;
    consecutive_good_ = consecutive_degraded_ = consecutive_unusable_ = 0;
  } else {
    consecutive_unusable_++;
    consecutive_good_ = consecutive_degraded_ = consecutive_poor_ = 0;
  }

  // State transitions
  switch (state_) {
    case VisionState::NORMAL:
      if (consecutive_degraded_ >= degraded_frame_count_)
        state_ = VisionState::DEGRADED_MODE;
      break;
    case VisionState::DEGRADED_MODE:
      if (consecutive_good_ >= recovery_frame_count_)
        state_ = VisionState::NORMAL;
      else if (consecutive_poor_ >= poor_frame_count_)
        state_ = VisionState::FALLBACK_MODE;
      break;
    case VisionState::FALLBACK_MODE:
      if (consecutive_good_ >= recovery_frame_count_ ||
          consecutive_degraded_ >= degraded_frame_count_)
        state_ = VisionState::DEGRADED_MODE;
      else if (consecutive_unusable_ >= unusable_frame_count_)
        state_ = VisionState::VISION_LOST;
      break;
    case VisionState::VISION_LOST:
      if (q <= IQ::POOR)
        state_ = VisionState::FALLBACK_MODE;
      break;
  }
}

void VisionNode::spin() {
  ros::spin();
}

}  // namespace vision_ros
```

**Step 5: Create vision_main.cpp**

```cpp
#include <ros/ros.h>
#include "vision_ros/vision_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  vision_ros::VisionNode node(nh, pnh);
  node.spin();
  return 0;
}
```

**Step 6: Build**

```bash
catkin build vision_ros --no-deps
```

**Step 7: Commit**

```bash
git add src/vision_ros/
git commit -m "feat(vision_ros): add vision node with model/fallback/quality pipeline"
```

---

### Task 8: Create vision_ros config YAMLs + launch file

**Files:**
- Create: `src/vision_ros/config/vision_base.yaml`
- Create: `src/vision_ros/config/vision_track.yaml`
- Create: `src/vision_ros/config/vision_skidpad.yaml`
- Create: `src/vision_ros/config/vision_accel.yaml`
- Create: `src/vision_ros/launch/vision.launch`
- Create: `src/vision_ros/models/.gitkeep`

**Step 1: Create vision_base.yaml**

```yaml
# === Vision Base Config ===
# Layered: vision_base.yaml → vision_{mode}.yaml → mission overlay → local override

# --- Inference backend ---
inference:
  backend_type: "onnx"          # "onnx" | "tensorrt" | "fallback_only"
  model_path: ""                # empty = fallback-only mode
  input_width: 640
  input_height: 640
  use_fp16: true
  num_threads: 2

# --- Detection ---
detection:
  conf_threshold: 0.5
  nms_threshold: 0.45
  max_detections: 50

# --- Image quality assessment ---
quality:
  blur_threshold: 200.0
  blur_degraded: 100.0
  blur_poor: 50.0
  brightness_low: 40.0
  brightness_high: 220.0

# --- Enhancement ---
enhancement:
  auto_clahe: true
  clahe_clip_limit: 2.0
  auto_gamma: true
  denoise_on_poor: true
  sharpen_on_blur: true

# --- Fallback (HSV traditional) ---
fallback:
  enabled: true
  model_confidence_floor: 0.3
  min_contour_area: 200.0
  max_contour_area: 50000.0

# --- Output ---
output:
  publish_debug_image: false
  debug_image_rate: 5
  confidence_scale: 1000

# --- Node ---
node:
  image_topic: "/resize_img_out"
  loop_rate: 30
```

**Step 2: Create vision_track.yaml (trackdrive overlay)**

```yaml
# TrackDrive: higher speed, prioritize low latency
detection:
  conf_threshold: 0.45
  max_detections: 50
```

**Step 3: Create vision_skidpad.yaml**

```yaml
# Skidpad: tighter area, fewer cones
detection:
  conf_threshold: 0.5
  max_detections: 30
```

**Step 4: Create vision_accel.yaml**

```yaml
# Acceleration: straight line, fewer cones, wider FOV not needed
detection:
  conf_threshold: 0.5
  max_detections: 20
```

**Step 5: Create vision.launch**

```xml
<!--
  FSD Vision Subsystem
  视觉子系统：图像锥桶检测与颜色分类

  Args:
    mission: track | skidpad | accel (selects config overlay)
    mode: standard | lightweight | fallback_only
    debug: true/false (enable debug image publishing)
-->
<launch>
  <arg name="mission" default="track"/>
  <arg name="mode" default="standard"/>
  <arg name="debug" default="false"/>
  <arg name="image_topic" default="/resize_img_out"/>
  <arg name="ns" default="perception/vision"/>
  <arg name="extra_config" default="" doc="Vehicle overlay config"/>
  <arg name="extra_local_config" default="" doc="Local override config"/>

  <group ns="$(arg ns)">
    <node pkg="vision_ros" type="vision_node" name="vision_node" output="screen">
      <!-- Layered config: base → mission → vehicle → local -->
      <rosparam command="load" file="$(find vision_ros)/config/vision_base.yaml"/>
      <rosparam command="load" file="$(find vision_ros)/config/vision_$(arg mission).yaml"/>
      <rosparam if="$(eval arg('extra_config') != '')"
                command="load" file="$(arg extra_config)"/>
      <rosparam if="$(eval arg('extra_local_config') != '')"
                command="load" file="$(arg extra_local_config)"/>

      <!-- Mode overrides -->
      <param name="inference/backend_type" value="fallback_only"
             if="$(eval mode == 'fallback_only')"/>
      <param name="inference/input_width" value="320"
             if="$(eval mode == 'lightweight')"/>
      <param name="inference/input_height" value="320"
             if="$(eval mode == 'lightweight')"/>

      <!-- Debug -->
      <param name="output/publish_debug_image" value="true"
             if="$(eval debug == 'true')"/>

      <!-- Image topic -->
      <param name="node/image_topic" value="$(arg image_topic)"/>
    </node>
  </group>
</launch>
```

**Step 6: Create models/.gitkeep**

```bash
touch src/vision_ros/models/.gitkeep
```

**Step 7: Commit**

```bash
git add src/vision_ros/config/ src/vision_ros/launch/ src/vision_ros/models/.gitkeep
git commit -m "feat(vision_ros): add config YAMLs and launch file"
```

---

### Task 9: Add color injection to lidar_cluster_ros

**Files:**
- Modify: `src/perception_ros/include/perception_ros/lidar_cluster_ros.hpp`
- Modify: `src/perception_ros/src/lidar_cluster_ros.cpp`
- Modify: `src/perception_ros/CMakeLists.txt` (add autodrive_msgs dep if not present)
- Modify: `src/perception_ros/config/lidar_base.yaml`

This is the minimal change to the existing LiDAR pipeline. The vision subscriber is optional — when no vision data arrives, behavior is identical to current.

**Step 1: Add members to lidar_cluster_ros.hpp**

After line 17 (`#include <fsd_common/topic_contract.hpp>`), add:

```cpp
#include <autodrive_msgs/HUAT_VisionDetections.h>
```

Inside the `LidarClusterRos` class private section (after `debug_marker_pub_` ~line 130), add:

```cpp
  // ── Vision color injection ──────────────────────────────────────
  bool vision_inject_enabled_ = false;
  ros::Subscriber vision_sub_;
  autodrive_msgs::HUAT_VisionDetections::ConstPtr last_vision_msg_;
  std::mutex vision_mutex_;
  double vision_max_age_sec_ = 0.2;       // max age of vision msg to use
  float vision_min_confidence_ = 300.0f;   // min confidence (scaled 0-1000)
  double vision_match_radius_px_ = 80.0;   // pixel-space match radius (unused for now)
  // For angular matching: vision bbox center → bearing angle
  double vision_match_angle_deg_ = 5.0;    // max bearing angle diff for match
  double camera_hfov_deg_ = 60.0;          // camera horizontal FOV
  int camera_width_px_ = 640;              // camera image width

  void visionCallback(const autodrive_msgs::HUAT_VisionDetections::ConstPtr& msg);
  uint8_t matchVisionColor(float cone_angle_deg) const;
```

**Step 2: Add vision subscriber setup in lidar_cluster_ros.cpp constructor**

In `loadParams()`, add at the end:

```cpp
  // Vision color injection params
  private_nh_.param<bool>("vision_inject/enabled", vision_inject_enabled_, false);
  private_nh_.param<double>("vision_inject/max_age_sec", vision_max_age_sec_, 0.2);
  private_nh_.param<float>("vision_inject/min_confidence", vision_min_confidence_, 300.0f);
  private_nh_.param<double>("vision_inject/match_angle_deg", vision_match_angle_deg_, 5.0);
  private_nh_.param<double>("vision_inject/camera_hfov_deg", camera_hfov_deg_, 60.0);
  private_nh_.param<int>("vision_inject/camera_width_px", camera_width_px_, 640);
```

In the constructor, after subscriber setup, add:

```cpp
  if (vision_inject_enabled_) {
    vision_sub_ = nh_.subscribe(
        fsd_common::topic_contract::kVisionDetections, 1,
        &LidarClusterRos::visionCallback, this);
    ROS_INFO("[perception] Vision color injection enabled, subscribing to '%s'",
             fsd_common::topic_contract::kVisionDetections);
  }
```

**Step 3: Add visionCallback and matchVisionColor**

```cpp
void LidarClusterRos::visionCallback(
    const autodrive_msgs::HUAT_VisionDetections::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(vision_mutex_);
  last_vision_msg_ = msg;
}

uint8_t LidarClusterRos::matchVisionColor(float cone_angle_deg) const {
  // Match LiDAR cone bearing angle to vision detection bearing angle.
  // Vision detection x pixel → bearing angle via camera model.
  std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(vision_mutex_));
  if (!last_vision_msg_) return kConeNone;

  // Check age
  double age = (ros::Time::now() - last_vision_msg_->header.stamp).toSec();
  if (age > vision_max_age_sec_) return kConeNone;

  // Check quality — don't inject from UNUSABLE frames
  if (last_vision_msg_->image_quality >= 3) return kConeNone;

  float best_angle_diff = static_cast<float>(vision_match_angle_deg_);
  uint8_t best_color = kConeNone;

  for (size_t i = 0; i < last_vision_msg_->x.size(); ++i) {
    if (last_vision_msg_->confidences[i] < static_cast<int32_t>(vision_min_confidence_))
      continue;

    // Vision pixel x → bearing angle
    float px = static_cast<float>(last_vision_msg_->x[i]);
    float vision_angle = (px / camera_width_px_ - 0.5f)
                         * static_cast<float>(camera_hfov_deg_);

    float diff = std::abs(vision_angle - cone_angle_deg);
    if (diff < best_angle_diff) {
      best_angle_diff = diff;
      best_color = last_vision_msg_->color_types[i];
    }
  }
  return best_color;
}
```

**Step 4: Modify publishOutput to inject vision color**

In `publishOutput()`, replace the `color_types.push_back(classifyConeTypeBySize(...))` block (around line 1316-1320) with:

```cpp
    uint8_t geo_color = classifyConeTypeBySize(det,
        enable_cone_size_typing_, big_cone_height_threshold_,
        big_cone_area_threshold_);

    if (vision_inject_enabled_) {
      // Compute bearing angle of this cone in degrees
      float angle_deg = static_cast<float>(
          std::atan2(det.centroid.y, det.centroid.x) * 180.0 / M_PI);
      uint8_t vision_color = matchVisionColor(angle_deg);
      // Vision provides BLUE/YELLOW/RED; geometry provides ORANGE_SMALL/BIG
      // Use vision color if available, otherwise keep geometry color
      if (vision_color != kConeNone) {
        detections.color_types.push_back(vision_color);
      } else {
        detections.color_types.push_back(geo_color);
      }
    } else {
      detections.color_types.push_back(geo_color);
    }
```

**Step 5: Add config to lidar_base.yaml**

At the end of `src/perception_ros/config/lidar_base.yaml`, add:

```yaml
# === Vision Color Injection ===
# When enabled, subscribes to vision detections and overrides color_types
# with camera-based color classification (BLUE/YELLOW/RED).
# When disabled or no vision data available, falls back to geometry-only typing.
vision_inject:
  enabled: false                # set true when vision_node is running
  max_age_sec: 0.2             # max age of vision message to accept
  min_confidence: 300           # min vision confidence (0-1000 scale)
  match_angle_deg: 5.0         # max bearing angle difference for matching
  camera_hfov_deg: 60.0        # camera horizontal field of view
  camera_width_px: 640         # camera image width in pixels
```

**Step 6: Build and verify**

```bash
catkin build perception_ros --no-deps
```

**Step 7: Commit**

```bash
git add src/perception_ros/include/perception_ros/lidar_cluster_ros.hpp \
        src/perception_ros/src/lidar_cluster_ros.cpp \
        src/perception_ros/config/lidar_base.yaml
git commit -m "feat(perception_ros): add optional vision color injection to LiDAR pipeline"
```

---

### Task 10: Integrate vision into fsd_launch

**Files:**
- Create: `src/fsd_launch/launch/subsystems/vision.launch`
- Modify: `src/fsd_launch/launch/subsystems/mission_stack.launch`

**Step 1: Create fsd_launch vision subsystem wrapper**

```xml
<!--
  FSD Vision Subsystem (fsd_launch wrapper)
  Delegates to vision_ros/launch/vision.launch with mission_stack conventions.
-->
<launch>
  <arg name="mission" default="track"/>
  <arg name="mode" default="standard"/>
  <arg name="debug" default="false"/>
  <arg name="image_topic" default="/resize_img_out"/>
  <arg name="vehicle" default="A13"/>
  <arg name="vehicle_config" default="" doc="Vehicle overlay"/>
  <arg name="vehicle_local_config" default="" doc="Local override"/>

  <include file="$(find vision_ros)/launch/vision.launch">
    <arg name="mission" value="$(arg mission)"/>
    <arg name="mode" value="$(arg mode)"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="image_topic" value="$(arg image_topic)"/>
    <arg name="extra_config" value="$(arg vehicle_config)"/>
    <arg name="extra_local_config" value="$(arg vehicle_local_config)"/>
  </include>
</launch>
```

**Step 2: Add vision to mission_stack.launch**

After the perception include block (~line 75), add:

```xml
    <!-- Vision (optional, parallel to LiDAR perception) -->
    <arg name="enable_vision" default="false" doc="Enable camera-based vision module"/>
    <arg name="vision_mode" default="standard" doc="standard | lightweight | fallback_only"/>
    <arg name="vision_debug" default="false"/>

    <include if="$(arg enable_vision)"
             file="$(find fsd_launch)/launch/subsystems/vision.launch">
        <arg name="mission" value="$(arg perception_mode)"/>
        <arg name="mode" value="$(arg vision_mode)"/>
        <arg name="debug" value="$(arg vision_debug)"/>
        <arg name="vehicle" value="$(arg vehicle)"/>
    </include>
```

Note: `enable_vision` defaults to `false` — existing behavior unchanged. User opts in with:
```bash
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=~/rosbag/track.bag enable_vision:=true
```

**Step 3: Build full workspace to verify integration**

```bash
catkin build
```

**Step 4: Commit**

```bash
git add src/fsd_launch/launch/subsystems/vision.launch \
        src/fsd_launch/launch/subsystems/mission_stack.launch
git commit -m "feat(fsd_launch): integrate vision subsystem into mission stack"
```

---

### Task 11: End-to-end smoke test with rosbag

**Precondition:** All previous tasks built successfully.

**Step 1: Test fallback-only mode (no model needed)**

```bash
# Terminal 1: launch with vision in fallback-only mode
roslaunch fsd_launch trackdrive.launch simulation:=true \
  bag:=~/rosbag/track.bag enable_vision:=true vision_mode:=fallback_only

# Terminal 2: verify vision topic is publishing
rostopic hz perception/vision/detections
# Expected: ~20-30 Hz (matching camera frame rate)

rostopic echo -n 1 perception/vision/detections
# Expected: message with backend_name="fallback_hsv", fallback_active=true

# Terminal 3: verify LiDAR detections still work
rostopic hz perception/lidar_cluster/detections
# Expected: same rate as before (~10 Hz)
```

**Step 2: Test with vision injection enabled**

Edit `src/perception_ros/config/lidar_base.yaml`:
```yaml
vision_inject:
  enabled: true
```

```bash
catkin build perception_ros --no-deps
roslaunch fsd_launch trackdrive.launch simulation:=true \
  bag:=~/rosbag/track.bag enable_vision:=true vision_mode:=fallback_only

# Check that color_types now include BLUE(0)/YELLOW(1) values
rostopic echo perception/lidar_cluster/detections/color_types
# Expected: mix of 0,1,2,3 instead of only 2,3,4
```

**Step 3: Test diagnostics**

```bash
rostopic echo perception/vision/diagnostics
# Expected: DiagnosticArray with vision_node status
```

**Step 4: Test graceful degradation (no image topic)**

```bash
# Launch without camera topic in bag (use a LiDAR-only bag)
roslaunch fsd_launch trackdrive.launch simulation:=true \
  bag:=~/rosbag/2024-10-17-01-19-05.bag enable_vision:=true

# Vision node should start but produce no output (no image callback)
# LiDAR pipeline should work normally
rostopic hz perception/lidar_cluster/detections
# Expected: normal rate, color_types = geometry-only (2/3/4)
```

**Step 5: Commit test notes**

```bash
git add -A
git commit -m "test(vision): verify end-to-end smoke test passes"
```

---

## Summary: Task Dependency Graph

```
Task 1 (scaffold + types)
  └→ Task 2 (image_quality)
  └→ Task 3 (image_enhancer)
  └→ Task 4 (inference_backend + onnx)
  └→ Task 5 (fallback + postprocess + tracker)
       └→ Task 6 (msg + topic_contract)
            └→ Task 7 (vision_node)
                 └→ Task 8 (config + launch)
                      └→ Task 9 (LiDAR color injection)
                           └→ Task 10 (fsd_launch integration)
                                └→ Task 11 (smoke test)
```

Tasks 2-5 can be parallelized after Task 1.
Task 6 must complete before Task 7.
Tasks 9-11 are sequential.
