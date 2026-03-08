#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <string>
#include <std_msgs/ColorRGBA.h>
#include <autodrive_msgs/topic_contract.hpp>

namespace fsd_viz {

// ============ 颜色配置 (RGBA) ============

// 锥桶颜色
constexpr std::array<float, 4> CONE_BLUE         = {0.0f, 0.3f, 1.0f, 1.0f};   // 蓝色锥桶
constexpr std::array<float, 4> CONE_YELLOW_SMALL = {1.0f, 0.9f, 0.0f, 1.0f};   // 小黄色锥桶
constexpr std::array<float, 4> CONE_YELLOW_BIG   = {1.0f, 0.85f, 0.0f, 1.0f};  // 大黄色锥桶（略深）
constexpr std::array<float, 4> CONE_RED          = {1.0f, 0.1f, 0.1f, 1.0f};   // 红色锥桶
constexpr std::array<float, 4> CONE_UNKNOWN      = {0.6f, 0.6f, 0.6f, 0.8f};   // 未知（灰）

// 路径颜色
constexpr std::array<float, 4> PATH_PARTIAL = {0.0f, 1.0f, 0.0f, 1.0f};   // 部分路径（绿）
constexpr std::array<float, 4> PATH_FULL    = {0.0f, 0.8f, 0.8f, 1.0f};   // 完整路径（青）
constexpr std::array<float, 4> PATH_CENTER  = {0.8f, 0.0f, 0.8f, 1.0f};   // 中心线（紫）

// 三角网格颜色
constexpr std::array<float, 4> TRI_EDGE     = {1.0f, 0.0f, 0.0f, 0.6f};   // 三角边（红）
constexpr std::array<float, 4> TRI_MIDPOINT = {0.0f, 1.0f, 0.0f, 1.0f};   // 中点（绿）

// 车辆颜色
constexpr std::array<float, 4> VEHICLE_BODY   = {1.0f, 1.0f, 1.0f, 0.9f}; // 车身（白）
constexpr std::array<float, 4> VEHICLE_ARROW  = {0.0f, 1.0f, 0.0f, 1.0f}; // 方向箭头（绿）
constexpr std::array<float, 4> VEHICLE_WHEEL  = {0.2f, 0.2f, 0.2f, 1.0f}; // 车轮（深灰）
constexpr std::array<float, 4> VEHICLE_TRAIL  = {0.0f, 0.8f, 1.0f, 0.7f}; // 轨迹（青）
constexpr std::array<float, 4> VEHICLE_VELOCITY = {1.0f, 0.0f, 0.5f, 1.0f}; // 速度矢量（品红）
constexpr std::array<float, 4> VEHICLE_STEERING = {1.0f, 0.5f, 0.0f, 1.0f}; // 转向指示（橙）

// 边界线颜色
constexpr std::array<float, 4> BOUNDARY_LEFT  = {1.0f, 1.0f, 0.0f, 0.8f}; // 左边界（黄）
constexpr std::array<float, 4> BOUNDARY_RIGHT = {0.0f, 0.3f, 1.0f, 0.8f}; // 右边界（蓝）

// ============ 尺寸配置 ============

// 锥桶
constexpr float CONE_RADIUS = 0.15f;
constexpr float CONE_HEIGHT = 0.30f;

// 路径
constexpr float PATH_WIDTH      = 0.08f;
constexpr float PATH_POINT_SIZE = 0.10f;

// 三角网格
constexpr float TRI_LINE_WIDTH  = 0.02f;
constexpr float MIDPOINT_SIZE   = 0.08f;

// 车辆 (方程式赛车尺寸)
constexpr float VEHICLE_LENGTH    = 2.8f;   // 车长
constexpr float VEHICLE_WIDTH     = 1.4f;   // 车宽
constexpr float VEHICLE_HEIGHT    = 0.5f;   // 车高
constexpr float VEHICLE_WHEELBASE = 1.55f;  // 轴距
constexpr float WHEEL_RADIUS      = 0.25f;
constexpr float WHEEL_WIDTH       = 0.15f;
constexpr float TRAIL_WIDTH       = 0.05f;  // 轨迹线宽

// ============ Frame ID ============
const std::string FRAME_GLOBAL  = autodrive_msgs::frame_contract::kWorld;    // 全局坐标系（ENU）
const std::string FRAME_VEHICLE = autodrive_msgs::frame_contract::kVelodyne; // 车辆坐标系（雷达）

// ============ 锥桶类型 ============
// 与 HUAT_ConeDetections.color_types / HUAT_Cone.type 对齐:
// 0=BLUE, 1=YELLOW_SMALL, 2=YELLOW_BIG, 3=RED, 4=NONE
// 注意：已移除橙色锥桶类型，改为大小黄色锥桶区分
enum class ConeType : uint8_t {
    BLUE = 0,          // 蓝色锥桶
    YELLOW_SMALL = 1,  // 小黄色锥桶
    YELLOW_BIG = 2,    // 大黄色锥桶
    RED = 3,           // 红色锥桶
    NONE = 4           // 无类型/未知
};

inline std::array<float, 4> getConeColor(ConeType type) {
    switch (type) {
        case ConeType::BLUE:         return CONE_BLUE;
        case ConeType::YELLOW_SMALL: return CONE_YELLOW_SMALL;
        case ConeType::YELLOW_BIG:   return CONE_YELLOW_BIG;
        case ConeType::RED:          return CONE_RED;
        default:                     return CONE_UNKNOWN;
    }
}

inline std::array<float, 4> getConeColor(int type) {
    return getConeColor(static_cast<ConeType>(type));
}

// ============ 3D 网格资源 URI ============

// --- FSSIM 锥桶 DAE 模型（内嵌颜色，无外部纹理） ---
const std::string MESH_CONE_BLUE       = "package://fsd_visualization/meshes/cone/cone_blue.dae";
const std::string MESH_CONE_YELLOW     = "package://fsd_visualization/meshes/cone/cone_yellow.dae";
const std::string MESH_CONE_ORANGE     = "package://fsd_visualization/meshes/cone/cone_orange.dae";
const std::string MESH_CONE_ORANGE_BIG = "package://fsd_visualization/meshes/cone/cone_orange_big.dae";

// --- Gazebo construction_cone（带纹理贴图的高质量锥桶） ---
const std::string MESH_CONE_GAZEBO     = "package://fsd_visualization/meshes/cone/construction_cone.dae";
// Gazebo cone DAE 单位为 inch (0.0254m)，SDF 中用 scale=10 → 实际约 0.3m 高
// RViz MESH_RESOURCE 的 scale 直接乘以几何体坐标，所以 scale=10 即可
constexpr float CONE_GAZEBO_SCALE      = 10.0f;

// --- 赛车模型 ---
const std::string MESH_VEHICLE_BODY    = "package://fsd_visualization/meshes/vehicle/whole_car.stl";
constexpr float VEHICLE_MESH_SCALE     = 0.001f;  // STL 单位为 mm，需缩放到 m

inline std::string getConeMeshURI(ConeType type) {
    switch (type) {
        case ConeType::BLUE:         return MESH_CONE_BLUE;
        case ConeType::YELLOW_SMALL: return MESH_CONE_YELLOW;
        case ConeType::YELLOW_BIG:   return MESH_CONE_YELLOW;  // 大黄色使用黄色模型（通过scale区分大小）
        case ConeType::RED:          return MESH_CONE_ORANGE;  // 红色使用橙色模型（红色模型暂缺）
        default:                     return MESH_CONE_YELLOW;  // 未知类型回退到黄色
    }
}

inline std::string getConeMeshURI(int type) {
    return getConeMeshURI(static_cast<ConeType>(type));
}

// ============ 速度色带 (Speed Color Gradient) ============
// 绿(低速) → 黄(中速) → 红(高速)
// speed_ratio: 0.0 = min_speed, 1.0 = max_speed
inline std_msgs::ColorRGBA speedToColor(double speed, double v_min, double v_max) {
    std_msgs::ColorRGBA c;
    c.a = 1.0f;
    double ratio = (v_max > v_min) ? (speed - v_min) / (v_max - v_min) : 0.0;
    ratio = std::max(0.0, std::min(1.0, ratio));
    if (ratio < 0.5) {
        // 绿 → 黄
        double t = ratio * 2.0;
        c.r = static_cast<float>(t);
        c.g = 1.0f;
        c.b = 0.0f;
    } else {
        // 黄 → 红
        double t = (ratio - 0.5) * 2.0;
        c.r = 1.0f;
        c.g = static_cast<float>(1.0 - t);
        c.b = 0.0f;
    }
    return c;
}

// 制动区域颜色（半透明红）
constexpr std::array<float, 4> BRAKE_ZONE = {1.0f, 0.1f, 0.1f, 0.6f};

// ============ 曲率色带 (Curvature Color Gradient) ============
// 蓝(低曲率/直道) → 青(中曲率) → 红(高曲率/急弯)
inline std_msgs::ColorRGBA curvatureToColor(double kappa, double k_min, double k_max) {
    std_msgs::ColorRGBA c;
    c.a = 1.0f;
    double ratio = (k_max > k_min) ? (std::abs(kappa) - k_min) / (k_max - k_min) : 0.0;
    ratio = std::max(0.0, std::min(1.0, ratio));
    if (ratio < 0.5) {
        // 蓝 → 青
        double t = ratio * 2.0;
        c.r = 0.0f;
        c.g = static_cast<float>(t);
        c.b = 1.0f;
    } else {
        // 青 → 红
        double t = (ratio - 0.5) * 2.0;
        c.r = static_cast<float>(t);
        c.g = static_cast<float>(1.0 - t);
        c.b = static_cast<float>(1.0 - t);
    }
    return c;
}

}  // namespace fsd_viz
