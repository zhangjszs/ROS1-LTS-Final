#pragma once

#include <array>
#include <string>

namespace fsd_viz {

// ============ 颜色配置 (RGBA) ============

// 锥桶颜色
constexpr std::array<float, 4> CONE_BLUE    = {0.0f, 0.3f, 1.0f, 1.0f};   // 左边界（蓝）
constexpr std::array<float, 4> CONE_YELLOW  = {1.0f, 0.9f, 0.0f, 1.0f};   // 右边界（黄）
constexpr std::array<float, 4> CONE_ORANGE  = {1.0f, 0.5f, 0.0f, 1.0f};   // 大橙桶
constexpr std::array<float, 4> CONE_UNKNOWN = {0.6f, 0.6f, 0.6f, 0.8f};   // 未知（灰）

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
const std::string FRAME_GLOBAL  = "world";    // 全局坐标系（ENU）
const std::string FRAME_VEHICLE = "velodyne"; // 车辆坐标系（雷达）

// ============ 锥桶类型 ============
enum class ConeType : uint8_t {
    UNKNOWN = 0,
    BLUE    = 1,
    YELLOW  = 2,
    ORANGE  = 3,
    ORANGE_BIG = 4
};

inline std::array<float, 4> getConeColor(ConeType type) {
    switch (type) {
        case ConeType::BLUE:       return CONE_BLUE;
        case ConeType::YELLOW:     return CONE_YELLOW;
        case ConeType::ORANGE:
        case ConeType::ORANGE_BIG: return CONE_ORANGE;
        default:                   return CONE_UNKNOWN;
    }
}

inline std::array<float, 4> getConeColor(int type) {
    return getConeColor(static_cast<ConeType>(type));
}

}  // namespace fsd_viz
