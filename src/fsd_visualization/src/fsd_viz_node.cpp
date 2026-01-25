/**
 * @file fsd_viz_node.cpp
 * @brief FSD 统一可视化节点
 * 
 * 集成锥桶、路径、车辆可视化
 */

#include <ros/ros.h>
#include "fsd_visualization/cone_visualizer.hpp"
#include "fsd_visualization/path_visualizer.hpp"
#include "fsd_visualization/vehicle_visualizer.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "fsd_viz_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("===========================================");
    ROS_INFO("FSD Visualization Node Starting...");
    ROS_INFO("===========================================");
    
    // 读取启用参数
    bool enable_cones, enable_path, enable_vehicle;
    pnh.param<bool>("enable_cones", enable_cones, true);
    pnh.param<bool>("enable_path", enable_path, true);
    pnh.param<bool>("enable_vehicle", enable_vehicle, true);
    
    // 创建可视化器
    std::unique_ptr<fsd_viz::ConeVisualizer> cone_viz;
    std::unique_ptr<fsd_viz::PathVisualizer> path_viz;
    std::unique_ptr<fsd_viz::VehicleVisualizer> vehicle_viz;
    
    if (enable_cones) {
        cone_viz = std::make_unique<fsd_viz::ConeVisualizer>(nh, pnh);
        ROS_INFO("[FSD_VIZ] Cone visualization: ENABLED");
    } else {
        ROS_INFO("[FSD_VIZ] Cone visualization: DISABLED");
    }
    
    if (enable_path) {
        path_viz = std::make_unique<fsd_viz::PathVisualizer>(nh, pnh);
        ROS_INFO("[FSD_VIZ] Path visualization: ENABLED");
    } else {
        ROS_INFO("[FSD_VIZ] Path visualization: DISABLED");
    }
    
    if (enable_vehicle) {
        vehicle_viz = std::make_unique<fsd_viz::VehicleVisualizer>(nh, pnh);
        ROS_INFO("[FSD_VIZ] Vehicle visualization: ENABLED");
    } else {
        ROS_INFO("[FSD_VIZ] Vehicle visualization: DISABLED");
    }
    
    ROS_INFO("-------------------------------------------");
    ROS_INFO("Published Topics (defaults):");
    ROS_INFO("  fsd/viz/cones      - Cone markers");
    ROS_INFO("  fsd/viz/path       - Path markers");
    ROS_INFO("  fsd/viz/boundaries - Boundary markers");
    ROS_INFO("  fsd/viz/vehicle    - Vehicle markers");
    ROS_INFO("-------------------------------------------");
    
    ros::spin();
    
    ROS_INFO("FSD Visualization Node Shutdown");
    return 0;
}
