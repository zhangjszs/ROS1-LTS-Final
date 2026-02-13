#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>

#include <simulation_core/types.hpp>
#include <simulation_core/vehicle_dynamics.hpp>
#include <simulation_core/cone_sensor.hpp>
#include <simulation_core/track_loader.hpp>

#include <autodrive_msgs/HUAT_CarState.h>
#include <autodrive_msgs/HUAT_SimState.h>
#include <autodrive_msgs/HUAT_ConeMap.h>
#include <autodrive_msgs/HUAT_VehicleCmd.h>
#include <autodrive_msgs/topic_contract.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>

using namespace simulation_core;

namespace {
std::uint32_t EncodeConfidenceScaled(double confidence_score) {
    const double clamped = std::max(0.0, std::min(1.0, confidence_score));
    return static_cast<std::uint32_t>(std::lround(clamped * 1000.0));
}
}  // namespace

class SimulationNode {
public:
    SimulationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_broadcaster_() {
        // Load parameters
        loadParameters();

        // Initialize simulation
        initSimulation();

        // Setup publishers
        pub_car_state_ = nh_.advertise<autodrive_msgs::HUAT_CarState>(
            pnh_.param<std::string>("car_state_topic", "~car_state"), 10);
        pub_sim_state_ = nh_.advertise<autodrive_msgs::HUAT_SimState>(
            pnh_.param<std::string>("sim_state_topic", "~sim_state"), 10);
        pub_cone_map_ = nh_.advertise<autodrive_msgs::HUAT_ConeMap>(
            pnh_.param<std::string>("cone_map_topic", "~cone_map"), 10);
        pub_track_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "fsd/viz/track", 1, true);  // Latched

        // Setup subscribers
        sub_cmd_ = nh_.subscribe(pnh_.param<std::string>("cmd_topic", "~cmd"), 1,
            &SimulationNode::cmdCallback, this);

        // Publish track visualization (once)
        publishTrackMarkers();

        ROS_INFO("[SimulationNode] Initialized with track: %s (%zu cones)",
                 track_.name.c_str(), track_.cones.size());
        ROS_INFO("[SimulationNode] Frames: world=%s, base_link=%s",
                 world_frame_.c_str(), base_link_frame_.c_str());
    }

    void run() {
        ros::Rate rate(sim_rate_);

        while (ros::ok()) {
            ros::spinOnce();

            // Get current control input
            ControlInput input;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                input = current_input_;
            }

            // Step simulation
            double dt = 1.0 / sim_rate_;
            dynamics_.step(input, dt);

            // Get observations
            std::vector<ConeObservation> observations;
            sensor_.observe(dynamics_.state().vehicle, observations);

            // Publish outputs
            publishCarState();
            publishSimState();
            publishConeMap(observations);

            if (publish_tf_) {
                publishTF();
            }

            rate.sleep();
        }
    }

private:
    void loadParameters() {
        // Simulation rate
        pnh_.param<int>("sim_rate", sim_rate_, 100);
        pnh_.param<bool>("publish_tf", publish_tf_, true);
        pnh_.param<std::string>("frames/world", world_frame_, autodrive_msgs::frame_contract::kWorld);
        pnh_.param<std::string>("frames/base_link", base_link_frame_,
                                autodrive_msgs::frame_contract::kBaseLink);

        // Track file
        std::string track_file;
        pnh_.param<std::string>("track_file", track_file, "");

        if (!track_file.empty()) {
            TrackLoader loader;
            if (!loader.load(track_file, track_)) {
                ROS_WARN("[SimulationNode] Failed to load track: %s, using default",
                         loader.lastError().c_str());
                track_ = TrackLoader::createOvalTrack();
            }
        } else {
            track_ = TrackLoader::createOvalTrack();
        }

        // Vehicle parameters
        VehicleParams vp;
        ros::NodeHandle sim_nh(nh_, "simulation");
        sim_nh.param<double>("vehicle/mass", vp.mass, 250.0);
        sim_nh.param<double>("vehicle/wheelbase", vp.wheelbase, 1.55);
        sim_nh.param<double>("vehicle/track_width", vp.track_width, 1.2);
        sim_nh.param<double>("vehicle/cg_height", vp.cg_height, 0.3);
        sim_nh.param<double>("vehicle/cg_to_front", vp.cg_to_front, 0.775);
        sim_nh.param<double>("vehicle/cg_to_rear", vp.cg_to_rear, 0.775);
        sim_nh.param<double>("vehicle/inertia_z", vp.inertia_z, 150.0);
        sim_nh.param<double>("vehicle/wheel_radius", vp.wheel_radius, 0.23);
        sim_nh.param<double>("vehicle/max_steering", vp.max_steering, 0.5);
        sim_nh.param<double>("vehicle/max_torque", vp.max_torque, 200.0);
        sim_nh.param<double>("vehicle/max_brake_torque", vp.max_brake_torque, 500.0);
        sim_nh.param<double>("aero/drag_coeff", vp.drag_coeff, 0.8);
        sim_nh.param<double>("aero/downforce_coeff", vp.downforce_coeff, 2.5);
        sim_nh.param<double>("aero/frontal_area", vp.frontal_area, 1.0);
        sim_nh.param<double>("aero/air_density", vp.air_density, 1.225);
        vehicle_params_ = vp;

        // Tire parameters
        TireParams tp;
        sim_nh.param<double>("tire/B", tp.B, 10.0);
        sim_nh.param<double>("tire/C", tp.C, 1.65);
        sim_nh.param<double>("tire/D", tp.D, 1.0);
        sim_nh.param<double>("tire/E", tp.E, -0.5);
        sim_nh.param<double>("tire/friction", tp.friction, 1.0);
        tire_params_ = tp;

        // Sensor parameters
        SensorParams sp;
        sim_nh.param<double>("lidar/max_range", sp.lidar_max_range, 20.0);
        sim_nh.param<double>("lidar/fov", sp.lidar_fov, M_PI);
        sim_nh.param<double>("lidar/noise_radial", sp.lidar_noise_radial, 0.05);
        sim_nh.param<double>("lidar/noise_angular", sp.lidar_noise_angular, 0.007);
        sim_nh.param<double>("lidar/detection_rate", sp.lidar_detection_rate, 0.95);
        sim_nh.param<double>("camera/max_range", sp.camera_max_range, 15.0);
        sim_nh.param<double>("camera/color_accuracy", sp.camera_color_accuracy, 0.9);
        sim_nh.param<double>("camera/delay", sp.camera_delay, 0.1);
        sensor_params_ = sp;
    }

    void initSimulation() {
        dynamics_.setVehicleParams(vehicle_params_);
        dynamics_.setTireParams(tire_params_);
        dynamics_.reset(track_);

        sensor_.setParams(sensor_params_);
        sensor_.setTrack(track_);
    }

    void cmdCallback(const autodrive_msgs::HUAT_VehicleCmd::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        // steering is uint8 (0-255), map to [-max_steering, max_steering]
        // 127 = center, 0 = full left, 255 = full right
        double steering_normalized = (static_cast<double>(msg->steering) - 127.0) / 127.0;
        current_input_.steering = steering_normalized * vehicle_params_.max_steering;

        // pedal_ratio is uint8 (0-255), map to [0, 1]
        current_input_.throttle = static_cast<double>(msg->pedal_ratio) / 255.0;

        // brake_force is uint8 (0-255), map to [0, 1]
        current_input_.brake = static_cast<double>(msg->brake_force) / 255.0;
    }

    void publishCarState() {
        const auto& state = dynamics_.state();

        autodrive_msgs::HUAT_CarState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame_;

        msg.car_state.x = state.vehicle.x;
        msg.car_state.y = state.vehicle.y;
        msg.car_state.theta = state.vehicle.yaw;

        // Front and rear points
        double cos_yaw = std::cos(state.vehicle.yaw);
        double sin_yaw = std::sin(state.vehicle.yaw);
        double lf = vehicle_params_.cg_to_front;
        double lr = vehicle_params_.cg_to_rear;

        msg.car_state_front.x = state.vehicle.x + lf * cos_yaw;
        msg.car_state_front.y = state.vehicle.y + lf * sin_yaw;
        msg.car_state_front.z = 0.0;

        msg.car_state_rear.x = state.vehicle.x - lr * cos_yaw;
        msg.car_state_rear.y = state.vehicle.y - lr * sin_yaw;
        msg.car_state_rear.z = 0.0;

        msg.V = std::hypot(state.vehicle.vx, state.vehicle.vy);
        msg.W = state.vehicle.yaw_rate;
        msg.A = 0.0;  // TODO: calculate acceleration

        pub_car_state_.publish(msg);
    }

    void publishSimState() {
        const auto& state = dynamics_.state();

        autodrive_msgs::HUAT_SimState msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame_;

        msg.pose.x = state.vehicle.x;
        msg.pose.y = state.vehicle.y;
        msg.pose.theta = state.vehicle.yaw;

        msg.velocity.x = state.vehicle.vx;
        msg.velocity.y = state.vehicle.vy;
        msg.velocity.z = state.vehicle.yaw_rate;

        msg.steering_angle = state.vehicle.steering;
        msg.throttle = state.input.throttle;
        msg.brake = state.input.brake;
        msg.sim_time = state.sim_time;

        pub_sim_state_.publish(msg);
    }

    void publishConeMap(const std::vector<ConeObservation>& observations) {
        autodrive_msgs::HUAT_ConeMap msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame_;

        const auto& state = dynamics_.state();
        double cos_yaw = std::cos(state.vehicle.yaw);
        double sin_yaw = std::sin(state.vehicle.yaw);

        uint32_t id = 0;
        for (const auto& obs : observations) {
            autodrive_msgs::HUAT_Cone cone;
            cone.header.stamp = msg.header.stamp;
            cone.header.frame_id = world_frame_;

            // Position in base_link frame
            cone.position_baseLink.x = obs.x;
            cone.position_baseLink.y = obs.y;
            cone.position_baseLink.z = 0.0;

            // Position in global frame
            cone.position_global.x = state.vehicle.x + obs.x * cos_yaw - obs.y * sin_yaw;
            cone.position_global.y = state.vehicle.y + obs.x * sin_yaw + obs.y * cos_yaw;
            cone.position_global.z = 0.0;

            cone.id = id++;
            cone.confidence = EncodeConfidenceScaled(obs.confidence);
            cone.type = static_cast<uint32_t>(obs.color);

            msg.cone.push_back(cone);
        }

        pub_cone_map_.publish(msg);
    }

    void publishTF() {
        const auto& state = dynamics_.state();

        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = base_link_frame_;

        tf.transform.translation.x = state.vehicle.x;
        tf.transform.translation.y = state.vehicle.y;
        tf.transform.translation.z = 0.0;

        double half_yaw = state.vehicle.yaw / 2.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = std::sin(half_yaw);
        tf.transform.rotation.w = std::cos(half_yaw);

        tf_broadcaster_.sendTransform(tf);
    }

    void publishTrackMarkers() {
        visualization_msgs::MarkerArray markers;

        // Delete all previous markers
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(delete_marker);

        int id = 0;
        for (const auto& cone : track_.cones) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "track_cones";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = cone.x;
            marker.pose.position.y = cone.y;
            marker.pose.position.z = 0.15;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.228;
            marker.scale.y = 0.228;
            marker.scale.z = 0.325;

            // Color based on cone type
            switch (cone.color) {
                case ConeColor::BLUE:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
                case ConeColor::YELLOW:
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                case ConeColor::ORANGE:
                case ConeColor::ORANGE_BIG:
                    marker.color.r = 1.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.0;
                    if (cone.color == ConeColor::ORANGE_BIG) {
                        marker.scale.x = 0.285;
                        marker.scale.y = 0.285;
                        marker.scale.z = 0.505;
                    }
                    break;
                default:
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
            }
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(0);  // Persistent

            markers.markers.push_back(marker);
        }

        pub_track_markers_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Publishers
    ros::Publisher pub_car_state_;
    ros::Publisher pub_sim_state_;
    ros::Publisher pub_cone_map_;
    ros::Publisher pub_track_markers_;

    // Subscribers
    ros::Subscriber sub_cmd_;

    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Simulation components
    VehicleDynamics dynamics_;
    ConeSensor sensor_;
    Track track_;

    // Parameters
    VehicleParams vehicle_params_;
    TireParams tire_params_;
    SensorParams sensor_params_;
    int sim_rate_;
    bool publish_tf_;
    std::string world_frame_;
    std::string base_link_frame_;

    // Control input
    ControlInput current_input_;
    std::mutex cmd_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simulation_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    SimulationNode node(nh, pnh);
    node.run();

    return 0;
}
