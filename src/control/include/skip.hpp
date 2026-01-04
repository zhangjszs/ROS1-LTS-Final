#include "control.h"
namespace control
{
    class Skip : public Car
    {
    public:
        Skip()
        {
            get_param("st");

            pub_cmd = nh.advertise<common_msgs::HUAT_VehcileCmd>("vehcileCMDMsg", 1000);
            sub_pose = nh.subscribe("/Carstate", 10, &Skip::pose_callback, this);                                 // 订阅车辆状态 								// 发布控制命令
            sub_path = nh.subscribe("/skidpad_detection_node/log_path", 100, &Skip::path_callback, this);         // 订阅路径
            sub_last = nh.subscribe("/skidpad_detection_node/approaching_goal", 100, &Skip::last_callback, this); // 订阅目标点抵达判断
        }

        void pose_callback(const common_msgs::HUAT_Carstate::ConstPtr &msgs)
        {
            geometry_msgs::Pose pose;
            car_x = pose.position.x = msgs->car_state.x;
            car_y = pose.position.y = msgs->car_state.y;
            pose.position.z = 0;
            car_veloc = msgs->V;
            car_fangle = msgs->car_state.theta;
            get_pose_judge = true;
        }

        void path_callback(const nav_msgs::Path::ConstPtr &msgs)
        {
            if (!get_pose_judge)
            {
                ROS_WARN("CarState not being received.");
                return;
            }
            get_pose_judge = false;
            get_path_judge = true;
            path_coordinate.clear();
            for (const auto &pose : msgs->poses)
            {
                double x = pose.pose.position.x;
                double y = pose.pose.position.y;
                path_coordinate.push_back({x, y});
            }
        }

        void last_callback(const std_msgs::Bool::ConstPtr &msg)
        {
            get_last_judge = msg->data;
        }

        int get_min()
        {
            double min_distance = std::numeric_limits<double>::max();
            int index, min;
            lookhead = angle_kv * car_veloc + angle_kl;
            for (min = index = 0; index < path_coordinate.size(); index++)
            {
                double tem_distance = distance_square(car_x, car_y, path_coordinate[index].x, path_coordinate[index].y);
                if (tem_distance < min_distance)
                {
                    min_distance = tem_distance;
                    min = index;
                }
            }
            tar = min;
            return min;
        }

        double count_error(double x1, double y1, double x2, double y2, double heading)
        {
            double dx = x1 - x2;
            double dy = y1 - y2;
            double error = sqrt(pow(dx, 2) + pow(dy, 2));
            return dy * cos(heading) - dx * sin(heading) > 0 ? -error : error;
        }

        int get_steering() override
        {
            int index_min = get_min();
            if (index_min >= path_coordinate.size() - 1 && get_last_judge)
            {
                pub_cmd.publish(stop_cmd);
                ros::shutdown();
            }
            double alpha = atan2(path_coordinate[index_min + 1].y - path_coordinate[index_min].y, path_coordinate[index_min + 1].x - path_coordinate[index_min].x) - car_fangle;

            alpha = angle_range(alpha);

            double e_y = count_error(path_coordinate[index_min].x, path_coordinate[index_min].y, car_x, car_y, alpha);

            double delta = atan2(0.5 * e_y, car_veloc + 6) + alpha;

            delta = angle_pid(delta);

            return int(delta / M_PI * 180 * 3.73) + 110;
        }

        int get_pedalling() override
        {
            double target = 4.0;
            double error = target - car_veloc;
            double accel;
            veloc_integra += error;
            accel = 0.5 * error + 0.1 * veloc_integra;

            if (car_veloc > 2)
                accel = 5;

            if (accel > 30)
                accel = 5;

            if (car_veloc <= 0.3)
                accel = 5;
            return (int)accel;
        }

        int get_braking() override
        {
            return 0;
        }

        int get_status() override
        {
            return 2;
        }
    };
}