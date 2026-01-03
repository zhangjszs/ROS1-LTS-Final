#include "control.h"
namespace control
{
    class Line : public Car
    {
    public:
        Line()
        {
            get_param("pp");

            pub_cmd = nh.advertise<common_msgs::HUAT_VehcileCmd>("vehcileCMDMsg", 1000);
            sub_pose = nh.subscribe("/Carstate", 10, &Line::pose_callback, this);                                 // 订阅车辆状态 								// 发布控制命令
            sub_path = nh.subscribe("/line_creat/line_global_path", 100, &Line::path_callback, this);             // 订阅路径
            sub_last = nh.subscribe("/skidpad_detection_node/approaching_goal", 100, &Line::last_callback, this); // 订阅目标点抵达判断
        }

        void pose_callback(const common_msgs::HUAT_Carstate::ConstPtr &msgs)
        {
            geometry_msgs::Pose pose;
            car_x = pose.position.x = msgs->car_state.x;
            car_y = pose.position.y = msgs->car_state.y;
            pose.position.z = 0;
            car_veloc = msgs->V;
            tf::Quaternion qAux;
            qAux.setRPY(0.0, 0.0, msgs->car_state.theta);
            tf::quaternionTFToMsg(qAux, pose.orientation);
            tf::poseMsgToEigen(pose, local_tf);
            local_tf = local_tf.inverse();
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
            // std::ifstream ifs;
            // ifs.open("/home/czy/line/src/control/src/path.txt", std::ios::in);
            // control::position temp;

            // while (ifs >> temp.x >> temp.y)
            //     path_coordinate.push_back(temp);
            // ifs.close();
        }

        void last_callback(const std_msgs::Bool::ConstPtr &msg)
        {
            get_last_judge = msg->data;
        }

        int get_target()
        {
            double min_distance = std::numeric_limits<double>::max();
            double diff_distance = 0;
            double tem_distance;
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

            for (index = min + 1, diff_distance = 0.0; index < path_coordinate.size(); index++)
            {
                tem_distance = sqrt(distance_square(path_coordinate[index - 1].x, path_coordinate[index - 1].y, path_coordinate[index].x, path_coordinate[index].y));
                diff_distance += tem_distance;
                if (diff_distance + tem_distance > lookhead)
                    break;
            }
            if (abs(lookhead - diff_distance - tem_distance) < abs(lookhead - diff_distance))
                return index;
            else
                return index - 1;
        }

        int get_steering() override
        {
            int index = get_target();
            tar = index;
            if (index >= path_coordinate.size() - 1 && get_last_judge)
            {
                pub_cmd.publish(stop_cmd);
                ros::shutdown();
            }
            Eigen::Vector3d product = local_tf * Eigen::Vector3d(path_coordinate[index].x, path_coordinate[index].y, 0.0);

            double goalX = product.x(), goalY = product.y();
            j = goalX, k = goalY;
            double alpha = atan2(goalY, goalX);

            alpha = angle_range(alpha);

            double delta = atan2(2 * car_length * sin(alpha) / lookhead, 1.0);

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