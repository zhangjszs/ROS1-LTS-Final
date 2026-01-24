#include "control.h"
#include <ros/package.h>
#include <stdlib.h>

namespace control
{
	void Car::get_param(std::string func)
	{
		nh.param(func + "/angle_kp", angle_kp, 1.0);
		nh.param(func + "/angle_ki", angle_ki, 0.0);
		nh.param(func + "/angle_kd", angle_kd, 0.0);
		nh.param(func + "/steering_delta_max", steering_delta_max, 0.5);
		nh.param(func + "/angle_kv", angle_kv, 0.0);
		nh.param(func + "/angle_kl", angle_kl, 2.0);
        nh.param("num", num, 1);
	}

	double Car::distance_square(double x1, double y1, double x2, double y2)
	{
		return pow(x1 - x2, 2) + pow(y1 - y2, 2);
	}

	double Car::angle_range(double alpha)
	{
		return (alpha > M_PI) ? (alpha - 2 * M_PI) : (alpha < -M_PI) ? (alpha + 2 * M_PI)
																	 : alpha;
	}

	double Car::angle_pid(double delta)
	{
		double error, differ;
		error = delta - car_fangle;

		if (error <= steering_delta_max)
			angle_integra = 0.0;
		angle_integra += error;
		differ = error - car_fangle;
		differ = error;

		delta = angle_kp * error + angle_ki * angle_integra + angle_kd * differ + car_fangle;

		if (delta > steering_delta_max)
			delta = steering_delta_max;
		else if (delta < -steering_delta_max)
			delta = -steering_delta_max;

		car_fangle = delta;
		return delta;
	}

	void Car::file_write(int i)
	{
		std::ofstream ofs;
		std::stringstream ss;
		std::string path = ros::package::getPath("control") + "/doc/my_cmd.txt";
		ofs.open(path.c_str(), std::ios_base::app);
		ss << "方向盘 = " << i << " ; car_x = " << car_x << " ; car_y = " << car_y
		   << " ; tar_x = " << path_coordinate[tar].x << " ; tar_y = " << path_coordinate[tar].y
		   << " ; gx = " << j << " ; gy = " << k << std::endl;

		ofs << ss.str();
		ofs.close();
	}

	void Car::control_cmd(autodrive_msgs::HUAT_VehicleCmd &cmd)
	{
		std::ifstream ifs;
		std::string home_path = getenv("HOME");
		std::string cmd_path = home_path + "/autoStartGkj/command";
		ifs.open(cmd_path.c_str(), std::ios::in);
		if (!ifs.is_open())
		{
			std::cout << "文件打开失败" << std::endl;
			get_last_judge = 1;
		}
		else if(!((char)ifs.get() - '0'))
			get_last_judge = 1;

		if (get_last_judge)
		{
			cmd.head1 = 0XAA;
			cmd.head2 = 0X55;
			cmd.length = 10;
			cmd.steering = 110;
			cmd.pedal_ratio = 0;
			cmd.brake_force = 80;
			cmd.gear_position = 0;
			cmd.working_mode = 1;
			cmd.racing_num = num;
			cmd.racing_status = 5;
			cmd.checksum = cmd.head1 + cmd.head2 + cmd.length +
						   cmd.steering + cmd.pedal_ratio + cmd.brake_force +
						   cmd.gear_position + cmd.working_mode + cmd.racing_num + cmd.racing_status;

			pub_cmd.publish(cmd);
			exit(0);
		}
		else
		{
			if (get_path_judge || num == 1 || num == 5)
			{
				cmd.head1 = 0XAA;
				cmd.head2 = 0X55;
				cmd.length = 10;

				cmd.steering = get_steering();	   // 方向盘
				cmd.pedal_ratio = get_pedalling(); // 油门
				cmd.brake_force = get_braking();   // 刹车
				cmd.gear_position = 1;			   // 汽车档位

				// 其他数据
				cmd.working_mode = 1;
				cmd.racing_num = num;
				cmd.racing_status = get_status();
				cmd.checksum = cmd.head1 + cmd.head2 + cmd.length +
							   cmd.steering + cmd.pedal_ratio + cmd.brake_force +
							   cmd.gear_position + cmd.working_mode + cmd.racing_num + cmd.racing_status;

				// file_write(cmd.steering);

				// 发布数据
				pub_cmd.publish(cmd);
			}
			else
				ROS_WARN("得不到有效的惯导路径信息");
		}

		++now;
	}
}
