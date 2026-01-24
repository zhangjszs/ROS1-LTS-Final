#include <perception_ros/imu_subscriber.hpp>
IMUSubscriber::IMUSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const autodrive_msgs::HUAT_InsP2ConstPtr imu_msg_ptr)
{
    IMUData imu_data;
    mtx.lock();
    imu_data.time = imu_msg_ptr->header.stamp.toSec();
    imu_data.linear_acceleration.x = imu_msg_ptr->Vn;
    imu_data.linear_acceleration.y = imu_msg_ptr->Ve;
    imu_data.linear_acceleration.z = imu_msg_ptr->Vu;
    imu_data.angular_velocity.x = imu_msg_ptr->gyro_x;
    imu_data.angular_velocity.y = imu_msg_ptr->gyro_y;
    imu_data.angular_velocity.z = imu_msg_ptr->gyro_z;
    imu_data.rpy.heading = imu_msg_ptr->Heading;
    imu_data.rpy.pitch = imu_msg_ptr->Pitch;
    // std::cout<<imu_msg_ptr->Heading<<" "<<imu_msg_ptr->Pitch<<std::endl;
    new_imu_data_.push_back(imu_data);
    mtx.unlock();
    // new_imu_data_.pop_front();
    // std::cout<<"linear_acceleration:"<<"("<<imu_data.linear_acceleration.x<<","<<imu_data.linear_acceleration.y<<","<<imu_data.linear_acceleration.z<<")"<<std::endl;
    // std::cout<<"angular_velocity:"<<"("<<imu_data.angular_velocity.x<<","<<imu_data.angular_velocity.y<<","<<imu_data.angular_velocity.z<<")"<<std::endl<<std::endl;
}

void IMUSubscriber::ParseData(std::deque<IMUData> &imu_data_buff)
{
    mtx.lock();
    if (new_imu_data_.size() > 0)
    {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        // std::cout<<"size: "<<new_imu_data_.size()<<"   "<<imu_data_buff.size()<<std::endl;
        new_imu_data_.clear();
    }
    mtx.unlock();
}

bool IMUSubscriber::SyncData(std::deque<IMUData> &UnsyncedData, IMUData &synced_data, double sync_time)
{
    IMUData tmp_imu_data;
    while (UnsyncedData.size() >= 2)
    {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time)
        {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2)
        {
            UnsyncedData.pop_front();
            break;
        }

        if (UnsyncedData.at(1).time - sync_time > 0.2)
        {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    // std::cout<<back_data.rpy.heading<<std::endl;
    // std::cout<<front_data.rpy.heading<<std::endl;
    // IMUData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    synced_data.rpy.heading = front_data.rpy.heading * front_scale + back_data.rpy.heading * back_scale;
    synced_data.rpy.pitch = front_data.rpy.pitch * front_scale + back_data.rpy.pitch * back_scale;
    tmp_imu_data = synced_data;
    tmp_imu_data.rpy.heading = synced_data.rpy.heading - pre_imu_data.rpy.heading;
    tmp_imu_data.rpy.pitch = synced_data.rpy.pitch - pre_imu_data.rpy.pitch;
    // tmp_imu_data
    pre_imu_data = synced_data;
    synced_data = tmp_imu_data;
    // std::cout<<synced_data.rpy.heading<<"   "<<synced_data.rpy.pitch<<std::endl;
    // synced_data.rpy.heading = back_data.rpy.heading  - front_data.rpy.heading;
    // synced_data.rpy.pitch = back_data.rpy.pitch  - front_data.rpy.pitch;
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    // synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    // synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    // synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    // synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 线性插值之后要归一化
    // synced_data.orientation.Normlize();

    // SyncedData.push_back(synced_data);
    // return true;
    return true;
}