#include <perception_ros/imu_subscriber.hpp>
IMUSubscriber::IMUSubscriber(ros::NodeHandle nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

void IMUSubscriber::msg_callback(const autodrive_msgs::HUAT_InsP2ConstPtr imu_msg_ptr)
{
    IMUData imu_data;
    mtx.lock();
    
    // 时间戳
    imu_data.time = imu_msg_ptr->header.stamp.toSec();
    
    // === 速度 (NED坐标系, m/s) ===
    // HUAT_InsP2 中速度为 NED 坐标系，Vd 向下为正
    imu_data.velocity.vn = imu_msg_ptr->Vn;
    imu_data.velocity.ve = imu_msg_ptr->Ve;
    imu_data.velocity.vd = imu_msg_ptr->Vd;  // Vd(向下)
    
    // === 加速度 (FRD车体系, m/s²) ===
    // HUAT_InsP2 中已经是 m/s²，直接使用
    imu_data.acceleration.ax = imu_msg_ptr->acc_x;
    imu_data.acceleration.ay = imu_msg_ptr->acc_y;
    imu_data.acceleration.az = imu_msg_ptr->acc_z;
    
    // === 角速度 (FRD车体系, rad/s) ===
    // HUAT_InsP2 中已经是 rad/s，直接使用
    imu_data.angular_velocity.wx = imu_msg_ptr->gyro_x;
    imu_data.angular_velocity.wy = imu_msg_ptr->gyro_y;
    imu_data.angular_velocity.wz = imu_msg_ptr->gyro_z;
    
    // === 姿态角 (度) ===
    imu_data.orientation.roll = imu_msg_ptr->Roll;
    imu_data.orientation.pitch = imu_msg_ptr->Pitch;
    imu_data.orientation.heading = imu_msg_ptr->Heading;
    
    // === INS 状态信息 ===
    imu_data.status.overall = imu_msg_ptr->Status;
    imu_data.status.num_sv = std::max(imu_msg_ptr->NSV1, imu_msg_ptr->NSV2);  // 取较大值
    imu_data.status.diff_age = imu_msg_ptr->Age * 0.1;  // 0.1s 单位转秒
    
    // 根据 INS_Status 判断数据有效性
    // Status=2 表示组合导航正常，所有数据有效
    imu_data.status.pos_valid = (imu_msg_ptr->Status == 2);
    imu_data.status.vel_valid = (imu_msg_ptr->Status >= 1);
    imu_data.status.att_valid = (imu_msg_ptr->Status >= 1);
    imu_data.status.heading_valid = (imu_msg_ptr->Status == 2);  // 仅 Status=2 时 Heading 为北参考
    
    new_imu_data_.push_back(imu_data);
    mtx.unlock();
}

void IMUSubscriber::ParseData(std::deque<IMUData> &imu_data_buff)
{
    mtx.lock();
    if (new_imu_data_.size() > 0)
    {
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        new_imu_data_.clear();
    }
    mtx.unlock();
}

bool IMUSubscriber::SyncData(std::deque<IMUData> &UnsyncedData, IMUData &synced_data, double sync_time)
{
    // 至少需要 1 条数据
    if (UnsyncedData.empty())
        return false;
    
    // 清理过期数据（比 sync_time 早超过 1 秒的数据）
    while (UnsyncedData.size() > 1 && sync_time - UnsyncedData.front().time > 1.0)
    {
        UnsyncedData.pop_front();
    }
    
    // Case 1: 点云时间早于或等于最早的 IMU 数据 -> 使用最早的 IMU 数据
    if (UnsyncedData.front().time >= sync_time)
    {
        // 如果时间差太大（>0.5秒），拒绝同步
        if (UnsyncedData.front().time - sync_time > 0.5)
            return false;
        
        synced_data = UnsyncedData.front();
        synced_data.time = sync_time;
        return true;
    }
    
    // Case 2: 只有一条数据且点云时间晚于它
    if (UnsyncedData.size() == 1)
    {
        // 如果时间差太大（>0.5秒），拒绝同步
        if (sync_time - UnsyncedData.front().time > 0.5)
            return false;
        
        synced_data = UnsyncedData.front();
        synced_data.time = sync_time;
        return true;
    }
    
    // Case 3: 有多条数据，找到包围 sync_time 的两条数据进行插值
    // 找到第一个时间 > sync_time 的数据
    size_t idx = 0;
    for (idx = 1; idx < UnsyncedData.size(); ++idx)
    {
        if (UnsyncedData.at(idx).time >= sync_time)
            break;
    }
    
    // 如果没找到（点云时间晚于所有 IMU 数据），使用最新的 IMU 数据
    if (idx >= UnsyncedData.size())
    {
        if (sync_time - UnsyncedData.back().time > 0.5)
            return false;
        
        synced_data = UnsyncedData.back();
        synced_data.time = sync_time;
        return true;
    }
    
    // Case 4: 正常情况，进行线性插值
    IMUData front_data = UnsyncedData.at(idx - 1);
    IMUData back_data = UnsyncedData.at(idx);
    
    double dt = back_data.time - front_data.time;
    if (dt < 1e-6) dt = 1e-6; // 防止除以零
    
    double front_scale = (back_data.time - sync_time) / dt;
    double back_scale = (sync_time - front_data.time) / dt;
    
    synced_data.time = sync_time;
    
    // 速度线性插值
    synced_data.velocity.vn = front_data.velocity.vn * front_scale + back_data.velocity.vn * back_scale;
    synced_data.velocity.ve = front_data.velocity.ve * front_scale + back_data.velocity.ve * back_scale;
    synced_data.velocity.vd = front_data.velocity.vd * front_scale + back_data.velocity.vd * back_scale;
    
    // 加速度线性插值
    synced_data.acceleration.ax = front_data.acceleration.ax * front_scale + back_data.acceleration.ax * back_scale;
    synced_data.acceleration.ay = front_data.acceleration.ay * front_scale + back_data.acceleration.ay * back_scale;
    synced_data.acceleration.az = front_data.acceleration.az * front_scale + back_data.acceleration.az * back_scale;
    
    // 角速度线性插值
    synced_data.angular_velocity.wx = front_data.angular_velocity.wx * front_scale + back_data.angular_velocity.wx * back_scale;
    synced_data.angular_velocity.wy = front_data.angular_velocity.wy * front_scale + back_data.angular_velocity.wy * back_scale;
    synced_data.angular_velocity.wz = front_data.angular_velocity.wz * front_scale + back_data.angular_velocity.wz * back_scale;
    
    // 姿态角线性插值
    synced_data.orientation.roll = front_data.orientation.roll * front_scale + back_data.orientation.roll * back_scale;
    synced_data.orientation.pitch = front_data.orientation.pitch * front_scale + back_data.orientation.pitch * back_scale;
    synced_data.orientation.heading = front_data.orientation.heading * front_scale + back_data.orientation.heading * back_scale;
    
    return true;
}
