#ifndef IMU_BPF_H_
#define IMU_BPF_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class Imu_Bpf{
public:
    Imu_Bpf();
    ~Imu_Bpf();
    bool init();
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

    void msgCallback(const sensor_msgs::Imu msg);

    double raw_x, raw_y, raw_z, raw_angular;
    double raw_prev_x, raw_prev_y, raw_prev_z, raw_prev_angular;
    double hpf_x, hpf_y, hpf_z, hpf_angular;
    double lpf_x, lpf_y, lpf_z, lpf_angular;

    bool flag = false;
    
    double low_factor_x, low_factor_y, low_factor_z, low_factor_angular;
    double high_factor_x, high_factor_y, high_factor_z, high_factor_angular;

    double factor;
};

#endif