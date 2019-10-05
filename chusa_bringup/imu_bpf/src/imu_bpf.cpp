#include <ros/ros.h>
#include <imu_bpf/imu_bpf.h>

Imu_Bpf::Imu_Bpf(){
    bool init_result = init();
    ROS_ASSERT(init_result);
}

Imu_Bpf::~Imu_Bpf(){

}

bool Imu_Bpf::init(){
    pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_bpf", 10);
    sub_ = nh_.subscribe<sensor_msgs::Imu>("imu/data_raw", 10, &Imu_Bpf::msgCallback, this);
    return true;
}

void Imu_Bpf::msgCallback(const sensor_msgs::Imu msg){
    if(flag == false){
        hpf_x = lpf_x = raw_prev_x = msg.linear_acceleration.x;
        hpf_y = lpf_y = raw_prev_y = msg.linear_acceleration.y;
        hpf_z = lpf_z = raw_prev_z = msg.linear_acceleration.z;
        hpf_angular = lpf_angular = raw_prev_angular = msg.angular_velocity.z;
        flag = true;
    }
    raw_x = msg.linear_acceleration.x;
    raw_y = msg.linear_acceleration.y;
    raw_z = msg.linear_acceleration.z;
    raw_angular = msg.angular_velocity.z;

    /////////////// X factor /////////////////////
    if(-0.2 < raw_x < 0.2){
        low_factor_x = 0.03;
        high_factor_x = 0.03;
    }
    else if(-0.5 < raw_x <= -0.2 || 0.2 <= raw_x < 0.5){
        low_factor_x = 0.2;
        high_factor_x = 0.2;
    }
    else{
        low_factor_x = 0.9;
        high_factor_x = 0.9;
    }
    ///////////////////////////////////////////////

    ////////////// Y factor ///////////////////////

    if(-0.2 < raw_y < 0.2){
        low_factor_y = 0.01;
        high_factor_y = 0.01;
    }
    else if(-0.5 < raw_y <= -0.2 || 0.2 <= raw_y < 0.5){
        low_factor_y = 0.1;
        high_factor_y = 0.1;
    }
    else{
        low_factor_y = 0.3;
        high_factor_y = 0.3;
    }

    ///////////////////////////////////////////////

    ////////////// Angular Z factor ///////////////

    high_factor_angular = 0.9;

    ///////////////////////////////////////////////

    hpf_x = high_factor_x * (lpf_x + raw_x - raw_prev_x);
    hpf_y = high_factor_y * (lpf_y + raw_y - raw_prev_y);
    hpf_z = high_factor_z * (lpf_z + raw_z - raw_prev_z);
    hpf_angular = high_factor_angular * (hpf_angular + raw_angular - raw_prev_angular);

    lpf_x = lpf_x + (low_factor_x * (hpf_x - lpf_x));
    lpf_y = lpf_y + (low_factor_y * (hpf_y - lpf_y));
    lpf_z = lpf_z + (low_factor_z * (hpf_z - lpf_z));
    //lpf_angular = lpf_angular + (low_factor_angular * (hpf_angular - lpf_angular));

    sensor_msgs::Imu imu_out;

    imu_out.header.stamp = ros::Time::now();

    imu_out.linear_acceleration.x = lpf_x;
    imu_out.linear_acceleration.y = lpf_y;
    imu_out.linear_acceleration.z = lpf_z;
    imu_out.angular_velocity.z = hpf_angular;

    pub_.publish(imu_out);

    raw_prev_x = raw_x;
    raw_prev_z = raw_z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_bpf");
    Imu_Bpf imu_bpf_node;
    
    ros::spin();

    return 0;
}