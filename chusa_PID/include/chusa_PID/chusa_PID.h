#ifndef CHUSA_PID_H_
#define CHUSA_PID_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <chusa_msgs/Pwm.h>

class Chusa_PID{
    public:
        Chusa_PID();
        ~Chusa_PID();
        bool init();

    private:
        ros::NodeHandle nh_;
	ros::Publisher publisher_;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_odom;
        void msgCallback_vel(const geometry_msgs::Twist twist_in);
        void msgCallback_odom(const nav_msgs::Odometry odom_in);
        int min(int a, int b);
        int max(int a, int b);

        bool vel_flag = false;
        bool odom_flag = false;
	bool time_flag = false;
	bool stop = false;
        int init_flag = 0;
        bool start = false;

        ros::Time prev_time, curr_time;

        double desired_vl, desired_vr, current_vl, current_vr;
        double k_p = 5;
        double k_i = 1;
        double k_d = 1;

        int min_pwm = -255;
        int max_pwm = 255;
        double min_vel = 0.0;
        double max_vel = 0.3;

        double robot_radius = 0.0625;

        double error_r, error_l;
	double prev_error_l = 0;
        double prev_error_r = 0;

        int output_pwm_l = 0;
        int output_pwm_r = 0;

        double I_l = 0.0;
        double I_r = 0.0;

        void PID(bool stop);
        int vel2PWM(double vel);
};

#endif

