#include <ros/ros.h>
#include <chusa_PID/chusa_PID.h>

Chusa_PID::Chusa_PID(){
    bool init_result = init();
    ROS_ASSERT(init_result);
}

Chusa_PID::~Chusa_PID(){

}

int Chusa_PID::min(int a, int b){
    if(a>b){
        return b;
    }
    else{
        return a;
    }
}

int Chusa_PID::max(int a, int b){
    if(a>b){
        return a;
    }
    else{
        return b;
    }
}

bool Chusa_PID::init(){
    publisher_ = nh_.advertise<chusa_msgs::Pwm>("pwm", 100);
    sub_vel = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Chusa_PID::msgCallback_vel, this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("odom", 1, &Chusa_PID::msgCallback_odom, this);
    if(time_flag == false){
        time_flag = true;
        prev_time = curr_time = ros::Time::now();
    }
    return true;
}

void Chusa_PID::msgCallback_vel(const geometry_msgs::Twist twist_in){
    vel_flag = true;
//  stop = false;
    if(twist_in.linear.x >= 0.01){
        start = true;
    }
    if(twist_in.linear.x == 0.0){
	    stop = true;
    }
//    if(twist_in.angular.z == 0){
//	    stop = true;
//    }
    if(twist_in.angular.z > 0.0){
	    desired_vl = twist_in.linear.x + 2 * robot_radius * twist_in.angular.z;
	    desired_vr = twist_in.linear.x;
    }
    else if(twist_in.angular.z < 0.0){
	    desired_vl = twist_in.linear.x;
	    desired_vr = twist_in.linear.x + 2 * robot_radius * twist_in.angular.z;
    }
    else{
	    desired_vl = twist_in.linear.x;
	    desired_vr = twist_in.linear.x;
    }
    //desired_vl = twist_in.linear.x + twist_in.angular.z * robot_radius;
    //desired_vr = twist_in.linear.x - twist_in.angular.z * robot_radius;

    if(vel_flag && odom_flag){
        PID(stop);
    }
}

void Chusa_PID::msgCallback_odom(const nav_msgs::Odometry odom_in){
    odom_flag = true;
    if(odom_in.twist.twist.angular.z > 0.0){
	    current_vl = odom_in.twist.twist.linear.x + 2 * robot_radius * odom_in.twist.twist.angular.z;
	    current_vr = odom_in.twist.twist.linear.x;
    }
    else if(odom_in.twist.twist.angular.z < 0.0){
	    current_vl = odom_in.twist.twist.linear.x;
	    current_vr = odom_in.twist.twist.linear.x + 2 * robot_radius * odom_in.twist.twist.angular.z;
    }
    else{
        current_vl = odom_in.twist.twist.linear.x;
        current_vr = odom_in.twist.twist.linear.x;
    }
    //current_vl = odom_in.twist.twist.linear.x + odom_in.twist.twist.angular.z * robot_radius;
    //current_vr = odom_in.twist.twist.linear.x - odom_in.twist.twist.angular.z * robot_radius;

    if(vel_flag && odom_flag){
        PID(stop);
    }
}

void Chusa_PID::PID(bool s){
    
    chusa_msgs::Pwm pwm;

    curr_time = ros::Time::now();
    double dt = (curr_time - prev_time).toSec();

    
    error_l = desired_vl - current_vl;
    error_r = desired_vr - current_vr;

    double P_l = k_p * error_l;
    I_l += k_i * error_l * dt;
    double D_l = k_d * (error_l - prev_error_l) / dt;

    double P_r = k_p * error_r;
    I_r += k_i * error_r * dt;
    double D_r = k_d * (error_r - prev_error_r) / dt;

    //ROS_INFO("dt : %f, desired_vl : %f, current_vl : %f, PID left : %f\n", dt, desired_vl, current_vl, P_l + I_l + D_l);
    output_pwm_l = vel2PWM(P_l + I_l + D_l);
    output_pwm_r = vel2PWM(P_r + I_r + D_r);
    
    if(output_pwm_l >= 0){
	    pwm.left_pwm = output_pwm_l >= max_pwm ? max_pwm : output_pwm_l;
    }
    else{
	    pwm.left_pwm = output_pwm_l <= min_pwm ? min_pwm : output_pwm_l;
    }
    if(output_pwm_r >= 0){
	    pwm.right_pwm = output_pwm_r >= max_pwm ? max_pwm : output_pwm_r;
    }
    else{
	    pwm.right_pwm = output_pwm_r <= min_pwm ? min_pwm : output_pwm_r;
    }

    pwm.right_pwm += 0;
    if(init_flag < 3){
        pwm.right_pwm = 0;
        pwm.left_pwm = 0;
    }
    if(start == true){
        init_flag++;
    }
    //pwm.left_pwm = max( min_pwm, min(output_pwm_l, max_pwm));
    //pwm.right_pwm = max( min_pwm, min(output_pwm_r, max_pwm));
    
    if(s==true){
	    pwm.left_pwm = 0;
	    pwm.right_pwm = 0;
    }
    publisher_.publish(pwm);
    
    prev_time = curr_time;
    prev_error_l = error_l;
    prev_error_r = error_r;

    vel_flag = false;
    odom_flag = false;
    stop = false;
}

int Chusa_PID::vel2PWM(double vel){
    return int(( vel - min_vel ) / (max_vel - min_vel) * (max_pwm - min_pwm) );
}

int main(int argc, char** argv){

    ros::init(argc, argv, "chusa_PID");

    Chusa_PID chusa_PID_node;

    ros::spin();

    return 0;
}
