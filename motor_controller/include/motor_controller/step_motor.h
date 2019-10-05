#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_

#include <ros/ros.h>
#include <chusa_msgs/Step.h>
#include <geometry_msgs/Twist.h>

class Step_Motor{
	public:
		Step_Motor();
		~Step_Motor();
		bool init();
		
	private:
		ros::NodeHandle nh_;
		ros::Publisher publisher_;
		ros::Subscriber subscriber_;
		void msgCallback(const geometry_msgs::Twist msg);

		bool initialized = false;
		double ROBOT_WIDTH = 0.16;
		double MAX_LIN_VEL = 0.22;
		double MAX_ANG_VEL = 2.75;
		double MAX_VEL = 0.44;

		chusa_msgs::Step msg;
};

#endif
