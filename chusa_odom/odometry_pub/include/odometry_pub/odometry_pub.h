#ifndef ODOMETRY_PUB_H_
#define ODOMETRY_PUB_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace message_filters;

class Chusa_Odom{
	public:
		Chusa_Odom();
		~Chusa_Odom();
		bool init();
	private:
		ros::NodeHandle nh_;
		ros::Publisher publisher_;
		message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub;
        message_filters::Subscriber<geometry_msgs::TwistStamped> wheel_odom_sub;
        TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::TwistStamped> sync;
		void msgCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& pose, const geometry_msgs::TwistStampedConstPtr& wheel_odom);

		bool initialized = false;

		nav_msgs::Odometry odom;
};

#endif
