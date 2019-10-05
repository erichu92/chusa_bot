#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>


using namespace message_filters;

ros::Publisher pub_;

void callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const geometry_msgs::TwistStampedConstPtr& wheel_odom)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = wheel_odom->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    for(int i=0; i<36; i++){
        odom.pose.covariance[i] = pose->pose.covariance[i];
        odom.twist.covariance[i] = 0;
    }

    odom.pose.pose.position.x = pose->pose.pose.position.x;
    odom.pose.pose.position.y = pose->pose.pose.position.y;
    odom.pose.pose.position.z = pose->pose.pose.position.z;

    odom.pose.pose.orientation.x = pose->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = pose->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = pose->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = pose->pose.pose.orientation.w;

    odom.twist.twist.linear.x = wheel_odom->twist.linear.x;
    odom.twist.twist.linear.y = wheel_odom->twist.linear.y;
    odom.twist.twist.linear.z = wheel_odom->twist.linear.z;

    odom.twist.twist.angular.x = wheel_odom->twist.angular.x;
    odom.twist.twist.angular.y = wheel_odom->twist.angular.y;
    odom.twist.twist.angular.z = wheel_odom->twist.angular.z;

    pub_.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_pub_node");
  ros::NodeHandle nh;
  pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh,"poseWithCovarianceStamped",1);
  message_filters::Subscriber<geometry_msgs::TwistStamped> wheel_odom_sub(nh,"wheel_odom",1);
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::TwistStamped> ChusaOdometry;
  Synchronizer<ChusaOdometry> sync(ChusaOdometry(10),pose_sub,wheel_odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}


/*
ros::Publisher pub_;
geometry_msgs::Twist twist;

void twistCallback(const geometry_msgs::Twist& msg){
    twist.linear.x = msg.linear.x;
    twist.angular.z = msg.angular.z;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = pose->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    for(int i=0; i<36; i++){
        odom.pose.covariance[i] = pose->pose.covariance[i];
        odom.twist.covariance[i] = 0;
    }

    odom.pose.pose.position.x = pose->pose.pose.position.x;
    odom.pose.pose.position.y = pose->pose.pose.position.y;
    odom.pose.pose.position.z = pose->pose.pose.position.z;

    odom.pose.pose.orientation.x = pose->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = pose->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = pose->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = pose->pose.pose.orientation.w;

    odom.twist.twist.linear.x = twist.linear.x;
    odom.twist.twist.linear.y = twist.linear.y;
    odom.twist.twist.linear.z = twist.linear.z;

    odom.twist.twist.angular.x = twist.angular.x;
    odom.twist.twist.angular.y = twist.angular.y;
    odom.twist.twist.angular.z = twist.angular.z;

    pub_.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_pub_node");
  ros::NodeHandle nh;
  pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1, twistCallback);
  ros::Subscriber sub_pose = nh.subscribe("/poseWithCovarianceStamped", 1, poseCallback);

  ros::spin();

  return 0;
}*/