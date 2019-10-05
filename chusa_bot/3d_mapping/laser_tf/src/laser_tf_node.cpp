#include <ros/ros.h>
#include <chusa_msgs/TfFromTilt.h>
#include <chusa_msgs/Tilt.h>
#include <chusa_msgs/TiltStamped.h>
#include <tf/transform_broadcaster.h>

float pitch = 0.0;
ros::Publisher pub_;

bool srvCallback(chusa_msgs::TfFromTilt::Request& req, chusa_msgs::TfFromTilt::Response& res){
    pitch = req.tilt;
    return true;
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "tilt_lidar_node");
    ros::NodeHandle n;
    pub_ = n.advertise<chusa_msgs::TiltStamped>("tilt", 1);
    ros::ServiceServer server = n.advertiseService("tf_tilt", srvCallback);
    ros::Rate loop_rate(10);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    chusa_msgs::TiltStamped msg;

    while(ros::ok()){
        tf::Quaternion q;
        q.setRPY(0,pitch,0);
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( q );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
        msg.header.stamp = ros::Time::now();
        msg.tilt.tilt = pitch;
        pub_.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}