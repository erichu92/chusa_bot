#include <ros/ros.h>
#include <chusa_msgs/Sensing3DOnce.h>
#include <chusa_msgs/TiltStamped.h>
#include <tf/transform_broadcaster.h>

bool _3d_sensing;
bool phase1;
bool phase2;
bool phase3;

bool srvCallback(chusa_msgs::Sensing3DOnce::Request &req, chusa_msgs::Sensing3DOnce::Response &res){
    if(req.start){
        _3d_sensing = true;
        phase1 = true;
        phase2 = false;
        phase3 = false;
        res.success = true;
        return true;
    }
    else{
        res.success = false;
        return false;
    }
    
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "sensing_3d_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<chusa_msgs::TiltStamped>("tilt",5);
    ros::ServiceServer server = n.advertiseService("sensing_3d", srvCallback);
    ros::Rate loop_rate(1);
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.122) );
    tf::Quaternion q;
    q.setRPY(0, 0, 1.57);
    transform.setRotation(q);

    _3d_sensing = false;
    phase1 = false;
    phase2 = false;
    phase3 = false;
    int MAX_TILT_ANGLE = 8;
    chusa_msgs::TiltStamped msg;
    msg.tilt.tick = 0.0;
    msg.header.stamp = ros::Time::now();
    double pitch = 0.0;
    double tick_angle = 3.6;
    int num = 1;
    while(ros::ok()){
        if(!_3d_sensing){
            msg.tilt.tick = 0;
            msg.header.stamp = ros::Time::now();
            q.setRPY(0, pitch, 1.57);
            transform.setRotation(q);
            pub.publish(msg);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
        }
        else{
            
            if(phase1){
                for(int i = 0 ; i < MAX_TILT_ANGLE ; i++){
                    msg.tilt.tick++;
                    pitch = tick_angle * ((double)msg.tilt.tick) / 180;
                    q.setRPY(0, pitch, 1.57);
                    transform.setRotation(q);
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
                    loop_rate.sleep();
                }
                phase1 = false;
                phase2 = true; 
            }
            else if(phase2){
                for(int i = 0 ; i < 2 * MAX_TILT_ANGLE ; i++){
                    msg.tilt.tick--;
                    
                    pitch = tick_angle * ((double)msg.tilt.tick+2) / 180;
                    
                    q.setRPY(0, pitch, 1.57);
                    transform.setRotation(q);
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
                    loop_rate.sleep();
                }
                phase2 = false;
                phase3 = true; 
            }
            else if(phase3){
                for(int i = 0 ; i < MAX_TILT_ANGLE ; i++){
                    msg.tilt.tick++;
                    pitch = tick_angle * ((double)msg.tilt.tick) / 180;
                    q.setRPY(0, pitch, 1.57);
                    transform.setRotation(q);
                    msg.header.stamp = ros::Time::now();
                    pub.publish(msg);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser"));
                    loop_rate.sleep();
                }
                phase3 = false;
                _3d_sensing = false; 
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}