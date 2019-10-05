#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv){
    //ros::Time begin = ros::Time::now();
    ros::init(argc, argv, "assemble_pointcloud2_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("cloud_in",10);
    ros::service::waitForService("assemble_scans2");
    ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    ros::Rate loop_rate(1);

    while(ros::ok()){
        laser_assembler::AssembleScans2 srv;
        ros::Duration d(1.0);
        srv.request.begin = ros::Time::now() - d;
        
        srv.request.end = ros::Time::now();

        if(client.call(srv)){
            ROS_INFO("success service");
            pub.publish(srv.response.cloud);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}