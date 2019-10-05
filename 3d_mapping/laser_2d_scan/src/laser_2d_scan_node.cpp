#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chusa_msgs/TiltStamped.h>
#include <sensor_msgs/LaserScan.h>


using namespace message_filters;

ros::Publisher pub_;
sensor_msgs::LaserScan prev;

void callback(const chusa_msgs::TiltStampedConstPtr& tilt_in, const sensor_msgs::LaserScanConstPtr& scan_in)
{
    sensor_msgs::LaserScan scan_out;
    if(!(-1 < tilt_in->tilt.tick && tilt_in->tilt.tick < 1)){
        scan_out.header.stamp = scan_in->header.stamp;
        scan_out.header.frame_id = "base_link";
        scan_out.angle_min = scan_in->angle_min;
        scan_out.angle_max = scan_in->angle_max;
        scan_out.angle_increment = 0;
        scan_out.time_increment = 0;
        scan_out.scan_time = scan_in->scan_time;
        scan_out.range_min = scan_in->range_min;
        scan_out.range_max = scan_in->range_max;
        scan_out.ranges = prev.ranges;
        scan_out.ranges = prev.intensities;
    }
    else{
        scan_out.header.frame_id = "base_link";
        prev.header.stamp = scan_out.header.stamp = scan_in->header.stamp;
        prev.angle_min = scan_out.angle_min = scan_in->angle_min;
        prev.angle_max = scan_out.angle_max = scan_in->angle_max;
        prev.angle_increment = scan_out.angle_increment = scan_in->angle_increment;
        prev.time_increment = scan_out.time_increment = scan_in->time_increment;
        prev.scan_time = scan_out.scan_time = scan_in->scan_time;
        prev.range_min = scan_out.range_min = scan_in->range_min;
        prev.range_max = scan_out.range_max = scan_in->range_max;
        prev.ranges = scan_out.ranges = scan_in->ranges;
        prev.intensities = scan_out.intensities = scan_in->intensities;
    }

    pub_.publish(scan_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_2d_scan_node");
  ros::NodeHandle nh;
  pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan_2d", 10);
  message_filters::Subscriber<chusa_msgs::TiltStamped> tilt_sub(nh,"tilt",10);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh,"scan",10);
  typedef sync_policies::ApproximateTime<chusa_msgs::TiltStamped, sensor_msgs::LaserScan> PlatScan;
  Synchronizer<PlatScan> sync(PlatScan(10),tilt_sub,scan_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}

/*
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <chusa_msgs/Tilt.h>
#include <sensor_msgs/LaserScan.h>


using namespace message_filters;

ros::Publisher pub_;
sensor_msgs::LaserScan prev;

void callback(const chusa_msgs::TiltConstPtr& tilt_in, const sensor_msgs::LaserScanConstPtr& scan_in)
{
    sensor_msgs::LaserScan scan_out;
    if(!(-1 < tilt_in->tick && tilt_in->tick < 1)){
        scan_out.header.stamp = ros::Time::now();
        scan_out.angle_min = prev.angle_min;
        scan_out.angle_max = prev.angle_max;
        scan_out.angle_increment = prev.angle_increment;
        scan_out.time_increment = prev.time_increment;
        scan_out.scan_time = prev.scan_time;
        scan_out.range_min = prev.range_min;
        scan_out.range_max = prev.range_max;
        scan_out.ranges = prev.ranges;
        scan_out.ranges = prev.intensities;
    }
    else{
        prev.header.stamp = scan_out.header.stamp = scan_in->header.stamp;
        prev.angle_min = scan_out.angle_min = scan_in->angle_min;
        prev.angle_max = scan_out.angle_max = scan_in->angle_max;
        prev.angle_increment = scan_out.angle_increment = scan_in->angle_increment;
        prev.time_increment = scan_out.time_increment = scan_in->time_increment;
        prev.scan_time = scan_out.scan_time = scan_in->scan_time;
        prev.range_min = scan_out.range_min = scan_in->range_min;
        prev.range_max = scan_out.range_max = scan_in->range_max;
        prev.ranges = scan_out.ranges = scan_in->ranges;
        prev.intensities = scan_out.intensities = scan_in->intensities;
    }

    pub_.publish(scan_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_2d_scan_node");
  ros::NodeHandle nh;
  pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan_2d", 10);
  message_filters::Subscriber<chusa_msgs::Tilt> tilt_sub(nh,"tilt",1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh,"scan",1);
  typedef sync_policies::ApproximateTime<chusa_msgs::Tilt, sensor_msgs::LaserScan> PlatScan;
  Synchronizer<PlatScan> sync(PlatScan(10),tilt_sub,scan_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
*/