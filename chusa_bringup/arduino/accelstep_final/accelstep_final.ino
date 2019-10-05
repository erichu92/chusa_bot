
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <AccelStepper.h>

AccelStepper stepper_l1 = AccelStepper(1, 3, 2);
AccelStepper stepper_r1 = AccelStepper(1, 5, 4);

ros::NodeHandle nh;
geometry_msgs::TwistStamped wheel_twist;
void msgCb(const geometry_msgs::Twist& twist);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", msgCb);
ros::Publisher pub("/wheel_twist", &wheel_twist);
//geometry_msgs::TwistStamped wheel_twist;
void msgCb(const geometry_msgs::Twist& twist){
  
  float step_vel_r = - 950.12 * (twist.linear.x - 0.16 * 0.5 * twist.angular.z);
  float step_vel_l = 950.12 * (twist.linear.x + 0.16 * 0.5 * twist.angular.z);
  long step_dist_r = step_vel_r * 1;
  long step_dist_l = step_vel_l * 1;
  
  stepper_l1.setCurrentPosition(0);
  //stepper_l2.setCurrentPosition(0);
  stepper_r1.setCurrentPosition(0);
  //stepper_r2.setCurrentPosition(0);
  
  if(fabs(step_dist_r) > fabs(step_dist_l)){
    while(stepper_r1.currentPosition() != step_dist_r){
      if(stepper_l1.currentPosition() != step_dist_l){
        stepper_l1.setSpeed(step_vel_l);
        //stepper_l2.setSpeed(step_vel_l);
        stepper_r1.setSpeed(step_vel_r);
        //stepper_r2.setSpeed(step_vel_r);
        stepper_l1.runSpeed();
        stepper_r1.runSpeed();
        //stepper_l2.runSpeed();
        //stepper_r2.runSpeed();
      }
      else{
        stepper_r1.setSpeed(step_vel_r);
        //stepper_r2.setSpeed(step_vel_r);
        stepper_r1.runSpeed();
        //stepper_r2.runSpeed();
      }
    }
  }
  else{
    while(stepper_l1.currentPosition() != step_dist_l){
      if(stepper_r1.currentPosition() != step_dist_r){
        stepper_l1.setSpeed(step_vel_l);
        //stepper_l2.setSpeed(step_vel_l);
        stepper_r1.setSpeed(step_vel_r);
        //stepper_r2.setSpeed(step_vel_r);
        stepper_l1.runSpeed();
        stepper_r1.runSpeed();
        //stepper_l2.runSpeed();
        //stepper_r2.runSpeed();
      }
      else{
        stepper_l1.setSpeed(step_vel_l);
        //stepper_l2.setSpeed(step_vel_l);
        stepper_l1.runSpeed();
        //stepper_l2.runSpeed();
      }
    }
  }
  
  wheel_twist.header.stamp = nh.now();
  wheel_twist.twist.linear.x = -(step_dist_r - step_dist_l) / (2 * 1 * 950.12);
  wheel_twist.twist.angular.z = (step_dist_r + step_dist_l) / (0.16 * 1 * 950.12);
  pub.publish(&wheel_twist);
  
}

void setup() {
  //Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  //nh.spinOnce();
  stepper_l1.setMaxSpeed(1000);
  //stepper_l2.setMaxSpeed(1000);
  stepper_r1.setMaxSpeed(1000);
  //stepper_r2.setMaxSpeed(1000);
}
void loop() {
  nh.spinOnce();
  delay(100);
}
