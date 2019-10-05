#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <chusa_msgs/TiltStamped.h>
#include <AccelStepper.h>

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define MotorInterfaceType 4

ros::NodeHandle nh;
void msgCb(const chusa_msgs::TiltStamped& msg);
ros::Subscriber<chusa_msgs::TiltStamped> sub("/tilt", msgCb);

AccelStepper stepper = AccelStepper(MotorInterfaceType, 8, 9, 10, 11);
int tick = 0;

void forward(int new_tick){
  //stepper.setCurrentPosition(tick);
  while (stepper.currentPosition() != new_tick)   {
    stepper.setSpeed(1);
    stepper.runSpeed();
    //tick++;
    //stepper.setCurrentPosition(tick);
  }
  tick++;
}

void reverse(int new_tick){
  //stepper.setCurrentPosition(tick);
  while (stepper.currentPosition() != new_tick)   {
    stepper.setSpeed(-1);
    stepper.runSpeed();
    //tick--;
    //stepper.setCurrentPosition(tick);
  }
  tick--;
}

void msgCb(const chusa_msgs::TiltStamped& msg){
  if(tick < msg.tilt.tick){
    forward(msg.tilt.tick);
    //delay(100);
  }
  else if(tick > msg.tilt.tick){
    reverse(msg.tilt.tick);
    //delay(100);
  }
  else{
    
    //delay(100);
  }
}

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  stepper.setMaxSpeed(1000);
  stepper.setCurrentPosition(0);
  forward(1);
  forward(2);
  forward(3);
  
  reverse(0);
}

void loop()
{
  //stepper.setCurrentPosition(tick);
  nh.spinOnce();
  //forward(100);
}
