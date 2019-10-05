This is code for Autonomous SLAM and 3D Mapping Robot Using Raspberry Pi, Arduino*2, 2D Lidar Sensor, A4988 Motor Drivers, Stepper Motors etc..

There is two main parts of these codes.

One is for raspberry pi code and another is for pc code.

First, this is our materials.

--------------------------------------------------------------------------

Materials

- Raspberry Pi 3 B+ ( 1EA )

- Arduino Uno ( 2EA )
	-> For Driving Motor Control
	-> For Tilting 2D Lidar Sensor Motor Control to Mapping 3D Map

- A4988 Motor Driver ( 3EA )
	-> Two for Driving Motor Driver
	-> One for Tilting 2D Lidar Sensor Motor

- Stepper Motor ( 5EA )
	-> Four for Driving
	-> One for Tilting 2D Lidar Sensor

- 2D RP Lidar Sensor ( 1EA )

---------------------------------------------------------------------------

Codes


Raspberry Pi

/chusa_bringup

/chusa_bringup/robot_base

-> This is our robot's urdf file. We operate robot in real space so we design it by measurement.

/chusa_bringup/imu_mpu9250_pub

This code is for imu sensor. We get the linear velocity and angular acceleration of the robot.
Be Careful!!! Unfortunately we put the imu sensor in wrong direction. So x, y coordinate is changed. :).
And from the imu sensor data, we publish odometry information.

/chusa_bringup/imu_tools
/chusa_bringup/imu_bpf

Also, there are some filter codes for getting accurate odometry. Complementary filter and band pass filter code. We write some codes, but we didn't use it. Because odometry measured by only imu and filter cannot be in error tolerance. So we use wheel odometry together instead of using only imu sensor odometry.

/chusa_bringup/rplidar_ros

This is node for RP Lidar Sensor

/chusa_bringup/chusa_teleop

This is node for controller our robot. Using this node, we can control the robot by keyboard input.

/chusa_bringup/rosserial-0.7.7

This is node for arduino rosserial connection.

/chusa_bringup/chusa_bot

Launch File for raspberry pi.


/motor_controller

There is motor controller node.

This code for stepper motor which is using for driving. This subscribe driving command and transform to left_tick and right_tick for each stepper motor.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% arduino code will be upload as soon as possible :) %%%%%%

------------------------------

PC

In the Laptop, we do a lot of computation.

/slam_gmapping

We use gmapping for making 2d map from lidar sensor data.

/chusa_odom

This node subscribe wheel_odom and laser scan matcher node and publish 
