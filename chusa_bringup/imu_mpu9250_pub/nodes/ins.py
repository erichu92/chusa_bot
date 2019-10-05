#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_mpu9250_pub')
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs import Odometry

import math
import time

def publisher():
    pub = rospy.Publisher('odom', Odometry)
    rospy.init_node('ins')
    imu = Imu()
    magnetic = MagneticField()

    mpu9250 = MPU9250(); # change

    while not rospy.is_shutdown():

	accel = mpu9250.readAccel()
	gyro = mpu9250.readGyro()
	mag = mpu9250.readMagnet()

        #roll = (float)(math.atan2(gyro['y'], gyro['z']))
        
        #imu.orientation.x = roll

        #pitch = (float)(0.0)

        #if(gyro['y']*math.sin(roll) + gyro['z'] * math.cos(roll) == 0):
        #        if( gyro['x'] > 0):
        #                pitch = (float)(PI /2)
        #        else:
        #                pitch = - (float)(PI / 2)
        #else:
        #        pitch = (float)(math.atan2(gyro['x'] , (gyro['y'] * math.sin(roll) + gyro['z'] * math.cos(roll))))

        #imu.orientation.y = pitch

        #yaw = (float)(math.atan2(mag['z'] - mag['y'] * math.cos(roll), mag['x'] * math.cos(pitch) + mag['y'] * math.sin(pitch) * math.sin(roll) + mag['z'] * #math.sin(pitch) * math.cos(roll)))
        #imu.orientation.z = yaw

        #imu.orientation.w = 1

        imu.angular_velocity.x = gyro['x'] * 0.07 * DPS_TO_RADS
        imu.angular_velocity.y = gyro['y'] * 0.07 * DPS_TO_RADS
        imu.angular_velocity.z = gyro['z'] * 0.07 * DPS_TO_RADS

        imu.linear_acceleration.x = accel['x']
        imu.linear_acceleration.y = accel['y']
        imu.linear_acceleration.z = accel['z']

        magnetic.magnetic_field.x = mag['x']
        magnetic.magnetic_field.y = mag['y']
        magnetic.magnetic_field.z = mag['z']

        pub_imu.publish(imu)
        pub_mag.publish(magnetic)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass
