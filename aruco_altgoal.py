#!/usr/bin/env python
#this code calculates alternate goal for the arm to reach to correct the orientation properly
#also 2 stage
#the output of this code mimics the joystick control that the jog_arm package utilizes (publishes to /joy topic)

import rospy
import time
import tf
import roslib
from math import cos, sin, radians, degrees
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# 4 gains, (x=forward/backward), (y=left/right), (z=up/down)
kx = 0.1
kx_back = 0.25
ky = 0.2
kz = 0.2

#gains for orientation
px = 0.001 
py = 0 #ignore roll
pz = 0.001

#define pi
pi = 3.14159

# distance in front of the tag that the ee should arrive at (unit = m)
desired_x = 0.3

arrived = 0
y_error1 = 0
z_error1 = 0

def aruco_jog():

    global arrived
    global y_error1
    global z_error1

    #name the node
    rospy.init_node('aruco_jog', anonymous=True)
    
    #listens to the different transform frames
    listener = tf.TransformListener()
    time.sleep(2)

    #publish a joy topic for the "joy_to_twist.py" file to listen to
    pub = rospy.Publisher('joy', Joy, queue_size=1)  
    pub2 = rospy.Publisher('y_error1', Float32, queue_size=10)
    pub3 = rospy.Publisher('z_error1', Float32, queue_size=10)  
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():

        j = Joy()

        #find position/rot of the aruco tag as well as the ee
        (position, rot) = listener.lookupTransform('/base_link', '/marker_701', rospy.Time(0))
        (position2, rot2) = listener.lookupTransform('/base_link', '/ur_arm_ee_link', rospy.Time(0))

        aruco_pos_x = position[0]
        aruco_pos_y = position[1]
        aruco_pos_z = position[2]
        aruco_ori_x = rot[0]
        aruco_ori_y = rot[1]
        aruco_ori_z = rot[2]
        aruco_ori_w = rot[3]

        ee_pos_x = position2[0]
        ee_pos_y = position2[1]
        ee_pos_z = position2[2]
        ee_ori_x = rot2[0]
        ee_ori_y = rot2[1]
        ee_ori_z = rot2[2]
        ee_ori_w = rot2[3]

	aruco_orientation_list = [aruco_ori_x, aruco_ori_y, aruco_ori_z, aruco_ori_w]
	(aruco_roll, aruco_pitch, aruco_yaw) = euler_from_quaternion(aruco_orientation_list)
	aruco_roll = aruco_roll * 180.0 / pi
        aruco_pitch = aruco_pitch * 180.0 / pi
        aruco_yaw = aruco_yaw * 180.0 / pi

	ee_orientation_list = [ee_ori_x, ee_ori_y, ee_ori_z, ee_ori_w]
	(ee_roll, ee_pitch, ee_yaw) = euler_from_quaternion(ee_orientation_list)
	ee_roll = ee_roll * 180.0 / pi
        ee_pitch = ee_pitch * 180.0 / pi
        ee_yaw = ee_yaw * 180.0 / pi

	if aruco_yaw > 0:
	    aruco_yaw = aruco_yaw - 180
	else:
	    aruco_yaw = aruco_yaw + 180
	
	#calculates alternate goal
	x_adj = desired_x*cos(radians(aruco_yaw))
	y_adj = desired_x*sin(radians(aruco_yaw))
	z_adj = desired_x*sin(radians(aruco_pitch))
	aruco_pos_x = aruco_pos_x - x_adj
	aruco_pos_y = aruco_pos_y - y_adj
	aruco_pos_z = aruco_pos_z - z_adj

        #find position error
        x_error = aruco_pos_x - ee_pos_x
        y_error = aruco_pos_y - ee_pos_y
        z_error = aruco_pos_z - ee_pos_z

	#find orientation error
	roll_error = aruco_roll - ee_roll
	pitch_error = aruco_pitch - ee_pitch
        yaw_error = aruco_yaw - ee_yaw

	#publish error for debuggin
	y_error1 = roll_error
	z_error1 = yaw_error

	#error threshold
	if abs(x_error) < 0.05 and abs(y_error) < 0.05 and abs(z_error) < 0.05:
            x_error = 0
	    y_error = 0
	    z_error = 0
	if abs(yaw_error) < 15 and abs(pitch_error) < 15:
	    pitch_error = 0
	    yaw_error = 0

        #position control law
        ee_vel_y = ky*y_error
        if x_error > 0:
            ee_vel_x = kx*x_error
        else:
            ee_vel_x = kx_back*x_error
        ee_vel_z = kz*z_error

	#orientation control law
	ee_ang_x = px*pitch_error  
	ee_ang_y = py*roll_error #ignore roll_error
	ee_ang_z = pz*yaw_error

	#repackage the velocity values as joy commands to publish
        j.axes = [-ee_vel_y, ee_vel_z, -ee_vel_x, ee_ang_x, ee_ang_y, -ee_ang_z, 0, 0]    
        j.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0]
        pub.publish(j)

	pub2.publish(y_error1)
	pub3.publish(z_error1)

        rate.sleep()

    ros.spin()

if __name__ == '__main__':
    try:
        aruco_jog()
    except rospy.ROSInterruptException:
        pass


