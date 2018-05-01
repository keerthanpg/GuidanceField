#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import math
from myatan import *

x_IMU=0
y_IMU=0
th_IMU=0
vx_IMU=0
vy_IMU=0
w0_IMU=0


def IMU_Readings(ROS_Message):

	global x_IMU, y_IMU, th_IMU,vx_IMU, vy_IMU, w0_IMU
	# Read Bag
	#Time=float(ROS_Message.Header.Stamp.Sec)+float(ROS_Message.Header.Stamp.Nsec/math.pow(10,9))
	#print(ROS_Message)
	q_x=ROS_Message.orientation.x
	q_y=ROS_Message.orientation.y
	q_z=ROS_Message.orientation.z
	q_w=ROS_Message.orientation.w

	# roll (x-axis rotation)
	sinr = +2.0 * (q_w * q_x + q_y * q_z)
	cosr = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
	roll = myatan(sinr, cosr)

	# pitch (y-axis rotation)
	sinp = +2.0 * (q_w * q_y - q_z * q_x)
	if (abs(sinp) >= 1):
		pitch = sign(M_PI / 2, sinp) # use 90 degrees if out of range
	else:
		pitch = math.asin(sinp)

	# yaw (z-axis rotation)
	siny = +2.0 * (q_w * q_z + q_x * q_y);
	cosy = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
	yaw = myatan(siny, cosy);

	# Final update for output
	dt=.1
	th0=yaw
	w0=(th0-th_IMU)/dt
	w0_IMU = w0
	th_IMU = th0
	print(w0_IMU)

def Localization_Readings(ROS_Message):
	global x_IMU, y_IMU, th_IMU,vx_IMU, vy_IMU, w0_IMU
	# Parse the content
	String1 = ROS_Message.data
	# print(String1)
	Read = String1.split(',')
	# Read the content
	x0=float(Read[1])
	y0=float(Read[2])
	z0=float(Read[3])
	x_IMU = x0
	y_IMU = y0

def get_Localization_Readings():
	return [x_IMU,y_IMU]

def get_IMU_Readings():
	return [th_IMU,w0_IMU] 

if __name__=='__main__':
	rospy.init_node('imu_readings')
	rospy.Subscriber('/imu', Imu, IMU_Readings)
	rospy.Subscriber('/location_coordinates', String, Localization_Readings)
	rospy.spin()