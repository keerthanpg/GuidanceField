#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import math
from myatan import *
from subprocess import check_output,Popen, PIPE
import pdb

x_IMU=0
y_IMU=0
th_IMU=0
vx_IMU=0
vy_IMU=0
w0_IMU=0


def IMU_Readings(ROS_Message):
	# Read Bag
	global th_IMU
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
	th0=2*math.pi-yaw
	w0=(th0-th_IMU)/dt
	w0_IMU = w0
	th_IMU = th0
	np.save('w0_IMU.npy', w0_IMU)
	np.save('th_IMU.npy', th_IMU)
	

def Localization_Readings(ROS_Message):
	global x_IMU, y_IMU, th_IMU,vx_IMU, vy_IMU, w0_IMU
	# Parse the content
	String1 = ROS_Message.data
	# print(String1)
	Read = String1.split(',')
	# Read the content
	if(len(Read)==5):
		x0=float(Read[1])
		y0=float(Read[2])
		z0=float(Read[3])
		x_IMU = x0
		y_IMU = y0
		np.save('y_IMU.npy', y_IMU)
		np.save('x_IMU.npy', x_IMU)

def get_Localization_Readings():
	global x_IMU, y_IMU
	try:
		x_IMU=np.load('x_IMU.npy')
		y_IMU=np.load('y_IMU.npy')
	except:
		a=0
	return [x_IMU,y_IMU]

def get_IMU_Readings():
	global w0_IMU, th_IMU
	try:
		w0_IMU=np.load('w0_IMU.npy')
		th_IMU=np.load('th_IMU.npy')
	except:
		a=0
	return th_IMU, w0_IMU 

if __name__=='__main__':
	rospy.init_node('imu_readings')
	rospy.Subscriber('/imu', Imu, IMU_Readings)
	rospy.Subscriber('/location_coordinates', String, Localization_Readings)
	rospy.spin()
