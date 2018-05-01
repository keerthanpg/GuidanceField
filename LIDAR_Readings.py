#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

distance=np.array([1000,1000])
bearing=np.array([0,0])

def LIDAR_Input(ROS_Message):
	global distance, bearing
	# Measure Ranges: Check that we remove the NAN
	distance=np.transpose(ROS_Message.ranges)
	# Manufacter angles (can be moved out to run only once)
	Angle_Min=ROS_Message.angle_min
	Angle_Max=ROS_Message.angle_max
	Angle_Increment1=(Angle_Max-Angle_Min)/726
	bearing=float(np.linspace(-726/2,726/2,num=1, endpoint=True)*Angle_Increment1)

def LIDAR_Readings():
	global distance, bearing
	return [distance,bearing]  


if __name__=='__main__':
	rospy.init_node('lidar_readings')
	rospy.Subscriber('/scan', LaserScan, LIDAR_Input)
	rospy.spin()
