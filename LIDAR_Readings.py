#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import pdb
from multiprocessing import Pool, Value

distance = np.zeros(727)
bearing = np.zeros(727)

def LIDAR_Input(ROS_Message):
	global distance
	global bearing
	# Measure Ranges: Check that we remove the NAN
	#with distance.get_lock():
	distance=np.transpose(ROS_Message.ranges)
	# Manufacter angles (can be moved out to run only once)
	Angle_Min=ROS_Message.angle_min
	Angle_Max=ROS_Message.angle_max
	Angle_Increment1=(Angle_Max-Angle_Min)/726.0
	#with bearing.get_lock():
	bearing=np.linspace(-726.0/2,726.0/2,num=726, endpoint=True)*Angle_Increment1
	np.save('distance.npy', distance)
	np.save('bearing.npy', bearing)
	
	
def LIDAR_Readings():
	global distance, bearing
	try:
		distance = np.load('distance.npy')
	except:
		a=0
	try:
		bearing = np.load('bearing.npy')
	except:
		a=0
	return [distance,bearing]

if __name__=='__main__':
	rospy.init_node('lidar_readings')
	rospy.Subscriber('/scan', LaserScan, LIDAR_Input)
	rospy.spin()
