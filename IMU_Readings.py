import numpy as np
import math

def IMU_Readings(ROS_Message):

	# Read Bag
	Time=double(ROS_Message.Header.Stamp.Sec)+double(ROS_Message.Header.Stamp.Nsec/math.pow(10,9))
	q_x=ROS_Message.Orientation.X
	q_y=ROS_Message.Orientation.Y
	q_z=ROS_Message.Orientation.Z
	q_w=mROS_Message.Orientation.W

	# roll (x-axis rotation)
	sinr = +2.0 * (q_w * q_x + q_y * q_z)
	cosr = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
	roll = myatan(sinr, cosr)

	# pitch (y-axis rotation)
	sinp = +2.0 * (q_w * q_y - q_z * q_x)
	if (abs(sinp) >= 1):
			pitch = sign(M_PI / 2, sinp) # use 90 degrees if out of range
	else:
			pitch = asin(sinp)

	# yaw (z-axis rotation)
	siny = +2.0 * (q_w * q_z + q_x * q_y);
	cosy = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
	yaw = myatan(siny, cosy);

	# Final update for output
	dt=.1
	th0=yaw
	w0=(th0-old_th0)/dt

	return [t0,th0,w0]  
