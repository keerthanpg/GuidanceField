#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
from LIDAR_Readings import *
from integrator import *
from motor_input import *
from guidance import *  
from over_camera import *
from imu import *
from myatan import *
import math


#Define initial state
x0=0.
y0=0.
th0=0.
vx0=0.
vy0=0.
w0=0.
t=0.
State0=[x0,y0,th0,vx0,vy0,w0]
State0Mem=State0
mem_dir=-1
Mode=0
Mode_mem=Mode

# Target State
x_target=30.
y_target=30.
Target=[x_target,y_target]

# Define Hard Obstacles (real coordinates)
x_H_Obs=np.array([30.,30.,20.,20.,20.,20.,20.,20.,20.,20.,20.])
y_H_Obs=np.array([25.,20.,30.,25.,20.,15.,10., 5., 0.,11.,13.])
N_H_Obs=x_H_Obs.shape[0]

# Define Soft Obstacles (real coordinates)
x_S_Obs=np.array([2., 4., 6., 8., 20., 12., 35., 20., 45.])
y_S_Obs=np.array([5., 5., 5., 5.,  5., 10., 15., 12.5, 5.])
N_S_Obs=x_S_Obs.shape[0]

Image_W=5000.0
Image_L=3000.0
Scaling=100.0

# Tank 
# L is the lenght of the tank, W the width, R_wh the wheel radius, Belt_length the lenght of the belt in the wheels
# dt is the integration time, Target_Cruise_speed is ratio to the maximum speed: 0 no movement, 1 to target full speed when travelling in straight lines
L=0.2
W=0.1
#R_wh=0.05
Max_size=1/2*max(L,W)
Clearance_distance=2.
Clearance_Target=Clearance_distance+Max_size/2
#Belt_Length=(2*math.pi*R_wh)+2*L
dt=0.01
Target_Cruise_speed=1. 


#% =================================================== Main loop starts here ==========================================================
#Log=[t,0.,0.,State0,Mode,mem_dir]
Delta_Target=0.

while(((State0[0]-x_target)**2+(State0[1]-y_target)**2)**(1/2)>(State0[3]**2+State0[4]**2)**(1/2)*dt*2):
	#%------------------------------------------------------------------------------------------------------------------------
	#% SENSOR INPUT: Get LIDAR readings
	[distance,bearing]=LIDAR_Readings()
	#%------------------------------------------------------------------------------------------------------------------------
	#% SENSOR INPUT: Get Over_Camera readings  
	[x_S_Obs_measure,y_S_Obs_measure]=Over_Camera(x_S_Obs,y_S_Obs,Image_W,Image_L,Scaling)
	#%------------------------------------------------------------------------------------------------------------------------
	#% SENSOR INPUT: IMU readings
	#[x_IMU,y_IMU,th_IMU,vx_IMU,vy_IMU,w0_IMU]=IMU()
	old_x=x_IMU
	old_y=y_IMU
	[x_IMU,y_IMU]=get_Localization_Readings()
	[th_IMU,w_IMU]=get_IMU_Readings()
	vx_IMU=(old_x-x_IMU)/dt
	vy_IMU=(old_y-y_IMU)/dt

	#%------------------------------------------------------------------------------------------------------------------------
	#% SENSOR FUSION: Here is where we decide what goes into the State update

	#% Convert LIDAR input to real world coordinates
	N_H_Obs=distance.shape[0]
	for i in range(N_H_Obs):
		if(not np.isnan(distance[i])):
			x_H_Obs[i]=State0[0]+distance[i]*math.cos(bearing[i])
			y_H_Obs[i]=State0[1]+distance[i]*math.sin(bearing[i])
		elif(np.isnan(distance[i])):
			x_H_Obs[i]=1000
			y_H_Obs[i]=1000

	#% convert soft obstacles to real world coordinates
	N_S_Obs=x_S_Obs_measure.shape[0]
	x_S_Obs=np.multiply(x_S_Obs_measure,(Scaling/Image_L))
	y_S_Obs=np.multiply(y_S_Obs_measure,(Scaling/Image_W))

	#% Convert current position to real world coordinates to real world coordinates (alternative use IMU or fuse both with Kalman filter)
	State0[0]=x_IMU
	State0[1]=y_IMU
	State0[2]=th_IMU
	State0[3]=vx_IMU
	State0[4]=vy_IMU
	State0[5]=w0_IMU

	#% Make changes here in case you want a dynamic target (changing accross time)
	Target[0]=x_target
	Target[1]=y_target
	
	#%------------------------------------------------------------------------------------------------------------------------
	#% GUIDANCE FIELD: In the simulation we compute for the whole space, but in the real thing we only compute at current location

	#% Return only the field at current location
	[Guidance_Field,Mode,final_A,final_B,mem_dir]=Guidance(Image_W,Image_L,Scaling,Target,x_H_Obs,y_H_Obs,x_S_Obs,y_S_Obs,State0,Clearance_Target,t,mem_dir,Mode)
	#print(Guidance_Field)

	#% Atan2 is confined by default inside [-pi,pi] NOT inside [0,2*pi]
	if np.linalg.norm(Guidance_Field)==0:
		th_target=State0[2]
	else:
		#th_target=atan2(Guidance_Field[1],Guidance_Field[0])
		th_target=myatan(Guidance_Field[1],Guidance_Field[0])

	#% Need to contain Delta_Target inside [0,2*pi]
	Delta_Target = th_target-State0[2]
	Delta_Target_old=Delta_Target
	
	if(Delta_Target>=2*math.pi):
		Delta_Target=Delta_Target-2*math.pi

	if (Delta_Target<=0):
		Delta_Target=Delta_Target+2*math.pi

	#% Get the multipliers
	[Motor_Left_Multiplier, Motor_Right_Multiplier] = Motor_input(Delta_Target)
	#pub.publish(std_msgs.msg.String(str(Delta_Target)))
	#% Motors and its inverse will go here
	Motor_Left_Speed=Motor_Left_Multiplier*Target_Cruise_speed
	Motor_Right_Speed=Motor_Right_Multiplier*Target_Cruise_speed

	#%   From here onwards is only relevant for simulation ==================================================================
	
	#% Integrate
	#State0=Integrator(State0,Motor_Left_Speed,Motor_Right_Speed,dt,W)
	#print(State0)
	
	#% Store State History
	#t=t+dt
	#Log=[Log,[t,Motor_Left_Multiplier, Motor_Right_Speed, State0, Mode, mem_dir]]
