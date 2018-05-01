import math
import numpy as np

def Integrator(State0,Motor_Left_Speed,Motor_Right_Speed,dt,W):

	#% Decode state0
	x0 =State0[0]
	y0 =State0[1]
	th0=State0[2]
	vx0=State0[3]
	vy0=State0[4]
	w0 =State0[5]

	#% Integrate th0
	w0=(Motor_Right_Speed-Motor_Left_Speed)/W
	th0=th0+w0*dt

	#% Ensure th0 stays between [0,2*pi]
	if(th0>2*math.pi):
		th0=th0-2*math.pi
	if (th0<0):
		th0=th0+2*math.pi

	#% Integrate speed
	v0=(Motor_Left_Speed+Motor_Right_Speed)/2
	vx0=v0*math.cos(th0)
	vy0=v0*math.sin(th0)

	#% Integrate position
	x0=x0+vx0*dt
	y0=y0+vy0*dt

	#% Update state0
	State0[0]=x0
	State0[1]=y0
	State0[2]=th0
	State0[3]=vx0
	State0[4]=vy0
	State0[5]=w0
	
	return State0
