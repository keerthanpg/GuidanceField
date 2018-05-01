import numpy as np
import math
from Dense_or_sparse import *

def Guidance(Image_W,Image_L,Scaling,Target,x_H_Obs,y_H_Obs,x_S_Obs,y_S_Obs,State0,Clearance_Target,t,mem_dir,Mode):

	# Key field constants
	Dispersion_rate=Clearance_Target**2/math.log(.2)**2
	Hard_Constant=1.
	Soft_Constant=1.

	#% Decode state0
	x0=State0[0]
	y0=State0[1]
	vx0=State0[3]
	vy0=State0[4]

	#% Decode Target
	x_target=Target[0]
	y_target=Target[1]

	#% Target
	Target_Constant=0.001
	Target_Field=-np.multiply(Target_Constant,[x0 - x_target, y0 - y_target])/math.sqrt((x0 - x_target)**2 + (y0 - y_target)**2)

	# Guidance Initialization
	Hard_Fieldx=0.0
	Hard_Fieldy=0.0
	Soft_Fieldx=0.0
	Soft_Fieldy=0.0
			 
	#=============================================== HardObstacles
	columns=y_H_Obs.shape[0]

	for i in range(columns):
		# Precomputation
		dx0=x_H_Obs[i]-x0
		dy0=y_H_Obs[i]-y0

		# What to do if you are not moving
		if(vx0==0 and vy0==0):
				check=1
		else:
			if Mode==0:
				check=max(0,np.sign(vx0*dx0+vy0*dy0))
			else:
				check=1
				
		# Selecting the direction to rotate
		if Mode==0:
				Direction=np.sign(dx0*(-y0 + y_target)-dy0*(-x0 + x_target))*check
		else:
				Direction=mem_dir

		# And now put it all together
		Hard_Fieldx=Hard_Fieldx+Hard_Constant*Direction*(-dy0)*math.exp(-(dx0*dx0 + dy0*dy0)/Dispersion_rate);
		Hard_Fieldy=Hard_Fieldy+Hard_Constant*Direction*(+dx0)*math.exp(-(dx0*dx0 + dy0*dy0)/Dispersion_rate);

	# Done: move to the next one  
	Hard_Field=[Hard_Fieldx,Hard_Fieldy]

	#=============================================== SoftObstacles
	columns=y_S_Obs.shape[0]

	for i in range(columns):
		# Precomputation
		dx0=x_S_Obs[i]-x0
		dy0=y_S_Obs[i]-y0

		# What to do if you are not moving
		if(vx0==0 and vy0==0):
				check=1
		else:
			if Mode==0:
				check=max(0,np.sign(vx0*dx0+vy0*dy0))
			else:
				check=1

		# Selecting the direction to rotate
		if Mode==0:
				Direction=np.sign(dx0*(-y0 + y_target)-dy0*(-x0 + x_target))*check
		else:
				Direction=mem_dir
	
		# And now put it all together
		Soft_Fieldx=Soft_Fieldx+Soft_Constant*Direction*(-dy0)*math.exp(-(dx0*dx0 + dy0*dy0)/Dispersion_rate)
		Soft_Fieldy=Soft_Fieldy+Soft_Constant*Direction*(+dx0)*math.exp(-(dx0*dx0 + dy0*dy0)/Dispersion_rate)

	# Done: move to the next one  
	Soft_Field=[Soft_Fieldx,Soft_Fieldy]

	#=============================================== SumAll
	Guidance_Field = Target_Field + Hard_Field + Soft_Field
	Obstacle_Field = Soft_Field + Hard_Field

	#% Test if direction is wrong before locking Mode to 1
	Rot_test= -np.sign(Obstacle_Field[0]*Target_Field[1]-Obstacle_Field[1]*Target_Field[0])
	
	#% Check wrong rotations
	if Mode==0:
		if (mem_dir*Rot_test<0 and t!=0):#% || (t==0 && Rot_test*(x_target-x0)<0) || (t==0 && Rot_test*(y_target-y0)<0)
				mem_dir=-mem_dir
				print('direction changed')

	#% Decide whether to apply Sparse (1) or Dense (0) mode
	[Mode,final_A,final_B]=Dense_or_sparse(x_H_Obs,y_H_Obs,x_S_Obs,y_S_Obs,x0,y0,x_target,y_target,Clearance_Target,Mode)
	#Mode=0
	#final_A=1
	#final_B=2

	return [Guidance_Field,Mode,final_A,final_B,mem_dir]
