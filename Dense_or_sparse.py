import numpy as np
import math
import pdb
from Dense_or_sparse import *

def Dense_or_sparse(x_H_Obs,y_H_Obs,x_S_Obs,y_S_Obs,x0,y0,x_target,y_target,Clearance_Target,Mode):

	# Preliminary
	final_A= float('inf')
	final_B= float('inf')

	#% HardObstacles
	columns_H=y_H_Obs.shape[0]

	#% SoftObstacles
	columns_S=y_S_Obs.shape[0]

	#% Obstacle classification (x,y) of obstacles, nearest point in line to target (2 columns), sign distance to that line, between xy0 and xyT, distance to current position
	x_Obs=np.concatenate((x_H_Obs.T,x_S_Obs.T),axis=0)
	y_Obs=np.concatenate((y_H_Obs.T,y_S_Obs.T),axis=0)
	Obs_prep=np.vstack((x_Obs,y_Obs))
	Obs=np.hstack((Obs_prep.T,np.zeros((columns_H+columns_S,6))))

	#% Straigh Line from Start to Goal
	m=(y_target-y0)/(x_target-x0)

	for i in range(columns_H+columns_S):

		#% Compute nearest point in the straight line to the target
		Obs[i,2]=(Obs[i,0]+m*(m*x0-y0+Obs[i,1]))/(1+m**2)
		Obs[i,3]= m*(Obs[i,2] - x0) + y0

		#% Compute the sign distance to the straight line
		Obs[i,4]=(m*(x0-Obs[i,0])-y0+Obs[i,1])/math.pow(1+m**2,1/2)

		#% Compute the sq distance to the x0,y0 for each obstacle (save on a square root processing time)
		Obs[i,5]=((x0-Obs[i,0])**2+(y0-Obs[i,1])**2)#%.^(1/2);

		#% Selection Criteria 1: Closer to line than clearance (factor of two from "up and down")
		Obs[i,6]=Obs[i,4]>-2*Clearance_Target and Obs[i,4]<2*Clearance_Target

		#% Selection Criteria 2: Points are in front of x0y0 and before xyTarget
		Obs[i,7]=(Obs[i,0]-x0)*(Obs[i,0]-x_target)+(Obs[i,1]-y0)*(Obs[i,1]-y_target)<0

		#% Sub selection for final criteria
		if (Obs[i,6] == 1 and Obs[i,7] == 1):
			# Get the row
			Sub_selection = Obs[i]

			if Sub_selection[4]>0:
				if Sub_selection[5]<final_A:
					final_A=Sub_selection[5]
					final_A2=Sub_selection[4]
			else:
				if Sub_selection[5]<final_B:
					final_B=Sub_selection[5]
					final_B2=Sub_selection[4]

	#print(Obs)
	#print(final_A2,final_B2)
	#print("=======================================================================================================")
	
	#pdb.run('Dense_or_sparse.test()')
	#print('here')
	

	if (final_A==float('inf') or final_B==float('inf')): #%&& t~=0
			Mode=0 #% Default is sparse
	else:
			dist=math.pow(final_A2**2+final_B2**2,1/2);
			if (dist<=2*Clearance_Target and Mode==1) or (dist<=Clearance_Target and Mode==0):
				Mode=1 #% Dense environment
			else:
				Mode=0 #% Sparse environment

	return [Mode,final_A,final_B]
