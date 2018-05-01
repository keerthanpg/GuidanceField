import numpy as np
import math

def Over_Camera(x_S_Obs,y_S_Obs,Image_W,Image_L,Scaling):

	#% Number of points detected by Lidar
	columns=x_S_Obs.shape[0]
	
	#% Prepare measurements
	x_S_Obs_measure=[]
	y_S_Obs_measure=[]

	for i in range(columns):
		x_S_Obs_measure.append(x_S_Obs[i]*Image_L/Scaling)
		y_S_Obs_measure.append(y_S_Obs[i]*Image_W/Scaling)

	return [np.array(x_S_Obs_measure),np.array(y_S_Obs_measure)]
