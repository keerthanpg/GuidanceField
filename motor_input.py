import numpy as np
import math

def Motor_input(th_target):
	#print(th_target)
	if((th_target>=0) and (th_target<math.pi/2)):
		Motor_Left_Multiplier=1-(th_target)/(math.pi/4)
		Motor_Right_Multiplier=1

	if((th_target>=math.pi/2) and th_target<math.pi):
		Motor_Left_Multiplier = -1
		Motor_Right_Multiplier=1-(th_target-math.pi/2)/(math.pi/4)

	if(th_target>=math.pi and th_target<3*math.pi/2):
		Motor_Left_Multiplier=-1+(th_target-math.pi)/(math.pi/4)
		Motor_Right_Multiplier=-1

	if(th_target>=3*math.pi/2 and th_target<=2*math.pi):
		Motor_Left_Multiplier=1
		Motor_Right_Multiplier=-1+(th_target-3*math.pi/2)/(math.pi/4)

	return [Motor_Left_Multiplier, Motor_Right_Multiplier]
