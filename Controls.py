import numpy as np
import math

def Controls(Right,Left):
	

	# Rescale
	freq_Max=.37
	Right=Right*freq_Max
	Left=Left*freq_Max

	# Constants Motor 1 (Right - TBD) - Coefficients of the motor calibration for positive (p) and negative (n)
	a3_p= -5709.7
	a2_p= +1871.3
	a1_p=  -415.66
	a0_p=   -46.127

	a3_n= -4210.6
	a2_n= -1862.7
	a1_n=  -520.76
	a0_n=   +42.593

	# Compute Motor 1
	if Right>0:
		In_1=a3_p*math.pow(Right,3)+a2_p*math.pow(Right,2)+a1_p*math.pow(Right,1)+a0_p
	else:
		In_1=a3_n*math.pow(Right,3)+a2_n*math.pow(Right,2)+a1_n*math.pow(Right,1)+a0_n


	# Constants Motor 2 (Left - TBD)
	b3_p= -7741.3
	b2_p= +2848.1
	b1_p=  -542.96
	b0_p=   -42.185

	b3_n= -6299.3
	b2_n= -1492.4
	b1_n=  -317.53
	b0_n=    55.362

	# Compute Motor 1
	if Left>0:
		In_2=b3_p*math.pow(Left,3)+b2_p*math.pow(Left,2)+b1_p*math.pow(Left,1)+b0_p
	else:
		In_2=b3_n*math.pow(Left,3)+b2_n*math.pow(Left,2)+b1_n*math.pow(Left,1)+b0_n

	return [int(In_1),int(In_2)]
