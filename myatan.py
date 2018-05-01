import numpy as np
import math

def myatan(y,x):

	if x>0:
		v=math.atan(y/x);

	if y>=0 and x<0:
		v= math.pi+math.atan(y/x);

	if y<0 and x<0:
		v=-math.pi+math.atan(y/x);

	if y>0 and x==0:
		v= math.pi/2;

	if y<0 and x==0:
		v=-math.pi/2;

	if v<0:
		v=v+2*math.pi;

	return v
