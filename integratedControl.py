# Control of the robot post calibration

import serial 
import syslog
import time
import serial
import argparse
import numpy as np
import math

#The following line is for serial over GPIO
port = '/dev/ttyACM0' # note I'm using Mac OS-X

ard = serial.Serial(port,9600,timeout=5)
time.sleep(2) # wait for Arduino


def Controls(Right,Left):

    # Rescale
    freq_Max=.37
    Right=Right*freq_Max
    Left=Left*freq_Max

    # Constants Motor 1 (Right - TBD)
    a3_p= -5709.7
    a2_p= +1871.3
    a1_p= -415.66
    a0_p= -46.127

    a3_n= -4210.6
    a2_n= -1862.7
    a1_n= -520.76
    a0_n= +42.593

    # Compute Motor 1
    if Right>0:
        In_1=a3_p*math.pow(Right,3)+a2_p*math.pow(Right,2)+a1_p*math.pow(Right,1)+a0_p
    else:
        In_1=a3_n*math.pow(Right,3)+a2_n*math.pow(Right,2)+a1_n*math.pow(Right,1)+a0_n

    # Constants Motor 2 (Left - TBD)
    b3_p= -7741.3
    b2_p= +2848.1
    b1_p= -542.96
    b0_p= -42.185

    b3_n= -6299.3
    b2_n= -1492.4
    b1_n= -317.53
    b0_n= +55.362

    # Compute Motor 1
    if Left>0:
        In_2=b3_p*math.pow(Left,3)+b2_p*math.pow(Left,2)+b1_p*math.pow(Left,1)+b0_p
    else:
        In_2=b3_n*math.pow(Left,3)+b2_n*math.pow(Left,2)+b1_n*math.pow(Left,1)+b0_n

    sendtoarduino(int(In_1),int(In_2))



def sendtoard():
    while (1):
    # Serial write section
        ard.flush()
        m1, m2 = Controls(r,l)  # Right and left

        # Because for Keerthana code, 355 is 0
        val1 = int(m1 + 355)
        val2 = int(m2 + 355)
        formatted_send=str(format(int(val1), '03d')+format(int(val2), '03d'));
        ard.write((str(val1)+str(val2)).encode('utf-8'))
        time.sleep(0.2) # I shortened this to match the new value in your Arduino

        # Serial read section
        msg = ard.read(ard.inWaiting()) # read all characters in buffer
        msg=msg.decode("utf-8") 
        print (msg)
        
    else:
        print ("Exiting")
    exit()

def sendtoarduino(motor_1, motor_2):
    
    	# Serial write section
        ard.flush()

        # Because for Keerthana code, 355 is 0
        val1 = int(motor_1 + 355)
        val2 = int(motor_2 + 355)
        formatted_send=str(format(int(val1), '03d')+format(int(val2), '03d'));
        ard.write((str(val1)+str(val2)).encode('utf-8'))
        time.sleep(0.2) # I shortened this to match the new value in your Arduino

        # Serial read section
        #msg = ard.read(ard.inWaiting()) # read all characters in buffer
        #msg=msg.decode("utf-8") 
	#print (msg)



# Main function
#if __name__ == "__main__":
#        sendtoard()


#if __name__ == "__main__":
	#while (1):
		# This is where you add Joao's stuff...
		#-----------------------------------
		# ... ...
		#-----------------------------------
		#mot1, mot2 = Controls(r,l)
        	#sendtoarduino(mot1, mot2)
