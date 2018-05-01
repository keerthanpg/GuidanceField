import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

time.sleep(2)
ser.write('\r\n')
ser.write('\r\n')
ser.write('\r\n')
time.sleep(1)

ser.write('lep\r')

while True:
	line = ser.readline()
	print(line)

ser.write('lep\r')
