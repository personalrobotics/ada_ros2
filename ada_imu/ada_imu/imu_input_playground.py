import serial
import time
import numpy as np

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200

print("connected to: " + ser.portstr)

# number of seconds to average the IMU data for
seconds = 3

# store the accelerometer data
accelX = []
accelY = []
accelZ = []

# ignore header information
while True:
	line = str(ser.readline())#read in a line from the IMU
	if line == "b'\\r\\n'":
		break


timeout = time.time() + seconds

while time.time() < timeout:
	line = str(ser.readline())#read in a line from the IMU
	#convert csv line to array
	data = list(map(str.strip, line.split(','))) 

	# store accelerometer data
	accelX.append(float(data[2]))
	accelY.append(float(data[3]))
	accelZ.append(float(data[4]))


# get averages
avgX = np.average(accelX)
avgY = np.average(accelY)
avgZ = np.average(accelZ)

print("average X acceleration: ", avgX)
print("average Y acceleration: ", avgY)
print("average Z acceleration: ", avgZ)


ser.close()