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

# ignore header information - wait for the empty line signifying header is over
while True:
	line = str(ser.readline()) #read in a line from the IMU
	if line == "b'\\r\\n'":
		break


timeout = time.time() + seconds

while time.time() < timeout:
	ser.flushInput() #flush the input stream to get most recent data
	ser.readline() #get rid of any leftover partial line from the flush
	line = str(ser.readline()) #read in a line from the IMU
	data = list(map(str.strip, line.split(','))) #convert csv line to array

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
