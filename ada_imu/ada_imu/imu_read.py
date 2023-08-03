import serial
import time
import numpy as np

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200

print("connected to: " + ser.portstr)

# number of seconds to average the IMU data for
seconds = 3
timeout = time.time() + seconds

# store the accelerometer data
accelX = []
accelY = []
accelZ = []

# tracker variable to know when data is being recorded 
# (ignore the header information)
logging = False

while time.time() < timeout:
	line = str(ser.readline())#read in a line from the IMU

	if logging:
		#convert csv line to array
		data = list(map(str.strip, line.split(','))) 

		# store accelerometer data
		accelX.append(float(data[2]))
		accelY.append(float(data[3]))
		accelZ.append(float(data[4]))

	
	# look for the empty line signifying the header is over
	# and the data stream is starting
	if line == "b'\\r\\n'":
		logging = True


# get averages
avgX = np.average(accelX)
avgY = np.average(accelY)
avgZ = np.average(accelZ)

print("average X acceleration: ", avgX)
print("average Y acceleration: ", avgY)
print("average Z acceleration: ", avgZ)


ser.close()