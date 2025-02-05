# -*- coding: utf-8 -*-
# Copyright (c) 2024-2025, Personal Robotics Laboratory
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
This module provides data for calibrating the IMU jointstate publisher.
The main function will read in the IMU's accelerometer data and print
the average over n seconds.
"""

import time
import serial
import numpy as np


def main():
    """
    Averages the IMU accelerometer data over n seconds and prints to terminal
    """
    ser = serial.Serial("/dev/ttyUSB0")
    ser.baudrate = 115200

    print("connected to: " + ser.portstr)

    # number of seconds to average the IMU data for
    seconds = 3

    # store the accelerometer data
    accel_x = []
    accel_y = []
    accel_z = []

    # ignore header information - wait for the empty line signifying header is over
    while True:
        line = str(ser.readline())  # read in a line from the IMU
        if line == "b'\\r\\n'":
            break

    timeout = time.time() + seconds

    while time.time() < timeout:
        ser.flushInput()  # flush the input stream to get most recent data
        ser.readline()  # get rid of any leftover partial line from the flush
        line = str(ser.readline())  # read in a line from the IMU
        data = list(map(str.strip, line.split(",")))  # convert csv line to array

        # store accelerometer data
        accel_x.append(float(data[2]))
        accel_y.append(float(data[3]))
        accel_z.append(float(data[4]))

    # get averages
    avg_x = np.average(accel_x)
    avg_y = np.average(accel_y)
    avg_z = np.average(accel_z)

    print("average X acceleration: ", avg_x)
    print("average Y acceleration: ", avg_y)
    print("average Z acceleration: ", avg_z)

    ser.close()


if __name__ == "__main__":
    main()
