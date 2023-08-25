#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module defines a node that continuously reads in accelerometer 
data from the IMU to calculate the angle at which the wheelchair is
currently tilted and publishes the angle and velocity to the 
/joint_states topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import serial

class IMUJointstatePublisher(Node):
    """
    Reads in IMU accelerometer data, calculates and smooths the angle and 
    velocity of the wheelchair's tilt, and publishes the values in a
    JointState message to /joint_states.
    """

    def __init__(self):
        super().__init__('IMU_jointstate_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        read_only = ParameterDescriptor(read_only = True)
        self.declare_parameter('joint_name', rclpy.Parameter.Type.STRING, read_only)
        self.declare_parameter('main_calib_vector', rclpy.Parameter.Type.DOUBLE_ARRAY, read_only)
        self.declare_parameter('tilt_calib_vector', rclpy.Parameter.Type.DOUBLE_ARRAY, read_only)
        self.declare_parameter('serial_port', '/dev/ttyUSB0', read_only)
        self.declare_parameter('velocity_thresh', rclpy.Parameter.Type.DOUBLE, read_only) # radians per second
        self.declare_parameter('position_smoothing_factor', rclpy.Parameter.Type.DOUBLE, read_only)
        self.declare_parameter('velocity_smoothing_factor', rclpy.Parameter.Type.DOUBLE, read_only)

        self.init_serial()
        self.init_vectors()
        self.init_smoothing()
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value

        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.publish)


    
    def init_vectors(self):
        """
        Initializes the calibration vectors and calculates other necessary direction vectors.
        """
        self.main_calib_vector = self.get_parameter('main_calib_vector').get_parameter_value().double_array_value
        self.tilt_calib_vector = self.get_parameter('tilt_calib_vector').get_parameter_value().double_array_value

        # calculate the vector for determining in which direction the IMU is being rotated
        self.direction_vector = np.cross(np.cross(self.main_calib_vector, self.tilt_calib_vector), self.main_calib_vector)


    def init_serial(self):
        """
        Initializes the serial port for IMU readings and waits for the IMU
        header information to be published before any function can try to
        read in data.
        """
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.ser = serial.Serial(port)
        self.ser.baudrate = 115200
        # ignore header information from the serial input
        # wait for the empty line signifying header is over
        while True:
            line = str(self.ser.readline()) #read in a line from the IMU
            if line == "b'\\r\\n'":
                break

    
    def init_smoothing(self):
        """
        Initializes all variables used in the exponential smoothing calculations.
        """
        # weights used in exponential rolling averages
        self.position_smoothing_factor = self.get_parameter('position_smoothing_factor').get_parameter_value().double_value
        self.velocity_smoothing_factor = self.get_parameter('velocity_smoothing_factor').get_parameter_value().double_value

        self.prev_smoothed_position = self.prev_angle  = self.get_IMU_angle()
        self.prev_smoothed_velocity = 0.0
        self.velocity_thresh = self.get_parameter('velocity_thresh').get_parameter_value().double_value



    def publish(self):
        """
        Reads in IMU data, applies smoothing, thresholds, and rounding, and then publishes
        the position and velocity in a JointState message.
        """
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity =  []
        msg.effort = []


        imu_angle = self.get_IMU_angle()
        smoothed_position = self.exponential_smoothing(self.position_smoothing_factor, imu_angle, self.prev_smoothed_position)
        # round the smoothed position to the nearest hundreth radian
        rounded_position = round(smoothed_position, 2)

        imu_velocity = (imu_angle - self.prev_angle) / self.timer_period
        smoothed_velocity = threshed_velocity = self.exponential_smoothing(self.velocity_smoothing_factor, imu_velocity, self.prev_smoothed_velocity)
        # apply threshold
        if np.abs(smoothed_velocity) < self.velocity_thresh:
            threshed_velocity = 0

        self.prev_angle = imu_angle
        self.prev_smoothed_position = smoothed_position
        self.prev_smoothed_velocity = smoothed_velocity


        msg.name.append(self.joint_name)
        msg.position.append(rounded_position)
        msg.velocity.append(threshed_velocity)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" \n' % msg)



    def get_IMU_angle(self):
        """
        Calculates and returns the signed angle of the wheelchair tilt in radians.
        """
        imu_vector = self.get_IMU_vector()
        imu_angle = self.angle(imu_vector, self.main_calib_vector)

        # to figure out the sign of the angle, project the imu vector onto the direction line
        scalar_projection = np.linalg.norm(imu_vector) * np.cos(self.angle(imu_vector, self.direction_vector))

        # the sign of the scalar projection is the direction of the angle
        imu_angle *= np.sign(scalar_projection)

        return imu_angle


    def get_IMU_vector(self):
        """
        Reads in the accelerometer data from the IMU and returns the acceleration vector
        as a floating point array.
        """
        self.ser.flushInput() # flush the input stream to get most recent data
        self.ser.readline() # get rid of any leftover partial line from the flush
        line = str(self.ser.readline()) # read in a line from the IMU
        data = list(map(str.strip, line.split(','))) # convert csv line to array

        accel_data = data[2:5]
        accel_data = list(map(float, accel_data)) # cast string data to floats

        return accel_data # return accelerometer values


    @staticmethod
    def angle(v1: np.ndarray, v2: np.ndarray):
        """
        Returns the angle in radians between vectors v1 and v2

        Parameters
        ----------
        v1, v2: vectors (numpy.ndarray type float) to calculate the angle between

        Returns
        -------
        angle: float (radians)
        """
        # get unit vectors
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)

        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


    @staticmethod
    def exponential_smoothing(smoothing_factor: float, current_datapoint: float, previous_average: float):
        """
        Calculates the exponential rolling average of the data

        Parameters
        ----------
        smoothing_factor: floating point variable between 0 and 1, determines how much smoothing is applied
        current_datapoint: the most recent datapoint that the average is being applied to
        previous_average: the output of the smoothing function on the previous datapoint

        Returns
        -------
        current_datapoint smoothed, float
        """
        return (smoothing_factor * current_datapoint) + ( (1-smoothing_factor) * previous_average)






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = IMUJointstatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
