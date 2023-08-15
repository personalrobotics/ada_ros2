import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor

import numpy as np
import serial


# default calibration values (level)
default_cX = -1003.4183333333335
default_cY = -24.198333333333334
default_cZ = 38.275000000000006

# default calibration values (tilt)
default_tX = -978.8416666666667
default_tY = -232.50333333333336
default_tZ = 40.96

class IMUJointstatePublisher(Node):

    def __init__(self):
        super().__init__('IMU_jointstate_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic', 10)

        self.declare_parameter('main_calib_vector', [default_cX, default_cY, default_cZ])
        self.declare_parameter('tilt_calib_vector', [default_tX, default_tY, default_tZ])
        self.declare_parameter('velocity_thresh', 0.05) # radians per second
        self.declare_parameter('serial_port', '/dev/ttyUSB0')

        self.init_serial()
        self.init_vectors()


        # weights used in exponential rolling averages, between 0 and 1
        self.position_smoothing_factor = 0.9 
        self.velocity_smoothing_factor = 0.9

        self.prev_smoothed_position = self.prev_angle  = self.get_IMU_angle()
        self.prev_smoothed_velocity = 0.0
        self.velocity_thresh = self.get_parameter('velocity_thresh').get_parameter_value().integer_value

        self.timer_period = 0.5 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)



    def init_vectors(self):
        self.main_calib_vector = self.get_parameter('main_calib_vector').get_parameter_value().double_array_value
        self.tilt_calib_vector = self.get_parameter('tilt_calib_vector').get_parameter_value().double_array_value

        # calculate the vector for determining in which direction the IMU is being rotated
        self.direction_vector = np.cross(np.cross(self.main_calib_vector, self.tilt_calib_vector), self.main_calib_vector)


    # initialize serial port for IMU readings
    def init_serial(self):
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.ser = serial.Serial(port)
        self.ser.baudrate = 115200
        # ignore header information from the serial input
        # wait for the empty line signifying header is over
        while True:
            line = str(self.ser.readline()) #read in a line from the IMU
            if line == "b'\\r\\n'":
                break



    def timer_callback(self):
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity =  []
        msg.effort = []


        imu_angle = self.get_IMU_angle()
        smoothed_position = exponential_smoothing(self.position_smoothing_factor, imu_angle, self.prev_smoothed_position)

        imu_velocity = (imu_angle - self.prev_angle) / self.timer_period
        smoothed_velocity = threshed_velocity = exponential_smoothing(self.velocity_smoothing_factor, imu_velocity, self.prev_smoothed_velocity)
        # apply threshhold
        if np.abs(smoothed_velocity) < self.velocity_thresh:
            threshed_velocity = 0

        self.prev_angle = imu_angle
        self.prev_smoothed_position = smoothed_position
        self.prev_smoothed_velocity = smoothed_velocity


        msg.name.append('IMU Joint')
        msg.position.append(smoothed_position)
        msg.velocity.append(threshed_velocity)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'root'


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" \n' % msg)



    # returns the signed angle of the IMU
    def get_IMU_angle(self):
        imu_vector = self.get_IMU_vector()
        imu_angle = angle(imu_vector, self.main_calib_vector)

        # to figure out the sign of the angle, project the imu vector onto the direction line
        scalar_projection = np.linalg.norm(imu_vector) * np.cos(angle(imu_vector, self.direction_vector))

        # the sign of the scalar projection is the direction of the angle
        imu_angle *= np.sign(scalar_projection)

        return imu_angle


    # reads in the accelerometer data from the IMU and returns the acceleration vector as an array
    def get_IMU_vector(self):
        self.ser.flushInput() # flush the input stream to get most recent data
        self.ser.readline() # get rid of any leftover partial line from the flush
        line = str(self.ser.readline()) # read in a line from the IMU
        data = list(map(str.strip, line.split(','))) # convert csv line to array

        accel_data = data[2:5]
        accel_data = list(map(float, accel_data)) # cast string data to floats

        return accel_data # return accelerometer values





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




# returns angle in radians between vectors v1 and v2
def angle(v1, v2):
    # get unit vectors
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


# calculates the exponential rolling average of the data
def exponential_smoothing(smoothing_factor, current_datapoint, previous_average):
    return (smoothing_factor * current_datapoint) + ( (1-smoothing_factor) * previous_average)