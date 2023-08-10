import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np
import serial


# calibration values (level)
cX = -1003.4183333333335
cY = -24.198333333333334
cZ = 38.275000000000006

# calibration values (tilt)
tX = -978.8416666666667
tY = -232.50333333333336
tZ = 40.96

main_calib_vector = np.array([cX, cY, cZ])
tilt_calib_vector = np.array([tX, tY, tZ])

# calculate the vector for determining in which direction the IMU is being rotated
direction_vector = np.cross(np.cross(main_calib_vector, tilt_calib_vector), main_calib_vector)

ser = serial.Serial('/dev/ttyUSB0')
ser.baudrate = 115200

# ignore header information from the serial input
# wait for the empty line signifying header is over
while True:
    line = str(ser.readline()) #read in a line from the IMU
    if line == "b'\\r\\n'":
        break


class IMUJointstatePublisher(Node):

    def __init__(self):
        super().__init__('IMU_jointstate_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic', 10)
        self.timer_period = 0.5 # seconds
        self.prev_angle = 0.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity =  []
        msg.effort = []


        imu_vector = get_IMU_vector()
        imu_angle = angle(imu_vector, main_calib_vector)
        # to figure out the sign of the angle, project the imu vector onto the direction line
        scalar_projection = np.linalg.norm(imu_vector) * np.cos(angle(imu_vector, direction_vector))
        # the sign of the scalar projection is the direction of the angle
        imu_angle *= np.sign(scalar_projection)

        imu_velocity = (imu_angle - self.prev_angle) / self.timer_period
        self.prev_angle = imu_angle

        msg.name.append('IMU Joint')
        msg.position.append(imu_angle)
        msg.velocity.append(imu_velocity)

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'root'


        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s" \n' % msg)



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

# reads in the accelerometer data from the IMU and returns the acceleration vector as an array
def get_IMU_vector():
    ser.flushInput() # flush the input stream to get most recent data
    ser.readline() # get rid of any leftover partial line from the flush
    line = str(ser.readline()) # read in a line from the IMU
    data = list(map(str.strip, line.split(','))) # convert csv line to array

    accel_data = data[2:5]
    accel_data = list(map(float, accel_data)) # cast string data to floats

    return accel_data # return accelerometer values
