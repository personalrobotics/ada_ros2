import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np




import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')





# calibration values (level)
cX = -1003.4183333333335
cY = -24.198333333333334
cZ = 38.275000000000006

#calibration values (tilt)
tX = -978.8416666666667
tY = -232.50333333333336
tZ = 40.96

main_calib_vector = np.array([cX, cY, cZ])
tilt_calib_vector = np.array([tX, tY, tZ])

#calculate the direction vector at the beginning
#vector for determining in which direction the IMU is being rotated
direction_vector = np.cross(np.cross(main_calib_vector, tilt_calib_vector), main_calib_vector)


ax.quiver(0, 0, 0, main_calib_vector[0], main_calib_vector[1], main_calib_vector[2], color='r', arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, tilt_calib_vector[0], tilt_calib_vector[1], tilt_calib_vector[2], color='b', arrow_length_ratio=0.1)
ax.quiver(0, 0, 0, direction_vector[0], direction_vector[1], direction_vector[2], color='g', arrow_length_ratio=0.1)

ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('3D Vector Plot')

plt.show()


class IMUJointstatePublisher(Node):

    def __init__(self):
        super().__init__('IMU_jointstate_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = []
        msg.position = []
        msg.velocity =  []
        msg.effort = []

        msg.name.append('IMU Joint')
        msg.position.append(1.0)
        msg.velocity.append(2.0)

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


#returns angle in radians between vectors v1 and v2
def angle_between(v1, v2):
    #get unit vectors
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)

    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


