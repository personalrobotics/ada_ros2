import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ada_imu'),
        'config',
        'imu_params.yaml'
        )
        
    imu_jointstate_publisher=Node(
        package = 'ada_imu',
        name = 'imu_jointstate_publisher',
        executable = 'imu_jointstate_publisher',
        parameters = [config]
    )

    ld.add_action(imu_jointstate_publisher)
    return ld