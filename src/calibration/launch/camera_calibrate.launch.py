import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    calibrate_camera_config = os.path.join(
        get_package_share_directory('calibration'),
        'configs',
        'camera_calibrate.yaml'
    )

    return LaunchDescription([
        Node(
            package='calibration',
            executable='camera_calibrate',
            name='camera_calibrate',
            output='screen',
            parameters=[{'config_file': calibrate_camera_config}]
        ),
    ])