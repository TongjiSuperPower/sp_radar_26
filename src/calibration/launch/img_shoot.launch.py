from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    img_shoot_config = os.path.join(
        get_package_share_directory('calibration'),
        'configs',
        'camera_shoot.yaml'
    )

    return LaunchDescription([
        Node(
            package='calibration',
            executable='img_shoot',
            name='img_shoot',
            output='screen',
            parameters=[{'config_file': img_shoot_config}]
        ),
    ])