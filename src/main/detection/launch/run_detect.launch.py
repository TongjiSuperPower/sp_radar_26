from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    run_detect_config = os.path.join(
        get_package_share_directory('camera_detection'),
        'config',
        'run_detect.yaml'
    )

    return LaunchDescription([
        Node(
            package='camera_detection',
            executable='runtime_car_armor',
            name='runtime_car_armor',
            output='screen',
            parameters=[{'config_file': run_detect_config}]
        ),
    ])