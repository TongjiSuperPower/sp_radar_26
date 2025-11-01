from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    locate_config = os.path.join(
        get_package_share_directory('pointcloud_locate'),
        'configs',
        'locate.yaml'
    )

    return LaunchDescription([
        Node(
            package='pointcloud_locate',
            executable='locate',
            name='locate',
            output='screen',
            parameters=[{'config_file': locate_config}]
        ),
    ])