from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_drawer_config = os.path.join(
        get_package_share_directory('minimap'),
        'config',
        'minimap_drawer.yaml'
    )

    return LaunchDescription([
        Node(
            package='minimap',
            executable='minimap_drawer',
            name='minimap_drawer',
            output='screen',
            parameters=[{'config_file': map_drawer_config}]
        ),
    ])