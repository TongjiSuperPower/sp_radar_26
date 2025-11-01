from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pc_proj = os.path.join(
        get_package_share_directory('calibration'),
        'configs',
        'pointcloud_projection.yaml'
    )

    return LaunchDescription([
        Node(
            package='calibration',
            executable='pointcloud_projection',
            name='pointcloud_projection',
            output='screen',
            parameters=[{'config_file': pc_proj},{'use_sim_time': True}]
        ),
    ])