from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    icp_node_config = os.path.join(
        get_package_share_directory('relocalization'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='relocalization',
            executable='icp_node',
            name='icp_node',
            output='screen',
            parameters=[{'config_file': icp_node_config},{'use_sim_time': True}]
        ),
    ])