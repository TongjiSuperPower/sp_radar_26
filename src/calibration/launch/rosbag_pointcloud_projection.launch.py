from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    icp_node_config = os.path.join(
        get_package_share_directory('calibration'),
        'configs',
        'rosbag_pointcloud_projection.yaml'
    )

    return LaunchDescription([
        Node(
            package='calibration',
            executable='rosbag_pointcloud_projection',
            name='rosbag_pointcloud_projection',
            output='screen',
            parameters=[{'config_file': icp_node_config},{'use_sim_time': True}]
        ),
    ])