from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node, LifecycleNode
from launch.actions import TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    static_scan_config = os.path.join(
        get_package_share_directory('map_scan'),
        'config',
        'static_scan.yaml'
    )

    return LaunchDescription([
        Node(
            package='map_scan',
            executable='checking_period_scan',
            name='checking_period_scan',
            output='screen',
            parameters=[{'config_file': static_scan_config},{'use_sim_time': True}]
        ),
        # TimerAction(
        #     period=6.0,  # 延迟3秒
        #     actions=[
        #         Node(
        #             package='map_scan',
        #             executable='pcd_process',
        #             name='static_scan',
        #             output='screen',
        #         ), 
        #         RegisterEventHandler(
        #             OnProcessExit(
        #                 on_exit=[EmitEvent(event=Shutdown())]
        #             )
        #         )
        #     ]
        # )
    ])