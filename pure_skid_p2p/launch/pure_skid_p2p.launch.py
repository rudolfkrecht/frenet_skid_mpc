# pure_skid_p2p.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='pure_skid_p2p',
            name='pure_skid_p2p',
            namespace='j100_0000',   # <-- THIS is the key
            output='screen',
            parameters=[
                {'base_frame': 'base_link'},   # stays relative
                {'cmdvel_topic': 'cmd_vel'},   # stays relative
                {'path_topic': '/micro_waypoints_path'}
            ]
        )
    ])
