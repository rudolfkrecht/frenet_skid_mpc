from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("torque_supervisor")
    params = os.path.join(pkg_share, "config", "config.yaml")

    return LaunchDescription([
        Node(
            package="torque_supervisor",
            executable="torque_supervisor",
            name="torque_supervisor",
            output="screen",
            parameters=[params],
        )
    ])
