import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("zrobot_joy"), "config"
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        "deadzone": 0.05,
                        "autorepeat_rate": 20.0,
                        "coalesce_interval_ms": 1,
                    }
                ],
            ),
            Node(
                package="zrobot_joy",
                executable="joy_teleop_node",
                name="joy_teleop_node",
                parameters=[
                    os.path.join(config_dir, "xbox_joy_params.yaml")
                ],
                output="screen",
            ),
        ]
    )
