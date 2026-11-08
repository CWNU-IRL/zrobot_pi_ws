"""battery_control 启动文件。

用法:
    ros2 launch battery_control battery.launch.py
    ros2 launch battery_control battery.launch.py channel:=can4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel = LaunchConfiguration('channel')
    address = LaunchConfiguration('device_address')
    rate = LaunchConfiguration('publish_rate')

    return LaunchDescription([
        DeclareLaunchArgument('channel', default_value='can4',
                              description='SocketCAN 接口名'),
        DeclareLaunchArgument('device_address', default_value='0',
                              description='电池设备地址（主机=0）'),
        DeclareLaunchArgument('publish_rate', default_value='2.0',
                              description='状态话题发布频率 Hz'),
        Node(
            package='battery_control',
            executable='battery_node',
            name='battery_control_node',
            output='screen',
            parameters=[{
                'channel': channel,
                'device_address': address,
                'publish_rate': rate,
            }],
        ),
    ])
