import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    启动电机控制节点的launch配置
    """
    
    # 获取包目录
    zrobot_bridge_dir = get_package_share_directory('zrobot_bridge')
    config_file = os.path.join(zrobot_bridge_dir, 'config', 'motor_config.yaml')
    
    # 创建节点
    motor_controller_node = Node(
        package='zrobot_bridge',
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    return LaunchDescription([
        motor_controller_node,
    ])
