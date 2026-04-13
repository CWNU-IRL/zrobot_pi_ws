import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('zrobot_gz_sim')
    share_root_dir = os.path.dirname(pkg_share_dir)

    pkg_share = FindPackageShare('zrobot_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    enable_controller_spawners = LaunchConfiguration('enable_controller_spawners')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf']),
        description='SDF world file')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    enable_controller_spawners_arg = DeclareLaunchArgument(
        'enable_controller_spawners',
        default_value='false',
        description='Enable controller_manager spawners (requires controller_manager package)')

    bridge_params_file = PathJoinSubstitution([pkg_share, 'config', 'bridge_params.yaml'])
    urdf_file = PathJoinSubstitution([pkg_share, 'resources', 'zrobot', 'urdf', 'zrobot.urdf'])

    robot_description = ParameterValue(
        Command([
            'cat ',
            urdf_file,
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[share_root_dir, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'zrobot', '-topic', 'robot_description'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=IfCondition(enable_controller_spawners),
    )

    joint_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_group_position_controller', '--controller-manager', '/controller_manager'],
        condition=IfCondition(enable_controller_spawners),
    )

    spawn_jsb_after_spawn = RegisterEventHandler(
        condition=IfCondition(enable_controller_spawners),
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_jgc_after_jsb = RegisterEventHandler(
        condition=IfCondition(enable_controller_spawners),
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_group_controller_spawner],
        )
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    gazebo_motor_bridge = Node(
        package='zrobot_gz_sim',
        executable='gazebo_motor_bridge_node',
        output='screen',
        parameters=[bridge_params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        world_arg,
        sim_time_arg,
        enable_controller_spawners_arg,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb_after_spawn,
        spawn_jgc_after_jsb,
        gz_bridge,
        gazebo_motor_bridge,
    ])
