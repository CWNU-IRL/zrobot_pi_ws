from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('zrobot_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf']),
        description='SDF world file')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    bridge_params_file = PathJoinSubstitution([pkg_share, 'config', 'bridge_params.yaml'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'zrobot_temp.urdf.xacro'])

    robot_description = Command([
        'xacro ',
        xacro_file,
        ' controllers_file:=',
        controllers_file,
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': [world, ' -r']}.items(),
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
        arguments=['-name', 'zrobot_temp', '-topic', 'robot_description'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    joint_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_group_position_controller', '--controller-manager', '/controller_manager'],
    )

    spawn_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_jgc_after_jsb = RegisterEventHandler(
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
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb_after_spawn,
        spawn_jgc_after_jsb,
        gz_bridge,
        gazebo_motor_bridge,
    ])
