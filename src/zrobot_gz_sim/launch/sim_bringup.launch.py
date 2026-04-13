import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('zrobot_gz_sim')
    share_root_dir = os.path.dirname(pkg_share_dir)

    try:
        get_package_share_directory('controller_manager')
        has_controller_manager = True
    except Exception:
        has_controller_manager = False

    try:
        get_package_share_directory('gz_ros2_control')
        has_gz_ros2_control = True
    except Exception:
        has_gz_ros2_control = False

    pkg_share = FindPackageShare('zrobot_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')
    enable_controller_spawners = LaunchConfiguration('enable_controller_spawners')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf']),
        description='SDF world file')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='1.05',
        description='Initial robot spawn height above ground (meters)')
    
    enable_controller_spawners_arg = DeclareLaunchArgument(
        'enable_controller_spawners',
        default_value='true',
        description='Enable controller_manager spawners (requires controller_manager package)')

    bridge_params_file = PathJoinSubstitution([pkg_share, 'config', 'bridge_params.yaml'])
    urdf_file_path = os.path.join(pkg_share_dir, 'resources', 'zrobot', 'urdf', 'zrobot.urdf')
    controllers_file_path = os.path.join(pkg_share_dir, 'config', 'controllers.yaml')

    controller_spawner_condition = IfCondition(enable_controller_spawners if has_controller_manager else 'false')

    with open(urdf_file_path, 'r', encoding='utf-8') as urdf_fp:
        robot_description_text = urdf_fp.read()

    # gz_ros2_control needs a filesystem params file path, package:// is not accepted.
    robot_description_text = robot_description_text.replace(
        'package://zrobot_gz_sim/config/controllers.yaml',
        controllers_file_path,
    )

    robot_description = ParameterValue(robot_description_text, value_type=str)

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
        arguments=['-name', 'zrobot', '-topic', 'robot_description', '-z', spawn_z],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        condition=controller_spawner_condition,
    )

    joint_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_group_position_controller', '--controller-manager', '/controller_manager'],
        condition=controller_spawner_condition,
    )

    spawn_jsb_after_spawn = RegisterEventHandler(
        condition=controller_spawner_condition,
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_jgc_after_jsb = RegisterEventHandler(
        condition=controller_spawner_condition,
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_group_controller_spawner],
        )
    )

    controller_manager_missing_warning = LogInfo(
        msg='[zrobot_gz_sim] controller_manager package not found. '
            'Controller spawners are disabled; install ros-jazzy-controller-manager and ros-jazzy-ros2-controllers.'
    )

    gz_ros2_control_missing_warning = LogInfo(
        msg='[zrobot_gz_sim] gz_ros2_control package not found. '
            'Install ros-jazzy-gz-ros2-control to enable ros2_control in Gazebo.'
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

    actions = [
        world_arg,
        sim_time_arg,
        spawn_z_arg,
        enable_controller_spawners_arg,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb_after_spawn,
        spawn_jgc_after_jsb,
        gz_bridge,
        gazebo_motor_bridge,
    ]

    if not has_controller_manager:
        actions.append(controller_manager_missing_warning)
    if not has_gz_ros2_control:
        actions.append(gz_ros2_control_missing_warning)

    return LaunchDescription(actions)
