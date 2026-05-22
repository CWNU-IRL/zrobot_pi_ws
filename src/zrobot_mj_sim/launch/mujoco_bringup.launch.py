from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    params_file = PathJoinSubstitution(
        [FindPackageShare("zrobot_mj_sim"), "config", "mujoco_bridge_params.yaml"]
    )

    mujoco_bridge = Node(
        package="zrobot_mj_sim",
        executable="mujoco_motor_bridge_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        mujoco_bridge,
    ])
