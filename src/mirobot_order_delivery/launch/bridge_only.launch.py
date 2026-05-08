from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = LaunchConfiguration("bridge_config")
    server_domain = LaunchConfiguration("server_domain")
    robot_domain = LaunchConfiguration("robot_domain")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([FindPackageShare("mirobot_order_delivery"), "config", "mirobot_domain_bridge.yaml"]),
            description="domain_bridge YAML config.",
        ),
        DeclareLaunchArgument("server_domain", default_value="18", description="Raspberry Pi/server ROS_DOMAIN_ID."),
        DeclareLaunchArgument("robot_domain", default_value="30", description="Laptop/Mirobot ROS_DOMAIN_ID."),
        ExecuteProcess(
            cmd=["ros2", "run", "domain_bridge", "domain_bridge", "--from", server_domain, "--to", robot_domain, bridge_config],
            output="screen",
        ),
    ])
