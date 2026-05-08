# Backward-compatible alias for robot_only.launch.py.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")
    robot_domain = LaunchConfiguration("robot_domain")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([FindPackageShare("mirobot_order_delivery"), "config", "order_delivery.yaml"]),
            description="Path to delivery node YAML parameters.",
        ),
        DeclareLaunchArgument(
            "robot_domain",
            default_value="30",
            description="ROS_DOMAIN_ID for the laptop/robot side.",
        ),
        Node(
            package="mirobot_order_delivery",
            executable="delivery_node",
            name="mirobot_order_delivery_node",
            output="screen",
            parameters=[params],
            additional_env={"ROS_DOMAIN_ID": robot_domain},
        ),
    ])
