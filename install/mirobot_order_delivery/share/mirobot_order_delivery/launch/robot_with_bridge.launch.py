from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")
    bridge_config = LaunchConfiguration("bridge_config")
    server_domain = LaunchConfiguration("server_domain")
    robot_domain = LaunchConfiguration("robot_domain")
    debug_view = LaunchConfiguration("debug_view")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution([FindPackageShare("mirobot_order_delivery"), "config", "order_delivery.yaml"]),
            description="Path to delivery node YAML parameters.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([FindPackageShare("mirobot_order_delivery"), "config", "mirobot_domain_bridge.yaml"]),
            description="domain_bridge YAML config.",
        ),
        DeclareLaunchArgument("server_domain", default_value="18", description="Raspberry Pi/server ROS_DOMAIN_ID."),
        DeclareLaunchArgument("robot_domain", default_value="30", description="Laptop/Mirobot ROS_DOMAIN_ID."),
        DeclareLaunchArgument("debug_view", default_value="false", description="Show OpenCV YOLO debug window on the laptop."),

        # Bridge /robot/cmd and /robot/stop from server_domain -> robot_domain,
        # and /robot/done(/robot/status) back from robot_domain -> server_domain.
        ExecuteProcess(
            cmd=["ros2", "run", "domain_bridge", "domain_bridge", "--from", server_domain, "--to", robot_domain, bridge_config],
            output="screen",
        ),

        # Run the actual YOLO + Mirobot worker only on the robot/laptop domain.
        Node(
            package="mirobot_order_delivery",
            executable="delivery_node",
            name="mirobot_order_delivery_node",
            output="screen",
            parameters=[params, {"debug.view": debug_view}],
            additional_env={"ROS_DOMAIN_ID": robot_domain},
        ),
    ])
