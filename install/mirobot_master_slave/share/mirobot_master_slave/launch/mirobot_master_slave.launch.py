from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ===== args =====
    port1 = LaunchConfiguration("port1")
    port2 = LaunchConfiguration("port2")
    baud  = LaunchConfiguration("baud")

    declare_args = [
        DeclareLaunchArgument("port1", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("port2", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("baud",  default_value="115200"),
    ]

    # ===== xacro path =====
    xacro_path = PathJoinSubstitution([
        FindPackageShare("mirobot_description"),
        "urdf",
        "mirobot_urdf_2.xacro"
    ])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str
    )

    # ================= Robot 1 =================
    bridge1 = Node(
        package="mirobot_driver",
        executable="serial_bridge",
        namespace="mirobot1",
        name="serial_bridge",
        output="screen",
        parameters=[{
            "port": port1,
            "baud": baud,
            "dry_run": False,
            "protocol": "gcode_example",
            "require_enable": False,
        }],
    )

    rsp1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="mirobot1",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": "mirobot1/"}
        ],
    )

    init1 = Node(
        package="mirobot_master_slave",
        executable="init_joint_once",
        name="mirobot1_init_joint_once",
        output="screen",
        parameters=[{
            "topic": "/mirobot1/target_joint_states",
            "delay_sec": 2.0,
            "names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }]
    )

    # ================= Robot 2 =================
    bridge2 = Node(
        package="mirobot_driver",
        executable="serial_bridge",
        namespace="mirobot2",
        name="serial_bridge",
        output="screen",
        parameters=[{
            "port": port2,
            "baud": baud,
            "dry_run": False,
            "protocol": "gcode_example",
            "require_enable": False,
        }],
    )

    rsp2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="mirobot2",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"frame_prefix": "mirobot2/"}
        ],
    )

    init2 = Node(
        package="mirobot_master_slave",
        executable="init_joint_once",
        name="mirobot2_init_joint_once",
        output="screen",
        parameters=[{
            "topic": "/mirobot2/target_joint_states",
            "delay_sec": 2.2,
            "names": ["joint1","joint2","joint3","joint4","joint5","joint6"],
            "positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }]
    )

    return LaunchDescription(
        declare_args + [
            bridge1, bridge2,
            rsp1, rsp2,
            init1, init2,  
        ]
    )

