# mirobot_driver.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mirobot_driver',
            executable='serial_bridge',
            name='mirobot_driver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'dry_run': False},           # 실제 구동이면 False
                {'protocol': 'gcode_example'},
                {'feedrate': 2000.0},
                {'joint_order': ['joint1','joint2','joint3','joint4','joint5','joint6']},
                {'joint_limits_deg_low':  [-170, -120, -170, -190, -120, -360]},
                {'joint_limits_deg_high': [ 170,  120,  170,  190,  120,  360]},
            ],
            remappings=[
                ('/target_joint_states', '/joint_states')  # ★ 여기 추가
            ]
        )
    ])
