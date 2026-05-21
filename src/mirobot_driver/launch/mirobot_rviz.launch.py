from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('mirobot_description')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'mirobot_urdf_2.urdf')
    rviz_cfg = os.path.join(desc_pkg, 'rviz', 'description.rviz')

    return LaunchDescription([
        # ✅ 실제 로봇 드라이버
        Node(
            package='mirobot_driver',
            executable='serial_bridge',
            name='mirobot_driver',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'dry_run': False},
                {'protocol': 'gcode_example'},
                {'feedrate': 2000},
                {'joint_order': ['joint1','joint2','joint3','joint4','joint5','joint6']},
            ],
            # ❌ remap 절대 넣지 마 (지금 너 파일엔 이게 문제였음)
        ),

        # ✅ RViz가 움직이려면 robot_state_publisher 필요
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # ❌ joint_state_publisher_gui 없음 (CLI 명령 덮어쓰기 방지)

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg]
        ),
    ])

