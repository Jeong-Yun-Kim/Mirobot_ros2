from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mirobot_description')

    urdf_path = os.path.join(pkg_share, 'urdf', 'mirobot_urdf_2.urdf')
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()
    robot_description = ParameterValue(urdf_xml, value_type=str)

    rviz_cfg = os.path.join(pkg_share, 'rviz', 'description.rviz')

    # 옵션: 하드웨어 없이도 RViz가 움직이도록 /target_joint_states를 /joint_states로 미러링
    mirror_arg = DeclareLaunchArgument(
        'mirror_to_joint_states',
        default_value='true',   # 하드웨어 없을 땐 true, 드라이버 켜면 false 추천
        description='Mirror /target_joint_states to /joint_states for RViz when no driver is running'
    )
    mirror = LaunchConfiguration('mirror_to_joint_states')

    # 1) robot_state_publisher : URDF을 파라미터로 전달
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2) joint_state_publisher_gui : 명령은 /target_joint_states 로 발행
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('/joint_states', '/target_joint_states')],
        # GUI도 robot_description을 읽을 수 있게 하면 초기 에러 감소
        parameters=[{'robot_description': robot_description}]
    )

    relay = Node(
        condition=IfCondition(mirror),
        package='topic_tools',
        executable='relay',
        name='target_to_joint_states_relay',
        arguments=['/target_joint_states', '/joint_states'],
        output='screen'
    )

    # 4) RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        mirror_arg,
        rsp,
        jsp_gui,
        relay,
        rviz,
    ])
