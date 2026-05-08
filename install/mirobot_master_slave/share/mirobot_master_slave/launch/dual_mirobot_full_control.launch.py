from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    mirobot_desc_dir = get_package_share_directory('mirobot_description')
    master_slave_dir = get_package_share_directory('mirobot_master_slave')

    urdf_file = os.path.join(mirobot_desc_dir, 'urdf', 'mirobot_urdf_2.xacro')

    xacro_exec = FindExecutable(name='xacro')

    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mirobot1_state_pub',
        namespace='mirobot1',
        output='screen',
        parameters=[{
	    'robot_description': ParameterValue(
		Command([
		    xacro_exec, ' ', urdf_file, ' ',
		    'base_offset_x:=0.0', ' ',
		    'namespace:=mirobot1'
		]),
		value_type=str
	    )
	}]
    )

    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mirobot2_state_pub',
        namespace='mirobot2',
        output='screen',
        parameters=[{
	    'robot_description': ParameterValue(
		Command([
		    xacro_exec, ' ', urdf_file, ' ',
		    'base_offset_x:=0.0', ' ',
		    'namespace:=mirobot2'
		]),
		value_type=str
	    )
	}]
    )

    driver1 = Node(
        package='mirobot_driver',
        executable='serial_bridge',
        name='mirobot1_driver',
        namespace='mirobot1',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'dry_run': False,
            'require_enable': True,
        }]
    )

    driver2 = Node(
        package='mirobot_driver',
        executable='serial_bridge',
        name='mirobot2_driver',
        namespace='mirobot2',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'dry_run': False,
            'require_enable': True,
        }]
    )

    master_cli = Node(
        package='mirobot_master_slave',
        executable='mirobot_xyz_cli.py',
        name='mirobot_cli',
        output='screen',
        namespace='mirobot1'
    )

    slave_replicator = Node(
        package='mirobot_master_slave',
        executable='mirobot_slave_replicator.py',
        name='mirobot_replicator',
        output='screen',
        namespace='mirobot2'
    )

    rviz_config = os.path.join(master_slave_dir, 'rviz', 'dual_mirobots.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot1_state_pub,
        robot2_state_pub,
        driver1,
        driver2,
        master_cli,
        slave_replicator,
        rviz_node
    ])

