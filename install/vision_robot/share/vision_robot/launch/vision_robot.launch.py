from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "/home/kjy/venvs/yolov5_env/bin/python",
                "/home/kjy/Mirobot_ros2/src/vision_robot/vision_robot/vision_robot_node.py",
            ],
            output="screen"
        )
    ])

