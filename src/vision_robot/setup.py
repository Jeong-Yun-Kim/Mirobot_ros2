from setuptools import setup

package_name = 'vision_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision_robot.launch.py']),
        ('share/' + package_name + '/data', ['data/static_board_pose.npz']),  # ⭐ 추가
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjy',
    maintainer_email='love2851030@naver.com',
    description='YOLO-based vision control for Mirobot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'vision_robot_node = vision_robot.vision_robot_node:main',
        ],
    },
)

