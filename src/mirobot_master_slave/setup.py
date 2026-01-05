from setuptools import setup

package_name = 'mirobot_master_slave'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ 실제 있는 파일만 나열
        ('share/' + package_name + '/launch', ['launch/mirobot_master_slave.launch.py']),
        ('share/' + package_name + '/launch', ['launch/dual_mirobot_full_control.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/dual_mirobots.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjy',
    maintainer_email='kjy@todo.todo',
    description='Master-slave control for WLKATA Mirobot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_cli = mirobot_master_slave.mirobot_xyz_cli:main',
            'slave_replicator = mirobot_master_slave.mirobot_slave_replicator:main',
            'init_joint_once = mirobot_master_slave.init_joint_once:main',
        ],
    },

)

