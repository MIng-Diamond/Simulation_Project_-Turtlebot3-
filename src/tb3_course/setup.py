from glob import glob

from setuptools import find_packages, setup

package_name = 'tb3_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mingseed',
    maintainer_email='luxm25@mails.tsinghua.edu.cn',
    description='TurtleBot3 ROS 2 Humble course examples and homework',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle = tb3_course.turtle_circle:main',
            'pose_listener = tb3_course.pose_listener:main',
            'move_square = tb3_course.move_square:main',
            'square_odom = tb3_course.square_odom:main',
            'nav_goal = tb3_course.nav_goal:main',
            'patrol = tb3_course.patrol:main',
            'velocity_mirror = tb3_course.velocity_mirror:main',
            'robot_distance = tb3_course.robot_distance:main',
            'leader_follower = tb3_course.leader_follower:main',
            'auto_mapper = tb3_course.auto_mapper:main',
        ],
    },
)
