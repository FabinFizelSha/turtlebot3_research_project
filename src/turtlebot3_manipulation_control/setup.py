from setuptools import setup, find_packages

setup(
    name='turtlebot3_manipulation_control',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'rclpy',
    ],
    entry_points={
        'console_scripts': [
            'turtlebot3_ik_demo = turtlebot3_manipulation_control.turtlebot3_ik_demo:main',
        ],
    },
)

