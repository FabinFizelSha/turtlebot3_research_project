from setuptools import setup

package_name = 'tb3_arm_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabin',
    maintainer_email='fabin@todo.todo',
    description='TurtleBot3 arm control Python package with multiple nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'goal_node = tb3_arm_ctrl.goal_node:main',
            'ik_node = tb3_arm_ctrl.ik_node:main',
            'ctrl_node = tb3_arm_ctrl.ctrl_node:main',
        ],
    },
)

