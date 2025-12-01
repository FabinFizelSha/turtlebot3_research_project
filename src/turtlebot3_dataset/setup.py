from setuptools import setup

package_name = 'turtlebot3_dataset'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fabin',
    maintainer_email='fabin@example.com',
    description='Dataset collector for TurtleBot3 sensors',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collect_data = turtlebot3_dataset.collect_data:main',
        ],
    },
)
