from setuptools import setup

package_name = 'tb3_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TurtleBot3 IK controller for pick and place',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tb3_ik_controller = tb3_ik.tb3_ik_controller:main',
        ],
    },
)

