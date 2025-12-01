from setuptools import setup

package_name = 'tb3_arm_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'h5py', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Logs TurtleBot3 arm and gripper states into HDF5',
    entry_points={
        'console_scripts': [
            'tb3_hdf5_logger = tb3_arm_logger.tb3_hdf5_logger:main'
        ],
    },
)
