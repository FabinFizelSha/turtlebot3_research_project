from setuptools import setup

package_name = 'random_nav_goals'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabin',
    maintainer_email='fabin@example.com',
    description='Random navigation goals sender for TurtleBot3',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'send_random_goals = random_nav_goals.random_nav_goal_sender:main',
        ],
    },
    package_data={
        '': ['package.xml'],  # explicitly include package.xml
    },
)
