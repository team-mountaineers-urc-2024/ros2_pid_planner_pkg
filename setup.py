from setuptools import find_packages, setup

package_name = 'ros2_pid_planner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_planner_node = ros2_pid_planner_pkg.pid_planner:main',
            'turtle_2_3d_node = ros2_pid_planner_pkg.turtle_2_3d:main',
            'pixhawk_node = ros2_pid_planner_pkg.test_localization:main',
            'gps2enu_repub_node = ros2_pid_planner_pkg.gps2enu_repub:main',
        ],
    },
)
