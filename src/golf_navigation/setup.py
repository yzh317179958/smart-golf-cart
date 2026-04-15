from setuptools import find_packages, setup

package_name = 'golf_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@golf.local',
    description='Golf cart navigation and control: mode manager, follow PID, GPS pulse waypoint follower',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mode_manager = golf_navigation.mode_manager_node:main',
            'follow_target_publisher = golf_navigation.follow_target_publisher:main',
            'lock_manager = golf_navigation.lock_manager_node:main',
            'gps_waypoint_follower = golf_navigation.gps_waypoint_follower:main',
        ],
    },
)
