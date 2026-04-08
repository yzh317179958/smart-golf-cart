from setuptools import find_packages, setup

package_name = 'golf_mapping'

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
    description='Golf cart mapping: GPS waypoint recorder, IMU fusion, test logger',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_path_recorder = golf_mapping.gps_path_recorder:main',
            'test_logger = golf_mapping.test_logger:main',
            'imu_ned_to_enu = golf_mapping.imu_ned_to_enu:main',
        ],
    },
)
