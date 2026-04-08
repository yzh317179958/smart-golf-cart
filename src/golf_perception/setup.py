from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'golf_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='杨子豪',
    maintainer_email='yzh317179958@gmail.com',
    description='Golf cart perception nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = golf_perception.detection_node:main',
            'camera_info_fix = golf_perception.camera_info_fix:main',
            'gesture_node = golf_perception.gesture_node:main',
            'image_resizer = golf_perception.image_resizer:main',
        ],
    },
)
