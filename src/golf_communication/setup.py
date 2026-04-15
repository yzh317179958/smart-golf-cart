from setuptools import find_packages, setup

package_name = 'golf_communication'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@golf.local',
    description='Golf cart communication: MQTT bridge and summon dispatcher for mobile APP integration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mqtt_bridge = golf_communication.mqtt_bridge_node:main',
            'summon_service = golf_communication.summon_service:main',
        ],
    },
)
