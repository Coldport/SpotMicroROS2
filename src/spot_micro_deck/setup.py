from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_micro_deck'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SpotMicro',
    maintainer_email='layillatoo@gmail.com',
    description='Bridge between the SpotMicro Steam Deck app and ROS2: streams camera, depth, '
                'lidar, map and robot state to the Deck and publishes its controller input',
    license='MIT',
    entry_points={
        'console_scripts': [
            'deck_bridge = spot_micro_deck.deck_bridge:main',
            'stack_launcher = spot_micro_deck.stack_launcher:main',
        ],
    },
)
