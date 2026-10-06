from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_micro_move'

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
    description='Trot gait node driving the real PCA9685 servos via spot_micro_ik, '
                'commanded by the Steam Deck\'s Joy input',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gait_node = spot_micro_move.gait_node:main',
        ],
    },
)
