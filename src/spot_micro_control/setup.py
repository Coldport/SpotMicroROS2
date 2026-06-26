from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_micro_control'

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
    maintainer='SpotMicro',
    maintainer_email='layillatoo@gmail.com',
    description='Gait controller and point navigation for SpotMicro',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gait_controller = spot_micro_control.gait_controller:main',
            'point_nav = spot_micro_control.point_nav:main',
        ],
    },
)
