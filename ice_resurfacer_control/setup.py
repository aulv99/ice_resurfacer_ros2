from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ice_resurfacer_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaro',
    maintainer_email='aaro.ulvila@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomy_node = ice_resurfacer_control.autonomy_seq:main',
            'conditioner_node = ice_resurfacer_control.conditioner_manager:main',
            'drive_bridge_node = ice_resurfacer_control.drive_bridge:main',
            'obstacle_detection_node = ice_resurfacer_control.obstacle_detection.py:main'
        ],
    },
)
