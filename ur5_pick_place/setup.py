from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ur5_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TU_NOMBRE',
    maintainer_email='tu_correo@ejemplo.com',
    description='Rutina Pick & Place simulada y nodo MoveIt para UR5 + gripper simple',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place_node = ur5_pick_place.pick_place_node:main',
            'moveit_goal_node = ur5_pick_place.moveit_goal_node:main',
        ],
    },
)
