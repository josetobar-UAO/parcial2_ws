from setuptools import setup
import os
from glob import glob

package_name = 'parcial_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='...',
    maintainer_email='...',
    description='...',
    license='...',
    entry_points={
        'console_scripts': [
            'moteus_joint_bridge = parcial_2.moteus_joint_bridge:main',
            'gui_moteus_angles = parcial_2.gui_moteus_angles:main',
            'moteus_sequential_routine = parcial_2.moteus_sequential_routine:main',
        ],
    },
)
