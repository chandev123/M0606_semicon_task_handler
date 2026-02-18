from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'm0609_semicon_task_handler'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pose_config.yaml']),
        ('share/' + package_name, glob('*.json')),
        ('share/' + package_name + '/frontend', glob('frontend/*.html')),
        ('share/' + package_name + '/frontend/css', glob('frontend/css/*')),
        ('share/' + package_name + '/frontend/js', glob('frontend/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MARKCH',
    maintainer_email='cocoffeechan09@gmail.com',
    description='ROS2 control package for M0609 robotic arm in semiconductor package task handling process',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hw_force = m0609_semicon_task_handler.hw_node.hw_node_force:main',
            'task = m0609_semicon_task_handler.task_node:main',
            'ui = m0609_semicon_task_handler.ui_node:main',
        ],
    },
)
