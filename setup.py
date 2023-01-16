from setuptools import setup

import os
from glob import glob

package_name = 'virtual_depth_camera'
sub_modules = []

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name]+sub_modules,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # needed by the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),  # needed by the launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='savvas',
    maintainer_email='savvas.sampaziotis@gmail.com',
    description='Virtual Camera Package. Purpose of this package is to synthesize an image of the robot, emulating the robot configurate and camera parameters. The output image is used as a mask, to determine the position of a robot in an RGBD image',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_depth_camera_node = virtual_depth_camera.virtual_depth_camera_node:main',
            'demo_node = virtual_depth_camera.demo_node:main',
        ],
    },
)
