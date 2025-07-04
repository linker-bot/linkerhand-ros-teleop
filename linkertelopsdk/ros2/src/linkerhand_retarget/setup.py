#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'linkerhand_retarget'

this_dir = os.path.abspath(os.path.dirname(__file__))
custom_dir = os.path.join(this_dir, package_name, "LinkerHand")

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

rescoure_dir = 'resource'
for dirpath, dirnames, filenames in os.walk(rescoure_dir):
    share_path = os.path.relpath(dirpath,rescoure_dir)
    for filename in filenames:
        file_path = os.path.join(dirpath,filename)
        data_files.append((os.path.join('share',package_name,share_path),[file_path]))
        
setup(
    name=package_name,
    version='2.6.3',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linker-robot',
    maintainer_email='linker-robot@todo.todo',
    description='ROS2 SDK for Linker Hand',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
         'handretarget  = linkerhand_retarget.handretarget:main',
        ],
    },
)
