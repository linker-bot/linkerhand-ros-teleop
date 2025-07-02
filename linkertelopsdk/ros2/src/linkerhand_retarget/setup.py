from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'linkerhand_retarget'
# 自动发现所有Python包（不包括resource/）
packages = find_packages(where='.', exclude=['resource*'])

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
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
    packages=packages,
    package_dir={'': '.'},  # 从当前目录搜索Python包
    include_package_data=True,  # 关键！启用非Python文件包含
    package_data={
        'linkerhand_retarget': [
            'motion/**/*',  # 递归包含所有子目录
        ],
    },
    # ROS2安装配置
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/linkerhand_retarget']),
        ('share/linkerhand_retarget', ['package.xml']),
        ('share/linkerhand_retarget/assets', glob('assets/*')),
        ('share/linkerhand_retarget/config', glob('config/*')),
    ],
    zip_safe=False,
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
         'handretarget  = linkerhand_retarget.handretarget:main',
        ],
    },
)
