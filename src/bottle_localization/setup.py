import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bottle_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='YOLO 2D 偵測框 + ZED 深度圖 -> 藥罐瓶身中心在 base_link 下的 3D 位置',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bottle_localizer = bottle_localization.bottle_localizer:main',
            'bottle_viewer = bottle_localization.bottle_viewer:main',
        ],
    },
)
