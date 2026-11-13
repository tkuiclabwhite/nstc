import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'handpackage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='5454sky@gmail.com',
    description='LinkerHand O6 雙手控制節點',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handpackage = handpackage.handpackage:main',
        ],
    },
)
