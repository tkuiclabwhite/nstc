import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'head_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='iclab@todo.todo',
    description='由頭部馬達刻度計算相機外參：動態 TF 節點 neck_tf_publisher、校正資料收集工具 head_sweep',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'head_sweep = head_tf.head_sweep:main',
            'neck_tf_publisher = head_tf.neck_tf_publisher:main',
        ],
    },
)
