from glob import glob

from setuptools import setup

package_name = 'sp3_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='5454sky@gmail.com',
    description='子三任務管理：接主控的取放任務，串接藥瓶辨識、手臂與手部抓取',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager = sp3_task_manager.task_manager:main',
            'reach_test = sp3_task_manager.reach_test:main',
        ],
    },
)
