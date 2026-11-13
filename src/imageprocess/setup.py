from setuptools import find_packages, setup

package_name = 'imageprocess'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iclab',
    maintainer_email='keninhuang920517@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image = imageprocess.image:main',
            'depth_process_node = imageprocess.depth_process_node:main',
            'overlap_node = imageprocess.overlap_node:main',
            'camera_param_bridge_node = imageprocess.camera_param_bridge_node:main',
        ],
    },
)
