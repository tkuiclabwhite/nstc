from setuptools import setup
setup(name='task_plan_demo', version='0.1.0', packages=['task_plan_demo'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/task_plan_demo']),
                ('share/task_plan_demo', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Demo Maintainer', maintainer_email='demo@example.com',
    description='ROS 2 task plan action example', license='Apache-2.0',
    entry_points={'console_scripts': ['server = task_plan_demo.server:main',
                                    'client_gui = task_plan_demo.client_gui:main']})
