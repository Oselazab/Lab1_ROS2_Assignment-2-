from glob import glob
from setuptools import setup
setup(name='lab1_control', version='1.0.0', packages=['lab1_control'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/lab1_control']),
                ('share/lab1_control', ['package.xml']),
                ('share/lab1_control/launch', glob('launch/*.launch.py')),
                ('share/lab1_control/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Osama Medhat', maintainer_email='iosamamedhat@gmail.com',
    description='ROS 2 Lab 1 turtlesim system', license='Apache-2.0',
    entry_points={'console_scripts': ['lab1_server = lab1_control.server:main']})
