from setuptools import setup
from glob import glob
import os
package_name='franka_body_servo_support'
setup(name=package_name,version='0.1.0',packages=[],data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),(os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),(os.path.join('share',package_name,'config'),glob('config/*.yaml'))],install_requires=['setuptools'],zip_safe=True,maintainer='Sajjad Hussain',maintainer_email='sajjad@example.com',description='MoveIt Servo support for Franka Panda MediaPipe body control',license='MIT')
