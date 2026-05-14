from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tl_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[
        package_name, 
        'sdk', 'sdk.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tlibot',
    maintainer_email='duyukun@fdrobot.com',
    description='ROS2 driver for TianLian TCB robot arm',
    license='MIT',
    package_data={
        package_name: 
        [
            'config/*.yaml',
            'launch/*.py',
        ],
        'sdk': [
            'TCB_SDK_2403/config/*.yaml', 'TCB_SDK_2403/models/*.urdf', 'TCB_SDK_2403/utils',
            'TCB_SDK_2207/config/*.yaml', 'TCB_SDK_2207/models/*.urdf', 'TCB_SDK_2207/utils',
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),  
    ],
    entry_points={
        'console_scripts': [
            'tl_driver_node = tl_driver.tl_driver:main',
        ],
    },
)