from setuptools import setup
import os, subprocess, platform
from glob import glob

package_name = 'ebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eyantra',
    maintainer_email='helpdesk@e-yantra.org',
    description='Package containing scripts to control ebot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reset_services.py = ebot_control.reset_services:main',
            'reset_arm.py=ebot_control.reset_arm:main',
            'hardware_docking.py = ebot_control.hardware_docking:main',
            'hardware_servo.py = ebot_control.hardware_servo:main',
            'hardware_aruco.py = ebot_control.hardware_aruco:main',  
            'ebot_dock = ebot_control.ebot_dock:main',
            'duplicate_imu = ebot_control.duplicate_imu:main',
        ],
    },
)
