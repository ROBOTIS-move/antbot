from glob import glob
import os

from setuptools import setup

package_name = 'antbot_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daun Jeong',
    maintainer_email='jdu@robotis.com',
    description='Keyboard teleoperation node for ANTBot swerve-drive robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = antbot_teleop.teleop_keyboard:main',
            'teleop_joystick = antbot_teleop.teleop_joystick:main',
            'swerve_sim = antbot_teleop.swerve_sim:main',
        ],
    },
)
