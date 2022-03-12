from glob import glob
from setuptools import setup
import os

package_name = 'pandabot_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Package to control Pandabot',
    license='GNU GENERAL PUBLIC LICENSE Version 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pandabot_key = pandabot_drive.pandabot_key:main',
			'pandabot_drive = pandabot_drive.pandabot_drive:main',
            'pandabot_teleop = pandabot_drive.pandabot_teleop:main',
            'pandabot_odometry = pandabot_drive.pandabot_odometry:main',
        ],
    },
)
