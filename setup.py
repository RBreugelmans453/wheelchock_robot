import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheelchock_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('description/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robert',
    maintainer_email='r.m.breugelmans@student.utwente.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'la_interface = wheelchock_robot.la_interface:main',
            'test_la_behavior = wheelchock_robot.test_la_behavior:main',
            'odrive_control = wheelchock_robot.odrive_control:main',
            'move_test_1 = wheelchock_robot.move_test_1:main',
        ],
    },
)
