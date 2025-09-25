from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gorm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Bjørndahl Mortensen',
    maintainer_email='antonbm2008@gmail.com',
    description='ROS 2 bringup package for launching the GORM rover system, including state publisher, control, and teleop.',
    license='GNU Affero General Public License v3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
