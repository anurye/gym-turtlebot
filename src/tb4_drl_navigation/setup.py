from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tb4_drl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='anurye.yesuf@gmail.com',
    description='TurtleBot4 Gymnasium Environment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example = tb4_drl_navigation.examples.sac:main',
            'rosgz = tb4_drl_navigation.environments.utils.ros_gz:main'
        ],
    },
)
