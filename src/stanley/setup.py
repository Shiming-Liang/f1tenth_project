import os
from glob import glob
from setuptools import setup

package_name = 'stanley'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzangupenn, Hongrui Zheng',
    maintainer_email='zzang@seas.upenn.edu, billyzheng.bz@gmail.com',
    description='f1tenth stanley',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanley_node = stanley.stanley_node:main',
            'mpc_node = stanley.mpc_node:main',
            'mpc_plus_stanley = stanley.mpc_plus_stanley:main'
        ],
    },
)
