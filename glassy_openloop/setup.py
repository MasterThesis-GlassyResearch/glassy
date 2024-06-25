import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'glassy_openloop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joaolehodey',
    maintainer_email='joao.lehodey@gmail.com',
    description='This package is used for openloop control (sending commands to the vehice)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'glassy_openloop = glassy_openloop.glassy_openloop:main',
            'glassy_openloop_analysis = glassy_openloop.glassy_openloop_analysis:main'
        ],
    },
)

