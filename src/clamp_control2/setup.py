from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'clamp_control2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bky',
    maintainer_email='bky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'clamp_node = clamp_control.ClampROSnode:main',
            'clamp_node = clamp_control.MotorControlNode:main',
        ],
    },
)
