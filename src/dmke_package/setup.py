from setuptools import find_packages, setup
import os

package_name = 'dmke_package'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_position_server = dmke_package.SetPosition_server:main'
            'set_position_client = dmke_package.SetPosition_client:main'
        ],
    },
)
