import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suchitha',
    maintainer_email='suchitha.channa@gmail.com',
    description='AK80-64 motor interface nodes for WORMS project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"pid_controller = {package_name}.pid_controller:main",
            f"motor_interface = {package_name}.motor_interface:main"
        ],
    },
)
