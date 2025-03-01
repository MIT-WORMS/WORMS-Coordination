import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'brain'

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
    maintainer='ethan',
    maintainer_email='ethanlai@mit.edu',
    description='Primary brain logic for WORMS project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"action_node = {package_name}.action_node:main",
            f"brain = {package_name}.brain:main",
            f"testing = {package_name}.testing:main"
        ],
    },
)
