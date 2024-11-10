from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gait_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elai',
    maintainer_email='ethanlai@mit.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_node = gait_package.action_node:main',
            'gait_manager = gait_package.gait_manager:main',
            'testing = gait_package.testing:main'
        ],
    },
)
