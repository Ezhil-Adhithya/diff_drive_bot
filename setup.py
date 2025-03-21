from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_urdf_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name,"urdf"),glob("urdf/*.urdf")),
        (os.path.join("share",package_name,"launch"),glob("launch/*.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ezhil',
    maintainer_email='ezhilns4002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node = gazebo_urdf_launch.voice:main",
        ],
    },
)
