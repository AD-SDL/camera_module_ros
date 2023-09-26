from setuptools import setup, find_packages
import os
from glob import glob

install_requires_list = ["setuptools", "opencv-python"]
# with open('requirements.txt') as reqs:
#     for line in reqs.readlines():
#         req = line.strip()
#         if not req or req.startswith('#'):
#             continue
#         install_requires_list.append(req)

package_name = 'camera_ros_client'

setup(
    name = package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools',"opencv-python"],
    zip_safe=True,
    maintainer='Doga Ozgulbas',
    maintainer_email='dozgulbas@anl.gov',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_ros_client.camera_publisher:main',
            'camera_subscriber = camera_ros_client.camera_subscriber:main',
            'capture_image = camera_ros_client.capture_image:main'


        ],
    },
)