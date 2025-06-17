from setuptools import setup
import os
from glob import glob

package_name = 'image_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 Node to Publish Images from a ZIP Archive',
    license='Apache-2.0',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'image_publisher_node = image_publisher.image_publisher_node:main',
        ],
    },
)
