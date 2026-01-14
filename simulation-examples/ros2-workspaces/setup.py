from setuptools import setup
import os
from glob import glob

package_name = 'robotics_course_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Course Maintainer',
    maintainer_email='course@example.com',
    description='Examples for the Physical AI & Humanoid Robotics Course',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = basic_nodes.simple_publisher:main',
            'simple_subscriber = basic_nodes.simple_subscriber:main',
        ],
    },
)