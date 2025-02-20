from setuptools import setup
import os
from glob import glob


package_name = 'examples'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='maintainer@example.com',
    description='Example nodes demonstrating usage of exray sdk',
    license='Copyrights Hydromea 2024 - All rights reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_square_path_demo = examples.linear_square_path_demo:main',
            'rotating_square_motion_demo = examples.rotating_square_motion_demo:main',
        ],
    },
)
