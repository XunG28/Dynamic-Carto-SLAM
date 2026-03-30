from glob import glob
import os

from setuptools import setup


package_name = 'dynamic_carto_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TurtleBot3 + Cartographer bringup for Dynamic-Carto-SLAM demo stack.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_gt_publisher = dynamic_carto_demo.gazebo_ground_truth_node:main',
            'bag_pose_to_tum = dynamic_carto_demo.bag_pose_to_tum:main',
        ],
    },
)
