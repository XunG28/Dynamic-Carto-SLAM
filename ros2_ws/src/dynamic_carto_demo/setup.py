from glob import glob
import os

from setuptools import setup


package_name = 'dynamic_carto_demo'


def _data_files_for_dir(src_dir: str, dst_dir: str):
    out = []
    for root, _, files in os.walk(src_dir):
        if not files:
            continue
        rel_root = os.path.relpath(root, src_dir)
        install_dir = dst_dir if rel_root == '.' else os.path.join(dst_dir, rel_root)
        out.append((install_dir, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), sorted(glob('launch/*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.lua') + glob('config/*.yaml')),
    ] + _data_files_for_dir(
        'models',
        os.path.join('share', package_name, 'models'),
    ),
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
            'odom_noisy = dynamic_carto_demo.odom_noisy_node:main',
            'odom_child_frame_fix = dynamic_carto_demo.odom_child_frame_fix_node:main',
            'odom_to_tf = dynamic_carto_demo.odom_to_tf_node:main',
            'laserscan_to_pointcloud2 = dynamic_carto_demo.laserscan_to_pointcloud2_node:main',
            'world_odom_bridge = dynamic_carto_demo.world_odom_bridge_node:main',
        ],
    },
)
