from setuptools import setup
import os
from glob import glob

package_name = 'ros2_pointpillars'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiachenli',
    maintainer_email='jiachenli@todo.todo',
    description='ROS2 package for PointPillars 3D object detection using LiDAR point clouds',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointpillars_node = ros2_pointpillars.ros2_pointpillars:main',
            'unitree_tf_publisher = ros2_pointpillars.unitree_tf_publisher:main',
        ],
    },
)
