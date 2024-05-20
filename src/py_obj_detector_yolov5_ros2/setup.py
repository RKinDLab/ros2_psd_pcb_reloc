import os
from glob import glob
from setuptools import setup

package_name = 'py_obj_detector_yolov5_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name,
              f"{package_name}.neural_net",
              f"{package_name}.submodules"
              ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Azmyin Md. Kamal',
    maintainer_email='azmyin12@gmail.com',
    description='Driver node that passes rgb, semantic matrix and depthmap to VSLAM nodes',
    license='GPL V3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "robot0_nn = py_obj_detector_yolov5_ros2.robot0_nn:main",
        "robot1_nn = py_obj_detector_yolov5_ros2.robot1_nn:main",
        "robot_combined_nn = py_obj_detector_yolov5_ros2.robot_combined_nn:main",
        ],
    },
)
