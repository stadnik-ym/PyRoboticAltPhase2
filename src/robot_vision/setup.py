from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_vision"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="artem",
    maintainer_email="artem@example.com",
    description="IMX708 camera, ArUco detector, debug vision processor and mission master",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_capture_node = robot_vision.camera_capture_node:main",
            "aruco_detector_node = robot_vision.aruco_detector_node:main",
            "vision_processor_node = robot_vision.vision_processor_node:main",
            "mission_master_node = robot_vision.mission_master_node:main",
            "lidar_avoidance_node = robot_vision.lidar_avoidance_node:main",

        ],
    },
)