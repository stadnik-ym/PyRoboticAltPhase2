# from setuptools import find_packages, setup

# package_name = 'stream_ros'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='artem',
#     maintainer_email='artem@todo.todo',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#             "ros_stream = stream_ros.image_stream_node:main",
#         ],
#     },
# )


from setuptools import find_packages, setup

package_name = "stream_ros"

setup(
    name=package_name,
    version="0.0.0",
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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="artem",
    maintainer_email="artem@example.com",
    description="ROS2 image stream web server",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ros_stream = stream_ros.ros_stream:main",
        ],
    },
)