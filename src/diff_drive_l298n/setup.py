from setuptools import setup

package_name = 'diff_drive_l298n'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='artem',
    maintainer_email='artem@todo.todo',
    description='L298N motor driver',
    license='TODO',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'diff_drive_node = diff_drive_l298n.motor_node:main',
            'route_executor = diff_drive_l298n.route_executor:main',
            'keyboard_teleop_node = diff_drive_l298n.keycontrol:main',
        ],
    },
)


