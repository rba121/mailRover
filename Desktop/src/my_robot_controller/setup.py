from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohit',
    maintainer_email='rohit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'test_node = my_robot_controller.my_first_node:main',
            'draw_circle = my_robot_controller.draw_circle:main',
            'pose_subscriber = my_robot_controller.pose_subscriber:main',
            'turtle_controller = my_robot_controller.turtle_controller:main',
            'mock_odom = my_robot_controller.mock_odom:main',
            'motor_controller = my_robot_controller.motor_controller:main',
            'motor_controller2 = my_robot_controller.motor_controller2:main',
            'motor_controller4 = my_robot_controller.motor_controller4:main',
            'keyboard_teleop = my_robot_controller.keyboard_teleop:main',
            'uart_bridge = my_robot_controller.uart_bridge:main',
        ],
    },
)
