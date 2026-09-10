import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    wheel_radius_m = 0.075
    track_width_m = 0.4572

    pkg_path = get_package_share_directory('mailrover_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mail_rover.urdf.xacro')
    start_lidar = LaunchConfiguration('start_lidar')

    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    rplidar = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('rplidar_ros'),
                        'launch',
                        'rplidar.launch.py',
                    )
                ),
                launch_arguments={
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'laser',
                }.items(),
                condition=IfCondition(start_lidar),
            )
        ],
    )

    encoder_odometry = Node(
        package='my_robot_controller',
        executable='encoder_odometry',
        name='encoder_odometry',
        output='screen',
        parameters=[{
            'wheel_radius_m': wheel_radius_m,
            'track_width_m': track_width_m,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'left_direction': 1.0,
            'right_direction': 1.0,
            'max_wheel_speed_mps': 1.0,
        }],
    )

    motor_controller = Node(
        package='my_robot_controller',
        executable='motor_controller_pid',
        output='screen',
        parameters=[{
            'wheel_radius_m': wheel_radius_m,
            'track_width_m': track_width_m,
            'left_motor_direction': 1.0,
            'right_motor_direction': 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_lidar',
            default_value='false',
            description='Start RPLIDAR during manual motion testing.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rplidar,
        encoder_odometry,
        motor_controller,
    ])
