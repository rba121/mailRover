import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    wheel_radius_m = 0.0635
    track_width_m = 0.4572
    robot_length_m = 0.4572
    robot_width_m = 0.6096
    front_wheel_overhang_m = 0.096

    pkg_path = get_package_share_directory('mailrover_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mail_rover.urdf.xacro')
    slam_params = LaunchConfiguration('slam_params')

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
            'max_wheel_speed_mps': 1.0,
        }],
    )

    scan_self_filter = Node(
        package='my_robot_controller',
        executable='scan_self_filter',
        name='scan_self_filter',
        output='screen',
        parameters=[{
            'input_scan_topic': '/scan',
            'output_scan_topic': '/scan_filtered',
            'laser_x_offset_m': 0.0,
            'laser_y_offset_m': 0.0,
            'self_filter_x_min_m': -(robot_length_m / 2.0) - 0.03,
            'self_filter_x_max_m': (robot_length_m / 2.0)
                                   + front_wheel_overhang_m + 0.02,
            'self_filter_y_min_m': -(robot_width_m / 2.0) - 0.03,
            'self_filter_y_max_m': (robot_width_m / 2.0) + 0.03,
        }],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={'params_file': slam_params}.items(),
    )

    motor_controller = Node(
        package='my_robot_controller',
        executable='motor_controller_pid',
        output='screen',
        parameters=[{
            'wheel_radius_m': wheel_radius_m,
            'track_width_m': track_width_m,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(pkg_path, 'config', 'my_slam_config.yaml'),
            description='SLAM Toolbox mapping parameter file.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rplidar,
        encoder_odometry,
        scan_self_filter,
        slam,
        motor_controller,
    ])
