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
    wheel_radius_m = 0.0635
    track_width_m = 0.4572

    pkg_path = get_package_share_directory('mailrover_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mail_rover.urdf.xacro')

    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_uart = LaunchConfiguration('enable_uart')

    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    obstacle_avoidance = Node(
        package='my_robot_controller',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{
            'cmd_vel_in': '/cmd_vel_nav',
            'cmd_vel_out': '/cmd_vel',
            'scan_topic': '/scan',
        }],
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

    uart_bridge = Node(
        package='my_robot_controller',
        executable='uart_bridge',
        name='uart_bridge',
        output='screen',
        condition=IfCondition(enable_uart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_path, 'maps', 'floor.yaml'),
            description='Saved occupancy grid map to use for localization.',
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time.',
        ),
        DeclareLaunchArgument(
            'enable_uart',
            default_value='false',
            description='Start UART delivery-task bridge.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rplidar,
        encoder_odometry,
        nav2,
        obstacle_avoidance,
        motor_controller,
        uart_bridge,
    ])
