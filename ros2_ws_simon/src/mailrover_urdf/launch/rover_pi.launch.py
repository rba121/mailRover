import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():

    # --- Robot Description (URDF) ---
    pkg_path = get_package_share_directory('mailrover_urdf')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mail_rover.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Static joint states for wheels (replaces joint_state_publisher_gui)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rplidar = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                   os.path.join(
                      get_package_share_directory('rplidar_ros'),
                      'launch',
                      'rplidar.launch.py'
                   )
               ),
               launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser'
               }.items()
            )
        ]
    )

    # --- RPLiDAR ---
    #rplidar_node = Node(
    #    package='rplidar_ros',
    #    executable='rplidar_composition',
    #    output='screen',
    #    parameters=[{
    #        'serial_port': '/dev/ttyUSB0',
    #        'serial_baudrate': 115200,
    #        'frame_id': 'laser',
    #        'angle_compensate': True,
    #        'scan_mode': 'Standard'
    #    }]
    #)

    #rplidar = TimerAction(
    #    period=3.0,
    #    actions=[rplidar_node]
    #)

    # --- RF2O Laser Odometry ---
    rf2o_node = Node(
       package='rf2o_laser_odometry',
       executable='rf2o_laser_odometry_node',
       name='rf2o_laser_odometry_node',
       output='screen',
       parameters=[{
         'laser_scan_topic': '/scan',
         'base_frame_id': 'base_footprint',
         'odom_frame_id': 'odom',
         'laser_frame_id': 'laser',
         'publish_tf': True,
       }]
    )

    rf2o = TimerAction(
        period=6.0,
        actions=[rf2o_node]
    )

    # --- SLAM Toolbox ---
    nav2_params_file = '/home/mypi/ros2_ws/nav2_params.yaml'

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'params_file': '/home/mypi/ros2_ws/my_slam_config.yaml'
        }.items()
    )

    # --- Nav2 Navigation ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items()
    )

    # --- Obstacle Safety Filter ---
    obstacle_avoidance = Node(
        package='my_robot_controller',
        executable='obstacle_avoidance',
        output='screen',
        parameters=[{
            'cmd_vel_in': '/cmd_vel_nav',
            'cmd_vel_out': '/cmd_vel',
            'scan_topic': '/scan',
            'ultrasonic_topic': '/ultrasonic/front',
            'blockage_wait_time': 5.0,
        }]
    )

    # --- Motor Controller ---
    motor_controller = Node(
        package='my_robot_controller',
        executable='motor_controller2',
        output='screen',
        parameters=[{
            'track_width_m': 0.4572,
            'max_linear_mps': 0.50,
            'max_pwm': 0.20,
            'min_effective_pwm': 0.15,
            'pwm_slew_per_sec': 0.35,
            'left_motor_direction': -1.0,
            'right_motor_direction': -1.0,
        }]
    )

    # --- Wheel Encoder Odometry ---
    # These are BCM GPIO numbers. Adjust them to match your Pi 5 wiring.
    encoder_odometry = Node(
        package='my_robot_controller',
        executable='encoder_odometry',
        output='screen',
        parameters=[{
            'left_encoder_a_pin': 5,
            'left_encoder_b_pin': 6,
            'right_encoder_a_pin': 13,
            'right_encoder_b_pin': 19,
            'encoder_pull_up': False,
            'ticks_per_wheel_rev': 735.0,
            'wheel_radius_m': 0.075,
            'track_width_m': 0.4572,
            'publish_tf': False,
        }]
    )

    # --- UART task bridge -> Nav2 goals ---
    uart_bridge = Node(
        package='my_robot_controller',
        executable='uart_bridge',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rplidar,
        rf2o,
        slam,
        nav2,
        obstacle_avoidance,
        motor_controller,
        encoder_odometry,
        uart_bridge,
    ])
