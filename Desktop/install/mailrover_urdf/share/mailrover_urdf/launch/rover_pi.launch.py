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

    # --- Motor Controller ---
    motor_controller = Node(
        package='my_robot_controller',
        executable='motor_controller4',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rplidar,
        rf2o,
        slam,
        motor_controller,
    ])
