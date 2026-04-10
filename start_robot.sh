#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Kill any existing session
tmux kill-session -t rover 2>/dev/null

# Start new session
tmux new-session -d -s rover -n main

# Pane 1 - robot state publisher
tmux send-keys -t rover 'ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"' Enter

# Pane 2 - joint state publisher
tmux split-window -t rover
tmux send-keys -t rover 'ros2 run joint_state_publisher joint_state_publisher' Enter

# Pane 3 - rplidar
#tmux split-window -t rover
#tmux send-keys -t rover 'ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0' Enter

# Wait for rplidar to fully initialize
sleep 10

# Pane 4 - rf2o odometry
tmux split-window -t rover
tmux send-keys -t rover 'ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args -p laser_scan_topic:=/scan -p base_frame_id:=base_footprint -p odom_frame_id:=odom -p laser_frame_id:=laser -p publish_tf:=true -p freq:=10.0' Enter

tmux attach -t rover
