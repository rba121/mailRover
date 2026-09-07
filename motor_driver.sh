#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws_simon/install/setup.bash

# Kill any existing session
tmux kill-session -t robot 2>/dev/null

# Start new session
tmux new-session -d -s robot -n main

# Pane 1 - Encoder Odometry
tmux send-keys -t robot "python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py" Enter

# Pane 2 - Motor Controller PID
tmux split-window -t robot
tmux send-keys -t robot "python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py" Enter

# Pane 3 - Scan Filter
tmux split-window -t robot
tmux send-keys -t robot "python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py" Enter

#tmux select-layout -t rover tiled

tmux attach -t robot
