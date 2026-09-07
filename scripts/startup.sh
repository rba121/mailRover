#!/bin/bash
# start_all.sh
#
# Runs every command in order. Each one starts in the background, then we
# wait until it's actually ready (checking a topic) before starting the
# next one. Press Ctrl+C to stop everything cleanly.

set -e

source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash

MAP_PATH="/home/mypi/maps/test.yaml"
NAV_PARAMS="/home/mypi/ros2_ws_simon/nav2_params.yaml"

PIDS=()

cleanup() {
    echo ""
    echo "Stopping everything..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    exit 0
}
trap cleanup SIGINT SIGTERM

wait_for_topic() {
    local topic=$1
    local timeout=$2
    echo "Waiting for $topic..."
    local waited=0
    while ! ros2 topic echo "$topic" --once --timeout 2 &>/dev/null; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$timeout" ]; then
            echo "TIMEOUT waiting for $topic"
            return 1
        fi
    done
    echo "$topic is ready."
    return 0
}

echo "===== Starting lidar ====="
ros2 launch sllidar_ros2 sllidar_s2_launch.py &
PIDS+=($!)
wait_for_topic "/scan" 20 || { cleanup; exit 1; }

echo "===== Starting robot_state_publisher ====="
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)" &
PIDS+=($!)
sleep 3

echo "===== Starting joint_state_publisher ====="
ros2 run joint_state_publisher joint_state_publisher &
PIDS+=($!)
wait_for_topic "/joint_states" 15 || { cleanup; exit 1; }

echo "===== Starting encoder_odometry ====="
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py &
PIDS+=($!)
wait_for_topic "/odom" 20 || { cleanup; exit 1; }

echo "===== Starting motor_controller_pid ====="
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py &
PIDS+=($!)
sleep 3

echo "===== Starting scan_self_filter ====="
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py &
PIDS+=($!)
wait_for_topic "/scan_filtered" 15 || { cleanup; exit 1; }

echo "===== Starting localization ====="
ros2 launch nav2_bringup localization_launch.py \
    map:="$MAP_PATH" use_sim_time:=false params_file:="$NAV_PARAMS" &
PIDS+=($!)
wait_for_topic "/map" 25 || { cleanup; exit 1; }

sleep 3
echo "===== Setting initial pose at loading station ====="
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 24.15984153883082, y: -45.741435235488034, z: 0.0}, orientation: {z: 0.24072647263611352, w: 0.9705929967664997}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}"

echo "Waiting for AMCL to accept the pose..."
wait_for_topic "/amcl_pose" 15 || { cleanup; exit 1; }
echo "AMCL pose confirmed - map -> odom transform is live."


echo "===== Starting navigation ====="
ros2 launch my_robot_controller navigation_launch.py \
    map:="$MAP_PATH" use_sim_time:=false params_file:="$NAV_PARAMS" &
PIDS+=($!)
sleep 10

echo ""
echo "===== ALL SYSTEMS UP - press Ctrl+C to stop everything ====="
wait
