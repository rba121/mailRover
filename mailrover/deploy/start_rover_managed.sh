#!/usr/bin/env bash

set -Ee -o pipefail

# ROS setup files expect some environment variables to be unset.
set +u
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
set -u

declare -a PIDS=()
declare -a NAMES=()

cleanup() {
    echo "Stopping rover base processes..."

    # Terminate each complete ROS process group, including child processes.
    for pid in "${PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done

    sleep 2

    for pid in "${PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_process() {
    local name="$1"
    shift

    echo "Starting $name..."

    # Start each ROS component in a separate process group.
    /usr/bin/setsid "$@" &
    PIDS+=("$!")
    NAMES+=("$name")

    sleep 1

    if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
        echo "ERROR: $name failed during startup."
        return 1
    fi
}

check_processes() {
    local index
    local pid
    local name
    local status

    for index in "${!PIDS[@]}"; do
        pid="${PIDS[$index]}"
        name="${NAMES[$index]}"

        if ! kill -0 "$pid" 2>/dev/null; then
            status=0
            wait "$pid" || status=$?

            echo "ERROR: $name exited with status $status."
            return 1
        fi
    done
}

start_process \
    lidar \
    ros2 launch sllidar_ros2 sllidar_s2_launch.py

start_process \
    robot_state_publisher \
    ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(xacro \
        /home/mypi/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"

start_process \
    joint_state_publisher \
    ros2 run joint_state_publisher joint_state_publisher

echo "Rover base processes started."

while true; do
    sleep 2
    check_processes
done
