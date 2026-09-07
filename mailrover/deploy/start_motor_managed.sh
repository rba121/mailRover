#!/usr/bin/env bash

set -Ee -o pipefail

# ROS setup scripts expect some variables to be initially unset.
set +u
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
set -u

ENCODER_SCRIPT=/home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
MOTOR_SCRIPT=/home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
FILTER_SCRIPT=/home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py

declare -a PIDS=()
declare -a NAMES=()

validate_files() {
    local file

    for file in \
        "$ENCODER_SCRIPT" \
        "$MOTOR_SCRIPT" \
        "$FILTER_SCRIPT"
    do
        if [[ ! -f "$file" ]]; then
            echo "ERROR: Required file is missing: $file" >&2
            return 1
        fi
    done
}

cleanup() {
    echo "Stopping motor-driver processes..."

    # Stop each complete process group, including child processes.
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

    /usr/bin/setsid "$@" &
    PIDS+=("$!")
    NAMES+=("$name")

    sleep 1

    if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
        echo "ERROR: $name failed during startup." >&2
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

            echo "ERROR: $name exited with status $status." >&2
            return 1
        fi
    done
}

validate_files

start_process \
    encoder_odometry \
    /usr/bin/python3 "$ENCODER_SCRIPT"

start_process \
    motor_controller_pid \
    /usr/bin/python3 "$MOTOR_SCRIPT"

start_process \
    scan_self_filter \
    /usr/bin/python3 "$FILTER_SCRIPT"

echo "Motor-driver processes started."

while true; do
    sleep 2
    check_processes
done
