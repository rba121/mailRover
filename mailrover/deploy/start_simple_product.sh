#!/usr/bin/env bash

set -Eeuo pipefail

MAILROVER_DIR=/home/mypi/mailrover
LOG_DIR=/var/log/mailrover
POSE_MARKER=/run/mailrover/initial_pose_published

MAP_FILE=/home/mypi/maps/test.yaml
NAV_PARAMS=/home/mypi/ros2_ws_simon/nav2_params.yaml

HOME_X=24.15984153883082
HOME_Y=-45.741435235488034
HOME_YAW=0.486

declare -a CHILD_PIDS=()

log() {
    printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

cleanup() {
    trap - EXIT INT TERM
    log "Stopping the simple MailRover stack..."

    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done

    sleep 5

    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL "$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_as_mypi() {
    local name="$1"
    shift

    log "Starting $name..."

    /usr/sbin/runuser -u mypi -- /bin/bash -lc '
        source /opt/ros/jazzy/setup.bash
        source /home/mypi/ros2_ws/install/setup.bash
        source /home/mypi/ros2_ws_simon/install/setup.bash
        export ROS_DOMAIN_ID=42
        export PYTHONUNBUFFERED=1
        exec "$@"
    ' bash "$@" >>"$LOG_DIR/$name.log" 2>&1 &

    CHILD_PIDS+=("$!")
}

mkdir -p "$LOG_DIR" /run/mailrover

if [[ -e "$POSE_MARKER" ]]; then
    log "ERROR: Home pose was already initialized during this boot."
    log "Physically place the robot at Home, press the E-stop, remove:"
    log "$POSE_MARKER"
    exit 1
fi

log "Waiting 30 seconds for the Pi and connected hardware to settle..."
sleep 30

log "Starting MailRover using the original manual sequence."

# Equivalent to start_robot.sh, without its interactive tmux attachment.
start_as_mypi lidar \
    ros2 launch sllidar_ros2 sllidar_s2_launch.py

log "Starting robot_state_publisher..."
/usr/sbin/runuser -u mypi -- /bin/bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /home/mypi/ros2_ws/install/setup.bash
    source /home/mypi/ros2_ws_simon/install/setup.bash
    export ROS_DOMAIN_ID=42
    robot_description="$(xacro /home/mypi/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
    exec ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p "robot_description:=$robot_description"
' >>"$LOG_DIR/robot_state_publisher.log" 2>&1 &
CHILD_PIDS+=("$!")

start_as_mypi joint_state_publisher \
    ros2 run joint_state_publisher joint_state_publisher

sleep 20

# Start odometry and the motor controller first.
start_as_mypi encoder_odometry \
    python3 \
    /home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py

start_as_mypi motor_controller_pid \
    python3 \
    /home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py

sleep 15

# Start the scan filter separately and let /scan_filtered stabilize before
# localization or navigation is allowed to consume it.
start_as_mypi scan_self_filter \
    python3 \
    /home/mypi/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py

sleep 20

# Equivalent to Terminal 3.
start_as_mypi localization \
    ros2 launch nav2_bringup localization_launch.py \
    "map:=$MAP_FILE" \
    "use_sim_time:=false" \
    "params_file:=$NAV_PARAMS"
sleep 25

# Equivalent to publishing the Home pose manually.
log "Publishing the confirmed Home pose..."
/usr/sbin/runuser -u mypi -- /bin/bash -lc '
    source /opt/ros/jazzy/setup.bash
    source /home/mypi/ros2_ws/install/setup.bash
    source /home/mypi/ros2_ws_simon/install/setup.bash
    export ROS_DOMAIN_ID=42
    exec python3 "$@"
' bash \
    "$MAILROVER_DIR/deploy/publish_initial_pose.py" \
    --x "$HOME_X" \
    --y "$HOME_Y" \
    --yaw "$HOME_YAW"

touch "$POSE_MARKER"
sleep 10

# Equivalent to Terminal 4.
start_as_mypi navigation \
    ros2 launch my_robot_controller navigation_launch.py \
    "map:=$MAP_FILE" \
    "use_sim_time:=false" \
    "params_file:=$NAV_PARAMS"
sleep 25

# Equivalent to Terminal 5.
log "Starting the MailRover navigation bridge..."
/usr/sbin/runuser -u mypi -- /bin/bash -lc '
    cd /home/mypi/mailrover
    set -a
    source .env
    set +a
    source /opt/ros/jazzy/setup.bash
    source /home/mypi/ros2_ws/install/setup.bash
    source /home/mypi/ros2_ws_simon/install/setup.bash
    export ROS_DOMAIN_ID=42
    export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
    export PYTHONUNBUFFERED=1
    exec python3 scripts/ros2_nav_bridge.py
' >>"$LOG_DIR/nav_bridge.log" 2>&1 &
CHILD_PIDS+=("$!")
sleep 5

# Equivalent to Terminal 6. The app remains root so it can own the GPIO lines.
log "Starting the MailRover application..."
/bin/bash -lc '
    cd /home/mypi/mailrover
    set -a
    source .env
    set +a
    export PYTHONUNBUFFERED=1
    exec .venv/bin/python app.py
' >>"$LOG_DIR/flask_app.log" 2>&1 &
CHILD_PIDS+=("$!")
sleep 5

log "MAILROVER SIMPLE PRODUCT READY"

# Intentionally stay simple: systemd owns the complete process cgroup and
# stops every descendant together. Individual component output remains in
# /var/log/mailrover, just as it remained visible in separate terminals.
while true; do
    sleep 60
done
