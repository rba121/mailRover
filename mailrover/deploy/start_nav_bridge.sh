#!/usr/bin/env bash

set -Eeuo pipefail

export HOME=/home/mypi
cd /home/mypi/mailrover

set +u
source /opt/ros/jazzy/setup.bash

if [[ -f /home/mypi/ros2_ws/install/setup.bash ]]; then
    source /home/mypi/ros2_ws/install/setup.bash
fi

source /home/mypi/ros2_ws_simon/install/setup.bash
set -u

set -a
source /home/mypi/mailrover/.env
set +a

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROOM_MAP_PATH="${ROOM_MAP_PATH:-/home/mypi/scripts/rooms.yaml}"
export PYTHONUNBUFFERED=1

exec /usr/bin/python3 \
    /home/mypi/mailrover/scripts/ros2_nav_bridge.py
