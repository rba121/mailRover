#!/usr/bin/env python3
"""
robot_supervisor.py

UPDATE: your teammate wired GPIO 18 to the physical start/stop button:
  - value 1 = STOPPED  (motors unpowered)
  - value 0 = STARTED  (motors powered)

This script now watches that pin directly:
  - Button goes to STARTED (0) -> bring up the entire software stack, in
    order, waiting for each stage to actually be ready before starting the
    next.
  - Button goes to STOPPED (1) -> cleanly shut everything down.

This means the operator's ONLY interaction is the physical button (plus the
web UI once running) - no terminal, no SSH, ever.

Stages (in order), triggered on STARTED:
  1. start_rover.sh        (lidar, robot_state_publisher)
  2. motor_driver.sh       (encoder_odometry, motor_controller, scan_filter, joint_publisher)
  3. localization_launch   (nav2_bringup, against your saved map)
  4. set initial pose      (robot always starts docked at loading_station)
  5. navigation_launch     (your trimmed navigation_launch.py)
  6. ros2_nav_bridge.py    (web UI <-> Nav2 bridge, HTTP on port 8765)

If any stage fails to become ready within its timeout, startup aborts and
everything already started is torn down cleanly.

A background watchdog also detects any UNEXPECTED crash while the stack is
supposed to be running (not caused by the button) and auto-restarts the
whole stack without human intervention.

--------------------------------------------------------------------------
BEFORE RUNNING: fill in the CONFIG section below - script paths, GPIO pin,
and loading station pose need to match your actual setup.
--------------------------------------------------------------------------

Run manually to test:
  python3 robot_supervisor.py

Run automatically on every Pi boot (so it's watching the button from the
moment the Pi powers on) - see the systemd section in the comment block at
the bottom of this file.
"""

import subprocess
import signal
import sys
import time
import threading
from gpiozero import DigitalInputDevice
import yaml

# ---------------------------------------------------------------------------
# CONFIG - fill these in for your actual setup
# ---------------------------------------------------------------------------
# Same pin your teammate wired for the physical start/stop button:
# value 1 = STOPPED (motors unpowered), value 0 = STARTED (motors powered)
ESTOP_GPIO_PIN = 18

START_ROVER_SH = "/home/mypi/start_robot.sh"
MOTOR_DRIVER_SH = "/home/mypi/motor_driver.sh"

MAP_PATH = "/home/mypi/maps/TASCmap.yaml"
NAV_PARAMS_PATH = "/home/mypi/ros2_ws_simon/nav2_params.yaml"
NAV_LAUNCH_PACKAGE = "my_robot_controller"       # package containing navigation_launch.py
NAV_LAUNCH_FILE = "navigation_launch.py"         # your trimmed launch file

ROOMS_FILE = "/home/mypi/rooms.yaml"             # used to get loading_station pose

NAV_BRIDGE_SCRIPT = "/home/mypi/mailrover/scripts/ros2_nav_bridge.py"
NAV_BRIDGE_HOST = "127.0.0.1"
NAV_BRIDGE_PORT = 8765
# Environment variables ros2_nav_bridge.py expects - fill in real values:
NAV_BRIDGE_ENV = {
    "ROOM_MAP_PATH": "/home/mypi/mailrover/maps/room_map.yaml",
    "MAILROVER_STATUS_URL": "http://127.0.0.1:8000/navigation/status",
    "SERVICE_KEY": "",  # match whatever your web app expects
    "NAV_BRIDGE_HOST": NAV_BRIDGE_HOST,
    "NAV_BRIDGE_PORT": str(NAV_BRIDGE_PORT),
}

# Readiness timeouts, in seconds - generous since a busy Pi can be slow to
# bring things up; tune down once you've seen real timings.
TIMEOUT_LIDAR_RSP = 20
TIMEOUT_MOTOR_DRIVER = 20
TIMEOUT_LOCALIZATION = 25
TIMEOUT_INITIAL_POSE = 10
TIMEOUT_NAVIGATION = 25
TIMEOUT_NAV_BRIDGE = 10

SHUTDOWN_GRACE_PERIOD = 5  # seconds to wait after SIGINT before SIGKILL
# ---------------------------------------------------------------------------


class ProcessManager:
    """Tracks started subprocesses (in start order) so they can be torn
    down cleanly, in reverse order, on shutdown or on a failed startup."""

    def __init__(self):
        self.processes = []  # list of (name, Popen)

    def start(self, name, cmd, shell=False):
        print(f"[supervisor] Starting: {name}")
        proc = subprocess.Popen(
            cmd, shell=shell,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.processes.append((name, proc))
        return proc

    def shutdown_all(self):
        print("[supervisor] Shutting down all processes...")
        # SIGINT everything, reverse of start order (last started, first stopped)
        for name, proc in reversed(self.processes):
            if proc.poll() is None:
                print(f"[supervisor]  -> SIGINT {name}")
                proc.send_signal(signal.SIGINT)

        deadline = time.time() + SHUTDOWN_GRACE_PERIOD
        while time.time() < deadline:
            if all(proc.poll() is not None for _, proc in self.processes):
                break
            time.sleep(0.3)

        # Force kill anything still alive
        for name, proc in reversed(self.processes):
            if proc.poll() is None:
                print(f"[supervisor]  -> SIGKILL {name} (did not exit cleanly)")
                proc.kill()

        self.processes.clear()
        print("[supervisor] Shutdown complete.")


def wait_for_topic(topic, timeout, description=""):
    """Blocks until a message is seen on `topic`, or timeout. Returns bool."""
    print(f"[supervisor] Waiting for {description or topic} (timeout {timeout}s)...")
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            timeout=timeout, capture_output=True, text=True,
        )
        ok = result.returncode == 0 and bool(result.stdout.strip())
        print(f"[supervisor]   -> {'ready' if ok else 'NOT ready'}: {description or topic}")
        return ok
    except subprocess.TimeoutExpired:
        print(f"[supervisor]   -> TIMEOUT waiting for {description or topic}")
        return False


def wait_for_action_server(action_name, timeout, description=""):
    """Blocks until an action server appears in `ros2 action list`, or timeout."""
    print(f"[supervisor] Waiting for {description or action_name} (timeout {timeout}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ["ros2", "action", "list"], timeout=5, capture_output=True, text=True
            )
            if action_name in result.stdout:
                print(f"[supervisor]   -> ready: {description or action_name}")
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    print(f"[supervisor]   -> TIMEOUT waiting for {description or action_name}")
    return False


def wait_for_port(host, port, timeout, description=""):
    """Blocks until a TCP port accepts connections, or timeout. Returns bool.
    Used for ros2_nav_bridge.py, which is a plain HTTP server (no ROS2 topic
    to probe)."""
    import socket
    print(f"[supervisor] Waiting for {description or f'{host}:{port}'} (timeout {timeout}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                print(f"[supervisor]   -> ready: {description or f'{host}:{port}'}")
                return True
        except OSError:
            time.sleep(0.5)
    print(f"[supervisor]   -> TIMEOUT waiting for {description or f'{host}:{port}'}")
    return False


def publish_initial_pose():
    """
    Publishes the robot's known starting pose (always docked at loading_station)
    to /initialpose, replacing the manual '2D Pose Estimate' click in RViz.
    """
    import math
    with open(ROOMS_FILE, "r") as f:
        data = yaml.safe_load(f)
    station = data["loading_station"]

    qz = math.sin(station["yaw"] / 2.0)
    qw = math.cos(station["yaw"] / 2.0)

    msg = (
        "{header: {frame_id: 'map'}, "
        f"pose: {{pose: {{position: {{x: {station['x']}, y: {station['y']}, z: 0.0}}, "
        f"orientation: {{z: {qz}, w: {qw}}}}}, "
        "covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, "
        "0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.06853]}}"
    )

    print("[supervisor] Publishing initial pose at loading station...")
    try:
        subprocess.run(
            ["ros2", "topic", "pub", "--once", "/initialpose",
             "geometry_msgs/msg/PoseWithCovarianceStamped", msg],
            timeout=TIMEOUT_INITIAL_POSE, capture_output=True, text=True,
        )
        return True
    except subprocess.TimeoutExpired:
        return False


class RobotSupervisor:
    def __init__(self):
        self.pm = ProcessManager()
        self.running = False
        self.lock = threading.Lock()

    def start_all(self):
        with self.lock:
            if self.running:
                print("[supervisor] Already running - ignoring start request.")
                return
            self.running = True

        print("\n===== START sequence initiated =====")
        try:
            # 1. Lidar + robot_state_publisher
            self.pm.start("start_rover.sh", [START_ROVER_SH])
            if not wait_for_topic("/scan", TIMEOUT_LIDAR_RSP, "lidar /scan"):
                raise RuntimeError("Lidar did not come up in time")

            # 2. Encoder odometry, motor controller, scan filter, joint publisher
            self.pm.start("motor_driver.sh", [MOTOR_DRIVER_SH])
            if not wait_for_topic("/odom", TIMEOUT_MOTOR_DRIVER, "encoder odometry /odom"):
                raise RuntimeError("Motor driver stack did not come up in time")
            if not wait_for_topic("/scan_filtered", 10, "scan_filter /scan_filtered"):
                raise RuntimeError("scan_filter did not come up in time")

            # 3. Localization
            self.pm.start("localization_launch", [
                "ros2", "launch", "nav2_bringup", "localization_launch.py",
                f"map:={MAP_PATH}", "use_sim_time:=false",
                f"params_file:={NAV_PARAMS_PATH}",
            ])
            if not wait_for_topic("/map", TIMEOUT_LOCALIZATION, "map_server /map"):
                raise RuntimeError("Localization did not come up in time")

            # 4. Initial pose (robot always starts docked here - known pose)
            time.sleep(2.0)  # give AMCL a moment after map_server is ready
            if not publish_initial_pose():
                raise RuntimeError("Failed to publish initial pose")
            if not wait_for_topic("/amcl_pose", TIMEOUT_INITIAL_POSE, "AMCL pose"):
                raise RuntimeError("AMCL did not accept initial pose in time")

            # 5. Navigation
            self.pm.start("navigation_launch", [
                "ros2", "launch", NAV_LAUNCH_PACKAGE, NAV_LAUNCH_FILE,
                f"map:={MAP_PATH}", "use_sim_time:=false",
                f"params_file:={NAV_PARAMS_PATH}",
            ])
            if not wait_for_action_server("/navigate_to_pose", TIMEOUT_NAVIGATION,
                                           "Nav2 navigate_to_pose action server"):
                raise RuntimeError("Navigation stack did not come up in time")

            # 6. ROS2 <-> web app navigation bridge
            import os as _os
            bridge_env = _os.environ.copy()
            bridge_env.update(NAV_BRIDGE_ENV)
            print(f"[supervisor] Starting: ros2_nav_bridge")
            proc = subprocess.Popen(
                ["python3", NAV_BRIDGE_SCRIPT], env=bridge_env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            )
            self.pm.processes.append(("ros2_nav_bridge", proc))
            if not wait_for_port(NAV_BRIDGE_HOST, NAV_BRIDGE_PORT, TIMEOUT_NAV_BRIDGE,
                                  "ros2_nav_bridge HTTP server"):
                raise RuntimeError("ros2_nav_bridge did not come up in time")

            print("===== START sequence COMPLETE - robot is ready for deliveries =====\n")

        except RuntimeError as e:
            print(f"[supervisor] STARTUP FAILED: {e}")
            print("[supervisor] Tearing down partially-started stack for safety...")
            self.pm.shutdown_all()
            with self.lock:
                self.running = False

    def stop_all(self):
        with self.lock:
            if not self.running:
                print("[supervisor] Nothing running - ignoring stop request.")
                return
            self.running = False

        print("\n===== STOP requested =====")
        self.pm.shutdown_all()
        print("===== Robot fully stopped =====\n")

    def watchdog_loop(self, check_interval=5):
        """
        Runs forever. If any tracked process dies unexpectedly (i.e. we did
        NOT intentionally stop it), this is a demo-day crash - tear down
        whatever's left and automatically restart the entire stack, with no
        human needing to touch the Pi.
        """
        while True:
            time.sleep(check_interval)
            with self.lock:
                if not self.running:
                    continue  # intentionally stopped, nothing to watch

                dead = [name for name, proc in self.pm.processes if proc.poll() is not None]

            if dead:
                print(f"[supervisor] WATCHDOG: detected unexpected crash in: {dead}")
                print("[supervisor] WATCHDOG: restarting the entire stack automatically...")
                self.stop_all()
                time.sleep(2.0)
                self.start_all()
                if self.running:
                    print("[supervisor] WATCHDOG: recovery successful, stack is back up.")
                else:
                    print("[supervisor] WATCHDOG: recovery FAILED - stack did not come back up. "
                          "Manual intervention needed.")


def main():
    supervisor = RobotSupervisor()

    def handle_sigterm(signum, frame):
        print("\n[supervisor] Termination signal received, shutting down...")
        supervisor.stop_all()
        sys.exit(0)

    signal.signal(signal.SIGTERM, handle_sigterm)
    signal.signal(signal.SIGINT, handle_sigterm)

    # Watchdog runs for the lifetime of the process, auto-recovering from any
    # UNEXPECTED crash (not caused by the button) without human intervention.
    watchdog_thread = threading.Thread(target=supervisor.watchdog_loop, daemon=True)
    watchdog_thread.start()

    estop = DigitalInputDevice(ESTOP_GPIO_PIN)

    def on_started():
        print("[supervisor] Button -> STARTED. Bringing up the full stack...")
        threading.Thread(target=supervisor.start_all, daemon=True).start()

    def on_stopped():
        print("[supervisor] Button -> STOPPED. Shutting down the full stack...")
        threading.Thread(target=supervisor.stop_all, daemon=True).start()

    # value 0 = STARTED, value 1 = STOPPED
    estop.when_deactivated = on_started  # fires when value goes to 0
    estop.when_activated = on_stopped    # fires when value goes to 1

    print(f"[supervisor] Watching E-stop on GPIO{ESTOP_GPIO_PIN}. "
          f"Current state: {'STOPPED' if estop.value else 'STARTED'}")

    # If the script is (re)launched while the button is already in STARTED
    # position, bring the stack up immediately rather than waiting for a
    # fresh press.
    if not estop.value:
        on_started()

    try:
        signal.pause()
    except KeyboardInterrupt:
        handle_sigterm(None, None)


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------------
# Run this automatically on every Pi boot, so the whole stack (lidar, motor
# driver nodes, localization, navigation, web bridge) is always up and ready
# the moment the Pi finishes booting - no SSH, no manual terminals needed.
#
# The physical START/STOP button now DOES control software directly, via
# GPIO 18: pressing STOP tears the whole stack down, pressing START brings
# it back up. This service just needs to be running (watching that pin)
# from the moment the Pi boots - the button does the rest.
#
# 1. Create /etc/systemd/system/robot-supervisor.service:
#
#   [Unit]
#   Description=Robot Supervisor (auto-starts full ROS2 stack at boot)
#   After=network.target
#
#   [Service]
#   ExecStart=/usr/bin/python3 /home/mypi/robot_supervisor.py
#   ExecStop=/bin/kill -TERM $MAINPID
#   Restart=on-failure
#   RestartSec=5
#   User=mypi
#
#   [Install]
#   WantedBy=multi-user.target
#
# 2. Enable it:
#   sudo systemctl daemon-reload
#   sudo systemctl enable robot-supervisor.service
#   sudo systemctl start robot-supervisor.service
#
# 3. Check logs if needed:
#   journalctl -u robot-supervisor.service -f
#
# 4. To manually stop/restart the whole software stack without rebooting:
#   sudo systemctl restart robot-supervisor.service
# ---------------------------------------------------------------------------
