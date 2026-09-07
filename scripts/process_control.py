#!/usr/bin/env python3

import ctypes
import os
import signal
import subprocess
import sys
import threading
import time

from gpiozero import DigitalInputDevice

ESTOP_GPIO_PIN = 18

ROS_SETUP = "source /opt/ros/jazzy/setup.bash && source /home/mypi/ros2_ws_simon/install/setup.bash"

URDF_PATH = "~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro"
MAP_PATH = "/home/mypi/maps/TASCmap.yaml"
NAV_PARAMS_PATH = "/home/mypi/ros2_ws_simon/nav2_params.yaml"
NAV_LAUNCH_PACKAGE = "my_robot_controller"

RESPAWN_DELAY_SEC = 3.0   # wait this long before relaunching a crashed child
CHECK_INTERVAL_SEC = 2.0  # how often the monitor loop polls child health


def shell_cmd(cmd_str):
    """Wraps a command string so it runs with the ROS2 environment sourced."""
    return ["/bin/bash", "-c", f"{ROS_SETUP} && {cmd_str}"]


CHILDREN = [
    {
        "name": "lidar",
        "cmd": shell_cmd("ros2 launch sllidar_ros2 sllidar_s2_launch.py"),
        "wait_topic": "/scan",
        "timeout": 20,
    },
    {
        "name": "robot_state_publisher",
        "cmd": shell_cmd(
            f'ros2 run robot_state_publisher robot_state_publisher '
            f'--ros-args -p robot_description:="$(xacro {URDF_PATH})"'
        ),
        "wait_topic": "/robot_description",
        "timeout": 15,
    },
    {
        "name": "joint_state_publisher",
        "cmd": shell_cmd("ros2 run joint_state_publisher joint_state_publisher"),
        "wait_topic": "/joint_states",
        "timeout": 15,
    },
    {
        "name": "encoder_odometry",
        "cmd": shell_cmd("python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py"),
        "wait_topic": "/odom",
        "timeout": 20,
    },
    {
        "name": "motor_controller_pid",
        "cmd": shell_cmd("python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py"),
        "wait_topic": None,
        "timeout": 10,
    },
    {
        "name": "scan_self_filter",
        "cmd": shell_cmd("python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py"),
        "wait_topic": "/scan_filtered",
        "timeout": 15,
    },
    {
        "name": "localization",
        "cmd": shell_cmd(
            f"ros2 launch nav2_bringup localization_launch.py "
            f"map:={MAP_PATH} use_sim_time:=false params_file:={NAV_PARAMS_PATH}"
        ),
        "wait_topic": "/map",
        "timeout": 25,
    },
    {
        "name": "navigation",
        "cmd": shell_cmd(
            f"ros2 launch {NAV_LAUNCH_PACKAGE} navigation_launch.py "
            f"map:={MAP_PATH} use_sim_time:=false params_file:={NAV_PARAMS_PATH}"
        ),
        "wait_action": "/navigate_to_pose",
        "timeout": 25,
    },
]


def set_pdeathsig():
    """Runs in the child right after fork: 'if my parent dies, SIGTERM me
    automatically' - guarantees cleanup even if the parent gets kill -9'd."""
    PR_SET_PDEATHSIG = 1
    libc = ctypes.CDLL("libc.so.6")
    libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM)
    os.setsid()  


def wait_for_topic(topic, timeout):
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            timeout=timeout, capture_output=True, text=True,
        )
        return result.returncode == 0 and bool(result.stdout.strip())
    except subprocess.TimeoutExpired:
        return False


def wait_for_action(action_name, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = subprocess.run(["ros2", "action", "list"], timeout=5, capture_output=True, text=True)
            if action_name in result.stdout:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


class ChildProcess:
    def __init__(self, spec):
        self.spec = spec
        self.name = spec["name"]
        self.proc = None
        self.pgid = None

    def launch(self):
        print(f"[supervisor]   Launching child: {self.name}")
        self.proc = subprocess.Popen(
            self.spec["cmd"], preexec_fn=set_pdeathsig,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        self.pgid = os.getpgid(self.proc.pid)

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def kill(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def wait_ready(self):
        if self.spec.get("wait_topic"):
            print(f"[supervisor]   Waiting for {self.name} ({self.spec['wait_topic']})...")
            return wait_for_topic(self.spec["wait_topic"], self.spec["timeout"])
        if self.spec.get("wait_action"):
            print(f"[supervisor]   Waiting for {self.name} ({self.spec['wait_action']})...")
            return wait_for_action(self.spec["wait_action"], self.spec["timeout"])
        time.sleep(min(self.spec.get("timeout", 5), 5))
        return True


class ProcessSupervisor:
    def __init__(self):
        self.children = {}
        self.running = False
        self.lock = threading.Lock()

    def start_all(self):
        with self.lock:
            if self.running:
                print("[supervisor] Already running - ignoring start request.")
                return
            self.running = True

        print("\n===== START sequence =====")
        for spec in CHILDREN:
            child = ChildProcess(spec)
            child.launch()
            self.children[child.name] = child

            if not child.wait_ready():
                print(f"[supervisor] STARTUP FAILED: {child.name} did not become ready in time.")
                self.stop_all()
                return

        print("===== START sequence COMPLETE =====\n")

    def stop_all(self):
        with self.lock:
            if not self.running:
                print("[supervisor] Nothing running - ignoring stop request.")
                return
            self.running = False

        print("\n===== STOP - killing all children =====")
        for name, child in reversed(list(self.children.items())):
            print(f"[supervisor]   Stopping {name}")
            child.kill()
        self.children.clear()
        print("===== All children stopped =====\n")

    def monitor_loop(self):
        """Watches every child independently. If one dies unexpectedly,
        ONLY that one gets relaunched - everything else is left alone."""
        while True:
            time.sleep(CHECK_INTERVAL_SEC)
            with self.lock:
                if not self.running:
                    continue
                dead_names = [name for name, c in self.children.items() if not c.is_alive()]

            for name in dead_names:
                print(f"[supervisor] WATCHDOG: '{name}' died unexpectedly - respawning just this child...")
                time.sleep(RESPAWN_DELAY_SEC)
                spec = next(s for s in CHILDREN if s["name"] == name)
                new_child = ChildProcess(spec)
                new_child.launch()
                if new_child.wait_ready():
                    self.children[name] = new_child
                    print(f"[supervisor] WATCHDOG: '{name}' recovered.")
                else:
                    print(f"[supervisor] WATCHDOG: '{name}' failed to come back up. Will retry next cycle.")


def main():
    supervisor = ProcessSupervisor()

    def cleanup_and_exit(signum, frame):
        print("\n[supervisor] Parent exiting - cleaning up all children...")
        supervisor.stop_all()
        sys.exit(0)

    signal.signal(signal.SIGTERM, cleanup_and_exit)
    signal.signal(signal.SIGINT, cleanup_and_exit)

    monitor_thread = threading.Thread(target=supervisor.monitor_loop, daemon=True)
    monitor_thread.start()

    estop = DigitalInputDevice(ESTOP_GPIO_PIN)

    def on_started():
        threading.Thread(target=supervisor.start_all, daemon=True).start()

    def on_stopped():
        threading.Thread(target=supervisor.stop_all, daemon=True).start()

    estop.when_deactivated = on_started 
    estop.when_activated = on_stopped   

    print(f"[supervisor] Watching GPIO{ESTOP_GPIO_PIN}. "
          f"Current: {'STOPPED' if estop.value else 'STARTED'}")
    if not estop.value:
        on_started()

    try:
        signal.pause()
    except KeyboardInterrupt:
        cleanup_and_exit(None, None)


if __name__ == "__main__":
    main()
