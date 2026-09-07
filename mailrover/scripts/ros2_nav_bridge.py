#!/usr/bin/env python3

import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


DEFAULT_ROOM_MAP = Path(__file__).resolve().parents[1] / "maps" / "room_map.yaml"


def yaw_to_quaternion(yaw: float):
    half = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def normalize_room(value: str) -> str:
    value = (value or "").strip()
    match = re.search(r"\d+", value)
    return match.group(0) if match else value


def normalize_alias(value: str) -> str:
    return re.sub(
        r"[^a-z0-9]+",
        "_",
        (value or "").strip().lower(),
    ).strip("_")


class MailRoverNavBridge(Node):
    def __init__(self):
        super().__init__("mailrover_nav_bridge")

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        self._active_task_id = None
        self._active_mission_type = None
        self._active_destination = None
        self._active_drawer_id = None
        self._active_generation = 0

        # Automatic Nav2 retry after an ABORTED result.
        self._abort_retry_delay_sec = max(
            0.5,
            float(
                os.environ.get(
                    "NAV_ABORT_RETRY_DELAY_SEC",
                    "2.0",
                )
            ),
        )
        try:
            self._max_recoveries = max(
                1,
                int(os.environ.get("NAV_MAX_RECOVERIES", "10")),
            )
        except (TypeError, ValueError):
            self._max_recoveries = 10
        self._recovery_counts = {}
        self._retry_timer = None

        # The currently accepted Nav2 action goal.
        self._active_goal_handle = None
        self._goal_send_pending_task_id = None

        # Operator return requests wait for the current goal cancellation
        # before the Home goal is dispatched.
        self._pending_return_home = {}
        self._pause_pending_tasks = set()

        # Tasks in this set were permanently canceled and must never retry.
        self._canceled_task_ids = set()

        self._lock = threading.Lock()

        self._room_map = self._load_room_map()
        self._rooms = self._room_map.get("rooms", {})

        self._app_status_url = os.environ.get(
            "MAILROVER_STATUS_URL",
            "http://127.0.0.1:8000/navigation/status",
        )

        self._service_key = os.environ.get(
            "SERVICE_KEY",
            "",
        )

        # Browser recovery controls publish the same conservative Twist
        # commands as keyboard_teleop.py. A short deadman timeout guarantees
        # a zero command if the browser, network, or app stops sending.
        self._teleop_publisher = self.create_publisher(
            Twist,
            os.environ.get("TELEOP_CMD_VEL_TOPIC", "/cmd_vel"),
            10,
        )
        self._teleop_enabled = False
        self._teleop_task_id = None
        self._teleop_deadline = 0.0
        self._teleop_motion_active = False
        self._teleop_timeout_sec = max(
            0.20,
            min(1.0, float(os.environ.get("TELEOP_DEADMAN_SEC", "0.35"))),
        )
        self.create_timer(0.10, self._check_teleop_deadman)

    def _publish_teleop(self, linear: float = 0.0, angular: float = 0.0):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self._teleop_publisher.publish(message)

    def _check_teleop_deadman(self):
        should_stop = False
        with self._lock:
            if (
                self._teleop_enabled
                and self._teleop_motion_active
                and time.monotonic() >= self._teleop_deadline
            ):
                self._teleop_motion_active = False
                should_stop = True
        if should_stop:
            self._publish_teleop()
            self.get_logger().warning("Manual recovery deadman stopped cmd_vel")

    def start_manual_teleop(self, task_id: str):
        with self._lock:
            if (
                self._active_task_id
                or self._goal_send_pending_task_id
                or self._pending_return_home
                or self._pause_pending_tasks
            ):
                return False, "NAVIGATION_ACTIVE"
            self._teleop_enabled = True
            self._teleop_task_id = task_id
            self._teleop_motion_active = False
            self._teleop_deadline = 0.0
        self._publish_teleop()
        self.get_logger().warning(f"Manual recovery enabled task={task_id}")
        return True, "TELEOP_STARTED"

    def command_manual_teleop(self, task_id: str, linear: float, angular: float):
        linear = max(-0.15, min(0.15, float(linear)))
        angular = max(-0.60, min(0.60, float(angular)))
        with self._lock:
            if not self._teleop_enabled or self._teleop_task_id != task_id:
                return False, "TELEOP_NOT_ACTIVE"
            self._teleop_deadline = time.monotonic() + self._teleop_timeout_sec
            self._teleop_motion_active = bool(linear or angular)
        self._publish_teleop(linear, angular)
        return True, "TELEOP_COMMAND_ACCEPTED"

    def stop_manual_teleop(self, task_id: str = None):
        with self._lock:
            if task_id and self._teleop_task_id not in {None, task_id}:
                return False, "TELEOP_TASK_MISMATCH"
            self._teleop_enabled = False
            self._teleop_task_id = None
            self._teleop_deadline = 0.0
            self._teleop_motion_active = False
        # Send zero more than once so the motor subscriber sees a definite stop.
        self._publish_teleop()
        self._publish_teleop()
        self.get_logger().warning(f"Manual recovery stopped task={task_id or '-'}")
        return True, "TELEOP_STOPPED"

    def _load_room_map(self):
        map_path = Path(
            os.environ.get(
                "ROOM_MAP_PATH",
                str(DEFAULT_ROOM_MAP),
            )
        )

        with map_path.open("r", encoding="utf-8") as file:
            room_map = yaml.safe_load(file) or {}

        rooms = room_map.get("rooms", {})

        self.get_logger().info(
            f"Loaded {len(rooms)} room navigation targets "
            f"from {map_path}"
        )

        if not room_map.get("loading_station"):
            self.get_logger().warning(
                "No loading_station target was found in the room map"
            )

        return room_map

    def target_for_destination(self, destination: str):
        room = normalize_room(destination)
        target = self._rooms.get(room)

        if target:
            return room, target

        alias = normalize_alias(destination)

        loading_aliases = {
            "home",
            "loading",
            "loading_station",
            "loading_bay",
            "load_station",
            "load_bay",
        }

        if alias in loading_aliases:
            return (
                "loading_station",
                self._room_map.get("loading_station"),
            )

        return room, None

    def dispatch_task(
        self,
        task_id: str,
        destination: str,
        drawer_id: str,
        mission_type: str = "delivery",
    ):
        mission_type = (
            mission_type or "delivery"
        ).strip().lower()

        if mission_type not in {"delivery", "return"}:
            self.get_logger().error(
                f"Invalid mission type: {mission_type!r}"
            )
            return False, "INVALID_MISSION_TYPE"

        with self._lock:
            if self._teleop_enabled:
                return False, "MANUAL_TELEOP_ACTIVE"
            if task_id in self._pause_pending_tasks:
                return False, "PAUSE_CONFIRMATION_PENDING"
            if task_id in self._canceled_task_ids:
                self.get_logger().warning(
                    f"Refusing canceled task={task_id}"
                )
                return False, "TASK_CANCELED"

        room, target = self.target_for_destination(destination)

        if not target:
            self.get_logger().error(
                f"No navigation target for "
                f"destination={destination!r} room={room!r}"
            )
            self.report_status(task_id, "FAULT")
            return False, f"ROOM_NOT_MAPPED:{room}"

        with self._lock:
            mission_key = (task_id, mission_type)
            if (
                self._active_task_id != task_id
                or self._active_mission_type != mission_type
            ):
                self._recovery_counts[mission_key] = 0
            self._active_task_id = task_id
            self._active_mission_type = mission_type
            self._active_destination = destination
            self._active_drawer_id = drawer_id
            self._active_goal_handle = None
            self._goal_send_pending_task_id = task_id
            self._active_generation += 1
            generation = self._active_generation

        self.get_logger().info(
            f"Dispatching task={task_id} "
            f"mission_type={mission_type} "
            f"drawer={drawer_id} "
            f"destination={destination} "
            f"room={room}"
        )

        if mission_type == "return":
            self.report_status(task_id, "RETURNING")
        else:
            self.report_status(task_id, "MOVING")

        if not self._action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "Nav2 navigate_to_pose action server "
                "is not available"
            )
            self.report_status(task_id, "FAULT")
            self._clear_active_mission(task_id)
            return (
                False,
                "NAV2_ACTION_SERVER_UNAVAILABLE",
            )

        goal = NavigateToPose.Goal()
        goal.pose = self._target_pose(target)

        future = self._action_client.send_goal_async(goal)

        future.add_done_callback(
            lambda result: self._goal_response_callback(
                task_id,
                mission_type,
                generation,
                result,
            )
        )

        return True, "GOAL_SENT"

    def _target_pose(
        self,
        target: dict,
    ) -> PoseStamped:
        pose = PoseStamped()

        pose.header.frame_id = os.environ.get(
            "NAV_FRAME_ID",
            "map",
        )

        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose.pose.position.x = float(target["x"])
        pose.pose.position.y = float(target["y"])
        pose.pose.position.z = 0.0

        quat = yaw_to_quaternion(
            float(target["yaw"])
        )

        pose.pose.orientation.x = quat["x"]
        pose.pose.orientation.y = quat["y"]
        pose.pose.orientation.z = quat["z"]
        pose.pose.orientation.w = quat["w"]

        return pose

    def _clear_active_mission(
        self,
        task_id: str = None,
    ):
        """
        Clear the mission and cancel a pending retry.
    
        A late result from an older task must not clear a newer task.
        """
        with self._lock:
            if (
                task_id
                and self._active_task_id not in {
                    None,
                    task_id,
                }
            ):
                return
    
            active_mission_key = (
                (self._active_task_id, self._active_mission_type)
                if self._active_task_id and self._active_mission_type
                else None
            )
            retry_timer = self._retry_timer
            self._retry_timer = None
            self._active_goal_handle = None
            if self._goal_send_pending_task_id == task_id or task_id is None:
                self._goal_send_pending_task_id = None
    
            if (
                task_id is None
                or self._active_task_id == task_id
            ):
                if active_mission_key:
                    self._recovery_counts.pop(active_mission_key, None)
                self._active_task_id = None
                self._active_mission_type = None
                self._active_destination = None
                self._active_drawer_id = None
                self._active_generation += 1
    
        if retry_timer is not None:
            retry_timer.cancel()
    
    
    def cancel_active_mission(
        self,
        task_id: str = None,
    ):
        """
        Cancel the current Nav2 goal and permanently suppress retries
        for the canceled task.
        """
        with self._lock:
            active_task_id = self._active_task_id
            active_mission_type = self._active_mission_type
    
            if (
                task_id
                and active_task_id
                and task_id != active_task_id
            ):
                return False, "TASK_NOT_ACTIVE"
    
            canceled_task_id = (
                active_task_id or task_id
            )
    
            if not canceled_task_id:
                return True, "NO_ACTIVE_MISSION"
    
            self._canceled_task_ids.add(
                canceled_task_id
            )
            self._pending_return_home.pop(canceled_task_id, None)
    
            retry_timer = self._retry_timer
            self._retry_timer = None
    
            goal_handle = self._active_goal_handle
            self._active_goal_handle = None
    
            if active_task_id == canceled_task_id:
                if active_mission_type:
                    self._recovery_counts.pop(
                        (canceled_task_id, active_mission_type),
                        None,
                    )
                self._active_task_id = None
                self._active_mission_type = None
                self._active_destination = None
                self._active_drawer_id = None
                self._active_generation += 1
    
        if retry_timer is not None:
            retry_timer.cancel()
    
        if goal_handle is not None:
            try:
                cancel_future = (
                    goal_handle.cancel_goal_async()
                )
    
                cancel_future.add_done_callback(
                    lambda future:
                    self._cancel_response_callback(
                        canceled_task_id,
                        future,
                    )
                )
    
            except Exception as exc:
                self.get_logger().error(
                    f"Could not request Nav2 cancellation "
                    f"task={canceled_task_id} "
                    f"error={exc}"
                )
    
        self.get_logger().warning(
            f"Navigation cancellation requested "
            f"task={canceled_task_id}"
        )
    
        self.report_status(
            canceled_task_id,
            "CANCELED",
        )
    
        return True, "CANCEL_REQUESTED"
    
    
    def _cancel_response_callback(
        self,
        task_id: str,
        future,
    ):
        try:
            response = future.result()
    
            goals_canceling = len(
                getattr(
                    response,
                    "goals_canceling",
                    [],
                )
            )
    
            self.get_logger().warning(
                f"Nav2 cancellation response "
                f"task={task_id} "
                f"goals_canceling={goals_canceling}"
            )
    
        except Exception as exc:
            self.get_logger().error(
                f"Nav2 cancellation response failed "
                f"task={task_id} "
                f"error={exc}"
            )

    def pause_active_mission(
        self,
        task_id: str = None,
    ):
        """Cancel motion without permanently canceling the MailRover task."""
        with self._lock:
            active_task_id = self._active_task_id
            active_mission_type = self._active_mission_type
            teleop_was_active = self._teleop_enabled
            if teleop_was_active:
                self._teleop_enabled = False
                self._teleop_task_id = None
                self._teleop_deadline = 0.0
                self._teleop_motion_active = False

            if task_id and active_task_id and task_id != active_task_id:
                return False, "TASK_NOT_ACTIVE"

            paused_task_id = active_task_id or task_id
            if not paused_task_id:
                return True, "NO_ACTIVE_MISSION"
            if paused_task_id in self._pause_pending_tasks:
                return False, "PAUSE_CONFIRMATION_PENDING"

            retry_timer = self._retry_timer
            self._retry_timer = None
            goal_handle = self._active_goal_handle
            self._active_goal_handle = None
            goal_send_was_pending = (
                self._goal_send_pending_task_id == paused_task_id
            )

            if active_task_id == paused_task_id:
                if active_mission_type:
                    self._recovery_counts.pop(
                        (paused_task_id, active_mission_type),
                        None,
                    )
                self._active_task_id = None
                self._active_mission_type = None
                self._active_destination = None
                self._active_drawer_id = None
                self._active_generation += 1

            if self._goal_send_pending_task_id == paused_task_id:
                self._goal_send_pending_task_id = None

            pause_confirmation_required = bool(
                goal_handle is not None or goal_send_was_pending
            )
            if pause_confirmation_required:
                self._pause_pending_tasks.add(paused_task_id)

        if retry_timer is not None:
            retry_timer.cancel()

        if teleop_was_active:
            self._publish_teleop()
            self._publish_teleop()

        if goal_handle is not None:
            try:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda future: self._pause_cancel_response_callback(
                        paused_task_id,
                        future,
                    )
                )
            except Exception as exc:
                with self._lock:
                    self._pause_pending_tasks.discard(paused_task_id)
                self.get_logger().error(
                    f"Could not pause Nav2 goal task={paused_task_id} "
                    f"error={exc}"
                )

        self.get_logger().warning(
            f"Navigation paused without canceling task={paused_task_id}"
        )
        if pause_confirmation_required:
            return False, "PAUSE_CONFIRMATION_PENDING"
        return True, "PAUSED"

    def _pause_cancel_response_callback(self, task_id: str, future):
        self._cancel_response_callback(task_id, future)
        with self._lock:
            self._pause_pending_tasks.discard(task_id)
        self.get_logger().warning(
            f"Navigation pause confirmed task={task_id}"
        )

    def request_return_home(
        self,
        task_id: str,
        drawer_id: str,
    ):
        """
        Cancel the current delivery goal and start Home only after Nav2 has
        processed that cancellation. This is separate from the E-stop path:
        the task remains active and is allowed to run its return mission.
        """
        with self._lock:
            if self._teleop_enabled:
                return False, "MANUAL_TELEOP_ACTIVE"
            if task_id in self._pause_pending_tasks:
                return False, "PAUSE_CONFIRMATION_PENDING"
            if task_id in self._canceled_task_ids:
                return False, "TASK_CANCELED"

            active_task_id = self._active_task_id
            if active_task_id and active_task_id != task_id:
                return False, "TASK_NOT_ACTIVE"

            if (
                active_task_id == task_id
                and self._active_mission_type == "return"
            ):
                return True, "ALREADY_RETURNING"

            if task_id in self._pending_return_home:
                return True, "RETURN_ALREADY_REQUESTED"

            self._pending_return_home[task_id] = drawer_id
            retry_timer = self._retry_timer
            self._retry_timer = None
            goal_handle = self._active_goal_handle
            self._active_goal_handle = None
            goal_send_pending = (
                self._goal_send_pending_task_id == task_id
            )

        if retry_timer is not None:
            retry_timer.cancel()

        if goal_handle is not None:
            self._cancel_goal_before_return_home(
                task_id,
                goal_handle,
            )
        elif not goal_send_pending:
            self._start_pending_return_home(task_id)

        self.get_logger().warning(
            f"Cancel-and-return requested task={task_id}"
        )
        return True, "CANCEL_RETURN_HOME_REQUESTED"

    def _cancel_goal_before_return_home(
        self,
        task_id: str,
        goal_handle,
    ):
        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._return_home_cancel_callback(
                    task_id,
                    future,
                )
            )
        except Exception as exc:
            self.get_logger().error(
                f"Could not cancel goal before Home task={task_id} "
                f"error={exc}"
            )
            self._start_pending_return_home(task_id)

    def _return_home_cancel_callback(
        self,
        task_id: str,
        future,
    ):
        try:
            response = future.result()
            goals_canceling = len(
                getattr(response, "goals_canceling", [])
            )
            self.get_logger().warning(
                f"Goal cancellation processed before Home "
                f"task={task_id} goals_canceling={goals_canceling}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Goal cancellation response failed before Home "
                f"task={task_id} error={exc}"
            )

        self._start_pending_return_home(task_id)

    def _start_pending_return_home(self, task_id: str):
        with self._lock:
            drawer_id = self._pending_return_home.pop(
                task_id,
                None,
            )
            if drawer_id is None or task_id in self._canceled_task_ids:
                return

            if self._active_task_id in {None, task_id}:
                if self._active_task_id and self._active_mission_type:
                    self._recovery_counts.pop(
                        (self._active_task_id, self._active_mission_type),
                        None,
                    )
                self._active_task_id = None
                self._active_mission_type = None
                self._active_destination = None
                self._active_drawer_id = None
                self._active_goal_handle = None
                if self._goal_send_pending_task_id == task_id:
                    self._goal_send_pending_task_id = None

        ok, reason = self.dispatch_task(
            task_id,
            "Home",
            drawer_id,
            mission_type="return",
        )
        if not ok:
            self.get_logger().error(
                f"Could not start Home goal task={task_id} reason={reason}"
            )
            self.report_status(task_id, "FAULT")

    def _schedule_aborted_retry(
        self,
        task_id: str,
        mission_type: str,
    ):
        """
        Schedule the same Nav2 destination again after an ABORTED result.
        """
        with self._lock:
            if task_id in self._canceled_task_ids:
                self.get_logger().warning(
                    f"Retry suppressed for canceled task={task_id}"
                )
                return
            if (
                self._active_task_id != task_id
                or self._active_mission_type != mission_type
            ):
                self.get_logger().warning(
                    f"Ignoring retry for stale mission "
                    f"task={task_id} "
                    f"mission_type={mission_type}"
                )
                return

            if self._retry_timer is not None:
                self.get_logger().warning(
                    f"Retry already scheduled for task={task_id}"
                )
                return

            mission_key = (task_id, mission_type)
            recovery_count = self._recovery_counts.get(mission_key, 0) + 1
            self._recovery_counts[mission_key] = recovery_count
            recovery_limit = self._max_recoveries
            limit_reached = recovery_count >= recovery_limit

            if not limit_reached:
                retry_timer = threading.Timer(
                    self._abort_retry_delay_sec,
                    self._retry_active_mission,
                    args=(task_id, mission_type),
                )
                retry_timer.daemon = True
                self._retry_timer = retry_timer
            else:
                retry_timer = None

        if limit_reached:
            aborting_status = (
                "RETURN_ABORTING"
                if mission_type == "return"
                else "ABORTING"
            )
            aborted_status = (
                "RETURN_ABORTED"
                if mission_type == "return"
                else "ABORTED"
            )
            self.get_logger().error(
                f"Navigation recovery limit reached task={task_id} "
                f"mission_type={mission_type} "
                f"recoveries={recovery_count}/{recovery_limit}"
            )
            self.report_status(
                task_id,
                aborting_status,
                recovery_count=recovery_count,
                recovery_limit=recovery_limit,
                reason="RECOVERY_LIMIT",
            )
            self.report_status(
                task_id,
                aborted_status,
                recovery_count=recovery_count,
                recovery_limit=recovery_limit,
                reason="RECOVERY_LIMIT",
            )
            self._clear_active_mission(task_id)
            return

        self.get_logger().warning(
            f"Nav2 goal aborted for task={task_id}; "
            f"recovery={recovery_count}/{recovery_limit}; "
            f"retrying the same destination in "
            f"{self._abort_retry_delay_sec:.1f} seconds"
        )

        self.report_status(
            task_id,
            "RETURN_RECOVERING"
            if mission_type == "return"
            else "RETRYING",
            recovery_count=recovery_count,
            recovery_limit=recovery_limit,
        )
        retry_timer.start()

    def _retry_active_mission(
        self,
        task_id: str,
        mission_type: str,
    ):
        """
        Resend the currently stored destination from the robot's current pose.
        """
        with self._lock:
            self._retry_timer = None
            if task_id in self._canceled_task_ids:
                self.get_logger().warning(
                    f"Retry canceled by E-stop/operator "
                    f"task={task_id}"
                )
                return

            if (
                self._active_task_id != task_id
                or self._active_mission_type != mission_type
            ):
                self.get_logger().warning(
                    f"Retry canceled because mission changed "
                    f"task={task_id}"
                )
                return

            destination = self._active_destination
            drawer_id = self._active_drawer_id

        if not destination:
            self.get_logger().error(
                f"Cannot retry task={task_id}: "
                "stored destination is missing"
            )
            self.report_status(task_id, "FAULT")
            self._clear_active_mission(task_id)
            return

        self.get_logger().warning(
            f"Resending Nav2 goal "
            f"task={task_id} "
            f"mission_type={mission_type} "
            f"destination={destination}"
        )

        ok, reason = self.dispatch_task(
            task_id,
            destination,
            drawer_id or "",
            mission_type,
        )

        if not ok:
            self.get_logger().error(
                f"Automatic retry failed "
                f"task={task_id} "
                f"reason={reason}"
            )
            self._clear_active_mission(task_id)

    def _goal_response_callback(
        self,
        task_id: str,
        mission_type: str,
        generation: int,
        future,
    ):
        with self._lock:
            stale_generation = generation != self._active_generation
        if stale_generation:
            self.get_logger().warning(
                f"Canceling stale goal response task={task_id} generation={generation}"
            )
            try:
                stale_goal_handle = future.result()
                if stale_goal_handle and stale_goal_handle.accepted:
                    cancel_future = stale_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(
                        lambda result: self._pause_cancel_response_callback(
                            task_id,
                            result,
                        )
                    )
                else:
                    with self._lock:
                        self._pause_pending_tasks.discard(task_id)
            except Exception as exc:
                with self._lock:
                    self._pause_pending_tasks.discard(task_id)
                self.get_logger().warning(
                    f"Stale goal could not be canceled task={task_id} error={exc}"
                )
            return
        try:
            goal_handle = future.result()
    
        except Exception as exc:
            with self._lock:
                if self._goal_send_pending_task_id == task_id:
                    self._goal_send_pending_task_id = None
                canceled = (
                    task_id in self._canceled_task_ids
                )
                return_requested = (
                    mission_type != "return"
                    and task_id in self._pending_return_home
                )

            if return_requested:
                self.get_logger().warning(
                    f"Delivery goal send failed while Home was requested; "
                    f"starting Home task={task_id}"
                )
                self._start_pending_return_home(task_id)
                return
    
            if canceled:
                self.get_logger().warning(
                    f"Goal-send result ignored for "
                    f"canceled task={task_id}"
                )
                return
    
            self.get_logger().error(
                f"Failed to send Nav2 goal "
                f"task={task_id} "
                f"mission_type={mission_type} "
                f"error={exc}"
            )
    
            self.report_status(task_id, "FAULT")
            self._clear_active_mission(task_id)
            return
    
        if not goal_handle.accepted:
            with self._lock:
                if self._goal_send_pending_task_id == task_id:
                    self._goal_send_pending_task_id = None
                canceled = (
                    task_id in self._canceled_task_ids
                )
                return_requested = (
                    mission_type != "return"
                    and task_id in self._pending_return_home
                )

            if return_requested:
                self.get_logger().warning(
                    f"Delivery goal was rejected while Home was requested; "
                    f"starting Home task={task_id}"
                )
                self._start_pending_return_home(task_id)
                return
    
            if canceled:
                self.get_logger().warning(
                    f"Rejected result ignored for "
                    f"canceled task={task_id}"
                )
                return
    
            self.get_logger().error(
                f"Nav2 rejected goal "
                f"task={task_id} "
                f"mission_type={mission_type}"
            )
    
            self.report_status(task_id, "FAULT")
            self._clear_active_mission(task_id)
            return
    
        with self._lock:
            if self._goal_send_pending_task_id == task_id:
                self._goal_send_pending_task_id = None
            canceled = (
                task_id in self._canceled_task_ids
            )
            return_requested = (
                mission_type != "return"
                and task_id in self._pending_return_home
            )

            stale = (
                self._active_task_id != task_id
                or self._active_mission_type
                != mission_type
            )
    
            if not canceled and not stale and not return_requested:
                self._active_goal_handle = (
                    goal_handle
                )

        if return_requested:
            self.get_logger().warning(
                f"Canceling newly accepted delivery goal before Home "
                f"task={task_id}"
            )
            self._cancel_goal_before_return_home(
                task_id,
                goal_handle,
            )
            return
    
        if canceled or stale:
            self.get_logger().warning(
                f"Canceling late/stale accepted goal "
                f"task={task_id}"
            )
    
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(
                    f"Could not cancel late goal "
                    f"task={task_id} "
                    f"error={exc}"
                )
    
            return
    
        self.get_logger().info(
            f"Nav2 accepted goal "
            f"task={task_id} "
            f"mission_type={mission_type}"
        )
    
        result_future = (
            goal_handle.get_result_async()
        )
    
        result_future.add_done_callback(
            lambda result:
            self._result_callback(
                task_id,
                mission_type,
                generation,
                result,
            )
        )
      

    def _result_callback(
        self,
        task_id: str,
        mission_type: str,
        generation: int,
        future,
    ):
        with self._lock:
            if generation != self._active_generation:
                self.get_logger().warning(
                    f"Ignoring stale Nav2 result task={task_id} generation={generation}"
                )
                return
        try:
            result = future.result()
    
        except Exception as exc:
            self.get_logger().error(
                f"Could not read Nav2 result "
                f"task={task_id} "
                f"mission_type={mission_type} "
                f"error={exc}"
            )
    
            self.report_status(task_id, "FAULT")
            self._clear_active_mission(task_id)
            return
    
        with self._lock:
            canceled = (
                task_id in self._canceled_task_ids
            )
            return_requested = (
                mission_type != "return"
                and task_id in self._pending_return_home
            )
    
            active = (
                self._active_task_id == task_id
                and self._active_mission_type
                == mission_type
            )
    
            if active:
                self._active_goal_handle = None

        if return_requested:
            self.get_logger().warning(
                f"Delivery goal finished while Home was requested "
                f"task={task_id} status={result.status}"
            )
            self._start_pending_return_home(task_id)
            return
    
        if canceled:
            self.get_logger().warning(
                f"Ignoring final Nav2 result for "
                f"canceled task={task_id} "
                f"status={result.status}"
            )
    
            self._clear_active_mission(task_id)
            return
    
        if not active:
            self.get_logger().warning(
                f"Ignoring stale Nav2 result "
                f"task={task_id} "
                f"status={result.status}"
            )
            return
    
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if mission_type == "return":
                self.get_logger().info(
                    f"Task returned to loading bay: "
                    f"{task_id}"
                )
    
                self.report_status(
                    task_id,
                    "RETURNED",
                )
    
            else:
                self.get_logger().info(
                    f"Task arrived at delivery "
                    f"destination: {task_id}"
                )
    
                self.report_status(
                    task_id,
                    "ARRIVED",
                )
    
            self._clear_active_mission(task_id)
            return
    
        if result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning(
                f"Navigation aborted "
                f"task={task_id} "
                f"mission_type={mission_type}"
            )
    
            self._schedule_aborted_retry(
                task_id,
                mission_type,
            )
            return
    
        if result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                f"Navigation canceled "
                f"task={task_id} "
                f"mission_type={mission_type}"
            )
    
            self.report_status(
                task_id,
                "CANCELED",
            )
    
            self._clear_active_mission(task_id)
            return
    
        self.get_logger().error(
            f"Navigation failed with an "
            f"unrecoverable result "
            f"task={task_id} "
            f"mission_type={mission_type} "
            f"status={result.status}"
        )
    
        self.report_status(task_id, "FAULT")
        self._clear_active_mission(task_id)
   
    def report_status(
        self,
        task_id: str,
        status: str,
        recovery_count: int = None,
        recovery_limit: int = None,
        reason: str = "",
    ):
        status_payload = {
            "task_id": task_id,
            "status": status,
        }
        if recovery_count is not None:
            status_payload["recovery_count"] = recovery_count
        if recovery_limit is not None:
            status_payload["recovery_limit"] = recovery_limit
        if reason:
            status_payload["reason"] = reason
        payload = json.dumps(status_payload).encode("utf-8")

        headers = {
            "Content-Type": "application/json",
            "X-MailRover-Service-Key": (
                self._service_key
            ),
        }

        request_object = urllib.request.Request(
            self._app_status_url,
            data=payload,
            headers=headers,
            method="POST",
        )

        try:
            with urllib.request.urlopen(
                request_object,
                timeout=2.0,
            ) as response:
                self.get_logger().info(
                    f"Reported {status} "
                    f"for task={task_id}: "
                    f"HTTP {response.status}"
                )

        except (
            urllib.error.URLError,
            TimeoutError,
        ) as exc:
            self.get_logger().error(
                f"Could not report {status} "
                f"for task={task_id}: {exc}"
            )


class BridgeRequestHandler(
    BaseHTTPRequestHandler
):
    bridge_node = None
    service_key = ""

    def do_POST(self):
        if self.path not in {
            "/task",
            "/cancel",
            "/pause",
            "/return-home",
            "/teleop/start",
            "/teleop/cmd",
            "/teleop/stop",
        }:
            self._send_json(
                404,
                {
                    "ok": False,
                    "error": "NOT_FOUND",
                },
            )
            return

        if self.service_key:
            provided = self.headers.get(
                "X-MailRover-Service-Key",
                "",
            )

            if provided != self.service_key:
                self._send_json(
                    401,
                    {
                        "ok": False,
                        "error": "SERVICE_KEY_REQUIRED",
                    },
                )
                return

        try:
            length = int(
                self.headers.get(
                    "Content-Length",
                    "0",
                )
            )

            raw_body = self.rfile.read(length)

            data = (
                json.loads(raw_body.decode("utf-8"))
                if raw_body
                else {}
            )

        except Exception:
            self._send_json(
                400,
                {
                    "ok": False,
                    "error": "INVALID_JSON",
                },
            )
            return

        if self.path.startswith("/teleop/"):
            task_id = (data.get("task_id") or "").strip()
            if not task_id:
                self._send_json(400, {"ok": False, "error": "TASK_ID_REQUIRED"})
                return

            if self.path == "/teleop/start":
                ok, reason = self.bridge_node.start_manual_teleop(task_id)
            elif self.path == "/teleop/stop":
                ok, reason = self.bridge_node.stop_manual_teleop(task_id)
            else:
                try:
                    linear = float(data.get("linear", 0.0))
                    angular = float(data.get("angular", 0.0))
                except (TypeError, ValueError):
                    self._send_json(400, {"ok": False, "error": "INVALID_VELOCITY"})
                    return
                if not math.isfinite(linear) or not math.isfinite(angular):
                    self._send_json(400, {"ok": False, "error": "INVALID_VELOCITY"})
                    return
                ok, reason = self.bridge_node.command_manual_teleop(
                    task_id,
                    linear,
                    angular,
                )

            self._send_json(
                200 if ok else 409,
                {"ok": ok, "message": reason, "task_id": task_id},
            )
            return


        if self.path == "/cancel":
            task_id = (
                data.get("task_id") or ""
            ).strip()

            ok, reason = (
                self.bridge_node.cancel_active_mission(
                    task_id or None
                )
            )

            self._send_json(
                200 if ok else 409,
                {
                    "ok": ok,
                    "message": reason,
                    "task_id": task_id,
                },
            )
            return

        if self.path == "/pause":
            task_id = (data.get("task_id") or "").strip()
            ok, reason = self.bridge_node.pause_active_mission(
                task_id or None
            )
            self._send_json(
                200 if ok else 409,
                {
                    "ok": ok,
                    "message": reason,
                    "task_id": task_id,
                },
            )
            return

        if self.path == "/return-home":
            task_id = (data.get("task_id") or "").strip()
            drawer_id = (data.get("drawer_id") or "D1").strip()

            if not task_id:
                self._send_json(
                    400,
                    {
                        "ok": False,
                        "error": "TASK_ID_REQUIRED",
                    },
                )
                return

            ok, reason = self.bridge_node.request_return_home(
                task_id,
                drawer_id,
            )
            self._send_json(
                202 if ok else 409,
                {
                    "ok": ok,
                    "message": reason,
                    "task_id": task_id,
                    "destination": "Home",
                },
            )
            return

        task_id = (
            data.get("task_id") or ""
        ).strip()

        destination = (
            data.get("destination") or ""
        ).strip()

        drawer_id = (
            data.get("drawer_id") or ""
        ).strip()

        mission_type = (
            data.get("mission_type")
            or "delivery"
        ).strip().lower()

        if not task_id or not destination:
            self._send_json(
                400,
                {
                    "ok": False,
                    "error": (
                        "TASK_ID_AND_DESTINATION_REQUIRED"
                    ),
                },
            )
            return

        if mission_type not in {
            "delivery",
            "return",
        }:
            self._send_json(
                400,
                {
                    "ok": False,
                    "error": "INVALID_MISSION_TYPE",
                },
            )
            return

        thread = threading.Thread(
            target=self.bridge_node.dispatch_task,
            args=(
                task_id,
                destination,
                drawer_id,
                mission_type,
            ),
            daemon=True,
        )

        thread.start()

        self._send_json(
            202,
            {
                "ok": True,
                "message": "TASK_ACCEPTED",
                "task_id": task_id,
                "destination": destination,
                "mission_type": mission_type,
            },
        )

    def log_message(self, fmt, *args):
        return

    def _send_json(
        self,
        status: int,
        payload: dict,
    ):
        body = json.dumps(payload).encode(
            "utf-8"
        )

        self.send_response(status)
        self.send_header(
            "Content-Type",
            "application/json",
        )
        self.send_header(
            "Content-Length",
            str(len(body)),
        )
        self.end_headers()

        try:
            self.wfile.write(body)
        except BrokenPipeError:
            pass


def main():
    rclpy.init()

    node = MailRoverNavBridge()

    BridgeRequestHandler.bridge_node = node
    BridgeRequestHandler.service_key = (
        os.environ.get("SERVICE_KEY", "")
    )

    host = os.environ.get(
        "NAV_BRIDGE_HOST",
        "127.0.0.1",
    )

    port = int(
        os.environ.get(
            "NAV_BRIDGE_PORT",
            "8765",
        )
    )

    server = ThreadingHTTPServer(
        (host, port),
        BridgeRequestHandler,
    )

    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )

    thread.start()

    node.get_logger().info(
        "MailRover navigation bridge listening "
        f"on http://{host}:{port} "
        "(/task, /cancel, /pause, /return-home, /teleop/*)"
    )

    try:
        rclpy.spin(node)

    finally:
        server.shutdown()
        node.stop_manual_teleop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
