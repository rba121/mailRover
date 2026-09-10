            "RETURN_RECOVERING"
            if mission_type == "return"
            else "RECOVERING"
        )

        self.get_logger().warning(
            f"Retrying task={task_id} after {reason}: "
            f"{retry_count}/{self._maximum_recoveries}"
        )

        self.report_status(
            task_id,
            retry_status,
            reason=reason,
            **details,
        )

        try:
            retry_delay = max(
                0.1,
                float(
                    os.environ.get(
                        "NAV_GOAL_RETRY_DELAY_SEC",
                        "1.0",
                    )
                ),
            )
        except ValueError:
            retry_delay = 1.0

        retry_timer = threading.Timer(
            retry_delay,
            self._send_retry_goal,
            args=(task_id, mission_type, target),
        )
        retry_timer.daemon = True
        retry_timer.start()

    def _send_retry_goal(
        self,
        task_id: str,
        mission_type: str,
        target: dict[str, Any],
    ) -> None:
        with self._lock:
            if (
                task_id != self._active_task_id
                or mission_type
                != self._active_mission_type
                or self._recovery_abort_requested
            ):
                return

        if not self._action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self._retry_active_goal(
                task_id,
                mission_type,
                "NAV2_ACTION_SERVER_UNAVAILABLE",
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._target_pose(target)

        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: (
                self._navigation_feedback_callback(
                    task_id,
                    mission_type,
                    feedback,
                )
            ),
        )

        future.add_done_callback(
            lambda result: self._goal_response_callback(
                task_id,
                mission_type,
                result,
            )
        )

    def _clear_active_task(self) -> None:
""",
    "retry helper methods",
)

replace_once(
    """    def _clear_active_task(self) -> None:
        with self._lock:
            self._active_task_id = None
            self._active_mission_type = None
            self._active_goal_handle = None
            self._recovery_abort_requested = False
""",
    """    def _clear_active_task(self) -> None:
        with self._lock:
            self._active_task_id = None
            self._active_mission_type = None
            self._active_goal_handle = None
            self._active_target = None
            self._recovery_count = 0
            self._goal_recovery_base = 0
            self._recovery_abort_requested = False
            self._last_pose = None
""",
    "clear retry state",
)

compile(text, str(path), "exec")
path.write_text(text, encoding="utf-8")

print("Retry patch applied successfully.")
print("Python syntax: OK")
PY

grep -nE '_active_target|_goal_recovery_base|def _retry_active_goal|def _send_retry_goal|Retrying task=' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover && python3 - <<'PY'
from pathlib import Path

path = Path("scripts/ros2_nav_bridge.py")
source = path.read_text(encoding="utf-8")
compile(source, str(path), "exec")

print("ros2_nav_bridge.py: syntax OK")
PY

sudo systemctl restart mailrover-product.service
sudo journalctl -u mailrover-product.service -n 80 --no-pager | grep -E 'MAILROVER PRODUCT READY|Ready: Flask|Traceback|ERROR|Failed|Exception'
sudo systemctl status mailrover-product.service --no-pager -l
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"route_recovery_count"|"recovery_limit"|"abort_notification"'
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -iE '"estop|"power_button|"fault|"lockout|"safe|"prompt'
grep -nC 10 -E 'UI communication lost|SAFE_HOLD|safe_hold' /home/mypi/mailrover/app.py
grep -RniC 12 -E 'def mark_ui_contact|class CommWatchdog|last_ui_ping|on_comm_restored' /home/mypi/mailrover/app.py /home/mypi/mailrover/services
sed -n '40,90p' /home/mypi/mailrover/services/watchdog.py
sleep 2 && curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"drawer_prompt"|"safe_hold_prev_state"'
sed -n '1,45p' /home/mypi/mailrover/services/watchdog.py
sudo journalctl -u mailrover-product.service --since "10 minutes ago" --no-pager | grep -E 'COMM_LOST|COMM_RESTORED|Exception in thread|Traceback|watchdog'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"drawer_prompt"|"safe_hold_prev_state"'
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"nav_status"|"route_recovery_count"|"recovery_limit"|"abort_reason"|"drawer_prompt"'
ls
cp /home/mypi/mailrover/scripts/ros2_nav_bridge.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py.before_retry_counter_fix
python3 - <<'PY'
from pathlib import Path
import re

path = Path(
    "/home/mypi/mailrover/scripts/ros2_nav_bridge.py"
)

text = path.read_text(encoding="utf-8")

old_zero = "self._goal_recovery_base = 0"
zero_count = text.count(old_zero)

if zero_count != 3:
    raise RuntimeError(
        "Expected 3 goal recovery base reset lines, "
        f"found {zero_count}"
    )

text = text.replace(
    old_zero,
    "self._nav2_recovery_count = 0",
)

old_retry_assignment = (
    "self._goal_recovery_base = retry_count"
)

if text.count(old_retry_assignment) != 1:
    raise RuntimeError(
        "Could not uniquely locate retry counter assignment"
    )

text = text.replace(
    old_retry_assignment,
    "self._nav2_recovery_count = 0",
    1,
)

new_feedback_method = '''    def _navigation_feedback_callback(
        self,
        task_id: str,
        mission_type: str,
        feedback_message,
    ) -> None:
        feedback = feedback_message.feedback

        nav2_recovery_count = int(
            feedback.number_of_recoveries
        )

        pose = feedback.current_pose.pose

        current_pose = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
        }

        report_counter = False

        with self._lock:
            if task_id != self._active_task_id:
                return

            self._last_pose = current_pose

            if (
                nav2_recovery_count
                != self._nav2_recovery_count
            ):
                self._nav2_recovery_count = (
                    nav2_recovery_count
                )
                report_counter = True

            # This counts complete goal resubmissions only.
            goal_retry_count = self._recovery_count

        if not report_counter:
            return

        status = (
            "RETURN_RECOVERING"
            if mission_type == "return"
            else "RECOVERING"
        )

        self.get_logger().warning(
            f"Nav2 recovery task={task_id}: "
            f"internal={nav2_recovery_count}, "
            f"goal retries={goal_retry_count}/"
            f"{self._maximum_recoveries}"
        )

        self.report_status(
            task_id,
            status,
            recovery_count=goal_retry_count,
            recovery_limit=self._maximum_recoveries,
            nav2_recovery_count=nav2_recovery_count,
            mission_type=mission_type,
            **current_pose,
        )

'''

pattern = (
    r"    def _navigation_feedback_callback\(\n"
    r".*?"
    r"(?=    def _request_recovery_limit_cancel\()"
)

text, method_count = re.subn(
    pattern,
    new_feedback_method,
    text,
    count=1,
    flags=re.DOTALL,
)

if method_count != 1:
    raise RuntimeError(
        "Could not uniquely replace navigation feedback method"
    )

if "_goal_recovery_base" in text:
    raise RuntimeError(
        "Old combined recovery counter still remains"
    )

compile(text, str(path), "exec")
path.write_text(text, encoding="utf-8")

print("Recovery counters separated successfully.")
print("Python syntax: OK")
PY

grep -nE '_recovery_count|_nav2_recovery_count|_goal_recovery_base|def _navigation_feedback_callback|def _retry_active_goal' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover && python3 - <<'PY'
from pathlib import Path

path = Path("scripts/ros2_nav_bridge.py")
source = path.read_text(encoding="utf-8")
compile(source, str(path), "exec")

print("ros2_nav_bridge.py: syntax OK")
PY

sudo systemctl restart mailrover-product.service
sudo journalctl -u mailrover-product.service -n 80 --no-pager | grep -E 'MAILROVER PRODUCT READY|Ready: Flask|Traceback|ERROR|Failed|Exception'
sudo systemctl status mailrover-product.service --no-pager -l
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"nav_status"|"route_recovery_count"|"recovery_limit"|"abort_reason"'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"route_recovery_count"|"abort_reason"'
python3 - <<'PY'
import json
import urllib.request

with urllib.request.urlopen("http://127.0.0.1:8000/status") as response:
    status = json.load(response)

print("nav2_recovery_count:", status.get("nav2_recovery_count", "not exposed"))
print("recovery_count:", status.get("route_recovery_count", 0))
PYpython3 - <<'PY'
import json
import urllib.request

with urllib.request.urlopen("http://127.0.0.1:8000/status") as response:
    status = json.load(response)

print("nav2_recovery_count:", status.get("nav2_recovery_count", "not exposed"))
print("recovery_count:", status.get("route_recovery_count", 0))
PY

cd mailrover/
ls
cd scripts/
ls
ros2_nav_bridge.py.before_retry_counter_fix
sudo systemctl stop mailrover-product.service
sudo systemctl disable --now mailrover-product.service
systemctl is-active mailrover-product.service
systemctl is-enabled mailrover-product.service
pgrep -af 'motor_controller_pid.py|encoder_odometry.py|navigation_launch.py'
cd /home/mypi/mailrover
if grep -q '^ROS_DOMAIN_ID=' .env; then     sudo sed -i 's/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=42/' .env; else     echo 'ROS_DOMAIN_ID=42' | sudo tee -a .env; fi
grep '^ROS_DOMAIN_ID=' /home/mypi/mailrover/.env
sudo systemctl start mailrover-product.service
PID=$(pgrep -n -f '/nav2_amcl/amcl')
echo "AMCL PID: $PID"
sudo tr '\0' '\n' < "/proc/$PID/environ" | grep -E '^(ROS_DOMAIN_ID|ROS_AUTOMATIC_DISCOVERY_RANGE)='
ros2 daemon stop
ros2 topic list | grep -E '^/(map|amcl_pose|odom|tf|tf_static)$'
ros2 run tf2_ros tf2_echo map base_footprint
dpkg -l | grep ros-jazzy-rmw-cyclonedds-cpp
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
grep -nE 'RecoveryNode|RetryUntilSuccessful|number_of_retries|num_retries' /home/mypi/ros2_ws/behavior_trees/navigate_to_pose_backup_first.xml
cp -v /home/mypi/mailrover/scripts/ros2_nav_bridge.py.before_retry_counter_fix /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover && python3 - <<'PY'
from pathlib import Path

path = Path("scripts/ros2_nav_bridge.py")
compile(path.read_text(encoding="utf-8"), str(path), "exec")

print("Restored ros2_nav_bridge.py: syntax OK")
PY

sudo systemctl restart mailrover-product.service
sleep 40
sudo journalctl -u mailrover-product.service -n 100 --no-pager | grep -E 'MAILROVER PRODUCT READY|Ready: Flask|Ready: navigation bridge|Traceback|ERROR|Failed|Exception'
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"nav_status"|"route_recovery_count"|"recovery_limit"|"abort_reason"'
grep -nE '_goal_recovery_base|_nav2_recovery_count|def _retry_active_goal|NAV_MAX_RECOVERIES' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"nav_status"|"route_recovery_count"|"recovery_limit"|"abort_reason"|"comm_ok"'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"nav_status"|"route_recovery_count"|"abort_reason"|"comm_ok"'
cd scripts/
ls -lt --full-time /home/mypi/mailrover/scripts/ros2_nav_bridge.py*
sudo systemctl stop mailrover-product.service
grep -nE 'NAV_MAX_RECOVERIES|_recovery_count|number_of_recoveries|def _retry_active_goal' /home/mypi/mailrover/scripts/ros2_nav_bridge.py.backup-20260805-210905 || echo "No recovery-counter code found"
python3 - <<'PY'
from pathlib import Path

path = Path(
    "/home/mypi/mailrover/scripts/"
    "ros2_nav_bridge.py.backup-20260805-210905"
)

compile(path.read_text(encoding="utf-8"), str(path), "exec")
print("Backup syntax: OK")
PY

cp -v /home/mypi/mailrover/scripts/ros2_nav_bridge.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py.with_recovery_counter && cp -v /home/mypi/mailrover/scripts/ros2_nav_bridge.py.backup-20260805-210905 /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sudo systemctl restart mailrover-product.service
sleep 40
sudo journalctl -u mailrover-product.service -n 100 --no-pager | grep -E 'Ready: navigation bridge|Ready: Flask|MAILROVER PRODUCT READY|Traceback|ERROR|Failed|Exception'
sudo systemctl status mailrover-product.service --no-pager -l
grep -nE 'NAV_MAX_RECOVERIES|_goal_recovery_base|_recovery_count|number_of_recoveries|def _schedule_aborted_retry|STATUS_ABORTED|RETRYING' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sudo ss -ltnp | grep -E ':8000|:8765'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"nav_status"|"abort_reason"'
for file in /home/mypi/mailrover/scripts/ros2_nav_bridge.py*; do     echo;     echo "========== $(basename "$file") ==========";     grep -nE     'schedule_aborted_retry|retry_active_mission|retry_active_goal|STATUS_ABORTED|RETRYING|NAV_MAX_RECOVERIES|number_of_recoveries'     "$file" || echo "No retry/counter markers"; done
clear
sudo systemctl stop mailrover-product.service
sudo systemctl stop mailrover-product.service
sudo sed -i 's/<RecoveryNode number_of_retries="6" name="NavigateRecovery">/<RecoveryNode number_of_retries="10" name="NavigateRecovery">/' /home/mypi/ros2_ws/behavior_trees/navigate_to_pose_backup_first.xml
grep -n 'NavigateRecovery' /home/mypi/ros2_ws/behavior_trees/navigate_to_pose_backup_first.xml
nano /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover/scripts
ls
rm ros2_nav_bridge.py
nano ros2_nav_bridge.py
cd ..
ls
nano .env
nano app.py
cd scripts/
nano ros2_nav_bridge.py
python3 -m py_compile /home/mypi/mailrover/app.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sudo rm -rf /home/mypi/mailrover/__pycache__ /home/mypi/mailrover/scripts/__pycache__
python3 -m py_compile /home/mypi/mailrover/app.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py
ls -l /home/mypi/mailrover/app.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py /home/mypi/mailrover/templates/admin.html
grep -nE 'estop_monitor_loop|cancel_delivery_for_estop|send_cancel_to_nav_bridge' /home/mypi/mailrover/app.py
grep -nE 'cancel_active_mission|cancel_goal_async|"/cancel"|_active_goal_handle' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
clear
grep -nE 'cancel_active_mission|cancel_goal_async|"/cancel"|_active_goal_handle' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
grep -n 'updateEstopPopup(s, estop)' /home/mypi/mailrover/templates/admin.html || echo "E-stop popup call removed"
sudo systemctl start mailrover-product.service
sleep 40
sudo journalctl -u mailrover-product.service -n 180 --no-pager | grep -E 'Ready: navigation bridge|Ready: Flask|MAILROVER PRODUCT READY|ESTOP_MONITOR_STARTED|Traceback|ERROR|Failed|Exception'
sudo systemctl is-active mailrover-product.service
sudo ss -ltnp | grep -E ':8000|:8765'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"nav_status"|"abort_reason"'
cd /home/mypi/mailrover
python3 - <<'PY'
import os
import config

print(os.path.join(config.LOG_DIR, config.LOG_FILE))
PY

grep -n 'ESTOP_MONITOR_STARTED' logs/events.log | tail -n 5
grep -nE 'ESTOP_MONITOR_ERROR|ESTOP_NAV_CANCEL_FAIL|Traceback|Exception' logs/events.log | tail -n 20
grep -nE 'ESTOP_MONITOR_STARTED|ESTOP_ACTIVE_DELIVERY|ESTOP_DELIVERY_CANCELED|ESTOP_NAV_CANCEL' logs/events.log | tail -n 30
grep -nE 'ESTOP_MONITOR_STARTED|ESTOP_ACTIVE_DELIVERY|ESTOP_DELIVERY_CANCELED|ESTOP_NAV_CANCEL' logs/events.log | tail -n 30
cd /home/mypi/mailrover
tail -F logs/events.log | grep --line-buffered -E 'ESTOP_ACTIVE_DELIVERY|ESTOP_PACKAGE_CANCELED|ESTOP_DELIVERY_CANCELED|ESTOP_NAV_CANCEL'
cd /home/mypi/mailrover
tail -F logs/events.log | grep --line-buffered -E 'ESTOP_ACTIVE_DELIVERY|ESTOP_PACKAGE_CANCELED|ESTOP_DELIVERY_CANCELED|ESTOP_NAV_CANCEL'
sudo journalctl -fu mailrover-product.service | grep --line-buffered -E 'Navigation cancellation requested|Nav2 cancellation response|Navigation canceled|Retry suppressed|Retry canceled'
clear
"sudo ss -ltnp | grep -E ':8000|:8765'"
sudo systemctl start mailrover-product.service
sleep 40
sudo journalctl -u mailrover-product.service -n 150 --no-pager | grep -E 'Ready: navigation bridge|Ready: Flask|MAILROVER PRODUCT READY|Traceback|ERROR|Failed|Exception'
for i in 1 2 3 4 5; do     curl -s http://127.0.0.1:8000/status >/dev/null;     sleep 1; done
curl -s http://127.0.0.1:8000/status | python3 -m json.tool | grep -E '"task_state"|"comm_ok"|"nav_status"|"abort_reason"'
sudo ss -ltnp | grep -E ':8000|:8765'
sudo journalctl -fu mailrover-product.service
sudo journalctl -fu mailrover-product.service | grep --line-buffered -E 'Dispatching|accepted goal|Navigation aborted|retrying the same destination|Resending Nav2 goal|ARRIVED|RETURNED|CANCELED|FAULT|ERROR'
clear
grep -nE 'ESTOP|E_STOP|estop|emergency|abortAlert|SAFE_HOLD' /home/mypi/mailrover/app.py
grep -RInE 'ESTOP_GPIO|ESTOP_STOPPED_HIGH|estop|E_STOP|emergency' /home/mypi/mailrover/adapters /home/mypi/mailrover/services /home/mypi/mailrover/deploy 2>/dev/null
grep -nE 'goal_handle|cancel_goal_async|retry_timer|do_POST|self.path|STATUS_CANCELED' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
grep -nE 'ESTOP|E_STOP|estop|emergency|abortAlert|alert' /home/mypi/mailrover/templates/admin.html
sed -n '390,490p' /home/mypi/mailrover/app.py
sed -n '1780,1905p' /home/mypi/mailrover/app.py
sed -n '2160,2250p' /home/mypi/mailrover/app.py
sed -n '50,90p' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sed -n '240,490p' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
clear
sed -n '390,490p' /home/mypi/mailrover/app.py
sed -n '1780,1905p' /home/mypi/mailrover/app.py
sed -n '2160,2250p' /home/mypi/mailrover/app.py
sed -n '50,90p' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sed -n '240,490p' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t rover
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t robot
clear
grep -nA120 -B20 'def send_task_to_nav_bridge' /home/mypi/mailrover/app.py
grep -nA180 -B20 'navigation/status' /home/mypi/mailrover/app.py
ls
ls
cd mailrover/
ls
cd scripts/
ls
pwd
cd ..
pwd
cd mailrover/
nano app.py
ls
nano app.py
nano admin.html 
grep -nE 'estop_monitor_loop|cancel_delivery_for_estop|send_cancel_to_nav_bridge' /home/mypi/mailrover/app.py
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
sudo systemctl restart mailrover-product.service
grep -nE 'estop_monitor_loop|cancel_delivery_for_estop|send_cancel_to_nav_bridge' /home/mypi/mailrover/app.py
grep -nE 'cancel_active_mission|cancel_goal_async|"/cancel"|_active_goal_handle' /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
grep -nE 'NAV_BRIDGE_CANCEL_FAIL|ESTOP_NAV_CANCEL_PENDING|ESTOP_NAV_CANCEL_CONFIRMED' logs/events.log | tail -n 40
sudo systemctl stop mailrover-product.service
sudo systemctl is-active mailrover-product.service
pgrep -af 'app.py|ros2_nav_bridge.py'
sudo systemctl stop mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
ros2 topic info /scan -v
ros2 topic info /scan_filtered -v
ros2 topic info /odom -v
ros2 topic info /cmd_vel -v
sudo systemctl status mailrover-product.service --no-pager
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
ros2 daemon stop
ros2 daemon start
sleep 3
ros2 node list | grep -E 'sllidar|scan_self|encoder|motor_controller'
ros2 topic info /scan
ros2 topic info /scan_filtered
ros2 topic info /odom
timeout 5s ros2 topic echo /scan --once >/dev/null && echo "/scan OK" || echo "/scan FAILED"
timeout 5s ros2 topic echo /scan_filtered --once >/dev/null && echo "/scan_filtered OK" || echo "/scan_filtered FAILED"
timeout 5s ros2 topic echo /odom --once >/dev/null && echo "/odom OK" || echo "/odom FAILED"
ros2 topic hz /scan
ros2 topic hz /scan_filtered
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /scan_filtered
cd /home/mypi/mailrover
tail -F logs/events.log | grep --line-buffered -E 'ESTOP_DELIVERY_ABORT_LATCHED|NAV_BRIDGE_CANCEL_SEND|NAV_BRIDGE_CANCEL_FAIL|ESTOP_NAV_STATUS_IGNORED|ESTOP_MONITOR_ERROR'
sudo systemctl stop mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
sudo systemctl stop mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
cd mailrover/
nano admin.html 
sudo systemctl restart mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
cd mailrover/templates/
nano admin.html
sudo systemctl restart mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
nano admin.html
sudo systemctl restart mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
nano admin.html
cd ..
nano app.py
sudo systemctl restart mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
ros
ros2 topic list
PID=$(pgrep -n -f '/nav2_amcl/amcl')
echo "AMCL PID: $PID"
sudo tr '\0' '\n' < "/proc/$PID/environ" | grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_STATIC_PEERS|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE)='
printf 'DISCOVERY=%s\nDOMAIN=%s\nLOCALHOST_ONLY=%s\nRMW=%s\n' "${ROS_AUTOMATIC_DISCOVERY_RANGE:-UNSET}" "${ROS_DOMAIN_ID:-UNSET}" "${ROS_LOCALHOST_ONLY:-UNSET}" "${RMW_IMPLEMENTATION:-UNSET}"
clear
sudo tail -n 50 -F /var/log/mailrover/navigation.log
clear
./start_robot.sh 
clear
cd ros2_ws_simon
nano nav2_params.yaml
cat nav2_params.yaml
nano nav2_params.yaml
clear
cd ..
clear
./motor_driver.sh 
clear
cd ros2_ws_simon
nano nav2_params.yaml
sudo systemctl stop mailrover-product.service
sleep 5
sudo systemctl is-active mailrover-product.service
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
curl -s http://127.0.0.1:8000/status | python3 -m json.tool
curl -s -X POST http://127.0.0.1:8000/admin/delivery/return_to_loading -H "Content-Type: application/json" | python3 -m json.tool
curl -i -X POST http://127.0.0.1:8000/admin/delivery/return_to_loading -H "Content-Type: application/json" -d '{}'
sudo journalctl -u mailrover-product.service --since "2 minutes ago" --no-pager
curl -i -X POST http://127.0.0.1:8000/admin/delivery/return_to_loading -H "Content-Type: application/json" -d '{}'
python3 - <<'PY'
import yaml

path = "/home/mypi/mailrover/maps/room_map.yaml"

with open(path, "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

print(data["loading_station"])
PY

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {
  header: {frame_id: map},
  pose: {
    position: {
      x: 24.35503060927691,
      y: -44.681053265815905,
      z: 0.0
    },
    orientation: {
      x: 0.0,
      y: 0.0,
      z: -0.4838074403,
      w: 0.8751744744
    }
  }
}}" --feedback
curl -v http://127.0.0.1:8000/login
curl -v http://172.16.210.186:8000/login
sudo ss -ltnp | grep 8000
curl -I http://127.0.0.1:8000/login
curl -I http://172.16.210.186:8000/login
hostname -I
clear
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
clear
tmux kill-session -t robot
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t rover
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t rover
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t robot
ssh -J root@mailrover.ca mypi@10.255.255.2
cd mailrover/
cd scripts/
python3 ros2_nav_bridge.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t robot
clear
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
# Stop the current app with Control+C
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
python3 - <<'PY'
import sqlite3

database = "data/mailrover.sqlite3"

with sqlite3.connect(database) as connection:
    rows = connection.execute("""
        SELECT request_id, recipient_email, status
        FROM package_requests
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """).fetchall()

    print("Stale records:", rows)

    connection.execute("""
        UPDATE package_requests
        SET status = 'CANCELED'
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """)

    print("Updated:", len(rows))
PY

set -a
source .env
set +a
sudo -E .venv/bin/python app.py
cd /home/mypi/mailrover
sudo cp -a data/mailrover.sqlite3 data/mailrover.sqlite3.before_status_cleanup
sudo python3 - <<'PY'
import sqlite3

database = "/home/mypi/mailrover/data/mailrover.sqlite3"

with sqlite3.connect(database) as connection:
    rows = connection.execute("""
        SELECT request_id, recipient_email, status
        FROM package_requests
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """).fetchall()

    print("Stale records:", rows)

    connection.execute("""
        UPDATE package_requests
        SET status = 'CANCELED'
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """)

    print("Updated:", len(rows))
PY

sudo python3 - <<'PY'
import sqlite3

database = "/home/mypi/mailrover/data/mailrover.sqlite3"

with sqlite3.connect(database) as connection:
    print(connection.execute("""
        SELECT request_id, status
        FROM package_requests
        WHERE request_id IN (
            'MR1786074978333',
            'MR1786082564153'
        )
    """).fetchall())
PY

set -a
source .env
set +a
sudo -E .venv/bin/python app.py
clear
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
killall python3
cd /home/mypi/mailrover
chmod 600 .env
bash -n .env
python3 -m py_compile app.py scripts/ros2_nav_bridge.py
grep -c '^NAV_MAX_RECOVERIES=10$' .env
cd ..
clear
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
killall python3
cd ..
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
clear
cd mailrover/
ls
pwd
sudo systemctl stop mailrover-product.service
sleep 5
sudo systemctl is-active mailrover-product.service
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sleep 40
sudo systemctl is-active mailrover-product.service
cleaqr
clear
sudo systemctl stop mailrover-product.service
sleep 5
sudo systemctl is-active mailrover-product.service
clear
sudo tail -n 50 -F /var/log/mailrover/navigation.log
clear
cd scripts
nano startup.py
nano startup.sh
./startup.sh
chmod +x startup.sh
./startup.sh 
cd ..
clear
ros2 launch nav2_bringup localization_launch.py map:=/home/mypi/maps/TASCmap.yaml use_sim_time:=false params_file:=/home/mypi/ros2_ws_simon/nav2_params.yaml
clear
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list | sort | uniq -c | grep -E 'amcl|slam|map_server|ekf|localization|robot_state'
ros2 topic info /amcl_pose -v
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
sudo systemctl show mailrover-product.service   -p FragmentPath   -p WorkingDirectory   -p ExecStart
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
cd mailrover/
ls
cd scripts/
ls
cd ..
python3 app.py
ls
cd /home/mypi/mailrover
ls -la | grep -E 'venv|\.venv'
python3 -m pip --version
source venv/bin/activate
pip install -r requirements.txt
python3 app.py
source venv/bin/activate
which python3
sudo -E /home/mypi/mailrover/venv/bin/python3 app.py
sudo -E python3 app.py
cd /home/mypi
find . -maxdepth 3 -type f -path "*/bin/activate" 2>/dev/null
find . -maxdepth 3 -type f -path "*/bin/python3" 2>/dev/null
cd /home/mypi/mailrover
cat requirements.txt
cd /home/mypi/mailrover
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
sudo -E /home/mypi/mailrover/.venv/bin/python3 /home/mypi/mailrover/app.py
clear
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
python3 /home/mypi/mailrover/scripts/ros2_nav_bridge.py
cd ..
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
python3 /home/mypi/mailrover/scripts/ros2_nav_bridge.py
clear
sudo ss -ltnp | grep -E ':8000|:8765'
cd /home/mypi/mailrover
grep -E 'PACKAGE_RECEIVED_EMAIL_STATUS|EMAIL_SENT|EMAIL_SEND_FAIL|EMAIL_DEV_LOG' logs/events.log | tail -20
nano /home/mypi/mailrover/.env
grep -E 'EMAIL_SENT|EMAIL_SEND_FAIL' /home/mypi/mailrover/logs/events.log | tail -5
cd /home/mypi/mailrover
set -a
source .env
set +a
python3 - <<'PY'
import os
key = os.environ.get("SMTP_PASSWORD", "")
print("Key starts correctly:", key.startswith("re_"))
print("Key length:", len(key))
PY

sudo ss -ltnp | grep ':8000'
curl -s http://127.0.0.1:8000/status | python3 -c 'import json,sys; s=json.load(sys.stdin); print("Task:",s.get("active_task_id"),"State:",s.get("task_state"))'
python3 - <<'PY'
import sqlite3

database = "data/mailrover.sqlite3"

with sqlite3.connect(database) as connection:
    rows = connection.execute("""
        SELECT request_id, recipient_email, status
        FROM package_requests
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """).fetchall()

    print("Stale records:", rows)

    connection.execute("""
        UPDATE package_requests
        SET status = 'CANCELED'
        WHERE status IN ('DISPATCHED', 'ARRIVED', 'RETURNING')
    """)

    print("Updated:", len(rows))
PY

set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
kiall python3
clear
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd mailrover/
nano .env
ls
rm .env
nano .env
timeout 60 ros2 topic hz /scan
ros2 node list | sort | uniq -c | sort -nr
ros2 topic info /cmd_vel -v
ps -ef | grep -E '[s]llidar_node|[a]mcl|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[m]otor_driver'
ros2 topic info /scan -v
ros2 topic info /odom -v
cd mailrover/
ls
cd scripts/
ls
cd ..
cd maps/
ls
nano room_map.yaml 
cd ..
ls
cd scripts/
ls
sed -n '1,160p' /home/mypi/scripts/rooms.yaml
cd /home/mypi/mailrover
sudo nano .env
sudo fuser -k 8765/tcp 2>/dev/null || true
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 /home/mypi/mailrover/scripts/ros2_nav_bridge.py
nl -ba /home/mypi/scripts/rooms.yaml | sed -n '1,35p'
nano /home/mypi/scripts/rooms.yaml
python3 - <<'PY'
import yaml

path = "/home/mypi/scripts/rooms.yaml"

with open(path, encoding="utf-8") as file:
    data = yaml.safe_load(file)

required = {"7004", "7006", "7011", "7016"}
rooms = data.get("rooms", {})
missing = required - set(rooms)

print("YAML valid")
print("Loading station:", data.get("loading_station"))
print("Required rooms present:", not missing)
print("Missing:", sorted(missing))
PY

cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
sudo cp -a /home/mypi/scripts/rooms.yaml /home/mypi/scripts/rooms.yaml.before_home_fix
sudo python3 - <<'PY'
import yaml

path = "/home/mypi/scripts/rooms.yaml"

with open(path, encoding="utf-8") as file:
    data = yaml.safe_load(file) or {}

rooms = data.get("rooms", {})
loading = rooms.pop("loading bay", None)

if loading is None:
    raise SystemExit('Could not find "loading bay" inside rooms')

corrected = {
    "loading_station": loading,
    "rooms": rooms,
}

with open(path, "w", encoding="utf-8") as file:
    yaml.safe_dump(corrected, file, sort_keys=False)

print("Home moved to loading_station:", loading)
print("Room count:", len(rooms))
PY

python3 - <<'PY'
import yaml

path = "/home/mypi/scripts/rooms.yaml"

with open(path, encoding="utf-8") as file:
    data = yaml.safe_load(file)

print("Top-level keys:", list(data))
print("Loading station:", data.get("loading_station"))
print("Rooms:", sorted(data.get("rooms", {})))
PY

cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
sudo sed -i.bak 's/^NAV_MAX_RECOVERIES=.*/NAV_MAX_RECOVERIES=10/' .env
grep '^NAV_MAX_RECOVERIES=' .env
sudo fuser -k 8765/tcp 2>/dev/null || true
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
echo "Recovery limit: $NAV_MAX_RECOVERIES"
echo "Room map: $ROOM_MAP_PATH"
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
sudo sed -i.bak 's/^NAV_MAX_RECOVERIES=.*/NAV_MAX_RECOVERIES=10/' .env
grep '^NAV_MAX_RECOVERIES=' .env
nano .env
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
echo "Recovery limit: $NAV_MAX_RECOVERIES"
echo "Room map: $ROOM_MAP_PATH"
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
set -a
source .env
set +a
echo "$NAV_MAX_RECOVERIES"
nano .env
cd /home/mypi/mailrover
set -a
source .env
set +a
echo "$NAV_MAX_RECOVERIES"
sudo ss -ltnp | grep ':8765'
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
ros2 topic hz /scan
ros2 topic delay /scan
ros2 param get /amcl use_sim_time
ros2 param get /collision_monitor use_sim_time
ros2 param get /global_costmap/global_costmap use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
top -b -n1 | head -20
vcgencmd get_throttled
ros2 node list | grep -Ei 'lidar|scan|sllidar'
ps -ef | grep '[s]llidar'
ros2 topic info /scan -v
ros2 param dump /sllidar_node
sudo dmesg -T | grep -Ei 'usb|ttyUSB|disconnect|reset|cp210|serial' | tail -50
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_s2_launch.py
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
clear
timeout 60 ros2 topic hz /scan
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run joint_state_publisher joint_state_publisher
tmuc kill-session -t robot
tmux kill-session -t robot
tmux kill-session -t rover
ls
cd ros2_ws
ls
cd src
ls
cd my_robot_controller/
ls
cd my_robot_controller/
ls
cd ../../../
cd ..
ls
cd ros2_ws_simon/
ls
cd src
ls
cd my_robot_controller/
ls
cd my_robot_controller/
ls
nano scan_self_filter.py 
top
cd ~
ls
cd mailrover
ls
nano run_pi.sh 
nano app.py
cd scripts
ls
nano ros2_nav_bridge.py
cd ../..
ls
nano start_robot.sh 
ls
cd deploy
cd mailrover/deploy
ls
nano start_complete_product.sh
nano ../.env
nano start_complete_product.sh
echo "$START_ROVER"
nano start_complete_product.sh
nano product.env 
nano start_rover_managed.sh 
nano product.env 
nano start_motor_managed.sh 
nano start_complete_product.sh
nano publish_initial_pose.py
ls
nano start_complete_product.sh
nano ../scripts/ros2_nav_bridge.py
ros2 node list
cd mailrover/
nano .env
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
echo "Recovery limit: $NAV_MAX_RECOVERIES"
echo "Room map: $ROOM_MAP_PATH"
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
sudo fuser -v 8765/tcp
sudo fuser -k 8765/tcp
sleep 2
sudo ss -ltnp | grep ':8765' || echo "Bridge port is free"
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t rover
tmux kill-session -t robot
ros2 node list
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t robot
tmux kill-session -t rover
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t rover
ros2 list nodes
ros2 node list
ros2 run nav2_util lifecycle_bringup map_server
exit
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
clear
cd scripts
nano process_control.py 
nano rooms.yaml 
cd ..
echo "shut up bitches. keep quiet. i need silence" | piper --model ~/piper_models/en_US-amy-medium.onnx --volume 30.0 --output_raw | aplay -D plughw:2,0 -r 22050 -f S16_LE -t raw -
aplay -l
echo "shut up bitches. keep quiet. i need silence" | piper --model ~/piper_models/en_US-amy-medium.onnx --volume 30.0 --output_raw | aplay -D plughw:0,0 -r 22050 -f S16_LE -t raw -
echo "where is my parcel" | piper --model ~/piper_models/en_US-amy-medium.onnx --volume 30.0 --output_raw | aplay -D plughw:0,0 -r 22050 -f S16_LE -t raw -
clear
ros2 node list | grep map_server
ros2 lifecycle get /map_server
ros2 topic echo /map --once
clear
ros2 topic list
clear
ros2 launch nav2_bringup localization_launch.py map:=/home/mypi/maps/TASCmap.yaml use_sim_time:=false params_file:=/home/mypi/ros2_ws_simon/nav2_params.yaml
./motor_driver.sh 
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
sudo fuser -v 8765/tcp
sudo fuser -k 8765/tcp
sleep 2
sudo ss -ltnp | grep ':8765' || echo "Bridge port is free"
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
ros2 list nodes
ros2 node list
exit
ls
cd maps
ls
cat ASBmap.yaml
cat TASCmap.yaml
cat officemap.yaml
nano TASCmap.yaml
cat officemap.yaml
cat TASCmap.yaml
cp TASCmap.yaml test.yaml
cat test.yaml
truncate
truncate --help
ls -l
truncate --size=132 test.yaml
cat test.yaml
cat testmap.yaml
exit
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t robot
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
tmux kill-session -t rover
sudo fuser -v 8765/tcp
sudo fuser -k 8765/tcp
sleep 2
sudo ss -ltnp | grep ':8765' || echo "Bridge port is free"
cd /home/mypi/mailrover
set -a
source .env
set +a
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
ros2 node list
exit
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
ros2 run joint_state_publisher joint_state_publisher
ros2 launch sllidar_ros2 sllidar_s2_launch.py
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/mailrover_urdf/urdf/mail_rover.urdf.xacro)"
tmux kill-session -t rover
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/encoder_odometry.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/motor_controller_pid.py
python3 ~/ros2_ws_simon/src/my_robot_controller/my_robot_controller/scan_self_filter.py
tmux kill-session -t robot
sudo kill 481938
sleep 2
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sudo systemctl stop mailrover-product.service
sudo systemctl is-active mailrover-product.service
sudo systemctl reset-failed mailrover-product.service
sudo systemctl start mailrover-product.service
sudo systemctl restart mailrover-product.service
cd
cd mailrover/
nano .env
cd ..
cd scripts/
ls
pwd
cd ..
cd mailrover/
cd scripts/
ls
cd ..
cd ...
cd ..
clear
sudo systemctl cat mailrover-product.service
sed -n '1,240p' /home/mypi/scripts/startup.sh
sed -n '1,260p' /home/mypi/scripts/process_control.py
sed -n '1,260p' /home/mypi/scripts/robot_supervisor.py
sudo sed -n '1,400p' /home/mypi/mailrover/deploy/start_complete_product.sh
cd mailrover/
nano .env
cd /home/mypi/mailrover
sudo sed -i.bak 's/^NAV_MAX_RECOVERIES=.*/NAV_MAX_RECOVERIES=50/' .env
grep '^NAV_MAX_RECOVERIES=' .env
sudo fuser -k 8765/tcp 2>/dev/null || true
cd /home/mypi/mailrover
set -a
source .env
set +a
echo "Recovery limit: $NAV_MAX_RECOVERIES"
export ROOM_MAP_PATH=/home/mypi/scripts/rooms.yaml
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
python3 scripts/ros2_nav_bridge.py
sudo nano /home/mypi/mailrover/deploy/product.env
clear
sudo cp -a /home/mypi/mailrover/deploy/product.env /home/mypi/mailrover/deploy/product.env.before_test_map
sudo nano /home/mypi/mailrover/deploy/product.env
nano .env
grep -E '^(ROOM_MAP_PATH|NAV_MAX_RECOVERIES|ROS_DOMAIN_ID)=' /home/mypi/mailrover/.env
grep -E '^(NAV_BRIDGE_ENABLED|NAV_BRIDGE_URL)=' /home/mypi/mailrover/.env
sudo sed -n '401,800p' /home/mypi/mailrover/deploy/start_complete_product.sh
sudo sed -n '1,300p' /home/mypi/mailrover/deploy/publish_initial_pose.py
sudo sed -n '1,260p' /home/mypi/mailrover/deploy/start_rover_managed.sh
sudo sed -n '1,260p' /home/mypi/mailrover/deploy/start_motor_managed.sh
clear
sudo systemctl stop mailrover-product.service
mailrover_backup_tag=$(date +%Y%m%d-%H%M%S)
sudo cp app.py "app.py.before-$mailrover_backup_tag"
sudo cp templates/admin.html "templates/admin.html.before-$mailrover_backup_tag"
sudo cp scripts/ros2_nav_bridge.py "scripts/ros2_nav_bridge.py.before-$mailrover_backup_tag"
sudo cp deploy/start_complete_product.sh "deploy/start_complete_product.sh.before-$mailrover_backup_tag"
sudo cp deploy/product.env "deploy/product.env.before-$mailrover_backup_tag"
sudo install -m 0644 /tmp/app.py /home/mypi/mailrover/app.py
sudo install -m 0644 /tmp/admin.html /home/mypi/mailrover/templates/admin.html
sudo install -m 0755 /tmp/ros2_nav_bridge.py /home/mypi/mailrover/scripts/ros2_nav_bridge.py
sudo install -m 0644 /tmp/product.env /home/mypi/mailrover/deploy/product.env
sudo install -m 0644 /tmp/mailrover-product.service /etc/systemd/system/mailrover-product.service
cd /home/mypi/mailrover
sudo patch --dry-run -p0 < /tmp/mailrover-startup-safety.patch
cd /home/mypi/mailrover
sudo patch -p0 < /tmp/mailrover-startup-safety.patch
sudo bash -n deploy/start_complete_product.sh
sudo bash -n deploy/start_rover_managed.sh
sudo bash -n deploy/start_motor_managed.sh
PYTHONPYCACHEPREFIX=/tmp/mailrover-pycache python3 -m py_compile app.py scripts/ros2_nav_bridge.py
sudo systemctl disable --now robot-supervisor.service mailrover.service mailrover-nav.service 2>/dev/null || true
sudo systemctl daemon-reload
sudo systemctl enable mailrover-product.service
sudo systemctl start mailrover-product.service
sudo journalctl -u mailrover-product.service -f
sudo systemctl is-active mailrover-product.service
sudo ss -ltnp | grep -E ':8000|:8765'
clear
sudo systemctl disable --now mailrover-product.service
sudo systemctl cat mailrover.service
sudo systemctl cat mailrover-nav.service
sudo sed -n '1,260p' /home/mypi/mailrover/deploy/start_mailrover_app.sh
sudo sed -n '1,300p' /home/mypi/mailrover/deploy/start_nav_bridge.sh
sudo systemctl is-enabled mailrover-product.service mailrover.service mailrover-nav.service
sudo systemctl is-active mailrover-product.service mailrover.service mailrover-nav.service
cd /home/mypi/mailrover
sudo systemctl stop mailrover-product.service
sudo cp app.py app.py.before-home-confirm
sudo cp templates/admin.html templates/admin.html.before-home-confirm
sudo install -m 0644 /tmp/app.py /home/mypi/mailrover/app.py
sudo install -m 0644 /tmp/admin.html /home/mypi/mailrover/templates/admin.html
PYTHONPYCACHEPREFIX=/tmp/mailrover-check python3 -m py_compile app.py
sudo rm -f /run/mailrover/initial_pose_published
sudo systemctl start mailrover-product.service
sudo journalctl -u mailrover-product.service -f
sed -n '1,18p' /home/mypi/scripts/rooms.yaml
sudo grep -Ei 'failed|abort|recover|spin|backup|collision|progress|costmap|transform|stale|goal' /var/log/mailrover/navigation.log | tail -250
sudo tail -200 /var/log/mailrover/nav_bridge.log
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
export ROS_DOMAIN_ID=42
ls -l /home/mypi/start_rover.sh /home/mypi/motor_driver.sh
find /home/mypi -maxdepth 5 -type f -name 'start_rover.sh' -print
grep -RIl 'sllidar_s2_launch.py' /home/mypi --include='*.sh' 2>/dev/null
sed -n '1,220p' /home/mypi/start_robot.sh
sed -n '1,260p' /home/mypi/scripts/startup.sh
sudo systemctl stop mailrover-product.service
sudo install -m 0755 /tmp/start_simple_product.sh /home/mypi/mailrover/deploy/start_simple_product.sh
sudo bash -n /home/mypi/mailrover/deploy/start_simple_product.sh
tmux ls
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list
sudo ss -ltnp | grep -E ':8000|:8765' || true
sudo rm -f /run/mailrover/initial_pose_published
sudo systemctl daemon-reload
sudo systemctl start mailrover-product.service
sudo journalctl -u mailrover-product.service -f
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list | sort | uniq -c | sort -nr
tmux ls
ps -ef | grep -E '[s]llidar_node|[s]can_self_filter|[e]ncoder_odometry|[m]otor_controller_pid'
ros2 topic info /scan -v
ros2 topic info /scan_filtered -v
source ~/.bashrc
clear
./start_robot.sh 
./motor_driver.sh 
sudo reboot
./motor_driver.sh 
clear
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
date
ros2 topic info /scan
ros2 node list | grep lidar
ros2 node list 
clear
./start_robot.sh 
clear
cd maps
nano TASCmap.yaml
nano ASBmap.yaml
nano TASCmap.yaml
cd ..
ros2 topic echo /amcl_pose --once
clear
sudo tail -n 50 -F /var/log/mailrover/navigation.log
cd scripts
ls
nano startup.sh 
cat startup.sh 
nano startup.sh 
cat startup.sh 
nano startup.sh 
cat startup.sh 
nano startup.sh 
./startup.sh 
nano startup.sh 
cat  startup.sh 
nano startup.sh 
cd ..
top
exit
sudo systemctl status startup.service
sudo reboot
systemctl
sudo systemctl status startup.service
sudo systemctl status mailrover-nav.service
sudo systemctl status startup.service
sudo systemctl status mailrover-nav.service
sudo systemctl status startup.service
systemctl
sudo systemctl status apport.service
sudo systemctl status apparmor.service
ls
nano .bashrc
cd mailrover/
ls
cd deploy
ls
nano start_mailrover_app.sh
./start_mailrover_app.sh
nano start_mailrover_app.sh
systemctl
sudo systemctl status startup.service
top
sudo systemctl status mailrover-product.service
sudo systemctl enable mailrover-product
sudo systemctl status mailrover-product.service
sudo systemctl start mailrover-product.service
sudo systemctl status mailrover-product.service
sudo systemctl restart startup
sudo systemctl status mailrover-product.service
sudo systemctl status startup.service
sudo systemctl restart mailrover-product
sudo systemctl status startup.service
sudo systemctl status mailrover-product.service
sudo systemctl status startup.service
nano /etc/systemd/system/startup.service
sudo systemctl disable startup.service
nano /etc/systemd/system/startup.service
sudo nano /etc/systemd/system/startup.service
sudo systemctl enable startup.service
sudo systemctl status startup.service
sudo systemctl restart startup.service
sudo systemctl status startup.service
sudo systemctl start mailrover-product.service
sudo systemctl status mailrover-product.service
sudo nano /etc/systemd/system/mailrover-product.service
sudo systemctl restart mailrover-product.service
sudo systemctl daemon-reload
sudo systemctl restart mailrover-product.service
sudo systemctl status mailrover-product.service
sudo nano /etc/systemd/system/mailrover-product.service
sudo systemctl daemon-reload
sudo systemctl restart mailrover-product.service
sudo systemctl stop startup
cd ~/scripts
ls
./startup.sh 
sudo ./startup.sh 
nano startup.sh
sudo ./startup.sh 
nano startup.sh
./startup.sh
sudo systemctl status mailrover-product.service
sudo systemctl restart mailrover-product.service
sudo systemctl status mailrover-product.service
sudo systemctl stop mailrover-product.service
sudo nano /etc/systemd/startup.service
sudo systemctl stop startup.service
sudo systemctl start mailrover-product.service
sudo systemctl status mailrover-product.service
sudo nano /etc/systemd/system/mailrover-product.service
cd mailrover/deploy/
cd /home/mypi/mailrover
sudo systemctl stop mailrover.service
sudo cp app.py app.py.before-battery-display
sudo cp templates/admin.html templates/admin.html.before-battery-display
sudo cp templates/receiver.html templates/receiver.html.before-step6-display
sudo install -m 0644 /tmp/app.py app.py
sudo install -m 0644 /tmp/admin.html templates/admin.html
if grep -q 'const recipientComplete' templates/receiver.html; then     echo "Recipient completion change is already installed."; elif grep -q 'Delivery Status Updating' templates/receiver.html; then     sudo patch -p0 < /tmp/recipient-step6-final.patch; else     sudo patch -p0 < /tmp/recipient-status-cleanup.patch; fi
PYTHONPYCACHEPREFIX=/tmp/mailrover-check python3 -m py_compile app.py
sudo systemctl start mailrover.service
sudo systemctl status mailrover.service --no-pager -l
sudo nano /etc/systemd/system/mailrover.service
sudo systemctl status mailrover.service
sudo systemctl status mailrover-nav.service
sudo systemctl status startup.service
sudo nano /etc/systemd/system/startup.service
sudo systemctl daemon-reload
sudo systemctl status startup.service
sudo systemctl start startup.service
sudo systemctl status startup.service
sudo systemctl status mailrover.service
sudo systemctl stop startup
cd scripts
./startup.sh 
ls
vi startup.sh 
nano startup.sh 
i2cget 1 0x48 0x48 w
ip a
sudo nano /etc/rc.local
sudo systemctl status rc-local
sudo nano /etc/rc.local
nano startup.sh 
exit
sudo systemctl status mailrover.service --no-pager -l
sudo systemctl enable --now mailrover-nav.service mailrover.service
sudo systemctl restart startup.service
sudo systemctl status startup.service
top
exit
killall startup.sh
cd scripts
./startup.sh 
./startup.sh &
exit
sudo systemctl status startup.service
sudo systemctl disable startup.service
ls
sudo nano /etc/rc.local
top
exit
killall startup.sh
cd scripts/
ls
/startup.sh
/startup.sh &
./startup.sh 
clear
killall startup.sh
clear
killall oython3
killall python3
./startup.sh
clear
ls
clear
ls
./startup.sh 
clear
cd /home/mypi/mailrover
grep -n 'voice.announce' app.py
grep -E 'VOICE_|TASK_CREATE_SENT|NAV_BRIDGE_STATUS|AUTH_SUCCESS|AUTO_UNLOCK_GRANTED|DELIVERY_ABORTED' logs/events.log | tail -100
sudo journalctl -u mailrover.service --since "30 minutes ago" --no-pager | tail -100
clear
cd /home/mypi/mailrover
tail -80 logs/events.log
head -1 /home/mypi/.local/bin/piper
sudo systemctl show mailrover.service -p User -p Environment -p EnvironmentFiles
cd /home/mypi/mailrover
set -a
source .env
set +a
sudo env HOME=/home/mypi PYTHONUSERBASE=/home/mypi/.local "$PIPER_BIN" --model "$PIPER_MODEL" --volume "$VOICE_VOLUME" --output_raw <<< "Mail Rover voice environment test." | sudo aplay -D "$VOICE_ALSA_DEVICE" -r "$VOICE_SAMPLE_RATE" -f S16_LE -t raw -
cd /home/mypi/mailrover
sudo systemctl stop mailrover.service
sudo cp app.py app.py.before-removing-voice
sudo install -m 0644 /tmp/app.py app.py
sudo sed -i 's/^VOICE_ENABLED=.*/VOICE_ENABLED=false/' .env
sudo mv services/voice.py services/voice.py.disabled 2>/dev/null || true
PYTHONPYCACHEPREFIX=/tmp/mailrover-check python3 -m py_compile app.py
sudo systemctl start mailrover.service
sleep 3
sudo systemctl status mailrover.service --no-pager
cd ..
cd scripts/
nano rooms.yaml
cd /home/mypi/mailrover
sudo systemctl stop mailrover.service
sudo cp app.py app.py.before-7201
sudo cp templates/admin.html templates/admin.html.before-7201
sudo cp /home/mypi/scripts/rooms.yaml /home/mypi/scripts/rooms.yaml.before-7201
sudo install -m 0644 /tmp/app.py app.py
sudo install -m 0644 /tmp/admin.html templates/admin.html
sudo sed -i -E -e 's/^([[:space:]]*)"7004":/\1"7201":/' -e 's/^([[:space:]]*)7004:/\1"7201":/' /home/mypi/scripts/rooms.yaml
diff -u /home/mypi/scripts/rooms.yaml.before-7201 /home/mypi/scripts/rooms.yaml
clear
grep -nE '^[[:space:]]*"?\(7004|7201\)"?:' /home/mypi/scripts/rooms.yaml
python3 - <<'PY'
import yaml

path = "/home/mypi/scripts/rooms.yaml"
with open(path) as file:
    rooms = yaml.safe_load(file)["rooms"]

print("7201:", rooms.get("7201"))
print("7004 present:", "7004" in rooms)
PY

cd /home/mypi/mailrover
PYTHONPYCACHEPREFIX=/tmp/mailrover-check python3 -m py_compile app.py
sudo systemctl start mailrover.service
sleep 3
sudo systemctl status mailrover.service --no-pager
grep -nE '^[[:space:]]*"?\(7004|7201\)"?:' /home/mypi/scripts/rooms.yaml
python3 - <<'PY'
import yaml

path = "/home/mypi/scripts/rooms.yaml"
with open(path) as file:
    rooms = yaml.safe_load(file)["rooms"]

print("7201:", rooms.get("7201"))
print("7004 present:", "7004" in rooms)
PY

sudo systemctl is-active mailrover-nav.service
sudo systemctl restart mailrover-nav.service
sleep 3
sudo systemctl status mailrover-nav.service --no-pager
killall startup.sh
cd scripts
killall startup.sh
./startup.sh
killall startup.sh
./startup.sh &
top
ros node list
ros2 node list
cd scripts
ls
./startup.sh &
kilall startup.sh
killall startup.sh
killall python3
ros2 node list
sudo systemctl status
ls
cd ..
ls
cd scripts
ls
cd ..
cd /etc
ls
./startup.sh
cd ~/scripts
./startup.sh
sudo nano /etc/rc.local
killall python3
killall ros2
sudo nano /etc/rc.local
sudo reboot
ros2 node list
systemctl status
cd scripts
ls
./startup.sh &
exit
clear
cd scripts/
nano startup.sh 
pi@121#
sudo systemctl is-enabled mailrover.service mailrover-nav.service
sudo systemctl is-active mailrover.service mailrover-nav.service
sudo systemctl start mailrover-nav.service
sleep 3
sudo systemctl is-active mailrover-nav.service
sudo systemctl status mailrover-nav.service --no-pager -l
sudo ss -ltnp | grep ':8765'
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws_simon/install/setup.bash
ros2 node list | grep mailrover_nav_bridge
systemctl status
sudo systemctl mailrover-nav-bridge status
sudo systemctl status mailrover-nav-bridge
sudo systemctl status
clear
ros2 topic list
./scripts/startup.sh &
systemctl status
sudo systemctl status mailrover
sudo systemctl start mailrover
sudo systemctl status mailrover
exit
jobs -l
cd scripts
ls
gpioset 
gpiodetect
sudo gpiodetect
gpioset 4
gpioset 4 26=1
gpioset 4 26=0
gpioset 4 26=1
gpioset 4 26=0
sudo shutdown now
ls
nmcli connection show
ip a
iwconfig wlan0
nmcli device wifi
sudo systemctl status ssh
ip a
ping 192.168.1.108
sudo ufw status
ip route | grep default
sudo nmcli connection delete "TELUSWiFi7275"
nmcli device wifi
sudo nmcli connection delete TELUSWiFi7275
nmcli connection show
sudo nmcli connection delete "netplan-wlan0-TELUSWiFi7275"
ip a
sudo nmcli device wifi connect "TELUSWiFi7275" 
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8"
ip a
ip route | grep default
nmcli device status
ip route | grep default 
ip a
nmcli device wifi
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:BD:B8:Fe
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
sudo nmcli connection delete "netplan-wlan0-TELUSWiFi7275"
nmcli device wifi
nmcli connection show
sudo nmcli connection delete "TELUSWiFi7275"
nmcli device wifi
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
nmcli device wifi
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
nmcli connection show

nmcli connection show
nmcli device wifi
nmcli connection show
nmcli connection status
nmcli device wifi
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
nmcli device wifi
nmcli connection status
nmcli connection show
sudo raspi-config nonint do_wifi_country CA
sudo reboot
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
nmcli connection show
nmcli device wifi
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:bd:b8:fe
sudo nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8" bssid 70:97:41:BD:B8:FE
nmcli device wifi
nmcli connection show
ip a
ip route | grep default
nmcli connection device
nmcli connection show
nmcli device wifi
ip a
ip route | grep default
sudo nmcli connection delete "TELUSWiFi7275"
nmcli connection show
ls
cd Desktop
ls
cd config
cd ..
cd config
ls
cd ..
p
sudo poweroff
ls
nmcli wifi list
nmcli device show
nmcli wifishow
nmcli wifi show
nmcli wifi 
nmcli connection
nmcli device show
ip addr
ip - a
ip
ip addres
nmcli device wifi list
nmcli device wifi connect "TELUSWiFi7275" password "zvGtK6t5b8"
nmcli device wifi list
nmcli device show
nmcli device wifi list
nmcli device show
nmcli device wifi list
nmcli device show
nmcli connection
ip addr
sudo raspi-config
clear
sudo systemclt enable sssh --now
sudo systemctl enable sssh --now
sudo systemctl enable ssh --now
sudo ufw status
sudo systemctl status ssh
hostname -I
iwconfig
nmcli connection show --active
sudo apt install wireless-tools
iwconfig
nmcli connection show --active
iwconfig
ip addr
ip route
ip route 
ip addr
ls
git status
ls
cd mailrover
ls
cd data 
ls
cd ..
ls
cd ..
git add ,
git add .
git commit -m "Project backup"
git remote add origin https://github.com/rba121/mailrover.git
git branch -M main
git push -u origin main
git status
git push -u origin main
git push -u origin main:pi-home-backup
ls
cd motor_ws
ls
cd ..
find ~/motor_ws -name ".git"
ls
cd robot_update/
ls
cd scripts
cd ..
cd scripts
ls
cs ros2_ws_simon
ls
cd ..
cd ros2_ws_simon
ls
cd src
ls
cd my_robot_controller/
ls
cd my_robot_controller/
ls
cd ..
ls
cd ..
ls
sudo poweroff
ls
hostname -I
sudo poweroff
ls
rm frames_2026-06-30_19.07.23.*
ls
mv machineshop_map.* /maps
mv machineshop_map.* ~/maps
ls
cd maps
ls
cd ..
ls
cd debs
ls
cd ..
nmcli wifi
nmcli device show
ping localhost
ping 192.168.1.108
hostname -I
ping 192.168.68.96
clear
ls
cd ~
tar --exclude='*/build' --exclude='*/install' --exclude='*/log'   -czvf pi_backup_$(date +%Y%m%d).tar.gz   config Desktop debs mailrover maps motor_driver.sh motor_ws odom.py   pid.py piper_models robot_update ros2_ws ros2_ws_backups ros2_ws_simon   ros2_ws_wendy scripts start_robot.sh test_script.py
ls -lh pi_backup_*.tar.gz
hostname -I
scp mypi@192.198.68.96:~/pi_backup_20260908.tar.gz ~/D/pi
clear
ls
cd ros2_ws
nano commands 
cat commands
ls
cd src
ls
cd ..
clea
clear
ls
git checkout main
git pull origin main
