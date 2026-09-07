# MailRover ROS2 Navigation Bridge

This connects the Flask app to autonomous ROS2/Nav2 movement.

## What It Does

1. The front desk enters the delivery destination, for example `Room 7001`.
2. When the scheduled delivery is dispatched, the Flask app sends the task to the local bridge at `http://127.0.0.1:8765/task`.
3. The bridge reads `maps/room_map.yaml`, converts `7001` into the saved x/y/yaw goal, and sends that goal to Nav2 using the `navigate_to_pose` action.
4. When Nav2 reaches the destination, the bridge reports `ARRIVED` back to the app at `/navigation/status`.
5. The app automatically sends the recipient arrival email with the one-time PIN.

## App `.env`

On the Pi, add or update these lines in `/home/mypi/mailrover/.env`:

```bash
NAV_BRIDGE_ENABLED=true
NAV_BRIDGE_URL=http://127.0.0.1:8765/task
NAV_BRIDGE_TIMEOUT_SEC=2
SERVICE_KEY=use_the_same_long_secret_for_app_and_bridge
```

Keep `SERVICE_KEY` the same for the Flask app and the ROS2 bridge.

## Start The Flask App

```bash
cd /home/mypi/mailrover
source .venv/bin/activate
set -a
source .env
set +a
sudo -E .venv/bin/python app.py
```

## Start The ROS2 Bridge

Open a second terminal on the Pi:

```bash
cd /home/mypi/mailrover
set -a
source .env
set +a
source /opt/ros/jazzy/setup.bash
source /home/mypi/ros2_ws/install/setup.bash
python3 scripts/ros2_nav_bridge.py
```

Nav2 must already be running and accepting goals on the `navigate_to_pose` action.

## Quick Test Without Dispatching From The App

With Flask and Nav2 running, send a room goal directly to the bridge:

```bash
curl -X POST http://127.0.0.1:8765/task \
  -H "Content-Type: application/json" \
  -H "X-MailRover-Service-Key: $SERVICE_KEY" \
  -d '{"task_id":"TEST7001","destination":"Room 7001","drawer_id":"D1"}'
```

The robot should navigate to room `7001`.

## Manual Arrival Callback Test

This only tests that the app can receive a navigation status:

```bash
curl -X POST http://127.0.0.1:8000/navigation/status \
  -H "Content-Type: application/json" \
  -H "X-MailRover-Service-Key: $SERVICE_KEY" \
  -d '{"task_id":"YOUR_ACTIVE_TASK_ID","status":"ARRIVED"}'
```

Replace `YOUR_ACTIVE_TASK_ID` with the active task id from the admin screen or logs.
