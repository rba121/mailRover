# MailRover Pi 5 Deployment

This guide assumes the Pi is reachable with:

```bash
ssh mypi@10.255.255.2
```

The app should be opened in a browser as an HTTP site. Use `/admin` for the
front desk/operator workflow.

## 1. Copy The App To The Pi

From your Mac, run this from the project folder:

```bash
rsync -av --exclude '.env' --exclude 'venv' --exclude '.venv' \
  "/Users/simon/Desktop/rover3 copy/" mypi@10.255.255.2:/home/mypi/mailrover/
```

Then SSH into the Pi:

```bash
ssh mypi@10.255.255.2
cd /home/mypi/mailrover
```

## 2. Create A Virtual Environment

On the Pi:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## 3. Create The Pi `.env`

On the Pi:

```bash
cp .env.example .env
nano .env
```

Minimum useful values:

```bash
MAILROVER_HOST=0.0.0.0
MAILROVER_PORT=8000
USE_REAL_GPIO_ACTUATORS=true
GPIO_CHIP_PATH=/dev/gpiochip4
D1_UNLOCK_GPIO=26
D1_LOCK_STATE_GPIO=16
SOLENOID_UNLOCK_ACTIVE_LOW=false
LOCK_STATE_LOCKED_HIGH=true
LOCK_FEEDBACK_REQUIRED=true

EMAIL_ENABLED=true
SMTP_HOST=smtp.resend.com
SMTP_PORT=587
SMTP_USERNAME=resend
SMTP_PASSWORD=your_resend_api_key
SMTP_USE_TLS=true
EMAIL_FROM="MailRover <robert@mailrover.com>"

MAILROVER_SALT=your_long_random_secret
ADMIN_PASSWORD=your_private_admin_password
SERVICE_KEY=your_long_random_service_key
```

Generate good secrets on the Pi:

```bash
python3 -c "import secrets; print(secrets.token_urlsafe(32))"
```

## 4. Run Manually For A First Test

```bash
bash run_pi.sh
```

Open:

```text
http://10.255.255.2:8000
```

Admin:

```text
http://10.255.255.2:8000/admin
```

## 5. Install As A Boot Service

On the Pi:

```bash
sudo cp /home/mypi/mailrover/deploy/mailrover.service.example /etc/systemd/system/mailrover.service
sudo systemctl daemon-reload
sudo systemctl enable mailrover
sudo systemctl start mailrover
sudo systemctl status mailrover
```

Logs:

```bash
sudo journalctl -u mailrover -f
```

Restart after changes:

```bash
sudo systemctl restart mailrover
```

## 6. Pi 5 Notes

- The web app can run on the Pi 5 immediately.
- Servo motors and PCA9685 are not used by this build.
- Real GPIO behavior depends on the solenoid lock, MOSFET, diode, and 12V supply wiring.
- For the 12V solenoid cabinet lock, follow `PI5_SOLENOID_LOCK.md`.
- For Pi UART, the likely device is `/dev/serial0` rather than the BBG path `/dev/ttyS1`.
- Keep `/admin` protected with `ADMIN_PASSWORD`.
- Keep dev/service actions protected with `SERVICE_KEY`.
