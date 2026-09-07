#!/usr/bin/env bash

set -Eeuo pipefail

cd /home/mypi/mailrover

set -a
source /home/mypi/mailrover/.env
set +a

export PYTHONUNBUFFERED=1

exec /home/mypi/mailrover/.venv/bin/python \
    /home/mypi/mailrover/app.py
