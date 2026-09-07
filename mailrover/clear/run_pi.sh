#!/bin/bash
set -e

cd "$(dirname "$0")"

if [ -d ".venv" ]; then
  source .venv/bin/activate
elif [ -d "venv" ]; then
  source venv/bin/activate
fi

if [ -f ".env" ]; then
  set -a
  source .env
  set +a
fi

if [ -x ".venv/bin/python" ]; then
  exec .venv/bin/python app.py
fi

exec python3 app.py
