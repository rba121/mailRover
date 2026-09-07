#!/bin/bash

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

python app.py
