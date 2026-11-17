#!/bin/bash
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
kill $(lsof -ti tcp:8765) 2>/dev/null || true
sleep 0.3
open "http://localhost:8765/"
exec python3 -B ./server.py
