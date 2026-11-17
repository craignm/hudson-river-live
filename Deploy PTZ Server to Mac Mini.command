#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

REMOTE_HOST="${MAC_MINI_HOST:-craignm@192.168.1.175}"
OVERWRITE_CALIB="${1:-}"

echo "===================================================================="
echo "  Deploying Hudson River PTZ Director Server -> $REMOTE_HOST"
echo "===================================================================="

# 1) Verify Python 3.9 compatibility before shipping
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B -m py_compile \
  server.py aisstream_client.py obs_camera_bridge.py ptz_drivers.py

# 2) Stream clean app bundle over SSH (no Dropbox or Homebrew on Mac mini)
echo "1) Syncing server & static UI bundle..."
COPYFILE_DISABLE=1 tar -cf - \
  server.py \
  aisstream_client.py \
  obs_camera_bridge.py \
  ptz_drivers.py \
  calibration.json \
  static/index.html \
  static/app.js \
  static/snapshot.jpg \
  $( [ -f .env ] && echo ".env" ) \
| ssh -o BatchMode=yes "$REMOTE_HOST" '
  set -euo pipefail
  APP_DIR="$HOME/Library/Application Support/HudsonRiverPTZ"
  STAGE_DIR="$APP_DIR/.stage_deploy"
  rm -rf "$STAGE_DIR"
  mkdir -p "$STAGE_DIR" "$APP_DIR/static"
  tar -xf - -C "$STAGE_DIR"

  rm -f "$APP_DIR/lan_camera_client.py"
  cp -f "$STAGE_DIR/server.py" \
        "$STAGE_DIR/aisstream_client.py" \
        "$STAGE_DIR/obs_camera_bridge.py" \
        "$STAGE_DIR/ptz_drivers.py" \
        "$APP_DIR/"
  cp -f "$STAGE_DIR/static/index.html" \
        "$STAGE_DIR/static/app.js" \
        "$STAGE_DIR/static/snapshot.jpg" \
        "$APP_DIR/static/"

  if [ -f "$STAGE_DIR/.env" ]; then
    cp -f "$STAGE_DIR/.env" "$APP_DIR/.env"
    chmod 600 "$APP_DIR/.env"
  fi

  # Preserve existing Mac mini calibration tweaks unless missing or --overwrite-calib
  if [ ! -f "$APP_DIR/calibration.json" ] || [ "'"$OVERWRITE_CALIB"'" = "--overwrite-calib" ]; then
    cp -f "$STAGE_DIR/calibration.json" "$APP_DIR/calibration.json"
  fi

  # Always point obs_url on the Mac mini at localhost loopback obs-websocket
  /usr/bin/python3 - "$APP_DIR/calibration.json" << "PYEOF"
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
cal = json.loads(p.read_text())
url = str(cal.get("obs_url") or cal.get("remote_cam_url") or "")
cal.pop("remote_cam_url", None)
if not url or "192.168.1.175:4455" in url or "craigs-mac-mini" in url.lower():
    cal["obs_url"] = "obs://127.0.0.1:4455"
else:
    cal["obs_url"] = url
p.write_text(json.dumps(cal, indent=2) + "\n")
PYEOF

  rm -rf "$STAGE_DIR"
'

# 3) Seed warm NY Harbor AIS static vessel profile cache if not yet on Mac mini
LOCAL_AIS_CACHE="$HOME/Library/Caches/HudsonRiverPTZ/ais_cache.json"
if [ -f "$LOCAL_AIS_CACHE" ]; then
  echo "2) Checking warm AIS vessel cache on Mac mini..."
  ssh -o BatchMode=yes "$REMOTE_HOST" '
    mkdir -p "$HOME/Library/Caches/HudsonRiverPTZ"
    if [ ! -s "$HOME/Library/Caches/HudsonRiverPTZ/ais_cache.json" ]; then
      cat > "$HOME/Library/Caches/HudsonRiverPTZ/ais_cache.json"
      echo "   -> Seeded warm ais_cache.json onto Mac mini."
    else
      cat > /dev/null
      echo "   -> Existing ais_cache.json already present on Mac mini (preserved)."
    fi
  ' < "$LOCAL_AIS_CACHE"
fi

# 4) Install & restart LaunchAgent on Mac mini
echo "3) Configuring & restarting LaunchAgent live.hudsonriver.ptz-server..."
ssh -o BatchMode=yes "$REMOTE_HOST" '
  set -euo pipefail
  APP_DIR="$HOME/Library/Application Support/HudsonRiverPTZ"
  PLIST_LABEL="live.hudsonriver.ptz-server"
  PLIST_PATH="$HOME/Library/LaunchAgents/$PLIST_LABEL.plist"
  LOG_PATH="$HOME/Library/Logs/hudson-ptz-server.log"
  mkdir -p "$HOME/Library/LaunchAgents" "$HOME/Library/Logs"

  cat > "$PLIST_PATH" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>$PLIST_LABEL</string>
    <key>ProgramArguments</key>
    <array>
        <string>/usr/bin/python3</string>
        <string>-B</string>
        <string>-u</string>
        <string>$APP_DIR/server.py</string>
    </array>
    <key>WorkingDirectory</key>
    <string>$APP_DIR</string>
    <key>EnvironmentVariables</key>
    <dict>
        <key>PYTHONDONTWRITEBYTECODE</key>
        <string>1</string>
        <key>PORT</key>
        <string>8765</string>
        <key>OBS_URL</key>
        <string>obs://127.0.0.1:4455</string>
        <key>OBS_WATCHDOG_ENABLED</key>
        <string>0</string>
        <key>AIS_CATCHER_URL</key>
        <string>http://127.0.0.1:8100/api/v1/vessels?limit=250&amp;max_age_s=300</string>
    </dict>
    <key>RunAtLoad</key>
    <true/>
    <key>KeepAlive</key>
    <true/>
    <key>StandardOutPath</key>
    <string>$LOG_PATH</string>
    <key>StandardErrorPath</key>
    <string>$LOG_PATH</string>
</dict>
</plist>
EOF

  plutil -lint "$PLIST_PATH" >/dev/null

  GUI_DOMAIN="gui/$(id -u)"
  if launchctl print "$GUI_DOMAIN/$PLIST_LABEL" >/dev/null 2>&1; then
    launchctl bootout "$GUI_DOMAIN/$PLIST_LABEL" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      launchctl print "$GUI_DOMAIN/$PLIST_LABEL" >/dev/null 2>&1 || break
      sleep 0.25
    done
  fi
  launchctl bootstrap "$GUI_DOMAIN" "$PLIST_PATH" 2>/dev/null || true
  launchctl kickstart -k "$GUI_DOMAIN/$PLIST_LABEL"
'

echo "4) Verifying live PTZ server health on Mac mini..."
sleep 2
curl -fsSL "http://192.168.1.175:8765/api/state" | /usr/bin/python3 -c '
import json, sys
s = json.load(sys.stdin)
cam = s.get("obs_cam", {})
print("   -> Server UP! ADS-B: {} | AIS: {} ({} local RF ships) | OBS Bridge: connected={} ({})".format(
    s.get("adsb_source"), s.get("ais_source"), s.get("ais_local_count", 0), cam.get("connected"), cam.get("status_text")
))
print("   -> Visible targets projected in Hudson FOV right now: {}".format(len(s.get("targets", []))))
'

echo ""
echo "===================================================================="
echo "  Hudson River PTZ Server is LIVE 24/7 on Craigs-Mac-Mini!"
echo "  - Mission Control (LAN):     http://192.168.1.175:8765/"
echo "  - Simulated PTZ Zoom View:   http://192.168.1.175:8765/?obs=1"
echo "  - Zero-Copy OBS AR HUD:      http://127.0.0.1:8765/?hud=1"
echo "===================================================================="
