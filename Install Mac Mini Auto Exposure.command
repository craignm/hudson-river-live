#!/bin/zsh
set -euo pipefail

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
LAN_SERVER="${LAN_SERVER:-http://192.168.1.245:8765}"
APP_SUPPORT="$HOME/Library/Application Support/HudsonRiverUVC"
PLIST_LABEL="live.hudsonriver.uvc-exposure"
PLIST_PATH="$HOME/Library/LaunchAgents/${PLIST_LABEL}.plist"
LOG_PATH="$HOME/Library/Logs/hudson-uvc-exposure.log"
if [[ -x "/opt/homebrew/bin/python3" ]]; then
  PY_BIN="/opt/homebrew/bin/python3"
else
  PY_BIN="/usr/bin/python3"
fi

echo "===================================================================="
echo "  Hudson River Cam — Insta360 Link 2C Pro Day/Night Exposure Setup"
echo "===================================================================="
echo ""

mkdir -p "$APP_SUPPORT" "$HOME/Library/LaunchAgents" "$HOME/Library/Logs"

if [[ -f "$SRC_DIR/uvc-util" && -f "$SRC_DIR/auto_exposure_day_night.py" ]]; then
  cp -f "$SRC_DIR/uvc-util" "$APP_SUPPORT/uvc-util"
  cp -f "$SRC_DIR/auto_exposure_day_night.py" "$APP_SUPPORT/auto_exposure_day_night.py"
  [[ -f "$SRC_DIR/calibration.json" ]] && cp -f "$SRC_DIR/calibration.json" "$APP_SUPPORT/calibration.json"
else
  echo "Fetching uvc-util & exposure controller over LAN from $LAN_SERVER..."
  curl -fsSL "$LAN_SERVER/dl/uvc-util" -o "$APP_SUPPORT/uvc-util"
  curl -fsSL "$LAN_SERVER/dl/auto_exposure_day_night.py" -o "$APP_SUPPORT/auto_exposure_day_night.py"
  curl -fsSL "$LAN_SERVER/dl/calibration.json" -o "$APP_SUPPORT/calibration.json"
fi

xattr -cr "$APP_SUPPORT" 2>/dev/null || true
chmod +x "$APP_SUPPORT/uvc-util" "$APP_SUPPORT/auto_exposure_day_night.py"

echo "1) Probing connected Insta360 UVC camera (zero OBS interruption)..."
echo "--------------------------------------------------------------------"
if ! "$APP_SUPPORT/uvc-util" --probe; then
  echo ""
  echo "NOTE: No Insta360 USB camera found on THIS Mac."
  echo "Make sure you run this on the Mac mini with the camera plugged in!"
  echo ""
  read -r "?Press Return to close..."
  exit 1
fi

echo ""
echo "2) Today's Hudson River Solar Exposure Schedule:"
echo "--------------------------------------------------------------------"
"$PY_BIN" "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util" --schedule-today

echo ""
echo "3) Registering standalone LaunchAgent..."
echo "--------------------------------------------------------------------"

cat > "$PLIST_PATH" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>${PLIST_LABEL}</string>
    <key>ProgramArguments</key>
    <array>
        <string>${PY_BIN}</string>
        <string>${APP_SUPPORT}/auto_exposure_day_night.py</string>
        <string>--uvc-bin</string>
        <string>${APP_SUPPORT}/uvc-util</string>
        <string>--day-ev</string>
        <string>0.0</string>
        <string>--night-ev</string>
        <string>-1.8</string>
        <string>--quiet-unchanged</string>
    </array>
    <key>RunAtLoad</key>
    <true/>
    <key>StartInterval</key>
    <integer>180</integer>
    <key>StandardOutPath</key>
    <string>${LOG_PATH}</string>
    <key>StandardErrorPath</key>
    <string>${LOG_PATH}</string>
</dict>
</plist>
EOF

GUI_DOMAIN="gui/$(id -u)"
launchctl bootout "$GUI_DOMAIN/$PLIST_LABEL" 2>/dev/null || true
if ! launchctl bootstrap "$GUI_DOMAIN" "$PLIST_PATH" 2>/dev/null; then
  launchctl unload "$PLIST_PATH" 2>/dev/null || true
  launchctl load "$PLIST_PATH"
fi

echo "Applying current exposure target right now..."
"$PY_BIN" "$APP_SUPPORT/auto_exposure_day_night.py" --uvc-bin "$APP_SUPPORT/uvc-util"

echo ""
echo "===================================================================="
echo "  Installed & Active!"
echo "  - LaunchAgent: $PLIST_PATH (checks every 3 min + across reboots)"
echo "  - Log file:    $LOG_PATH"
echo "  - Manual test anytime:"
echo "      ./uvc-util -V 0x2e1a -s exposure-comp-ev=-1.8   # Night"
echo "      ./uvc-util -V 0x2e1a -s exposure-comp-ev=0.0    # Day"
echo "===================================================================="
echo ""
read -r "?Press Return to close..."
