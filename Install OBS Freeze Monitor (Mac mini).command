#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

if [ -f "./.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "./.env"
  set +a
fi

TOPIC="${NTFY_TOPIC:-}"
if [ -z "$TOPIC" ]; then
  echo "ERROR: NTFY_TOPIC is not set. Add NTFY_TOPIC=<your-private-topic> to .env or pass NTFY_TOPIC=... in the environment." >&2
  exit 1
fi
INSTALL_DIR="$HOME/Library/Application Support/HudsonRiverMonitor"
SCRIPT_DST="$INSTALL_DIR/obs_freeze_monitor.py"
LOG_PATH="$HOME/Library/Logs/hudson-obs-freeze-monitor.log"
PLIST_LABEL="live.hudsonriver.obs-freeze-monitor"
PLIST_PATH="$HOME/Library/LaunchAgents/$PLIST_LABEL.plist"

if [ -x "/opt/homebrew/bin/python3" ]; then
  PY_BIN="/opt/homebrew/bin/python3"
else
  PY_BIN="/usr/bin/python3"
fi

echo "=== Installing 24/7 Hudson OBS Freeze Monitor ($PLIST_LABEL) ==="
mkdir -p "$INSTALL_DIR" "$HOME/Library/Logs" "$HOME/Library/LaunchAgents"
cp -f "./obs_freeze_monitor.py" "$SCRIPT_DST"
chmod +x "$SCRIPT_DST"
xattr -d com.apple.quarantine "$SCRIPT_DST" 2>/dev/null || true
if [ -f "./uvc-util" ]; then
  cp -f "./uvc-util" "$INSTALL_DIR/uvc-util"
  chmod +x "$INSTALL_DIR/uvc-util"
  xattr -d com.apple.quarantine "$INSTALL_DIR/uvc-util" 2>/dev/null || true
fi

cat <<EOF > "$PLIST_PATH"
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>Label</key>
    <string>$PLIST_LABEL</string>
    <key>ProgramArguments</key>
    <array>
        <string>$PY_BIN</string>
        <string>-u</string>
        <string>$SCRIPT_DST</string>
        <string>--obs-url</string>
        <string>obs://127.0.0.1:4455</string>
        <string>--ntfy-topic</string>
        <string>$TOPIC</string>
        <string>--data-dir</string>
        <string>$INSTALL_DIR</string>
    </array>
    <key>RunAtLoad</key>
    <true/>
    <key>KeepAlive</key>
    <true/>
    <key>ThrottleInterval</key>
    <integer>10</integer>
    <key>StandardOutPath</key>
    <string>$LOG_PATH</string>
    <key>StandardErrorPath</key>
    <string>$LOG_PATH</string>
</dict>
</plist>
EOF

# Remove legacy label if ever installed under old name
launchctl bootout "gui/$(id -u)/local.hudson.obs-freeze-monitor" 2>/dev/null || true
rm -f "$HOME/Library/LaunchAgents/local.hudson.obs-freeze-monitor.plist"

launchctl bootout "gui/$(id -u)/$PLIST_LABEL" 2>/dev/null || true
launchctl bootstrap "gui/$(id -u)" "$PLIST_PATH"

echo ""
echo "✅ LaunchAgent '$PLIST_LABEL' installed and running (auto-starts on reboot)!"
echo "   Script & incidents: $INSTALL_DIR"
echo "   Log file:           $LOG_PATH"
echo "   ntfy topic:         $TOPIC"
echo "   Web dashboard:      https://ntfy.sh/$TOPIC"
echo ""
echo "Sending live test push notification from $(hostname)..."
"$PY_BIN" "$SCRIPT_DST" --obs-url "obs://127.0.0.1:4455" --ntfy-topic "$TOPIC" --data-dir "$INSTALL_DIR" --test-alert || true
echo ""
echo "Done!"
