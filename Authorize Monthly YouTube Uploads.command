#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"
REMOTE_HOST="${MAC_MINI_HOST:-craignm@192.168.1.175}"

PY_BIN="/opt/homebrew/bin/python3"
[ -x "$PY_BIN" ] || PY_BIN="python3"

"$PY_BIN" - "$REMOTE_HOST" << 'PYEOF'
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.parse
import urllib.request
import webbrowser

remote_host = sys.argv[1]
print("====================================================================")
print("  Hudson River Monthly 4K YouTube Uploader — One-Time OAuth Setup")
print("====================================================================")
print("")
print("In Google Cloud Console (project: youtube-playlist-automator-20):")
print("  1. Go to: https://console.cloud.google.com/apis/credentials?project=youtube-playlist-automator-20")
print("  2. Click '+ Create Credentials' -> 'OAuth client ID' -> Application type: 'Desktop app'")
print("  3. Download the JSON file (or copy the Client ID & Client Secret)")
print("")

inp = input("Drag & drop the downloaded client_secret.json file here (or paste Client ID): ").strip().strip("'").strip('"')
client_id = ""
client_secret = ""

p = Path(os.path.expanduser(inp))
if p.is_file():
    raw = json.loads(p.read_text())
    block = raw.get("installed") or raw.get("web") or raw
    client_id = str(block.get("client_id") or "").strip()
    client_secret = str(block.get("client_secret") or "").strip()
else:
    client_id = inp
    client_secret = input("Paste OAuth Client Secret: ").strip()

if not client_id or not client_secret:
    print("Error: Client ID and Client Secret are required.")
    sys.exit(1)

import socket

redirect_port = 8799
redirect_uri = f"http://localhost:{redirect_port}"
scopes = "https://www.googleapis.com/auth/youtube https://www.googleapis.com/auth/youtube.upload"
auth_code_box = {"code": "", "error": ""}


class DualStackHTTPServer(HTTPServer):
    address_family = socket.AF_INET6

    def server_bind(self):
        self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        super().server_bind()


class CallbackHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        if "code" in q:
            auth_code_box["code"] = q["code"][0]
            msg = b"<h2>YouTube authorization succeeded! You can close this tab and return to Terminal.</h2>"
        else:
            auth_code_box["error"] = q.get("error", ["unknown"])[0]
            msg = f"<h2>Authorization failed: {auth_code_box['error']}</h2>".encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(msg)))
        self.end_headers()
        self.wfile.write(msg)


srv = DualStackHTTPServer(("::", redirect_port), CallbackHandler)
auth_url = "https://accounts.google.com/o/oauth2/v2/auth?" + urllib.parse.urlencode(
    {
        "client_id": client_id,
        "redirect_uri": redirect_uri,
        "response_type": "code",
        "scope": scopes,
        "access_type": "offline",
        "prompt": "consent",
    }
)
print("\nOpening Google OAuth consent screen in your browser...")
print(f"If it does not open automatically, visit:\n  {auth_url}\n")
webbrowser.open(auth_url)

while not auth_code_box["code"] and not auth_code_box["error"]:
    srv.handle_request()
srv.server_close()

if auth_code_box["error"]:
    print(f"OAuth failed: {auth_code_box['error']}")
    sys.exit(1)

tok_body = urllib.parse.urlencode(
    {
        "code": auth_code_box["code"],
        "client_id": client_id,
        "client_secret": client_secret,
        "redirect_uri": redirect_uri,
        "grant_type": "authorization_code",
    }
).encode("utf-8")
tok_req = urllib.request.Request(
    "https://oauth2.googleapis.com/token",
    data=tok_body,
    headers={"Content-Type": "application/x-www-form-urlencoded"},
    method="POST",
)
with urllib.request.urlopen(tok_req, timeout=20.0) as resp:
    tok_json = json.loads(resp.read().decode("utf-8"))

refresh_token = tok_json.get("refresh_token", "")
access_token = tok_json.get("access_token", "")
if not refresh_token:
    print("Error: Google did not return a refresh_token. Please re-run and approve consent.")
    sys.exit(1)

# Verify connected YouTube channel
ch_req = urllib.request.Request(
    "https://www.googleapis.com/youtube/v3/channels?part=snippet&mine=true",
    headers={"Authorization": f"Bearer {access_token}"},
)
try:
    with urllib.request.urlopen(ch_req, timeout=15.0) as ch_resp:
        ch_data = json.loads(ch_resp.read().decode("utf-8"))
        items = ch_data.get("items") or []
        if items:
            sn = items[0].get("snippet") or {}
            print(f"\nVerified YouTube Channel: {sn.get('title')} ({items[0].get('id')})")
except Exception:
    pass

# Push credentials directly to Mac mini PTZ server via /api/settings over SSH loopback
payload = json.dumps(
    {
        "yt_oauth_client_id": client_id,
        "yt_oauth_client_secret": client_secret,
        "yt_oauth_refresh_token": refresh_token,
        "monthly_youtube_enabled": True,
    }
)
print(f"Saving YouTube upload credentials to {remote_host}...")
subprocess.run(
    [
        "ssh",
        remote_host,
        "curl -fsSL -X POST http://127.0.0.1:8765/api/settings -H 'Content-Type: application/json' -d @-",
    ],
    input=payload.encode("utf-8"),
    check=True,
)
print("\nAll set! Monthly 3-hour Dawn, Morning, Afternoon, Sunset, and Night 4K uploads are armed 24/7 on Craigs-Mac-Mini.")
PYEOF
