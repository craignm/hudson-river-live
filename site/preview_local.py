#!/usr/bin/env python3
"""
Local preview server for hudsonriver.live (zero dependencies).
Serves site/public on http://localhost:8090 and emulates the Cloudflare Pages
/api/live endpoint so you can test auto-healing YouTube stream lookup locally.
"""

import json
import re
import time
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PUBLIC_DIR = Path(__file__).resolve().parent / "public"
CHANNEL_ID = "UCTtqzzGVBuwkACkLPau0GTQ"
DEFAULT_VIDEO_ID = "kE_N1WY7UAY"

_cache = {"ts": 0.0, "payload": None}


def resolve_live_stream() -> dict:
    now = time.time()
    if _cache["payload"] and (now - _cache["ts"] < 60):
        return _cache["payload"]

    url = f"https://www.youtube.com/channel/{CHANNEL_ID}/live"
    try:
        req = urllib.request.Request(
            url,
            headers={
                "User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7)",
                "Accept-Language": "en-US,en;q=0.9",
            },
        )
        html = urllib.request.urlopen(req, timeout=6).read().decode("utf-8", errors="ignore")
        m_vid = re.search(
            r'<link\s+rel="canonical"\s+href="https://www\.youtube\.com/watch\?v=([a-zA-Z0-9_-]{11})"',
            html,
        )
        is_live = bool(re.search(r'"isLive"\s*:\s*true', html) or re.search(r'"style"\s*:\s*"LIVE"', html))
        vid = m_vid.group(1) if m_vid else DEFAULT_VIDEO_ID
        payload = {
            "videoId": vid,
            "isLive": bool(m_vid and is_live),
            "channelId": CHANNEL_ID,
            "watchUrl": f"https://www.youtube.com/watch?v={vid}" if m_vid else "https://www.youtube.com/@craignm/live",
        }
    except Exception:
        payload = {
            "videoId": DEFAULT_VIDEO_ID,
            "isLive": True,
            "fallback": True,
            "channelId": CHANNEL_ID,
            "watchUrl": f"https://www.youtube.com/watch?v={DEFAULT_VIDEO_ID}",
        }

    _cache["ts"] = now
    _cache["payload"] = payload
    return payload


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(PUBLIC_DIR), **kwargs)

    def do_GET(self):
        if self.path.startswith("/api/live"):
            data = json.dumps(resolve_live_stream()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
            return
        super().do_GET()


if __name__ == "__main__":
    port = 8090
    print(f"hudsonriver.live preview running at http://localhost:{port}")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
