"""Tiny armed-mode dashboard: stdlib HTTP, JSON status, e-stop flag."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable
from urllib.parse import urlparse

HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>airbearing dashboard</title>
<style>
body{font-family:ui-monospace,monospace;background:#111;color:#eee;margin:24px}
button{font-size:22px;padding:12px 28px;background:#c0392b;color:#fff;border:0;cursor:pointer}
.ok{color:#2ecc71}.bad{color:#e74c3c} pre{background:#1b1b1b;padding:12px}
</style></head>
<body>
<h1>airbearing — laboratory only</h1>
<p>Not flight software. E-stop zeros commands this host would send.</p>
<p><button onclick="estop()">E-STOP</button></p>
<pre id="s">loading…</pre>
<script>
async function tick(){
  const r = await fetch('/status.json');
  const j = await r.json();
  document.getElementById('s').textContent = JSON.stringify(j, null, 2);
  document.getElementById('s').className = j.estop ? 'bad' : 'ok';
}
async function estop(){ await fetch('/estop', {method:'POST'}); tick(); }
setInterval(tick, 250); tick();
</script>
</body></html>
"""


class Dashboard:
    def __init__(self, status_fn: Callable[[], dict[str, Any]], estop_fn: Callable[[], None], host: str = "127.0.0.1", port: int = 8765):
        self.status_fn = status_fn
        self.estop_fn = estop_fn
        self.host = host
        self.port = port
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    body = HTML.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/status.json":
                    body = json.dumps(dash.status_fn()).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_error(404)

            def do_POST(self):
                path = urlparse(self.path).path
                if path == "/estop":
                    dash.estop_fn()
                    body = b'{"estop": true}'
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_error(404)

        self._Handler = Handler

    def start(self) -> str:
        self._httpd = ThreadingHTTPServer((self.host, self.port), self._Handler)
        self.port = self._httpd.server_address[1]
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return f"http://{self.host}:{self.port}/"

    def stop(self) -> None:
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
