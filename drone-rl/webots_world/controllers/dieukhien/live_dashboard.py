"""Read-only local dashboard. HTTP requests never call Webots or ns-3 APIs."""
from collections import deque
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from threading import Lock, Thread
import time
import uuid
from urllib.parse import urlsplit

ASSETS = Path(__file__).with_name('live')


def clean(value):
    """Emit strict JSON even if an instrument temporarily has no valid reading."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class LiveDashboard:
    def __init__(self, port=8765):
        self.port = int(port)
        self.session = uuid.uuid4().hex
        self.lock = Lock()
        self.state = None
        self.updated_at = None
        self.last_publish = -math.inf
        self.events = deque(maxlen=30)
        self.received = self.forwarded = self.dropped = 0
        self.last_verdict = None
        self.server = self.thread = None

    def record_packet(self, sender, seq, forwarded, sim_time, reason='', verdict=None):
        # Called only by the simulation thread; readers use the published copy.
        self.received += 1
        self.forwarded += int(forwarded)
        self.dropped += int(not forwarded)
        self.last_verdict = dict(verdict) if verdict is not None else self.last_verdict
        self.events.appendleft(dict(sender=sender, seq=seq, forwarded=bool(forwarded),
                                    sim_time=sim_time, reason=reason))

    def ready(self):
        return time.monotonic() - self.last_publish >= 0.1

    def publish(self, state, force=False):
        now = time.monotonic()
        if not force and now - self.last_publish < 0.1:
            return  # At most 10 updates / wall-clock second, even in fast simulation.
        self.last_publish = now
        state = clean(deepcopy(state))
        state['session'] = self.session
        state['packets'] = dict(received=self.received, forwarded=self.forwarded,
                                dropped=self.dropped, verdict=clean(self.last_verdict))
        state['events'] = clean(list(self.events))
        with self.lock:
            self.state = state
            self.updated_at = now

    def snapshot(self):
        with self.lock:
            return dict(state=deepcopy(self.state),
                        age_s=None if self.updated_at is None else time.monotonic() - self.updated_at)

    def start(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = urlsplit(self.path).path
                if path == '/api/state':
                    body = json.dumps(dashboard.snapshot(), allow_nan=False).encode()
                    mime = 'application/json; charset=utf-8'
                elif path in ('/', '/app.js', '/style.css'):
                    name = {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}[path]
                    body = (ASSETS / name).read_bytes()
                    mime = {'/': 'text/html', '/app.js': 'text/javascript', '/style.css': 'text/css'}[path] + '; charset=utf-8'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', self.port), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.thread = Thread(target=self.server.serve_forever, daemon=True, name='live-dashboard')
        self.thread.start()
        print(f'[LIVE] http://127.0.0.1:{self.port}', flush=True)
        return self

    def close(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
            self.thread.join(timeout=2)
            self.server = self.thread = None
