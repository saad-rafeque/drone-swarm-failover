"""Ground-control web server (standard library only).

  GET  /              the control page (static/index.html)
  GET  /static/<f>    page assets (app.js, app.css)
  GET  /api/state     one JSON snapshot of the swarm
  GET  /api/stream    Server-Sent Events: a snapshot every 100 ms
  GET  /api/obstacles buildings / woods of the current mission as lat-lon polygons
  GET  /api/config    map service keys from config/map_keys.local.yaml (git-ignored), read on every call
  GET  /reports/<f>   result files (charts, reports, logs summaries, the PX4 replay page)
  GET  /docs/<f>      handover documents (docs/ and the README)
  POST /api/cmd       JSON command (start, pause, resume, reset, speed, fault, partition, heal)
Bound to 127.0.0.1: only this laptop can open it.
"""
from __future__ import annotations

import json
import time
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import yaml

STATIC = Path(__file__).resolve().parent / "static"
REPO = Path(__file__).resolve().parents[3]
KEYS_FILE = REPO / "config" / "map_keys.local.yaml"
TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg",
         ".md": "text/markdown; charset=utf-8", ".json": "application/json", ".jsonl": "text/plain; charset=utf-8",
         ".csv": "text/csv; charset=utf-8", ".pdf": "application/pdf", ".txt": "text/plain; charset=utf-8",
         ".glb": "model/gltf-binary"}
SHARED = {"/reports/": REPO / "reports", "/docs/": REPO / "docs"}   # read-only file areas


def shared_file(path: str) -> Path | None:
    """A file under reports/ or docs/ (or README.md) for a /reports/... or /docs/... URL; None if outside.
    A /docs/<path>.md URL that is not in docs/ is looked up from the repository root, so the README's links to
    CHANGELOG.md or scripts/README.md open in the Docs page too (Markdown only; never hidden folders
    or *.local.* files)."""
    if path == "/docs/README.md":
        return REPO / "README.md"
    for prefix, root in SHARED.items():
        if path.startswith(prefix):
            f = (root / path[len(prefix):]).resolve()
            if f.is_file() and f.suffix in TYPES and root.resolve() in f.parents:
                return f
    if path.startswith("/docs/") and path.endswith(".md"):
        repo = REPO.resolve()
        f = (repo / path[len("/docs/"):]).resolve()
        if f.is_file() and repo in f.parents and ".local." not in f.name and \
                not any(part.startswith(".") for part in f.relative_to(repo).parts):
            return f
    return None


def map_keys() -> dict:
    """Mapbox / Cesium ion tokens the user pasted into the local key file (empty strings if missing)."""
    keys = {"mapbox_token": "", "cesium_ion_token": ""}
    try:
        data = yaml.safe_load(KEYS_FILE.read_text()) or {}
        keys.update({k: str(data.get(k) or "").strip() for k in keys})
    except (OSError, yaml.YAMLError):
        pass
    return keys
STREAM_PERIOD_S = 0.1


def make_handler(backend):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args) -> None:  # keep the terminal quiet
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj, separators=(",", ":")).encode(), "application/json")

        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                path = "/static/index.html"
            if path.startswith("/static/"):
                f = (STATIC / path[len("/static/"):]).resolve()
                if f.parent != STATIC or f.suffix not in TYPES or not f.exists():
                    self._send(404, b"not found", "text/plain")
                    return
                self._send(200, f.read_bytes(), TYPES[f.suffix])
            elif path == "/api/state":
                self._json(backend.snapshot())
            elif path == "/api/obstacles":
                self._json(backend.obstacles_payload())
            elif path == "/api/config":
                self._json(map_keys())
            elif path.startswith(("/reports/", "/docs/")):
                f = shared_file(urllib.parse.unquote(path))
                if f is None:
                    self._send(404, b"not found", "text/plain")
                else:
                    self._send(200, f.read_bytes(), TYPES[f.suffix])
            elif path == "/api/stream":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                try:
                    while True:
                        data = json.dumps(backend.snapshot(), separators=(",", ":"))
                        self.wfile.write(f"data: {data}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(STREAM_PERIOD_S)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self) -> None:
            if self.path != "/api/cmd":
                self._send(404, b"not found", "text/plain")
                return
            try:
                n = int(self.headers.get("Content-Length", "0"))
                cmd = json.loads(self.rfile.read(n) or b"{}")
                self._json(backend.command(cmd))
            except (ValueError, TypeError, KeyError) as exc:
                self._json({"ok": False, "msg": f"bad command: {exc}"}, 400)

    return Handler


def serve(backend, port: int) -> None:
    srv = ThreadingHTTPServer(("127.0.0.1", port), make_handler(backend))
    srv.daemon_threads = True
    print(f"Swarm ground control: http://localhost:{port}  (backend: {backend.label})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
