"""Loopback-only HTTP transport for the local demo."""
import json
import importlib.util
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .devices import list_serial_ports
from .service import ROOT


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, service, allowed_hosts=()):
        self.service = service
        # Hosts allowed in the Host header. The default is loopback only;
        # passing --allow-lan adds the machine's LAN address so another
        # computer on the same network can drive the demo.
        self.allowed_hosts = set(allowed_hosts)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        # Avoid writing hundreds of polling requests to the terminal.
        if args and str(args[1] if len(args) > 1 else "") not in ("200", "304"):
            super().log_message(fmt, *args)

    def _trusted(self):
        host = self.headers.get("Host", "")
        allowed = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        allowed |= getattr(self.server, "allowed_hosts", set())
        if host not in allowed:
            self._json({"error": "仅接受本机地址"}, 403)
            return False
        origin = self.headers.get("Origin")
        if origin and origin not in {"http://" + h for h in allowed}:
            self._json({"error": "拒绝跨来源请求"}, 403)
            return False
        if self.headers.get("Sec-Fetch-Site") == "cross-site":
            self._json({"error": "拒绝跨站点请求"}, 403)
            return False
        return True

    def _send(self, body, status=200, content_type="application/json; charset=utf-8", filename=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; script-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _json(self, value, status=200, filename=None):
        self._send(json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8"), status, filename=filename)

    def do_GET(self):
        if not self._trusted():
            return
        path = urlparse(self.path).path
        try:
            if path == "/api/state":
                self._json(self.server.service.state())
            elif path == "/api/sessions":
                self._json(self.server.service.sessions())
            elif path == "/api/ports":
                try:
                    ports = list_serial_ports()
                    self._json({"ports": ports, "available": importlib.util.find_spec("serial") is not None})
                except (ImportError, RuntimeError) as exc:
                    self._json({"ports": [], "available": False, "message": str(exc)})
            elif path.startswith("/api/sessions/"):
                parts = path.split("/")
                if len(parts) not in (4, 5) or (len(parts) == 5 and parts[-1] != "download"):
                    raise FileNotFoundError("记录地址不存在")
                value = self.server.service.session(parts[3])
                self._json(value, filename=parts[3] + ".json" if len(parts) == 5 else None)
            elif path in ("/", "/index.html", "/styles.css", "/app.js", "/favicon.svg"):
                name = "index.html" if path == "/" else path[1:]
                file = ROOT / "web" / name
                mime = {".html": "text/html", ".css": "text/css", ".js": "text/javascript", ".svg": "image/svg+xml"}[file.suffix]
                self._send(file.read_bytes(), content_type=mime + "; charset=utf-8")
            else:
                self._json({"error": "地址不存在"}, 404)
        except ValueError as exc:
            self._json({"error": str(exc)}, 400)
        except FileNotFoundError:
            self._json({"error": "文件不存在或记录尚未保存"}, 404)
        except OSError:
            self._json({"error": "文件读取失败，请检查本地目录权限"}, 500)

    def do_POST(self):
        if not self._trusted():
            return
        if urlparse(self.path).path != "/api/action":
            self._json({"error": "地址不存在"}, 404)
            return
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self._json({"error": "请求必须使用 application/json"}, 415)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 16384:
                raise ValueError("请求体大小无效")
            def reject_constant(value):
                raise ValueError("JSON 不允许非有限数字")
            payload = json.loads(self.rfile.read(size), parse_constant=reject_constant)
            self._json(self.server.service.action(payload))
        except (ValueError, UnicodeDecodeError) as exc:
            self._json({"error": str(exc)}, 400)
        except (OSError, RuntimeError) as exc:
            self._json({"error": "操作未完成：" + str(exc)}, 500)
