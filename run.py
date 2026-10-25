#!/usr/bin/env python3
"""Launch Lumbar Lab; simulation has no third-party requirements."""
import argparse
import importlib
import sys

if sys.version_info < (3, 10):
    print("Lumbar Lab requires Python 3.10+. On Windows, double-click start_windows.bat.")
    raise SystemExit(1)

import threading
import webbrowser
from lumbar.server import Server
from lumbar.service import AppService


def load_actuator_factory(spec):
    """Import only the module/function explicitly selected on the command line."""
    if not isinstance(spec, str) or spec.count(":") != 1:
        raise ValueError("actuator factory must use module:callable, e.g. integrations.hardware_adapter:create_actuator")
    module_name, name = spec.split(":")
    if not module_name or not all(part.isidentifier() for part in module_name.split(".")) or not name.isidentifier():
        raise ValueError("actuator factory must name a Python module and a function as module:callable")
    try:
        module = importlib.import_module(module_name)
        factory = getattr(module, name)
    except Exception as exc:
        raise ValueError(f"cannot load actuator factory {spec}: {exc}") from exc
    if not callable(factory):
        raise ValueError(f"actuator factory {spec} is not callable")
    return factory


def main(argv=None):
    parser = argparse.ArgumentParser(description="Lumbar Lab local demo")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address to bind; use 0.0.0.0 to let other machines "
                             "on the LAN reach the demo page (e.g. 192.168.x.x)")
    parser.add_argument("--allow-lan", action="store_true",
                        help="accept requests whose Host header names this "
                             "machine's LAN address (needed together with "
                             "--host 0.0.0.0 so another computer can drive the demo)")
    parser.add_argument("--actuator-factory", metavar="MODULE:CALLABLE",
                        help="explicit adapter factory, e.g. integrations.hardware_adapter:create_actuator; default serial output is disabled")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    app_options = {}
    if args.actuator_factory is not None:
        try:
            app_options["actuator_factory"] = load_actuator_factory(args.actuator_factory)
        except ValueError as exc:
            parser.error(str(exc))
    try:
        app = AppService(**app_options)
    except Exception as exc:
        print(f"Cannot initialize Lumbar Lab or the selected actuator factory: {exc}", file=sys.stderr)
        return 1
    lan_hosts = set()
    lan_ip = ""
    if args.allow_lan:
        try:
            import socket as _socket
            _probe = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            _probe.connect(("8.8.8.8", 80))
            lan_ip = _probe.getsockname()[0]
            _probe.close()
            lan_hosts.add(f"{lan_ip}:{args.port}")
            lan_hosts.add(f"{lan_ip}")
        except OSError:
            pass
    try:
        server = Server((args.host, args.port), app)
    except OSError as exc:
        app.close()
        print(f"Cannot open port {args.port}: {exc}\nTry: python run.py --port 8766")
        return 1
    #  局域网访问：把本机 LAN 地址加入 Host 白名单。
    #  默认（不加 --allow-lan）只允许 127.0.0.1/localhost —— 这是队友
    #  原本的安全设定，避免网页被别人从局域网控制电机。
    server.allowed_hosts = set(lan_hosts)
    if args.host == "0.0.0.0" and lan_ip:
        print(f"LAN access: http://{lan_ip}:{args.port}"
              f"{'  (--allow-lan enabled)' if args.allow_lan else '  (add --allow-lan to allow remote control)'}")
    stopped = threading.Event()
    def control_loop():
        while not stopped.wait(0.05):
            try:
                app.tick()
            except Exception as exc:
                app.fail("内部错误 · 已停止，请重启实验台")
                print(f"Control loop stopped: {exc}", file=sys.stderr)
                break
    worker = threading.Thread(target=control_loop, name="lumbar-control", daemon=True)
    worker.start()
    url = f"http://127.0.0.1:{args.port}"
    print(f"Lumbar Lab: {url}\nSimulation mode. Press Ctrl+C to close.", flush=True)
    if args.actuator_factory is not None:
        print(f"Actuator factory loaded: {args.actuator_factory}. Startup remains in simulation mode.", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        worker.join(timeout=2)
        app.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
