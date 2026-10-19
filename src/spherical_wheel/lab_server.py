"""Loopback-only browser lab for the live MuJoCo simulation.

The physics worker owns elapsed simulation time. A second worker renders copied
poses in its own MuJoCo model, so HTTP clients never step or edit the plant.
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import ipaddress
import json
import mimetypes
import os
from pathlib import Path
import socket
import sys
import threading
import time
from urllib.parse import urlsplit
import webbrowser

# MuJoCo chooses its GL backend on first import, including imports through sim.
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

REPO_ROOT = Path(__file__).resolve().parents[2]
UI_ROOT = REPO_ROOT / "ui"
MAX_BODY_BYTES = 64 * 1024


def _loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON value: {value}")


class LabService:
    """Workers and shared JPEG cache, independent of the HTTP request threads."""

    def __init__(self, runtime, width=800, height=600, fps=12):
        self.runtime = runtime
        self.width, self.height, self.fps = width, height, fps
        self.stop_event = threading.Event()
        self.frame_ready = threading.Condition()
        self.frame = None
        self.frame_number = 0
        self.frame_time = None
        self.render_error = None
        self.physics_error = None
        self.render_fps = 0.0
        self._threads = []

    def start(self):
        for name, target in (("wheel-physics", self._physics), ("wheel-render", self._render)):
            worker = threading.Thread(name=name, target=target, daemon=True)
            self._threads.append(worker)
            worker.start()

    def stop(self):
        self.stop_event.set()
        with self.frame_ready:
            self.frame_ready.notify_all()
        for worker in self._threads:
            worker.join(timeout=5)

    def state(self):
        with self.runtime.lock:
            result = dict(self.runtime.state())
        with self.frame_ready:
            result["server"] = {
                "render_error": self.render_error,
                "physics_error": self.physics_error,
                "render_fps": round(self.render_fps, 1),
                "frame_number": self.frame_number,
                "frame_age_s": None if self.frame_time is None else round(time.monotonic() - self.frame_time, 3),
            }
        return result

    def _physics(self):
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            try:
                with self.runtime.lock:
                    running = self.runtime.running and not self.runtime.failed
                    dt = float(self.runtime.dt)
                if not running:
                    deadline = time.monotonic()
                    self.stop_event.wait(.02)
                    continue
                now = time.monotonic()
                if now < deadline:
                    self.stop_event.wait(min(deadline - now, .02))
                    continue
                # Bounded catch-up prevents a busy spiral after a slow command.
                steps = min(4, 1 + int((now - deadline) / dt))
                with self.runtime.lock:
                    if self.runtime.running and not self.runtime.failed:
                        self.runtime.advance(steps=steps)
                with self.frame_ready:
                    self.physics_error = None
                deadline += steps * dt
                if time.monotonic() - deadline > .1:
                    deadline = time.monotonic()
            except Exception as exc:
                with self.frame_ready:
                    self.physics_error = f"{type(exc).__name__}: {exc}"
                with self.runtime.lock:
                    self.runtime.running = False
                # Data and reset commands remain available for inspection.
                self.stop_event.wait(.05)

    @staticmethod
    def _path(mujoco, scene, points, rgba, width=2.0):
        import numpy as np
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return
        if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
            return
        points = points[np.unique(np.linspace(0, len(points)-1, min(100, len(points))).astype(int))]
        points = np.column_stack((points[:, :2], np.full(len(points), .004)))
        for a, b in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom or np.linalg.norm(a-b) < 1e-6:
                continue
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE, np.ones(3), np.zeros(3), np.eye(3).ravel(), np.asarray(rgba))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width, a, b)
            geom.category = mujoco.mjtCatBit.mjCAT_DECOR
            scene.ngeom += 1

    def _render(self):
        renderer = None
        try:
            import mujoco
            import numpy as np
            from PIL import Image
            with self.runtime.lock:
                model_path = getattr(self.runtime, "model_path", self.runtime.plant.model_path)
            model = mujoco.MjModel.from_xml_path(str(model_path))
            model.vis.global_.offwidth = max(self.width, model.vis.global_.offwidth)
            model.vis.global_.offheight = max(self.height, model.vis.global_.offheight)
            data = mujoco.MjData(model)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth, camera.elevation, camera.distance = 135, -23, 1.1
            option = mujoco.MjvOption()
            option.geomgroup[3:] = 0
            option.sitegroup[3:] = 0
            renderer = mujoco.Renderer(model, height=self.height, width=self.width)
            fork_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
            previous_time = time.monotonic()
            deadline = previous_time
            while not self.stop_event.is_set():
                with self.runtime.lock:
                    snapshot = self.runtime.render_snapshot()
                data.qpos[:] = snapshot["qpos"]
                data.time = snapshot["time_s"]
                # Positions are already integrated by the physics worker. These
                # calls update only world transforms, avoiding duplicate contacts.
                mujoco.mj_kinematics(model, data)
                mujoco.mj_comPos(model, data)
                mujoco.mj_camlight(model, data)
                camera_settings = snapshot.get("camera", {})
                for key in ("azimuth", "elevation", "distance"):
                    if key in camera_settings:
                        setattr(camera, key, float(camera_settings[key]))
                camera.lookat[:] = data.qpos[:3] + np.array([0., 0., .08])
                renderer.update_scene(data, camera=camera, scene_option=option)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                self._path(mujoco, renderer.scene, snapshot.get("reference_path_xy_m", []), [1., .64, .13, 1.], 3.)
                self._path(mujoco, renderer.scene, snapshot.get("path_xy_m", []), [0., .85, .94, 1.], 3.)
                reference = np.asarray(snapshot.get("reference_xy_m", []), dtype=float)
                if reference.shape == (2,) and renderer.scene.ngeom < renderer.scene.maxgeom:
                    geom = renderer.scene.geoms[renderer.scene.ngeom]
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3, .006),
                                       np.r_[reference, .008], np.eye(3).ravel(), np.array([1., .64, .13, 1.]))
                    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                    renderer.scene.ngeom += 1
                force = np.asarray(snapshot.get("force_N", [0., 0., 0.]), dtype=float)[:3]
                if force.shape == (3,) and np.linalg.norm(force) > .01 and fork_id >= 0:
                    start = data.xipos[fork_id].copy()
                    length = min(.30, .035 + .025 * np.linalg.norm(force))
                    end = start + force / np.linalg.norm(force) * length
                    scene = renderer.scene
                    if scene.ngeom < scene.maxgeom:
                        geom = scene.geoms[scene.ngeom]
                        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.ones(3), np.zeros(3), np.eye(3).ravel(), np.array([.98, .23, .18, 1.]))
                        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .008, start, end)
                        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                        scene.ngeom += 1
                frame = renderer.render()
                encoded = BytesIO()
                Image.fromarray(frame).save(encoded, format="JPEG", quality=85)
                now = time.monotonic()
                with self.frame_ready:
                    self.frame = encoded.getvalue()
                    self.frame_number += 1
                    self.frame_time = now
                    delta = now - previous_time
                    self.render_fps = 1 / delta if delta > 0 else 0
                    self.render_error = None
                    self.frame_ready.notify_all()
                previous_time = now
                deadline = max(deadline + 1/self.fps, now)
                self.stop_event.wait(max(0., deadline - time.monotonic()))
        except Exception as exc:
            with self.frame_ready:
                self.render_error = f"{type(exc).__name__}: {exc}"
                self.frame_ready.notify_all()
        finally:
            if renderer is not None:
                renderer.close()


def _handler(service):
    class Handler(BaseHTTPRequestHandler):
        server_version = "SphericalWheelLab/1.0"

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, format, *args):
            # The page polls state; request logging would hide useful diagnostics.
            if len(args) < 2 or str(args[1]) not in ("200", "304"):
                super().log_message(format, *args)

        def _valid_host(self):
            try:
                host = urlsplit("http://" + self.headers.get("Host", ""))
                return (host.hostname is not None and _loopback(host.hostname)
                        and host.port == self.server.server_port and not host.username)
            except ValueError:
                return False

        def _headers(self, status, content_type, length=None, extra=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            if length is not None:
                self.send_header("Content-Length", str(length))
            for name, value in (extra or {}).items():
                self.send_header(name, value)
            self.end_headers()

        def _json(self, value, status=200):
            content = json.dumps(value, allow_nan=False).encode("utf-8")
            self._headers(status, "application/json; charset=utf-8", len(content))
            self.wfile.write(content)

        def _error(self, message, status=400):
            self._json({"error": message}, status)

        def do_GET(self):
            if not self._valid_host():
                self._error("This application accepts local loopback requests only", 403)
                return
            path = urlsplit(self.path).path
            try:
                if path == "/api/state":
                    self._json(service.state())
                elif path == "/api/info":
                    self._json({
                        "engine": "MuJoCo", "transport": "local HTTP + live MJPEG",
                        "physics_dt_s": service.runtime.dt, "render_target_fps": service.fps,
                        "units": {"position": "m", "time": "s", "force": "N", "impulse": "N s", "torque": "N m", "belt_speed": "m/s", "angle": "deg"},
                        "frame_url": "/frame.jpg", "stream_url": "/stream.mjpg",
                        "export_url": "/api/export", "local_only": True,
                        "scenarios": ["shell", "polar", "transition"],
                        "trajectories": ["hold", "line", "sideways", "curve", "transition"],
                        "model_note": "Provisional mass, friction and actuator limits; ideal belt speed servos.",
                    })
                elif path == "/api/export":
                    with service.runtime.lock:
                        content = service.runtime.export_csv().encode("utf-8")
                    self._headers(200, "text/csv; charset=utf-8", len(content), {"Content-Disposition": 'attachment; filename="spherical-wheel-run.csv"'})
                    self.wfile.write(content)
                elif path == "/frame.jpg":
                    with service.frame_ready:
                        frame, error = service.frame, service.render_error
                    if frame is None:
                        self._error(error or "Renderer is starting; retry shortly", 503)
                    else:
                        self._headers(200, "image/jpeg", len(frame))
                        self.wfile.write(frame)
                elif path == "/stream.mjpg":
                    self._stream()
                elif path in ("/", "/index.html", "/app.js", "/styles.css"):
                    filename = "index.html" if path == "/" else path[1:]
                    target = UI_ROOT / filename
                    if not target.is_file():
                        self._error("Browser interface is unavailable", 404)
                        return
                    content = target.read_bytes()
                    content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
                    self._headers(200, content_type + "; charset=utf-8", len(content))
                    self.wfile.write(content)
                elif path == "/favicon.ico":
                    self._headers(204, "image/x-icon", 0)
                else:
                    self._error("Not found", 404)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self._error(f"{type(exc).__name__}: {exc}", 500)

        def _stream(self):
            with service.frame_ready:
                if service.frame is None and not service.render_error:
                    service.frame_ready.wait(timeout=2)
                frame, error = service.frame, service.render_error
            if frame is None:
                self._error(error or "Renderer is starting; retry shortly", 503)
                return
            self._headers(200, "multipart/x-mixed-replace; boundary=frame", extra={"Connection": "close"})
            previous = -1
            while not service.stop_event.is_set():
                with service.frame_ready:
                    service.frame_ready.wait_for(lambda: service.frame_number != previous or service.stop_event.is_set(), timeout=2)
                    if service.stop_event.is_set():
                        break
                    frame, number = service.frame, service.frame_number
                if frame is None or number == previous:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
                self.wfile.flush()
                previous = number
            self.close_connection = True

        def do_POST(self):
            if not self._valid_host():
                self._error("This application accepts local loopback requests only", 403)
                return
            if urlsplit(self.path).path != "/api/command":
                self._error("Not found", 404)
                return
            origin = self.headers.get("Origin")
            if origin:
                try:
                    parsed = urlsplit(origin)
                    valid = (parsed.scheme == "http" and _loopback(parsed.hostname or "")
                             and parsed.port == self.server.server_port and not parsed.username)
                except ValueError:
                    valid = False
                if not valid:
                    self._error("Cross-origin commands are disabled", 403)
                    return
            if self.headers.get("Sec-Fetch-Site") == "cross-site":
                self._error("Cross-site commands are disabled", 403)
                return
            if self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower() != "application/json":
                self._error("Commands require application/json", 415)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self._error("Invalid Content-Length")
                return
            if not 0 < length <= MAX_BODY_BYTES:
                self._error("Command body must contain 1 to 65536 bytes", 413)
                return
            try:
                command = json.loads(self.rfile.read(length).decode("utf-8"), parse_constant=_reject_constant)
                if not isinstance(command, dict):
                    raise ValueError("Command must be a JSON object")
                with service.runtime.lock:
                    response = service.runtime.command(command)
                self._json(response)
            except (ValueError, TypeError, KeyError) as exc:
                self._error(str(exc))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self._error(f"{type(exc).__name__}: {exc}", 500)

    return Handler


def run_server(host="127.0.0.1", port=8765, open_browser=False):
    """Run the browser lab until Ctrl+C; never bind to a public interface."""
    if not _loopback(host):
        raise ValueError("The interactive lab binds only to a loopback address")
    if not 0 <= int(port) <= 65535:
        raise ValueError("Port must be in [0, 65535]")
    from .lab_runtime import SimulationLab
    print("Loading the MuJoCo model and local balance controllers…", flush=True)
    runtime = SimulationLab()
    service = LabService(runtime)
    server_type = ThreadingHTTPServer
    if ":" in host:
        class IPv6Server(ThreadingHTTPServer):
            address_family = socket.AF_INET6
        server_type = IPv6Server
    server = server_type((host, int(port)), _handler(service))
    server.daemon_threads = True
    server.block_on_close = False
    display_host = f"[{host}]" if ":" in host else host
    url = f"http://{display_host}:{server.server_port}/"
    service.start()
    print(f"Spherical wheel lab: {url}\nPress Ctrl+C to stop.", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        print("\nStopping the spherical wheel lab.", flush=True)
    finally:
        service.stop()
        server.server_close()
