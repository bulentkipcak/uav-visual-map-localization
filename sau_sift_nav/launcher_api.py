from __future__ import annotations

import asyncio
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field


DEFAULTS: dict[str, Any] = {
    "map_source": "sift-master",
    "host": "127.0.0.1",
    "port": 8080,
    "modern_dashboard_port": 8090,
    "mavlink": "udp:127.0.0.1:14550",
    "video_source": "udp",
    "udp_port": 5600,
    "send_vision": False,
    "telemetry_seed_source": "global",
    "search_radius_m": 150.0,
    "max_search_radius_m": 300.0,
    "max_tiles_per_scale": 9,
    "early_stop_inliers": 160,
    "min_nav_inliers": 80,
    "resize_w": None,
    "gazebo_truth": True,
    "gz_topic": "/world/iris_runway/pose/info",
    "gz_model": "iris_with_gimbal",
    "log_enabled": True,
    "log_csv": "live_sift_dashboard.csv",
    "duration_sec": 0.0,
}


PRESETS: list[dict[str, Any]] = [
    {
        "id": "observe_recommended",
        "name": "Observe recommended",
        "description": "SIFT-master, Gazebo truth, VPE kapali. Rapor/gözlem için güvenli başlangıç.",
        "config": {
            **DEFAULTS,
            "log_csv": "dashboard_observe.csv",
            "send_vision": False,
        },
    },
    {
        "id": "observe_fast_768",
        "name": "Fast observe 768",
        "description": "Son kamera ayarina uygun hafif gözlem preset'i; maksimum 9 patch arar.",
        "config": {
            **DEFAULTS,
            "log_csv": "dashboard_fast_768.csv",
            "search_radius_m": 150.0,
            "max_search_radius_m": 300.0,
            "max_tiles_per_scale": 9,
            "resize_w": None,
            "send_vision": False,
        },
    },
    {
        "id": "validation_log_only",
        "name": "Validation log only",
        "description": "EKF/source-set degistirmeden validation log almak icin.",
        "config": {
            **DEFAULTS,
            "run_mode": "validation_log_only",
            "send_vision": False,
            "log_csv": "dashboard_validation.csv",
            "validation_log_csv": "dashboard_validation_detail.csv",
        },
    },
    {
        "id": "vpe_experimental",
        "name": "VPE experimental",
        "description": "Deneysel VPE yayini. Source-set testlerinden once dikkatli kullan.",
        "config": {
            **DEFAULTS,
            "send_vision": True,
            "send_vision_speed": False,
            "vision_publish_mode": "fix",
            "vision_stream_mode": "fixed_rate_kalman_predict",
            "vision_z_source": "zero",
            "vision_attitude_source": "zero",
            "vision_timestamp_source": "frame",
            "log_csv": "dashboard_vpe_experimental.csv",
        },
    },
]


CONFIG_FIELDS: list[dict[str, Any]] = [
    {"name": "mavlink", "type": "text", "label": "MAVLink"},
    {"name": "video_source", "type": "text", "label": "Video source"},
    {"name": "udp_port", "type": "number", "label": "UDP video port"},
    {"name": "port", "type": "number", "label": "Old dashboard port"},
    {"name": "modern_dashboard_port", "type": "number", "label": "Live API port"},
    {"name": "send_vision", "type": "boolean", "label": "Send VPE"},
    {"name": "gazebo_truth", "type": "boolean", "label": "Gazebo truth"},
    {"name": "telemetry_seed_source", "type": "select", "label": "Telemetry seed", "options": ["auto", "local", "global"]},
    {"name": "search_radius_m", "type": "number", "label": "Search radius m"},
    {"name": "max_search_radius_m", "type": "number", "label": "Max search radius m"},
    {"name": "max_tiles_per_scale", "type": "number", "label": "Max patches"},
    {"name": "early_stop_inliers", "type": "number", "label": "Early stop inliers"},
    {"name": "min_nav_inliers", "type": "number", "label": "Min nav inliers"},
    {"name": "resize_w", "type": "number", "label": "Resize width"},
    {"name": "duration_sec", "type": "number", "label": "Duration sec"},
    {"name": "log_enabled", "type": "boolean", "label": "Log CSV"},
    {"name": "log_csv", "type": "text", "label": "Log base name"},
]


class StartRequest(BaseModel):
    preset_id: str = Field(default="observe_recommended")
    overrides: dict[str, Any] = Field(default_factory=dict)
    extra_args: str = ""


class GimbalRequest(BaseModel):
    mavlink: Optional[str] = None
    pitch_deg: float = -90.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    timeout_sec: float = 5.0


class DummyVisionRequest(BaseModel):
    mavlink: Optional[str] = None
    rate_hz: float = 10.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    reset_counter: int = 0
    timeout_sec: float = 5.0


class SourceSetRequest(BaseModel):
    mavlink: Optional[str] = None
    source_set: int = Field(default=1, ge=1, le=3)
    timeout_sec: float = 5.0


class DummyVisionStreamer(threading.Thread):
    def __init__(self, request: DummyVisionRequest, connection_string: str) -> None:
        super().__init__(daemon=True)
        self.request = request
        self.connection_string = connection_string
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.started_at: Optional[float] = time.time()
        self.sent_count = 0
        self.last_sent_at: Optional[float] = None
        self.last_error: Optional[str] = None
        self.heartbeat_seen = False
        self.finished = False

    def stop(self) -> None:
        self.stop_event.set()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            running = self.started_at is not None and not self.finished and not self.stop_event.is_set()
            return {
                "running": running,
                "mavlink": self.connection_string,
                "rate_hz": self.request.rate_hz,
                "pose": {
                    "x": self.request.x,
                    "y": self.request.y,
                    "z": self.request.z,
                    "roll": self.request.roll,
                    "pitch": self.request.pitch,
                    "yaw": self.request.yaw,
                },
                "sent_count": self.sent_count,
                "started_at": self.started_at,
                "uptime_sec": time.time() - self.started_at if self.started_at and running else None,
                "last_sent_age_sec": time.time() - self.last_sent_at if self.last_sent_at else None,
                "heartbeat_seen": self.heartbeat_seen,
                "last_error": self.last_error,
            }

    def run(self) -> None:
        conn = None
        try:
            from pymavlink import mavutil

            conn = mavutil.mavlink_connection(
                self.connection_string,
                autoreconnect=True,
                source_system=247,
                source_component=189,
            )
            hb = conn.wait_heartbeat(timeout=max(1.0, float(self.request.timeout_sec)))
            if hb is None:
                self._set_error("MAVLink heartbeat timeout")
                return
            with self.lock:
                self.heartbeat_seen = True

            period = 1.0 / max(0.5, float(self.request.rate_hz))
            next_send = time.monotonic()
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now < next_send:
                    time.sleep(min(0.05, next_send - now))
                    continue
                self._send_vpe(conn)
                next_send += period
        except Exception as exc:
            self._set_error(str(exc))
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass
            with self.lock:
                self.finished = True

    def _send_vpe(self, conn: Any) -> None:
        req = self.request
        usec = int(time.time() * 1_000_000)
        covariance = [float("nan")] + [0.0] * 20
        try:
            conn.mav.vision_position_estimate_send(
                usec,
                float(req.x),
                float(req.y),
                float(req.z),
                float(req.roll),
                float(req.pitch),
                float(req.yaw),
                covariance,
                int(req.reset_counter) & 0xFF,
            )
        except TypeError:
            conn.mav.vision_position_estimate_send(
                usec,
                float(req.x),
                float(req.y),
                float(req.z),
                float(req.roll),
                float(req.pitch),
                float(req.yaw),
            )
        with self.lock:
            self.sent_count += 1
            self.last_sent_at = time.time()

    def _set_error(self, message: str) -> None:
        with self.lock:
            self.last_error = message


class Launcher:
    def __init__(self, repo_root: Path) -> None:
        self.repo_root = repo_root
        self.process: Optional[subprocess.Popen[str]] = None
        self.command: list[str] = []
        self.started_at: Optional[float] = None
        self.config: dict[str, Any] = {}
        self.log_tail: deque[str] = deque(maxlen=300)
        self._lock = threading.RLock()
        self._reader_thread: Optional[threading.Thread] = None
        self._dummy_vision: Optional[DummyVisionStreamer] = None

    def start(self, request: StartRequest) -> dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            if self.process and self.process.poll() is None:
                raise HTTPException(status_code=409, detail="live_sift_nav is already running")

            config = self._build_config(request)
            command = self._build_command(config, request.extra_args)
            self._check_ports_available(config)
            self.log_tail.clear()
            self.log_tail.append("$ " + " ".join(shlex.quote(part) for part in command))
            self.process = subprocess.Popen(
                command,
                cwd=self.repo_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            self.command = command
            self.started_at = time.time()
            self.config = config
            self._reader_thread = threading.Thread(target=self._read_output, daemon=True)
            self._reader_thread.start()
            return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            proc = self.process
            if not proc or proc.poll() is not None:
                return self.status()
            self.log_tail.append("launcher: stopping live_sift_nav")
            proc.send_signal(signal.SIGINT)

        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)
        return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_locked()
            running = bool(self.process and self.process.poll() is None)
            return {
                "running": running,
                "pid": self.process.pid if self.process and running else None,
                "returncode": self.process.poll() if self.process else None,
                "started_at": self.started_at,
                "uptime_sec": time.time() - self.started_at if self.started_at and running else None,
                "command": self.command,
                "config": self.config,
                "log_tail": list(self.log_tail),
                "live_api_url": self._live_api_url(),
                "old_dashboard_url": self._old_dashboard_url(),
                "dummy_vision": self._dummy_vision_status_locked(),
            }

    def point_gimbal(self, request: GimbalRequest) -> dict[str, Any]:
        connection_string = request.mavlink or self.config.get("mavlink") or DEFAULTS["mavlink"]
        result = send_gimbal_mount_control(
            connection_string=str(connection_string),
            pitch_deg=float(request.pitch_deg),
            roll_deg=float(request.roll_deg),
            yaw_deg=float(request.yaw_deg),
            timeout_sec=max(1.0, float(request.timeout_sec)),
        )
        with self._lock:
            self.log_tail.append(
                "launcher: gimbal command "
                f"pitch={request.pitch_deg:.1f} roll={request.roll_deg:.1f} yaw={request.yaw_deg:.1f} "
                f"result={result.get('ack_result_name', result.get('status'))}"
            )
        return result

    def start_dummy_vision(self, request: DummyVisionRequest) -> dict[str, Any]:
        connection_string = request.mavlink or self.config.get("mavlink") or DEFAULTS["mavlink"]
        with self._lock:
            if self._dummy_vision and self._dummy_vision.is_alive():
                return self.status()
            streamer = DummyVisionStreamer(request=request, connection_string=str(connection_string))
            self._dummy_vision = streamer
            self.log_tail.append(
                "launcher: dummy VPE starting "
                f"mavlink={connection_string} rate={request.rate_hz:.1f}Hz "
                f"pose=({request.x:.2f},{request.y:.2f},{request.z:.2f})"
            )
            streamer.start()
            return self.status()

    def stop_dummy_vision(self) -> dict[str, Any]:
        streamer: Optional[DummyVisionStreamer]
        with self._lock:
            streamer = self._dummy_vision
            if streamer is None:
                return self.status()
            self.log_tail.append("launcher: dummy VPE stopping")
            streamer.stop()
        streamer.join(timeout=2.0)
        return self.status()

    def switch_source_set(self, request: SourceSetRequest) -> dict[str, Any]:
        connection_string = request.mavlink or self.config.get("mavlink") or DEFAULTS["mavlink"]
        result = send_ekf_source_set(
            connection_string=str(connection_string),
            source_set=int(request.source_set),
            timeout_sec=max(1.0, float(request.timeout_sec)),
        )
        with self._lock:
            self.log_tail.append(
                "launcher: EKF source-set "
                f"set={request.source_set} result={result.get('ack_result_name', result.get('status'))}"
            )
        return result

    def _read_output(self) -> None:
        proc = self.process
        if not proc or not proc.stdout:
            return
        for line in proc.stdout:
            with self._lock:
                self.log_tail.append(line.rstrip())

    def _refresh_locked(self) -> None:
        if self.process and self.process.poll() is not None:
            if not self.log_tail or not self.log_tail[-1].startswith("launcher: process exited"):
                self.log_tail.append(f"launcher: process exited rc={self.process.poll()}")
        if self._dummy_vision and not self._dummy_vision.is_alive():
            status = self._dummy_vision.snapshot()
            if status.get("last_error") and not any("dummy VPE stopped" in item for item in self.log_tail):
                self.log_tail.append(f"launcher: dummy VPE stopped error={status['last_error']}")

    def _build_config(self, request: StartRequest) -> dict[str, Any]:
        preset = next((item for item in PRESETS if item["id"] == request.preset_id), None)
        if not preset:
            raise HTTPException(status_code=400, detail=f"unknown preset: {request.preset_id}")
        config = dict(preset["config"])
        for key, value in request.overrides.items():
            if value == "" or value is None:
                continue
            config[key] = value
        self._prepare_log_config(config)
        return config

    def _prepare_log_config(self, config: dict[str, Any]) -> None:
        if not config.get("log_enabled", True):
            config["log_csv"] = ""
            config.pop("validation_log_csv", None)
            config.pop("vision_publish_log_csv", None)
            return

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_log = str(config.get("log_csv") or DEFAULTS["log_csv"])
        config["log_csv"] = self._timestamped_log_path(base_log, stamp)
        for key in ("validation_log_csv", "vision_publish_log_csv"):
            if config.get(key):
                config[key] = self._timestamped_log_path(str(config[key]), stamp)

    @staticmethod
    def _timestamped_log_path(value: str, stamp: str) -> str:
        path = Path(value)
        suffix = path.suffix or ".csv"
        stem = path.stem or "live_sift_log"
        return str(path.with_name(f"{stem}_{stamp}{suffix}"))

    def _build_command(self, config: dict[str, Any], extra_args: str) -> list[str]:
        command = [sys.executable, "live_sift_nav.py"]
        add = command.extend
        add(["--map-source", str(config.get("map_source", "sift-master"))])
        add(["--host", str(config.get("host", "127.0.0.1"))])
        add(["--port", str(config.get("port", 8080))])
        add(["--modern-dashboard-port", str(config.get("modern_dashboard_port", 8090))])
        add(["--video-source", str(config.get("video_source", "udp"))])
        add(["--udp-port", str(config.get("udp_port", 5600))])
        add(["--mavlink", str(config.get("mavlink", DEFAULTS["mavlink"]))])
        add(["--telemetry-seed-source", str(config.get("telemetry_seed_source", "global"))])
        add(["--search-radius-m", str(config.get("search_radius_m", 150.0))])
        add(["--max-search-radius-m", str(config.get("max_search_radius_m", 300.0))])
        add(["--max-tiles-per-scale", str(config.get("max_tiles_per_scale", 9))])
        add(["--early-stop-inliers", str(config.get("early_stop_inliers", 160))])
        add(["--min-nav-inliers", str(config.get("min_nav_inliers", 80))])
        add(["--log-csv", str(config.get("log_csv", "live_sift_dashboard.csv"))])

        if config.get("run_mode"):
            add(["--run-mode", str(config["run_mode"])])
        if config.get("validation_log_csv"):
            add(["--validation-log-csv", str(config["validation_log_csv"])])
        if config.get("resize_w") not in (None, "", 0, "0"):
            add(["--resize-w", str(config["resize_w"])])
        if float(config.get("duration_sec") or 0) > 0:
            add(["--duration-sec", str(config["duration_sec"])])
        if config.get("send_vision"):
            add(["--send-vision"])
        else:
            add(["--no-send-vision"])
        if config.get("send_vision_speed") is False:
            add(["--no-send-vision-speed"])
        if config.get("gazebo_truth"):
            add(["--gazebo-truth"])
            add(["--gz-topic", str(config.get("gz_topic", DEFAULTS["gz_topic"]))])
            add(["--gz-model", str(config.get("gz_model", DEFAULTS["gz_model"]))])

        optional_keys = {
            "vision_publish_mode": "--vision-publish-mode",
            "vision_stream_mode": "--vision-stream-mode",
            "vision_z_source": "--vision-z-source",
            "vision_attitude_source": "--vision-attitude-source",
            "vision_timestamp_source": "--vision-timestamp-source",
            "vision_rate_hz": "--vision-rate-hz",
            "vision_kf_config": "--vision-kf-config",
        }
        for key, flag in optional_keys.items():
            if config.get(key) not in (None, ""):
                add([flag, str(config[key])])

        if extra_args.strip():
            add(shlex.split(extra_args))
        return command

    def _check_ports_available(self, config: dict[str, Any]) -> None:
        host = str(config.get("host", DEFAULTS["host"]))
        checks = [
            ("old dashboard", config.get("port", DEFAULTS["port"])),
            ("live API", config.get("modern_dashboard_port", DEFAULTS["modern_dashboard_port"])),
        ]
        conflicts: list[str] = []
        for label, value in checks:
            try:
                port = int(value)
            except (TypeError, ValueError):
                continue
            if port <= 0:
                continue
            if not self._can_bind(host, port):
                conflicts.append(f"{label} {host}:{port}")
        if conflicts:
            detail = (
                "Port already in use: "
                + ", ".join(conflicts)
                + ". Stop the existing live_sift_nav process or choose different ports."
            )
            raise HTTPException(status_code=409, detail=detail)

    @staticmethod
    def _can_bind(host: str, port: int) -> bool:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
        except OSError:
            return False
        finally:
            sock.close()
        return True

    def _live_api_url(self) -> str:
        host = self.config.get("host", DEFAULTS["host"])
        port = self.config.get("modern_dashboard_port", DEFAULTS["modern_dashboard_port"])
        return f"http://{host}:{port}"

    def _old_dashboard_url(self) -> str:
        host = self.config.get("host", DEFAULTS["host"])
        port = self.config.get("port", DEFAULTS["port"])
        return f"http://{host}:{port}"

    def _dummy_vision_status_locked(self) -> dict[str, Any]:
        if self._dummy_vision is None:
            return {"running": False, "sent_count": 0}
        return self._dummy_vision.snapshot()


def send_gimbal_mount_control(
    connection_string: str,
    pitch_deg: float,
    roll_deg: float,
    yaw_deg: float,
    timeout_sec: float,
) -> dict[str, Any]:
    try:
        from pymavlink import mavutil
    except Exception as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=f"pymavlink import failed: {exc}") from exc

    conn = None
    try:
        conn = mavutil.mavlink_connection(
            connection_string,
            autoreconnect=True,
            source_system=246,
            source_component=190,
        )
        hb = conn.wait_heartbeat(timeout=timeout_sec)
        if hb is None:
            raise HTTPException(status_code=504, detail="MAVLink heartbeat timeout")

        target_system = int(conn.target_system or 1)
        target_component = int(conn.target_component or 1)
        command = mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL
        mount_mode = mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING

        conn.mav.command_long_send(
            target_system,
            target_component,
            command,
            0,
            float(pitch_deg),
            float(roll_deg),
            float(yaw_deg),
            0,
            0,
            0,
            float(mount_mode),
        )

        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout_sec)
        ack_command = int(getattr(ack, "command", command)) if ack is not None else None
        ack_result = int(getattr(ack, "result", -1)) if ack is not None else None
        ack_name = mavutil.mavlink.enums.get("MAV_RESULT", {}).get(ack_result)
        return {
            "status": "sent",
            "mavlink": connection_string,
            "target_system": target_system,
            "target_component": target_component,
            "command": command,
            "ack_command": ack_command,
            "ack_result": ack_result,
            "ack_result_name": ack_name.name if ack_name else None,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,
            "yaw_deg": yaw_deg,
        }
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"gimbal command failed: {exc}") from exc
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass


def send_ekf_source_set(connection_string: str, source_set: int, timeout_sec: float) -> dict[str, Any]:
    try:
        from pymavlink import mavutil
    except Exception as exc:  # pragma: no cover
        raise HTTPException(status_code=500, detail=f"pymavlink import failed: {exc}") from exc

    conn = None
    try:
        conn = mavutil.mavlink_connection(
            connection_string,
            autoreconnect=True,
            source_system=246,
            source_component=188,
        )
        hb = conn.wait_heartbeat(timeout=timeout_sec)
        if hb is None:
            raise HTTPException(status_code=504, detail="MAVLink heartbeat timeout")

        target_system = int(conn.target_system or 1)
        target_component = int(conn.target_component or 1)
        command = getattr(mavutil.mavlink, "MAV_CMD_SET_EKF_SOURCE_SET", 42007)
        conn.mav.command_long_send(
            target_system,
            target_component,
            command,
            0,
            float(source_set),
            0,
            0,
            0,
            0,
            0,
            0,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout_sec)
        ack_command = int(getattr(ack, "command", command)) if ack is not None else None
        ack_result = int(getattr(ack, "result", -1)) if ack is not None else None
        ack_name = mavutil.mavlink.enums.get("MAV_RESULT", {}).get(ack_result)
        return {
            "status": "sent",
            "mavlink": connection_string,
            "target_system": target_system,
            "target_component": target_component,
            "command": command,
            "source_set": source_set,
            "ack_command": ack_command,
            "ack_result": ack_result,
            "ack_result_name": ack_name.name if ack_name else None,
        }
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"source-set command failed: {exc}") from exc
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass


def create_launcher_app(repo_root: Path, static_dir: Optional[Path] = None) -> FastAPI:
    launcher = Launcher(repo_root)
    app = FastAPI(title="SAU SIFT Launcher", version="0.1.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/api/launcher/config")
    def get_config() -> dict[str, Any]:
        return {"defaults": DEFAULTS, "fields": CONFIG_FIELDS, "presets": PRESETS}

    @app.get("/api/launcher/status")
    def get_status() -> dict[str, Any]:
        return launcher.status()

    @app.post("/api/launcher/start")
    def start(request: StartRequest) -> dict[str, Any]:
        return launcher.start(request)

    @app.post("/api/launcher/stop")
    def stop() -> dict[str, Any]:
        return launcher.stop()

    @app.post("/api/launcher/gimbal/down")
    def gimbal_down(request: GimbalRequest) -> dict[str, Any]:
        return launcher.point_gimbal(request)

    @app.post("/api/launcher/dummy-vision/start")
    def dummy_vision_start(request: DummyVisionRequest) -> dict[str, Any]:
        return launcher.start_dummy_vision(request)

    @app.post("/api/launcher/dummy-vision/stop")
    def dummy_vision_stop() -> dict[str, Any]:
        return launcher.stop_dummy_vision()

    @app.post("/api/launcher/vpe/off")
    def vpe_off() -> dict[str, Any]:
        return launcher.stop_dummy_vision()

    @app.post("/api/launcher/source-set")
    def source_set(request: SourceSetRequest) -> dict[str, Any]:
        return launcher.switch_source_set(request)

    @app.websocket("/ws/launcher")
    async def ws_launcher(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(launcher.status())
                await asyncio.sleep(1.0)
        except WebSocketDisconnect:
            return

    if static_dir and static_dir.exists():
        assets_dir = static_dir / "assets"
        index_file = static_dir / "index.html"
        if assets_dir.exists():
            app.mount("/assets", StaticFiles(directory=assets_dir), name="dashboard_assets")
        if index_file.exists():
            @app.get("/")
            def dashboard_index() -> FileResponse:
                return FileResponse(index_file)

            @app.get("/{path:path}")
            def dashboard_fallback(path: str) -> FileResponse:
                if path.startswith(("api/", "ws/", "assets/")):
                    raise HTTPException(status_code=404, detail="not found")
                return FileResponse(index_file)

    return app


def run_launcher(host: str = "127.0.0.1", port: int = 8099, static_dir: str = "dashboard/dist") -> None:
    import uvicorn

    app = create_launcher_app(Path.cwd(), Path(static_dir))
    uvicorn.run(app, host=host, port=port, log_level="info")
