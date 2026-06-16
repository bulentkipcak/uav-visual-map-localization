from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Dict, Optional

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except Exception:  # pragma: no cover - handled at runtime
    mavutil = None

from .state import SharedState
from .logging import append_vision_publish_log


EKF_EXTERNAL_NAV_XY_PARAMS: Dict[str, float] = {
    "VISO_TYPE": 1,
    "VISO_POS_M_NSE": 2.0,
    "VISO_VEL_M_NSE": 2.0,
    "AHRS_EKF_TYPE": 3,
    "EK3_ENABLE": 1,
    "EK2_ENABLE": 0,
    "EK3_SRC1_POSXY": 6,
    "EK3_SRC1_VELXY": 0,
    "EK3_SRC1_POSZ": 1,
    "EK3_SRC1_VELZ": 0,
    "EK3_SRC1_YAW": 1,
    "EK3_SRC_OPTIONS": 0,
}


class MavlinkBridge(threading.Thread):
    def __init__(
        self,
        state: SharedState,
        connection_string: str,
        configure_ekf: bool = False,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.connection_string = connection_string
        self.configure_ekf = configure_ekf
        self.stop_event = threading.Event()
        self.conn: Any = None
        self.conn_lock = threading.Lock()
        self.target_system = 1
        self.target_component = 1
        self.params_sent = False

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        if mavutil is None:
            self.state.update_telemetry({"status": "pymavlink_missing"})
            self.state.set_error("pymavlink is not installed")
            return

        try:
            self.conn = mavutil.mavlink_connection(
                self.connection_string,
                autoreconnect=True,
                source_system=245,
                source_component=191,
            )
        except Exception as exc:
            self.state.update_telemetry({"status": "connect_failed"})
            self.state.set_error(f"MAVLink open failed: {exc}")
            return

        self.state.update_telemetry({"status": "listening", "connection": self.connection_string})
        self._wait_heartbeat_and_configure()

        while not self.stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1.0)
            except Exception as exc:
                self.state.update_telemetry({"status": "read_error"})
                self.state.set_error(f"MAVLink read error: {exc}")
                time.sleep(1.0)
                continue

            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            self._handle_message(msg)

    def send_vision_position(
        self,
        north: float,
        east: float,
        down: float,
        roll: float,
        pitch: float,
        yaw: float,
        reset_counter: int,
        usec: Optional[int] = None,
    ) -> bool:
        if self.conn is None:
            return False

        usec = int(usec if usec is not None else time.time() * 1_000_000)
        covariance = [float("nan")] + [0.0] * 20
        try:
            with self.conn_lock:
                self.conn.mav.vision_position_estimate_send(
                    usec,
                    float(north),
                    float(east),
                    float(down),
                    float(roll),
                    float(pitch),
                    float(yaw),
                    covariance,
                    int(reset_counter) & 0xFF,
                )
        except TypeError:
            with self.conn_lock:
                self.conn.mav.vision_position_estimate_send(
                    usec,
                    float(north),
                    float(east),
                    float(down),
                    float(roll),
                    float(pitch),
                    float(yaw),
                )
        except Exception as exc:
            self.state.set_error(f"VISION_POSITION_ESTIMATE send failed: {exc}")
            return False

        return True

    def send_vision_speed(
        self,
        vx: float,
        vy: float,
        vz: float,
        reset_counter: int,
    ) -> bool:
        if self.conn is None:
            return False

        usec = int(time.time() * 1_000_000)
        covariance = [float("nan")] + [0.0] * 8
        try:
            with self.conn_lock:
                self.conn.mav.vision_speed_estimate_send(
                    usec,
                    float(vx),
                    float(vy),
                    float(vz),
                    covariance,
                    int(reset_counter) & 0xFF,
                )
        except TypeError:
            with self.conn_lock:
                self.conn.mav.vision_speed_estimate_send(
                    usec,
                    float(vx),
                    float(vy),
                    float(vz),
                )
        except Exception as exc:
            self.state.set_error(f"VISION_SPEED_ESTIMATE send failed: {exc}")
            return False

        return True

    def _wait_heartbeat_and_configure(self) -> None:
        if self.conn is None:
            return
        try:
            hb = self.conn.wait_heartbeat(timeout=5)
            if hb is not None:
                self.target_system = int(self.conn.target_system or self.target_system)
                self.target_component = int(self.conn.target_component or self.target_component)
                self.state.update_telemetry(
                    {
                        "status": "heartbeat",
                        "target_system": self.target_system,
                        "target_component": self.target_component,
                    }
                )
        except Exception as exc:
            self.state.set_error(f"Heartbeat wait failed: {exc}")

        if self.configure_ekf:
            self.send_external_nav_xy_params()

    def send_external_nav_xy_params(self) -> None:
        if self.conn is None or mavutil is None:
            return

        with self.conn_lock:
            for name, value in EKF_EXTERNAL_NAV_XY_PARAMS.items():
                self.conn.mav.param_set_send(
                    self.target_system,
                    self.target_component,
                    name.encode("ascii"),
                    float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                )
                time.sleep(0.05)
        self.params_sent = True
        self.state.update_telemetry({"ekf_params_sent": True})

    def _handle_message(self, msg: Any) -> None:
        received_at = time.time()
        msg_type = msg.get_type()
        patch: Dict[str, Any] = {
            "status": "receiving",
            "last_message": msg_type,
            "last_message_time": received_at,
        }

        if msg_type == "HEARTBEAT":
            try:
                self.target_system = int(msg.get_srcSystem())
                self.target_component = int(msg.get_srcComponent())
            except Exception:
                pass
            patch["heartbeat"] = {
                "target_system": self.target_system,
                "target_component": self.target_component,
            }
        elif msg_type == "LOCAL_POSITION_NED":
            patch["local_ned"] = {
                "north": float(msg.x),
                "east": float(msg.y),
                "down": float(msg.z),
                "vx": float(msg.vx),
                "vy": float(msg.vy),
                "vz": float(msg.vz),
                "timestamp": received_at,
            }
        elif msg_type == "GLOBAL_POSITION_INT":
            patch["global_position"] = {
                "lat": float(msg.lat) / 1e7,
                "lon": float(msg.lon) / 1e7,
                "relative_alt_m": float(msg.relative_alt) / 1000.0,
                "alt_m": float(msg.alt) / 1000.0,
                "heading_deg": None if msg.hdg == 65535 else float(msg.hdg) / 100.0,
                "timestamp": received_at,
            }
        elif msg_type == "ATTITUDE":
            patch["attitude"] = {
                "roll_rad": float(msg.roll),
                "pitch_rad": float(msg.pitch),
                "yaw_rad": float(msg.yaw),
                "roll_deg": math.degrees(float(msg.roll)),
                "pitch_deg": math.degrees(float(msg.pitch)),
                "yaw_deg": math.degrees(float(msg.yaw)),
                "timestamp": received_at,
            }
        elif msg_type == "VFR_HUD":
            patch["vfr_hud"] = {
                "heading_deg": float(msg.heading),
                "groundspeed_mps": float(msg.groundspeed),
                "airspeed_mps": float(msg.airspeed),
                "alt_m": float(msg.alt),
                "timestamp": received_at,
            }

        self.state.update_telemetry(patch)


class VisionMavlinkSender(threading.Thread):
    def __init__(
        self,
        state: SharedState,
        connection_string: str,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.connection_string = connection_string
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.conn: Any = None
        self.conn_lock = threading.Lock()
        self.target_system = 1
        self.target_component = 1

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        if mavutil is None:
            self.state.update_vision_tx({"status": "pymavlink_missing"})
            self.state.set_error("pymavlink is not installed")
            return

        try:
            self.conn = mavutil.mavlink_connection(
                self.connection_string,
                autoreconnect=True,
                source_system=246,
                source_component=192,
            )
        except Exception as exc:
            self.state.update_vision_tx({"status": "connect_failed"})
            self.state.set_error(f"Vision MAVLink open failed: {exc}")
            return

        self.state.update_vision_tx({"status": "connecting", "connection": self.connection_string})
        try:
            hb = self.conn.wait_heartbeat(timeout=5)
            if hb is not None:
                self.target_system = int(self.conn.target_system or self.target_system)
                self.target_component = int(self.conn.target_component or self.target_component)
                self.ready.set()
                self.state.update_vision_tx(
                    {
                        "status": "connected",
                        "connection": self.connection_string,
                        "target_system": self.target_system,
                        "target_component": self.target_component,
                    }
                )
        except Exception as exc:
            self.state.set_error(f"Vision heartbeat wait failed: {exc}")

        while not self.stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as exc:
                self.state.set_error(f"Vision MAVLink drain failed: {exc}")
                time.sleep(1.0)
                continue
            if msg is None:
                continue

    def send_vision_position(
        self,
        north: float,
        east: float,
        down: float,
        roll: float,
        pitch: float,
        yaw: float,
        reset_counter: int,
        usec: Optional[int] = None,
    ) -> bool:
        if self.conn is None or not self.ready.is_set():
            return False

        usec = int(usec if usec is not None else time.time() * 1_000_000)
        covariance = [float("nan")] + [0.0] * 20
        try:
            with self.conn_lock:
                self.conn.mav.vision_position_estimate_send(
                    usec,
                    float(north),
                    float(east),
                    float(down),
                    float(roll),
                    float(pitch),
                    float(yaw),
                    covariance,
                    int(reset_counter) & 0xFF,
                )
        except TypeError:
            with self.conn_lock:
                self.conn.mav.vision_position_estimate_send(
                    usec,
                    float(north),
                    float(east),
                    float(down),
                    float(roll),
                    float(pitch),
                    float(yaw),
                )
        except Exception as exc:
            self.state.set_error(f"VISION_POSITION_ESTIMATE send failed: {exc}")
            return False

        return True

    def send_vision_speed(
        self,
        vx: float,
        vy: float,
        vz: float,
        reset_counter: int,
    ) -> bool:
        if self.conn is None or not self.ready.is_set():
            return False

        usec = int(time.time() * 1_000_000)
        covariance = [float("nan")] + [0.0] * 8
        try:
            with self.conn_lock:
                self.conn.mav.vision_speed_estimate_send(
                    usec,
                    float(vx),
                    float(vy),
                    float(vz),
                    covariance,
                    int(reset_counter) & 0xFF,
                )
        except TypeError:
            with self.conn_lock:
                self.conn.mav.vision_speed_estimate_send(
                    usec,
                    float(vx),
                    float(vy),
                    float(vz),
                )
        except Exception as exc:
            self.state.set_error(f"VISION_SPEED_ESTIMATE send failed: {exc}")
            return False

        return True


class VisionPublisherThread(threading.Thread):
    def __init__(
        self,
        state: SharedState,
        mavlink: Any,
        rate_hz: float = 8.0,
        max_age_sec: float = 10.0,
        allow_gps_seed: bool = False,
        send_speed: bool = False,
        velocity_source: str = "zero",
        z_source: str = "zero",
        attitude_source: str = "telemetry",
        publish_mode: str = "rate",
        timestamp_source: str = "frame",
        extra_predict_sec: float = 0.0,
        stream_mode: str = "event_based_raw",
        publish_log_csv: Optional[str] = None,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.mavlink = mavlink
        self.rate_hz = max(0.1, rate_hz)
        self.max_age_sec = max_age_sec
        self.allow_gps_seed = allow_gps_seed
        self.send_speed = send_speed
        self.velocity_source = velocity_source
        self.z_source = z_source
        self.attitude_source = attitude_source
        self.publish_mode = publish_mode
        self.timestamp_source = timestamp_source
        self.extra_predict_sec = max(0.0, extra_predict_sec)
        self.stream_mode = stream_mode
        self.publish_log_csv = publish_log_csv
        self.stop_event = threading.Event()
        self.sent_count = 0
        self.speed_sent_count = 0
        self.last_sent_nav_timestamp: Optional[float] = None
        self.last_publish_log_time: Optional[float] = None

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        period = 1.0 / self.rate_hz
        while not self.stop_event.is_set():
            started = time.time()
            self._send_once()
            elapsed = time.time() - started
            time.sleep(max(0.0, period - elapsed))

    def _send_once(self) -> None:
        send_time = time.time()
        publish_dt = None
        if self.last_publish_log_time is not None:
            publish_dt = send_time - self.last_publish_log_time
        self.last_publish_log_time = send_time
        effective_timestamp_source = self.timestamp_source
        effective_publish_mode = self.publish_mode
        if self.stream_mode == "fixed_rate_kalman_predict":
            pose = self.state.current_kalman_vision_pose(max_age_sec=self.max_age_sec)
            effective_timestamp_source = "send"
            effective_publish_mode = "rate"
        else:
            predict_position = self.stream_mode not in {"event_based_raw", "fixed_rate_hold_last"}
            pose = self.state.current_vision_pose(
                max_age_sec=self.max_age_sec,
                allow_gps_seed=self.allow_gps_seed,
                velocity_source=self.velocity_source,
                predict_position=predict_position,
                extra_predict_sec=self.extra_predict_sec,
            )
            if self.stream_mode == "event_based_raw":
                effective_publish_mode = "fix"
                effective_timestamp_source = "frame"
            elif self.stream_mode == "fixed_rate_hold_last":
                effective_publish_mode = "rate"
                effective_timestamp_source = "send"
        snapshot = self.state.snapshot()
        attitude = snapshot.get("telemetry", {}).get("attitude", {})
        if pose is None:
            nav = snapshot.get("nav_estimate") or {}
            gates = snapshot.get("nav_gates") or {}
            alignment = snapshot.get("vision_alignment")
            nav_age = None
            if nav.get("timestamp") is not None:
                nav_age = max(0.0, time.time() - float(nav["timestamp"]))
            status = "stale_estimate" if nav_age is not None else "waiting_estimate"
            if (
                nav_age is not None
                and gates.get("vision_align_source") not in (None, "", "none")
                and not alignment
            ):
                status = "waiting_alignment"
            self.state.update_vision_tx(
                {
                    "status": status,
                    "rate_hz": self.rate_hz,
                    "age_sec": nav_age,
                    "source": None,
                    "speed_source": self.velocity_source,
                    "z_source": self.z_source,
                    "attitude_source": self.attitude_source,
                    "publish_mode": effective_publish_mode,
                    "stream_mode": self.stream_mode,
                    "timestamp_source": effective_timestamp_source,
                    "extra_predict_sec": self.extra_predict_sec,
                    "publish_time": send_time,
                    "publish_dt_s": publish_dt,
                    "align_source": gates.get("vision_align_source"),
                }
            )
            self._log_publish(
                {
                    "send_time": send_time,
                    "status": "skipped",
                    "reason": status,
                    "sent_count": self.sent_count,
                    "publish_dt_s": publish_dt,
                    "publish_mode": effective_publish_mode,
                    "stream_mode": self.stream_mode,
                }
            )
            return

        nav_timestamp = pose.get("nav_timestamp")
        if effective_publish_mode == "fix":
            if nav_timestamp is None:
                self.state.update_vision_tx(
                    {
                        "status": "waiting_visual_fix",
                        "rate_hz": self.rate_hz,
                        "send_speed": self.send_speed,
                        "speed_source": self.velocity_source,
                        "z_source": self.z_source,
                        "attitude_source": self.attitude_source,
                        "publish_mode": effective_publish_mode,
                        "stream_mode": self.stream_mode,
                        "timestamp_source": effective_timestamp_source,
                        "extra_predict_sec": self.extra_predict_sec,
                        "publish_time": send_time,
                        "publish_dt_s": publish_dt,
                    }
                )
                self._log_publish(
                    {
                        "send_time": send_time,
                        "status": "skipped",
                        "reason": "waiting_visual_fix",
                        "sent_count": self.sent_count,
                        "publish_dt_s": publish_dt,
                        "publish_mode": effective_publish_mode,
                        "stream_mode": self.stream_mode,
                    }
                )
                return
            nav_timestamp = float(nav_timestamp)
            if self.last_sent_nav_timestamp == nav_timestamp:
                self.state.update_vision_tx(
                    {
                        "status": "waiting_new_fix",
                        "rate_hz": self.rate_hz,
                        "send_speed": self.send_speed,
                        "speed_source": self.velocity_source,
                        "z_source": self.z_source,
                        "attitude_source": self.attitude_source,
                        "publish_mode": effective_publish_mode,
                        "stream_mode": self.stream_mode,
                        "timestamp_source": effective_timestamp_source,
                        "extra_predict_sec": self.extra_predict_sec,
                        "source": pose["source"],
                        "age_sec": pose["age_sec"],
                        "sent_count": self.sent_count,
                        "publish_time": send_time,
                        "publish_dt_s": publish_dt,
                    }
                )
                self._log_publish(
                    {
                        "send_time": send_time,
                        "status": "skipped",
                        "reason": "waiting_new_fix",
                        "sent_count": self.sent_count,
                        "publish_dt_s": publish_dt,
                        "publish_mode": effective_publish_mode,
                        "stream_mode": self.stream_mode,
                        "source": pose.get("source"),
                        "frame_id": pose.get("frame_id"),
                    }
                )
                return

        roll = pitch = yaw = 0.0
        if self.attitude_source == "telemetry" and attitude.get("fresh", True):
            roll = float(attitude.get("roll_rad") or 0.0)
            pitch = float(attitude.get("pitch_rad") or 0.0)
            yaw = float(attitude.get("yaw_rad") or 0.0)

        usec = None
        if effective_timestamp_source == "frame" and nav_timestamp is not None:
            usec = int(float(nav_timestamp) * 1_000_000)
        vpe_usec = int(usec if usec is not None else send_time * 1_000_000)

        down = 0.0 if self.z_source == "zero" else float(pose["down"])
        pos_ok = self.mavlink.send_vision_position(
            north=float(pose["north"]),
            east=float(pose["east"]),
            down=down,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            reset_counter=int(pose["reset_counter"]),
            usec=vpe_usec,
        )
        speed_ok = True
        if self.send_speed:
            speed_ok = self.mavlink.send_vision_speed(
                vx=float(pose.get("vx") or 0.0),
                vy=float(pose.get("vy") or 0.0),
                vz=float(pose.get("vz") or 0.0),
                reset_counter=int(pose["reset_counter"]),
            )
        if pos_ok and speed_ok:
            self.sent_count += 1
            if self.send_speed:
                self.speed_sent_count += 1
            if effective_publish_mode == "fix" and nav_timestamp is not None:
                self.last_sent_nav_timestamp = float(nav_timestamp)
            kf = pose.get("kf") or {}
            self.state.update_vision_tx(
                {
                    "status": "sending",
                    "sent_count": self.sent_count,
                    "speed_sent_count": self.speed_sent_count,
                    "rate_hz": self.rate_hz,
                    "send_speed": self.send_speed,
                    "speed_source": self.velocity_source,
                    "z_source": self.z_source,
                    "attitude_source": self.attitude_source,
                    "publish_mode": effective_publish_mode,
                    "stream_mode": self.stream_mode,
                    "timestamp_source": effective_timestamp_source,
                    "extra_predict_sec": self.extra_predict_sec,
                    "source": pose["source"],
                    "north": pose["north"],
                    "east": pose["east"],
                    "map_north": pose.get("map_north"),
                    "map_east": pose.get("map_east"),
                    "down": down,
                    "vx": pose.get("vx"),
                    "vy": pose.get("vy"),
                    "vz": pose.get("vz"),
                    "age_sec": pose["age_sec"],
                    "prediction_horizon_sec": pose.get("prediction_horizon_sec"),
                    "pose_extra_predict_sec": pose.get("extra_predict_sec"),
                    "frame_id": pose.get("frame_id"),
                    "nav_timestamp": pose.get("nav_timestamp"),
                    "vision_timestamp_source": effective_timestamp_source,
                    "reset_counter": pose["reset_counter"],
                    "alignment": pose.get("alignment"),
                    "vpe_usec": vpe_usec,
                    "publish_time": send_time,
                    "publish_dt_s": publish_dt,
                    "kf": kf,
                }
            )
            self._log_publish(
                {
                    "send_time": send_time,
                    "status": "sent",
                    "sent_count": self.sent_count,
                    "vpe_usec": vpe_usec,
                    "vpe_N": pose.get("north"),
                    "vpe_E": pose.get("east"),
                    "kf_N": kf.get("north", pose.get("north")),
                    "kf_E": kf.get("east", pose.get("east")),
                    "kf_vN": kf.get("vn", pose.get("vx")),
                    "kf_vE": kf.get("ve", pose.get("vy")),
                    "last_fix_age_s": pose.get("age_sec"),
                    "publish_dt_s": publish_dt,
                    "publish_mode": effective_publish_mode,
                    "stream_mode": self.stream_mode,
                    "source": pose.get("source"),
                    "frame_id": pose.get("frame_id"),
                    "reset_counter": pose.get("reset_counter"),
                    "pos_ok": pos_ok,
                    "speed_ok": speed_ok,
                }
            )
        else:
            self.state.update_vision_tx(
                {
                    "status": "send_failed",
                    "rate_hz": self.rate_hz,
                    "position_ok": pos_ok,
                    "speed_ok": speed_ok,
                    "publish_time": send_time,
                    "publish_dt_s": publish_dt,
                }
            )
            self._log_publish(
                {
                    "send_time": send_time,
                    "status": "send_failed",
                    "sent_count": self.sent_count,
                    "vpe_usec": vpe_usec,
                    "vpe_N": pose.get("north"),
                    "vpe_E": pose.get("east"),
                    "last_fix_age_s": pose.get("age_sec"),
                    "publish_dt_s": publish_dt,
                    "publish_mode": effective_publish_mode,
                    "stream_mode": self.stream_mode,
                    "source": pose.get("source"),
                    "frame_id": pose.get("frame_id"),
                    "reset_counter": pose.get("reset_counter"),
                    "pos_ok": pos_ok,
                    "speed_ok": speed_ok,
                }
            )

    def _log_publish(self, record: Dict[str, Any]) -> None:
        if not self.publish_log_csv:
            return
        try:
            append_vision_publish_log(self.publish_log_csv, record)
        except Exception as exc:
            self.state.set_error(f"Vision publish log failed: {exc}")
