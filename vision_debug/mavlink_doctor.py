#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import Any, Dict, Iterable, Optional

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"pymavlink import failed: {exc}") from exc

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
except Exception as exc:  # pragma: no cover
    Pose_V = None
    Node = None
    GZ_IMPORT_ERROR: Optional[Exception] = exc
else:
    GZ_IMPORT_ERROR = None


EXTNAV_XY_PARAMS: Dict[str, float] = {
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

GPS_TAKEOFF_EXTNAV_PARAMS: Dict[str, float] = {
    "SIM_GPS_DISABLE": 0,
    "VISO_TYPE": 1,
    "VISO_POS_M_NSE": 2.0,
    "VISO_VEL_M_NSE": 2.0,
    "AHRS_EKF_TYPE": 3,
    "EK3_ENABLE": 1,
    "EK2_ENABLE": 0,
    "EK3_SRC1_POSXY": 3,
    "EK3_SRC1_VELXY": 3,
    "EK3_SRC1_POSZ": 1,
    "EK3_SRC1_VELZ": 3,
    "EK3_SRC1_YAW": 1,
    "EK3_SRC2_POSXY": 6,
    "EK3_SRC2_VELXY": 0,
    "EK3_SRC2_POSZ": 1,
    "EK3_SRC2_VELZ": 0,
    "EK3_SRC2_YAW": 1,
    "EK3_SRC_OPTIONS": 0,
}

STATUS_PARAM_NAMES = list(dict.fromkeys([*EXTNAV_XY_PARAMS.keys(), *GPS_TAKEOFF_EXTNAV_PARAMS.keys()]))
STATUS_PARAM_NAMES.extend(name for name in ("VISO_DELAY_MS",) if name not in STATUS_PARAM_NAMES)

EKF_FLAG_NAMES = [
    (1, "ATTITUDE"),
    (2, "VEL_HORIZ"),
    (4, "VEL_VERT"),
    (8, "POS_HORIZ_REL"),
    (16, "POS_HORIZ_ABS"),
    (32, "POS_VERT_ABS"),
    (64, "POS_VERT_AGL"),
    (128, "CONST_POS_MODE"),
    (256, "PRED_POS_HORIZ_REL"),
    (512, "PRED_POS_HORIZ_ABS"),
    (1024, "GPS_GLITCH"),
    (2048, "ACCEL_ERROR"),
]


WATCH_MESSAGE_IDS = [
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
    mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
]

WATCH_STREAMS = [
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
]


class GazeboPoseSubscriber:
    def __init__(self, topic: str, model: str) -> None:
        if Node is None:
            raise RuntimeError(f"Gazebo Python bindings import failed: {GZ_IMPORT_ERROR}")
        self.topic = topic
        self.model = model
        self.node = Node()
        self.lock = threading.Lock()
        self.latest: Optional[Dict[str, float]] = None

    def start(self) -> None:
        self.node.subscribe(Pose_V, self.topic, self._callback)

    def get(self) -> Optional[Dict[str, float]]:
        with self.lock:
            return dict(self.latest) if self.latest is not None else None

    def _callback(self, msg: Any, *_args: Any) -> None:
        pose = self._find_pose(msg)
        if pose is None:
            return
        with self.lock:
            self.latest = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
                "timestamp": time.time(),
            }

    def _find_pose(self, msg: Any) -> Optional[Any]:
        exact = [pose for pose in msg.pose if pose.name == self.model]
        if exact:
            return exact[0]
        leaf = [pose for pose in msg.pose if pose.name.split("::")[-1] == self.model]
        if leaf:
            return leaf[0]
        contains = [pose for pose in msg.pose if self.model in pose.name]
        return contains[0] if contains else None

REQUIRED_EKF_FLAGS = 8 | 16 | 256 | 512


def connect(args: argparse.Namespace) -> Any:
    conn = mavutil.mavlink_connection(
        args.mavlink,
        autoreconnect=True,
        source_system=args.source_system,
        source_component=args.source_component,
    )
    print(f"CONNECT {args.mavlink}")
    hb = conn.wait_heartbeat(timeout=args.heartbeat_timeout)
    if hb is None:
        raise SystemExit("HEARTBEAT timeout")
    print(
        "HEARTBEAT "
        f"target={conn.target_system}/{conn.target_component} "
        f"type={getattr(hb, 'type', '-')} autopilot={getattr(hb, 'autopilot', '-')} "
        f"mode={mavutil.mode_string_v10(hb)}"
    )
    return conn


def request_message_intervals(conn: Any, hz: float) -> None:
    request_data_streams(conn, hz)
    interval_us = int(1_000_000 / hz)
    for msg_id in WATCH_MESSAGE_IDS:
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(0.02)


def request_data_streams(conn: Any, hz: float) -> None:
    rate = max(1, int(round(hz)))
    components = sorted({int(conn.target_component or 0), 1})
    for component in components:
        for stream_id in WATCH_STREAMS:
            conn.mav.request_data_stream_send(
                conn.target_system,
                component,
                stream_id,
                rate,
                1,
            )
            time.sleep(0.02)


def param_name(raw: Any) -> str:
    if isinstance(raw, bytes):
        return raw.decode("ascii", errors="ignore").strip("\x00")
    return str(raw).strip("\x00")


def read_param(conn: Any, name: str, timeout: float = 2.0) -> Optional[float]:
    conn.mav.param_request_read_send(
        conn.target_system,
        conn.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg is None:
            continue
        if param_name(msg.param_id) == name:
            return float(msg.param_value)
    return None


def set_param(conn: Any, name: str, value: float) -> None:
    conn.mav.param_set_send(
        conn.target_system,
        conn.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def collect_messages(conn: Any, duration: float) -> Dict[str, Any]:
    latest: Dict[str, Any] = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        latest[msg.get_type()] = msg
        if msg.get_type() == "STATUSTEXT":
            print(format_message(msg))
    return latest


def format_message(msg: Any) -> str:
    t = msg.get_type()
    if t == "LOCAL_POSITION_NED":
        return (
            "LOCAL_POSITION_NED "
            f"x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f} "
            f"vx={msg.vx:.3f} vy={msg.vy:.3f} vz={msg.vz:.3f}"
        )
    if t == "GLOBAL_POSITION_INT":
        hdg = None if msg.hdg == 65535 else msg.hdg / 100.0
        return (
            "GLOBAL_POSITION_INT "
            f"lat={msg.lat / 1e7:.7f} lon={msg.lon / 1e7:.7f} "
            f"rel_alt={msg.relative_alt / 1000.0:.2f} hdg={hdg}"
        )
    if t == "ATTITUDE":
        return (
            "ATTITUDE "
            f"roll={math.degrees(msg.roll):.2f} "
            f"pitch={math.degrees(msg.pitch):.2f} "
            f"yaw={math.degrees(msg.yaw):.2f}"
        )
    if t == "EKF_STATUS_REPORT":
        fields = msg.to_dict()
        flags = int(fields.get("flags") or 0)
        parts = ["EKF_STATUS_REPORT"]
        for key in (
            "flags",
            "velocity_variance",
            "pos_horiz_variance",
            "pos_vert_variance",
            "compass_variance",
            "terrain_alt_variance",
        ):
            if key in fields:
                value = fields[key]
                if key == "flags":
                    parts.append(f"flags={flags}:{decode_ekf_flags(flags)}")
                elif isinstance(value, float):
                    parts.append(f"{key}={value:.4f}")
                else:
                    parts.append(f"{key}={value}")
        return " ".join(parts)
    if t == "VFR_HUD":
        return (
            "VFR_HUD "
            f"heading={msg.heading} groundspeed={msg.groundspeed:.2f} "
            f"alt={msg.alt:.2f}"
        )
    if t == "STATUSTEXT":
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        return f"STATUSTEXT severity={msg.severity} text={text}"
    return f"{t} {msg.to_dict()}"


def print_params(conn: Any, names: Iterable[str]) -> None:
    for name in names:
        value = read_param(conn, name)
        if value is None:
            print(f"PARAM {name}=<timeout>")
        else:
            print(f"PARAM {name}={value:g}")


def cmd_status(args: argparse.Namespace) -> int:
    conn = connect(args)
    request_message_intervals(conn, args.stream_hz)
    print_params(conn, STATUS_PARAM_NAMES)
    latest = collect_messages(conn, args.duration)
    for key in (
        "LOCAL_POSITION_NED",
        "GLOBAL_POSITION_INT",
        "ATTITUDE",
        "VFR_HUD",
        "EKF_STATUS_REPORT",
    ):
        msg = latest.get(key)
        if msg is None:
            print(f"{key} <not received>")
        else:
            print(format_message(msg))
    return 0


def cmd_set_params(args: argparse.Namespace) -> int:
    conn = connect(args)
    params = dict(EXTNAV_XY_PARAMS)
    if args.velxy_external:
        params["EK3_SRC1_VELXY"] = 6
    if args.velz_external:
        params["EK3_SRC1_VELZ"] = 6
    for name, value in params.items():
        print(f"SET {name}={value:g}")
        set_param(conn, name, value)
        time.sleep(args.param_delay)
    print("VERIFY")
    print_params(conn, params.keys())
    print("NOTE VISO_TYPE requires SITL reboot/restart.")
    return 0


def cmd_set_takeoff_switch_params(args: argparse.Namespace) -> int:
    conn = connect(args)
    params = dict(GPS_TAKEOFF_EXTNAV_PARAMS)
    if args.src2_velxy_external:
        params["EK3_SRC2_VELXY"] = 6
    for name, value in params.items():
        print(f"SET {name}={value:g}")
        set_param(conn, name, value)
        time.sleep(args.param_delay)
    print("VERIFY")
    print_params(conn, params.keys())
    print("NOTE VISO_TYPE and source changes should be tested after SITL reboot/restart.")
    return 0


def cmd_set_param(args: argparse.Namespace) -> int:
    conn = connect(args)
    name = args.name.upper()
    print(f"SET {name}={args.value:g}")
    set_param(conn, name, args.value)
    time.sleep(args.param_delay)
    if args.verify:
        print("VERIFY")
        print_params(conn, [name])
    return 0


def cmd_switch_source_set(args: argparse.Namespace) -> int:
    conn = connect(args)
    return 0 if send_source_set(conn, args.set, args.ack_timeout) else 1


def cmd_safe_switch_source_set(args: argparse.Namespace) -> int:
    conn = connect(args)
    request_message_intervals(conn, args.stream_hz)
    gz_sub = None
    gz_origin = None
    if args.gz_truth_guard:
        try:
            gz_sub = GazeboPoseSubscriber(args.gz_topic, args.gz_model)
            gz_sub.start()
        except Exception as exc:
            print(f"ABORT Gazebo truth guard unavailable: {exc}")
            return 2
        gz_deadline = time.time() + args.gz_pose_timeout
        while time.time() < gz_deadline:
            gz_pose = gz_sub.get()
            if gz_pose is not None:
                gz_origin = (float(gz_pose["x"]), float(gz_pose["y"]))
                print(
                    "GZ_GUARD origin="
                    f"({gz_origin[0]:.3f},{gz_origin[1]:.3f}) "
                    f"topic={args.gz_topic} model={args.gz_model}"
                )
                break
            time.sleep(0.05)
        if gz_origin is None:
            print("ABORT Gazebo truth guard timed out waiting for pose")
            return 2

    print(f"PRECHECK {args.precheck_sec:.1f}s")
    pre = collect_health(conn, args.precheck_sec)
    print_health("PRE", pre)
    if not health_ok(pre, args.max_velocity_variance, args.max_pos_horiz_variance):
        print("ABORT precheck unhealthy; source set not changed")
        return 2

    print(f"SWITCH set={args.to_set}")
    if not send_source_set(conn, args.to_set, args.ack_timeout):
        return 1

    local_origin = None
    pre_local = pre.get("LOCAL_POSITION_NED")
    if pre_local is not None:
        local_origin = (float(pre_local.x), float(pre_local.y))

    deadline = None if args.monitor_sec <= 0 else time.time() + args.monitor_sec
    rollback = False
    latest: Dict[str, Any] = {}
    next_print = 0.0
    try:
        while deadline is None or time.time() < deadline:
            drain_messages(conn, latest, timeout=0.2)
            now = time.time()
            if now >= next_print:
                next_print = now + 1.0
                print_health("MON", latest)
            failure_reason = health_failure_reason(
                latest,
                args.max_velocity_variance,
                args.max_pos_horiz_variance,
                max_local_drift_m=args.max_local_drift_m,
                max_xy_speed_mps=args.max_xy_speed_mps,
                local_origin=local_origin,
            )
            if failure_reason is None and gz_sub is not None and gz_origin is not None:
                gz_pose = gz_sub.get()
                if gz_pose is None:
                    failure_reason = "gz_pose_missing"
                else:
                    gz_age = now - float(gz_pose["timestamp"])
                    gz_drift = math.hypot(float(gz_pose["x"]) - gz_origin[0], float(gz_pose["y"]) - gz_origin[1])
                    if args.max_gz_pose_age_sec > 0.0 and gz_age > args.max_gz_pose_age_sec:
                        failure_reason = f"gz_pose_age={gz_age:.3f}>{args.max_gz_pose_age_sec:.3f}"
                    elif args.max_gz_drift_m > 0.0 and gz_drift > args.max_gz_drift_m:
                        failure_reason = f"gz_drift={gz_drift:.3f}>{args.max_gz_drift_m:.3f}"
            if failure_reason:
                print(f"ROLLBACK_REASON {failure_reason}")
                print_health("FAIL", latest)
                rollback = True
                break
    except KeyboardInterrupt:
        print("INTERRUPTED monitor stopped; source set is unchanged")
        return 130

    if rollback:
        print(f"ROLLBACK set={args.rollback_set}")
        send_source_set(conn, args.rollback_set, args.ack_timeout)
        return 3

    print("SAFE_SWITCH_OK")
    return 0


def cmd_stream_vision(args: argparse.Namespace) -> int:
    conn = connect(args)
    request_message_intervals(conn, args.stream_hz)
    latest: Dict[str, Any] = {}
    wait_for_pose(conn, latest, args, timeout=args.pose_timeout)

    frozen_pose = None
    if args.freeze:
        frozen_pose = pose_from_latest(latest, args)
        print(
            "FREEZE "
            f"x={frozen_pose['x']:.3f} y={frozen_pose['y']:.3f} z={frozen_pose['z']:.3f}"
        )

    sent = 0
    last_print = 0.0
    period = 1.0 / args.rate
    deadline = None if args.duration <= 0 else time.time() + args.duration
    stream_started = time.time()
    next_send = time.time()
    try:
        while deadline is None or time.time() < deadline:
            drain_messages(conn, latest, timeout=0.0)
            now = time.time()
            if now < next_send:
                time.sleep(min(0.02, next_send - now))
                continue
            pose = frozen_pose if frozen_pose is not None else pose_from_latest(
                latest,
                args,
                elapsed=now - stream_started,
            )
            attitude = latest.get("ATTITUDE")
            roll = pitch = yaw = 0.0
            if args.attitude_source == "telemetry":
                roll = float(getattr(attitude, "roll", 0.0))
                pitch = float(getattr(attitude, "pitch", 0.0))
                yaw = float(getattr(attitude, "yaw", 0.0))
            send_vision_position(conn, pose["x"], pose["y"], pose["z"], roll, pitch, yaw)
            if args.send_speed:
                velocity = velocity_from_latest(latest, args)
                send_vision_speed(conn, velocity["vx"], velocity["vy"], velocity["vz"])
            sent += 1
            next_send += period
            if now - last_print >= 1.0:
                last_print = now
                print_stream_line(sent, pose, latest)
    except KeyboardInterrupt:
        print("INTERRUPTED")
    print(f"DONE sent={sent}")
    return 0


def wait_for_pose(conn: Any, latest: Dict[str, Any], args: argparse.Namespace, timeout: float) -> None:
    deadline = time.time() + timeout
    attitude_required = args.attitude_source == "telemetry"
    while time.time() < deadline:
        drain_messages(conn, latest, timeout=0.5)
        if attitude_required and latest.get("ATTITUDE") is None:
            continue
        if args.pose_source == "manual":
            return
        if args.pose_source == "local" and args.bootstrap_sec > 0:
            return
        if args.pose_source == "local" and latest.get("LOCAL_POSITION_NED") is not None:
            return
    if attitude_required:
        raise SystemExit("Timed out waiting for ATTITUDE")
    raise SystemExit("Timed out waiting for pose")


def manual_pose_from_args(args: argparse.Namespace) -> Dict[str, float]:
    return {"x": float(args.x), "y": float(args.y), "z": float(args.z), "source": "manual"}


def drain_messages(conn: Any, latest: Dict[str, Any], timeout: float) -> None:
    while True:
        msg = conn.recv_match(blocking=timeout > 0, timeout=timeout)
        timeout = 0.0
        if msg is None:
            return
        if msg.get_type() == "BAD_DATA":
            continue
        latest[msg.get_type()] = msg
        if msg.get_type() == "STATUSTEXT":
            print(format_message(msg))
            if statustext_is_ekf_failure(msg):
                latest["_ekf_failure_text"] = statustext_text(msg)


def pose_from_latest(
    latest: Dict[str, Any],
    args: argparse.Namespace,
    elapsed: Optional[float] = None,
) -> Dict[str, float]:
    local = latest.get("LOCAL_POSITION_NED")
    if args.pose_source == "manual":
        z = float(args.z)
        if args.use_current_z and local is not None:
            z = float(local.z)
        return {"x": float(args.x), "y": float(args.y), "z": z, "source": "manual"}
    if args.bootstrap_sec > 0 and local is None:
        return manual_pose_from_args(args)
    if local is None:
        raise SystemExit("LOCAL_POSITION_NED missing")
    return {"x": float(local.x), "y": float(local.y), "z": float(local.z), "source": "local"}


def velocity_from_latest(latest: Dict[str, Any], args: argparse.Namespace) -> Dict[str, float]:
    local = latest.get("LOCAL_POSITION_NED")
    if local is not None:
        return {"vx": float(local.vx), "vy": float(local.vy), "vz": float(local.vz)}
    return {"vx": float(args.vx), "vy": float(args.vy), "vz": float(args.vz)}


def send_vision_position(
    conn: Any,
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> None:
    usec = int(time.time() * 1_000_000)
    covariance = [float("nan")] + [0.0] * 20
    try:
        conn.mav.vision_position_estimate_send(
            usec,
            float(x),
            float(y),
            float(z),
            float(roll),
            float(pitch),
            float(yaw),
            covariance,
            0,
        )
    except TypeError:
        conn.mav.vision_position_estimate_send(
            usec,
            float(x),
            float(y),
            float(z),
            float(roll),
            float(pitch),
            float(yaw),
        )


def send_vision_speed(conn: Any, vx: float, vy: float, vz: float) -> None:
    usec = int(time.time() * 1_000_000)
    covariance = [float("nan")] + [0.0] * 8
    try:
        conn.mav.vision_speed_estimate_send(
            usec,
            float(vx),
            float(vy),
            float(vz),
            covariance,
            0,
        )
    except TypeError:
        conn.mav.vision_speed_estimate_send(
            usec,
            float(vx),
            float(vy),
            float(vz),
        )


def print_stream_line(sent: int, pose: Dict[str, float], latest: Dict[str, Any]) -> None:
    local = latest.get("LOCAL_POSITION_NED")
    ekf = latest.get("EKF_STATUS_REPORT")
    local_text = "local=<none>"
    if local is not None:
        local_text = f"local=({local.x:.2f},{local.y:.2f},{local.z:.2f})"
    ekf_text = "ekf=<none>"
    if ekf is not None:
        fields = ekf.to_dict()
        flags = int(fields.get("flags") or 0)
        ekf_text = (
            f"ekf_flags={flags}:{decode_ekf_flags(flags)} "
            f"pos_h={fields.get('pos_horiz_variance', float('nan')):.4f}"
        )
    print(
        f"VISION sent={sent} "
        f"pose[{pose.get('source', '-') }]=({pose['x']:.2f},{pose['y']:.2f},{pose['z']:.2f}) "
        f"{local_text} {ekf_text}"
    )


def decode_ekf_flags(flags: int) -> str:
    active = [name for bit, name in EKF_FLAG_NAMES if flags & bit]
    return ",".join(active) if active else "-"


def send_source_set(conn: Any, source_set: int, ack_timeout: float) -> bool:
    command = getattr(mavutil.mavlink, "MAV_CMD_SET_EKF_SOURCE_SET", 42007)
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
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
    ack = wait_command_ack(conn, command, ack_timeout)
    if ack is None:
        print(f"COMMAND_ACK command={command} <timeout>")
        return False
    result = int(getattr(ack, "result", -1))
    print(f"COMMAND_ACK command={command} result={result}:{mav_result_name(result)}")
    return result == mavutil.mavlink.MAV_RESULT_ACCEPTED


def collect_health(conn: Any, duration: float) -> Dict[str, Any]:
    latest: Dict[str, Any] = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        drain_messages(conn, latest, timeout=0.2)
    return latest


def print_health(label: str, latest: Dict[str, Any]) -> None:
    heartbeat = latest.get("HEARTBEAT")
    local = latest.get("LOCAL_POSITION_NED")
    ekf = latest.get("EKF_STATUS_REPORT")
    mode = mavutil.mode_string_v10(heartbeat) if heartbeat is not None else "-"
    local_text = "local=-"
    if local is not None:
        local_text = (
            f"local=({local.x:.2f},{local.y:.2f},{local.z:.2f}) "
            f"vel=({local.vx:.2f},{local.vy:.2f},{local.vz:.2f})"
        )
    ekf_text = "ekf=-"
    if ekf is not None:
        fields = ekf.to_dict()
        flags = int(fields.get("flags") or 0)
        ekf_text = (
            f"flags={flags}:{decode_ekf_flags(flags)} "
            f"vel_var={float(fields.get('velocity_variance') or 0.0):.3f} "
            f"pos_var={float(fields.get('pos_horiz_variance') or 0.0):.3f}"
        )
    failure = latest.get("_ekf_failure_text")
    failure_text = f" failure={failure}" if failure else ""
    print(f"{label} mode={mode} {local_text} {ekf_text}{failure_text}")


def health_ok(latest: Dict[str, Any], max_velocity_variance: float, max_pos_horiz_variance: float) -> bool:
    return not health_failure_reason(
        latest,
        max_velocity_variance=max_velocity_variance,
        max_pos_horiz_variance=max_pos_horiz_variance,
        require_all_messages=True,
    )


def health_failed(
    latest: Dict[str, Any],
    max_velocity_variance: float,
    max_pos_horiz_variance: float,
    require_all_messages: bool = False,
) -> bool:
    return bool(
        health_failure_reason(
            latest,
            max_velocity_variance=max_velocity_variance,
            max_pos_horiz_variance=max_pos_horiz_variance,
            require_all_messages=require_all_messages,
        )
    )


def health_failure_reason(
    latest: Dict[str, Any],
    max_velocity_variance: float,
    max_pos_horiz_variance: float,
    require_all_messages: bool = False,
    max_local_drift_m: float = 0.0,
    max_xy_speed_mps: float = 0.0,
    local_origin: Optional[tuple[float, float]] = None,
) -> Optional[str]:
    heartbeat = latest.get("HEARTBEAT")
    local = latest.get("LOCAL_POSITION_NED")
    ekf = latest.get("EKF_STATUS_REPORT")
    failure_text = latest.get("_ekf_failure_text")
    if failure_text:
        return f"statustext={failure_text}"
    if require_all_messages and (local is None or ekf is None):
        missing = []
        if local is None:
            missing.append("LOCAL_POSITION_NED")
        if ekf is None:
            missing.append("EKF_STATUS_REPORT")
        return "missing=" + ",".join(missing)
    if heartbeat is not None and mavutil.mode_string_v10(heartbeat) == "LAND":
        return "mode=LAND"
    if local is not None and max_xy_speed_mps > 0:
        xy_speed = math.hypot(float(local.vx), float(local.vy))
        if xy_speed > max_xy_speed_mps:
            return f"xy_speed={xy_speed:.3f}>{max_xy_speed_mps:.3f}"
    if local is not None and local_origin is not None and max_local_drift_m > 0:
        local_drift = math.hypot(float(local.x) - local_origin[0], float(local.y) - local_origin[1])
        if local_drift > max_local_drift_m:
            return f"local_drift={local_drift:.3f}>{max_local_drift_m:.3f}"
    if ekf is None:
        return None
    fields = ekf.to_dict()
    flags = int(fields.get("flags") or 0)
    velocity_variance = float(fields.get("velocity_variance") or 0.0)
    pos_horiz_variance = float(fields.get("pos_horiz_variance") or 0.0)
    if (flags & REQUIRED_EKF_FLAGS) != REQUIRED_EKF_FLAGS:
        missing_flags = REQUIRED_EKF_FLAGS & ~flags
        return f"flags_missing={missing_flags}:{decode_ekf_flags(missing_flags)} current={flags}:{decode_ekf_flags(flags)}"
    if velocity_variance > max_velocity_variance:
        return f"velocity_variance={velocity_variance:.4f}>{max_velocity_variance:.4f}"
    if pos_horiz_variance > max_pos_horiz_variance:
        return f"pos_horiz_variance={pos_horiz_variance:.4f}>{max_pos_horiz_variance:.4f}"
    return None


def statustext_text(msg: Any) -> str:
    text = getattr(msg, "text", "")
    if isinstance(text, bytes):
        text = text.decode("utf-8", errors="replace")
    return str(text)


def statustext_is_ekf_failure(msg: Any) -> bool:
    text = statustext_text(msg).lower()
    patterns = (
        "ekf variance",
        "ekf failsafe",
        "stopped aiding",
        "bad position",
        "gps glitch",
        "compass error",
    )
    return any(pattern in text for pattern in patterns)


def mav_result_name(result: int) -> str:
    enum = getattr(mavutil.mavlink, "enums", {}).get("MAV_RESULT", {})
    item = enum.get(result)
    if item is not None:
        return str(item.name)
    return str(result)


def wait_command_ack(conn: Any, command: int, timeout: float) -> Optional[Any]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
        if msg is None:
            continue
        if int(getattr(msg, "command", -1)) == int(command):
            return msg
        print(format_message(msg))
    return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MAVLink ExternalNav debug helper",
        allow_abbrev=False,
    )
    parser.add_argument("--mavlink", default="udpin:127.0.0.1:14550")
    parser.add_argument("--source-system", type=int, default=246)
    parser.add_argument("--source-component", type=int, default=191)
    parser.add_argument("--heartbeat-timeout", type=float, default=8.0)
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="Read params and telemetry", allow_abbrev=False)
    status.add_argument("--duration", type=float, default=6.0)
    status.add_argument("--stream-hz", type=float, default=5.0)
    status.set_defaults(func=cmd_status)

    set_params = sub.add_parser("set-params", help="Set ExternalNav XY params", allow_abbrev=False)
    set_params.add_argument("--param-delay", type=float, default=0.08)
    set_params.add_argument("--velxy-external", action="store_true", help="Set EK3_SRC1_VELXY=6 for VISION_SPEED_ESTIMATE tests")
    set_params.add_argument("--velz-external", action="store_true", help="Set EK3_SRC1_VELZ=6 for VISION_SPEED_ESTIMATE tests")
    set_params.set_defaults(func=cmd_set_params)

    switch_params = sub.add_parser(
        "set-takeoff-switch-params",
        help="Set source set 1=GPS and source set 2=ExternalNav params",
        allow_abbrev=False,
    )
    switch_params.add_argument("--param-delay", type=float, default=0.08)
    switch_params.add_argument(
        "--src2-velxy-external",
        action="store_true",
        help="Set EK3_SRC2_VELXY=6 for VPE+VSE comparison tests; default is VPE-only with EK3_SRC2_VELXY=0",
    )
    switch_params.set_defaults(func=cmd_set_takeoff_switch_params)

    one_param = sub.add_parser("set-param", help="Set one MAVLink parameter and optionally verify it", allow_abbrev=False)
    one_param.add_argument("name")
    one_param.add_argument("value", type=float)
    one_param.add_argument("--param-delay", type=float, default=0.12)
    one_param.add_argument("--verify", action=argparse.BooleanOptionalAction, default=True)
    one_param.set_defaults(func=cmd_set_param)

    source_set = sub.add_parser(
        "switch-source-set",
        help="Switch EKF source set with MAV_CMD_SET_EKF_SOURCE_SET",
        allow_abbrev=False,
    )
    source_set.add_argument("--set", type=int, choices=[1, 2, 3], required=True)
    source_set.add_argument("--ack-timeout", type=float, default=3.0)
    source_set.set_defaults(func=cmd_switch_source_set)

    safe_source_set = sub.add_parser(
        "safe-switch-source-set",
        help="Switch EKF source set and auto-rollback on EKF variance or LAND",
        allow_abbrev=False,
    )
    safe_source_set.add_argument("--to-set", type=int, choices=[1, 2, 3], default=2)
    safe_source_set.add_argument("--rollback-set", type=int, choices=[1, 2, 3], default=1)
    safe_source_set.add_argument("--precheck-sec", type=float, default=3.0)
    safe_source_set.add_argument("--monitor-sec", type=float, default=60.0, help="Seconds to monitor after switching; 0 means until Ctrl-C")
    safe_source_set.add_argument("--stream-hz", type=float, default=10.0)
    safe_source_set.add_argument("--max-velocity-variance", type=float, default=0.45)
    safe_source_set.add_argument("--max-pos-horiz-variance", type=float, default=0.35)
    safe_source_set.add_argument(
        "--max-local-drift-m",
        type=float,
        default=0.0,
        help="Rollback if LOCAL_POSITION_NED XY drifts this many meters from the pre-switch local position; 0 disables",
    )
    safe_source_set.add_argument(
        "--max-xy-speed-mps",
        type=float,
        default=0.0,
        help="Rollback if LOCAL_POSITION_NED horizontal speed exceeds this value; 0 disables",
    )
    safe_source_set.add_argument(
        "--gz-truth-guard",
        action="store_true",
        help="Also monitor Gazebo model world XY drift and rollback if it exceeds --max-gz-drift-m",
    )
    safe_source_set.add_argument("--gz-topic", default="/world/iris_runway/pose/info")
    safe_source_set.add_argument("--gz-model", default="iris_with_gimbal")
    safe_source_set.add_argument("--gz-pose-timeout", type=float, default=5.0)
    safe_source_set.add_argument("--max-gz-pose-age-sec", type=float, default=1.0)
    safe_source_set.add_argument(
        "--max-gz-drift-m",
        type=float,
        default=0.0,
        help="Rollback if Gazebo model XY drifts this many world meters from pre-switch pose; 0 disables",
    )
    safe_source_set.add_argument("--ack-timeout", type=float, default=3.0)
    safe_source_set.set_defaults(func=cmd_safe_switch_source_set)

    stream = sub.add_parser("stream-vision", help="Send VISION_POSITION_ESTIMATE", allow_abbrev=False)
    stream.add_argument("--duration", type=float, default=30.0, help="Seconds to stream; 0 means until Ctrl-C")
    stream.add_argument("--rate", type=float, default=10.0)
    stream.add_argument("--stream-hz", type=float, default=10.0)
    stream.add_argument("--pose-timeout", type=float, default=8.0)
    stream.add_argument("--pose-source", "--source", dest="pose_source", choices=["local", "manual"], default="local")
    stream.add_argument("--freeze", action="store_true")
    stream.add_argument("--x", type=float, default=0.0)
    stream.add_argument("--y", type=float, default=0.0)
    stream.add_argument("--z", type=float, default=0.0)
    stream.add_argument("--vx", type=float, default=0.0)
    stream.add_argument("--vy", type=float, default=0.0)
    stream.add_argument("--vz", type=float, default=0.0)
    stream.add_argument("--send-speed", action="store_true", help="Also send VISION_SPEED_ESTIMATE")
    stream.add_argument("--attitude-source", choices=["telemetry", "zero"], default="telemetry")
    stream.add_argument("--bootstrap-sec", type=float, default=0.0, help="For local source, send manual pose while LOCAL_POSITION_NED is missing")
    stream.add_argument("--use-current-z", action="store_true")
    stream.set_defaults(func=cmd_stream_vision)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
