#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, Tuple

os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"pymavlink import failed: {exc}") from exc

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"Gazebo Python bindings import failed: {exc}") from exc


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


@dataclass
class GazeboPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    timestamp: float


class GazeboPoseSubscriber:
    def __init__(self, topic: str, model: str) -> None:
        self.topic = topic
        self.model = model
        self.node = Node()
        self.lock = threading.Lock()
        self.latest: Optional[GazeboPose] = None

    def start(self) -> None:
        self.node.subscribe(Pose_V, self.topic, self._callback)

    def get(self) -> Optional[GazeboPose]:
        with self.lock:
            return self.latest

    def _callback(self, msg: Pose_V, *_args: Any) -> None:
        pose = self._find_pose(msg)
        if pose is None:
            return
        with self.lock:
            self.latest = GazeboPose(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
                qx=float(pose.orientation.x),
                qy=float(pose.orientation.y),
                qz=float(pose.orientation.z),
                qw=float(pose.orientation.w),
                timestamp=time.time(),
            )

    def _find_pose(self, msg: Pose_V) -> Optional[Any]:
        exact = [pose for pose in msg.pose if pose.name == self.model]
        if exact:
            return exact[0]
        leaf = [pose for pose in msg.pose if pose.name.split("::")[-1] == self.model]
        if leaf:
            return leaf[0]
        contains = [pose for pose in msg.pose if self.model in pose.name]
        return contains[0] if contains else None


def connect_mavlink(args: argparse.Namespace) -> Any:
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
        f"mode={mavutil.mode_string_v10(hb)}"
    )
    request_message_intervals(conn, args.stream_hz)
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


def drain_mavlink(conn: Any, latest: dict[str, Any], timeout: float = 0.0) -> None:
    while True:
        msg = conn.recv_match(blocking=timeout > 0, timeout=timeout)
        timeout = 0.0
        if msg is None:
            return
        if msg.get_type() == "BAD_DATA":
            continue
        latest[msg.get_type()] = msg
        latest[f"_{msg.get_type()}_time"] = time.time()
        if msg.get_type() == "STATUSTEXT":
            text = msg.text.decode("utf-8", errors="replace") if isinstance(msg.text, bytes) else msg.text
            print(f"STATUSTEXT severity={msg.severity} text={text}")


def wait_for_bootstrap(
    gz_sub: GazeboPoseSubscriber,
    conn: Any,
    latest: dict[str, Any],
    timeout: float,
    attitude_required: bool,
) -> Tuple[GazeboPose, Any, Optional[Any]]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        drain_mavlink(conn, latest, timeout=0.1)
        gz_pose = gz_sub.get()
        local = latest.get("LOCAL_POSITION_NED")
        attitude = latest.get("ATTITUDE")
        if gz_pose is not None and local is not None and (attitude is not None or not attitude_required):
            return gz_pose, local, attitude
    if attitude_required:
        raise SystemExit("Timed out waiting for Gazebo pose + LOCAL_POSITION_NED + ATTITUDE")
    raise SystemExit("Timed out waiting for Gazebo pose + LOCAL_POSITION_NED")


def delta_world_to_ned(dx: float, dy: float, mode: str) -> Tuple[float, float]:
    if mode == "enu":
        return dy, dx
    if mode == "xy":
        return dx, dy
    if mode == "neg_enu":
        return -dy, -dx
    if mode == "neg_xy":
        return -dx, -dy
    raise ValueError(f"Unknown axis mode: {mode}")


def send_vision_position(
    conn: Any,
    north: float,
    east: float,
    down: float,
    roll: float,
    pitch: float,
    yaw: float,
    reset_counter: int,
) -> None:
    usec = int(time.time() * 1_000_000)
    covariance = [float("nan")] + [0.0] * 20
    try:
        conn.mav.vision_position_estimate_send(
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
        conn.mav.vision_position_estimate_send(
            usec,
            float(north),
            float(east),
            float(down),
            float(roll),
            float(pitch),
            float(yaw),
        )


def send_vision_speed(conn: Any, vn: float, ve: float, vd: float, reset_counter: int) -> None:
    usec = int(time.time() * 1_000_000)
    covariance = [float("nan")] + [0.0] * 8
    try:
        conn.mav.vision_speed_estimate_send(
            usec,
            float(vn),
            float(ve),
            float(vd),
            covariance,
            int(reset_counter) & 0xFF,
        )
    except TypeError:
        conn.mav.vision_speed_estimate_send(usec, float(vn), float(ve), float(vd))


def ekf_text(latest: dict[str, Any]) -> str:
    ekf = latest.get("EKF_STATUS_REPORT")
    if ekf is None:
        return "ekf=-"
    fields = ekf.to_dict()
    flags = int(fields.get("flags") or 0)
    return (
        f"ekf_flags={flags} "
        f"vel_var={float(fields.get('velocity_variance') or 0.0):.3f} "
        f"pos_var={float(fields.get('pos_horiz_variance') or 0.0):.3f}"
    )


def run(args: argparse.Namespace) -> int:
    gz_sub = GazeboPoseSubscriber(args.gz_topic, args.model)
    gz_sub.start()
    conn = connect_mavlink(args)
    latest: dict[str, Any] = {}

    ref_gz, ref_local, _attitude = wait_for_bootstrap(
        gz_sub,
        conn,
        latest,
        args.bootstrap_timeout,
        attitude_required=args.attitude_source == "telemetry",
    )
    print(
        "BOOTSTRAP "
        f"gz=({ref_gz.x:.3f},{ref_gz.y:.3f},{ref_gz.z:.3f}) "
        f"local=({ref_local.x:.3f},{ref_local.y:.3f},{ref_local.z:.3f}) "
        f"axis={args.axis} speed_source={args.speed_source} "
        f"attitude_source={args.attitude_source} "
        f"pose_delay={args.pose_delay_sec:.2f}s "
        f"bias=({args.pos_bias_north:.2f},{args.pos_bias_east:.2f}) "
        f"noise_std={args.pos_noise_std:.2f} "
        f"observe_only={args.observe_only}"
    )

    period = 1.0 / args.rate
    sent = 0
    reset_counter = 0
    previous_pose: Optional[GazeboPose] = None
    filtered_vn = 0.0
    filtered_ve = 0.0
    filtered_vd = 0.0
    next_send = time.time()
    next_print = 0.0
    next_stream_request = time.time() + args.stream_request_sec
    next_interval_request = time.time() + max(5.0, args.stream_request_sec)
    deadline = None if args.duration <= 0 else time.time() + args.duration
    pose_history: deque[Tuple[float, float, float, float]] = deque()
    rng = random.Random(args.noise_seed)
    noise_n = 0.0
    noise_e = 0.0

    try:
        while deadline is None or time.time() < deadline:
            drain_mavlink(conn, latest, timeout=args.mavlink_poll_timeout)
            now = time.time()
            if args.stream_request_sec > 0 and now >= next_stream_request:
                request_data_streams(conn, args.stream_hz)
                next_stream_request = now + args.stream_request_sec
            if args.stream_request_sec > 0 and now >= next_interval_request:
                request_message_intervals(conn, args.stream_hz)
                next_interval_request = now + max(5.0, args.stream_request_sec)
            if now < next_send:
                time.sleep(min(0.02, next_send - now))
                continue

            gz_pose = gz_sub.get()
            attitude = latest.get("ATTITUDE")
            local = latest.get("LOCAL_POSITION_NED")
            if gz_pose is None or (attitude is None and args.attitude_source == "telemetry"):
                next_send += period
                continue

            dn, de = delta_world_to_ned(gz_pose.x - ref_gz.x, gz_pose.y - ref_gz.y, args.axis)
            north = float(ref_local.x) + dn
            east = float(ref_local.y) + de
            down = float(local.z) if local is not None else float(ref_local.z) - (gz_pose.z - ref_gz.z)
            pose_history.append((now, north, east, down))
            if args.pose_delay_sec > 0:
                target_time = now - args.pose_delay_sec
                while len(pose_history) > 1 and pose_history[1][0] <= target_time:
                    pose_history.popleft()
                _sample_time, north, east, down = pose_history[0]
            while len(pose_history) > 2 and pose_history[0][0] < now - max(2.0, args.pose_delay_sec + 2.0):
                pose_history.popleft()

            if args.pos_noise_std > 0:
                if args.pos_noise_tau_sec > 0:
                    rho = math.exp(-period / args.pos_noise_tau_sec)
                    sigma = math.sqrt(max(0.0, 1.0 - rho * rho)) * args.pos_noise_std
                    noise_n = rho * noise_n + rng.gauss(0.0, sigma)
                    noise_e = rho * noise_e + rng.gauss(0.0, sigma)
                else:
                    noise_n = rng.gauss(0.0, args.pos_noise_std)
                    noise_e = rng.gauss(0.0, args.pos_noise_std)
            else:
                noise_n = 0.0
                noise_e = 0.0
            north += args.pos_bias_north + noise_n
            east += args.pos_bias_east + noise_e

            vn = ve = vd = 0.0
            if args.speed_source == "gz" and previous_pose is not None:
                dt = max(1.0e-3, gz_pose.timestamp - previous_pose.timestamp)
                raw_vn, raw_ve = delta_world_to_ned(
                    (gz_pose.x - previous_pose.x) / dt,
                    (gz_pose.y - previous_pose.y) / dt,
                    args.axis,
                )
                raw_vd = -(gz_pose.z - previous_pose.z) / dt
                alpha = min(1.0, max(0.0, args.speed_alpha))
                filtered_vn = alpha * raw_vn + (1.0 - alpha) * filtered_vn
                filtered_ve = alpha * raw_ve + (1.0 - alpha) * filtered_ve
                filtered_vd = alpha * raw_vd + (1.0 - alpha) * filtered_vd
                vn, ve, vd = clamp_velocity(filtered_vn, filtered_ve, filtered_vd, args.max_speed)
            elif args.speed_source == "local" and local is not None:
                vn, ve, vd = float(local.vx), float(local.vy), float(local.vz)

            if not args.observe_only:
                roll = pitch = yaw = 0.0
                if args.attitude_source == "telemetry" and attitude is not None:
                    roll = float(attitude.roll)
                    pitch = float(attitude.pitch)
                    yaw = float(attitude.yaw)
                send_vision_position(
                    conn,
                    north,
                    east,
                    down,
                    roll,
                    pitch,
                    yaw,
                    reset_counter,
                )
                if args.send_speed:
                    send_vision_speed(conn, vn, ve, vd, reset_counter)

            previous_pose = gz_pose
            sent += 1
            next_send += period

            if now >= next_print:
                next_print = now + 1.0
                err_text = "err=-"
                if local is not None:
                    err_text = f"err=({north - local.x:.2f},{east - local.y:.2f},{down - local.z:.2f})"
                    local_age = now - float(latest.get("_LOCAL_POSITION_NED_time", now))
                    err_text += f" local_age={local_age:.2f}s"
                else:
                    err_text = "err=- local_age=-"
                axes_text = ""
                if args.compare_axes and local is not None:
                    axes_text = " " + format_axis_errors(ref_gz, ref_local, gz_pose, local)
                print(
                    "GT "
                    f"sent={sent} pose=({north:.2f},{east:.2f},{down:.2f}) "
                    f"vel=({vn:.2f},{ve:.2f},{vd:.2f}) {err_text}{axes_text} {ekf_text(latest)}"
                )
    except KeyboardInterrupt:
        print("INTERRUPTED")
    print(f"DONE sent={sent}")
    return 0


def clamp_velocity(vn: float, ve: float, vd: float, limit: float) -> Tuple[float, float, float]:
    speed = math.sqrt(vn * vn + ve * ve + vd * vd)
    if speed <= limit:
        return vn, ve, vd
    scale = limit / speed
    return vn * scale, ve * scale, vd * scale


def format_axis_errors(ref_gz: GazeboPose, ref_local: Any, gz_pose: GazeboPose, local: Any) -> str:
    items = []
    best_axis = "-"
    best_error = float("inf")
    for axis in ("enu", "xy", "neg_enu", "neg_xy"):
        dn, de = delta_world_to_ned(gz_pose.x - ref_gz.x, gz_pose.y - ref_gz.y, axis)
        north = float(ref_local.x) + dn
        east = float(ref_local.y) + de
        err = math.hypot(north - float(local.x), east - float(local.y))
        items.append(f"{axis}={err:.2f}")
        if err < best_error:
            best_error = err
            best_axis = axis
    return "axis_err[" + " ".join(items) + f" best={best_axis}" + "]"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish Gazebo ground truth as MAVLink ExternalNav")
    parser.add_argument("--mavlink", default="udp:127.0.0.1:14550")
    parser.add_argument("--source-system", type=int, default=247)
    parser.add_argument("--source-component", type=int, default=193)
    parser.add_argument("--heartbeat-timeout", type=float, default=8.0)
    parser.add_argument("--gz-topic", default="/world/iris_runway/pose/info")
    parser.add_argument("--model", default="iris_with_gimbal")
    parser.add_argument("--axis", choices=["enu", "xy", "neg_enu", "neg_xy"], default="enu")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--stream-hz", type=float, default=10.0)
    parser.add_argument("--stream-request-sec", type=float, default=2.0)
    parser.add_argument("--mavlink-poll-timeout", type=float, default=0.02)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until Ctrl-C")
    parser.add_argument("--bootstrap-timeout", type=float, default=10.0)
    parser.add_argument("--send-speed", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--speed-source", choices=["gz", "zero", "local"], default="gz")
    parser.add_argument("--attitude-source", choices=["telemetry", "zero"], default="telemetry")
    parser.add_argument("--speed-alpha", type=float, default=0.35)
    parser.add_argument("--max-speed", type=float, default=8.0)
    parser.add_argument("--pose-delay-sec", type=float, default=0.0, help="Publish an older Gazebo pose to emulate localization latency")
    parser.add_argument("--pos-bias-north", type=float, default=0.0, help="Constant north bias added to VPE meters")
    parser.add_argument("--pos-bias-east", type=float, default=0.0, help="Constant east bias added to VPE meters")
    parser.add_argument("--pos-noise-std", type=float, default=0.0, help="Gaussian XY noise standard deviation added to VPE meters")
    parser.add_argument("--pos-noise-tau-sec", type=float, default=0.0, help="Correlated noise time constant; 0 uses white noise")
    parser.add_argument("--noise-seed", type=int, default=7)
    parser.add_argument("--observe-only", action="store_true", help="Do not publish VPE/VSE; only compare Gazebo pose with ArduPilot local position")
    parser.add_argument("--compare-axes", action="store_true", help="Print error for all axis mappings against LOCAL_POSITION_NED")
    return parser


def main() -> int:
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
