from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Optional, Tuple

from .state import SharedState

try:
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node
except Exception as exc:  # pragma: no cover - depends on local Gazebo bindings
    Pose_V = None
    Node = None
    GZ_IMPORT_ERROR: Optional[Exception] = exc
else:
    GZ_IMPORT_ERROR = None


@dataclass(frozen=True)
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
        if Node is None:
            raise RuntimeError(f"Gazebo Python bindings import failed: {GZ_IMPORT_ERROR}")
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

    def _callback(self, msg: Any, *_args: Any) -> None:
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

    def _find_pose(self, msg: Any) -> Optional[Any]:
        exact = [pose for pose in msg.pose if pose.name == self.model]
        if exact:
            return exact[0]
        leaf = [pose for pose in msg.pose if pose.name.split("::")[-1] == self.model]
        if leaf:
            return leaf[0]
        contains = [pose for pose in msg.pose if self.model in pose.name]
        return contains[0] if contains else None


class GazeboTruthThread(threading.Thread):
    def __init__(
        self,
        state: SharedState,
        topic: str,
        model: str,
        bootstrap_mode: str = "telemetry",
        rate_hz: float = 10.0,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.topic = topic
        self.model = model
        self.bootstrap_mode = bootstrap_mode
        self.rate_hz = max(0.5, rate_hz)
        self.stop_event = threading.Event()
        self.subscriber: Optional[GazeboPoseSubscriber] = None
        self.ref_gz_ned: Optional[Tuple[float, float]] = None
        self.ref_truth_ned: Optional[Tuple[float, float]] = None

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        try:
            self.subscriber = GazeboPoseSubscriber(self.topic, self.model)
            self.subscriber.start()
        except Exception as exc:
            self.state.update_truth({"status": "gazebo_import_failed"})
            self.state.set_error(str(exc))
            return

        self.state.update_truth(
            {
                "status": "waiting_pose",
                "source": "gazebo",
                "topic": self.topic,
                "model": self.model,
                "bootstrap_mode": self.bootstrap_mode,
            }
        )

        interval = 1.0 / self.rate_hz
        while not self.stop_event.is_set():
            loop_started = time.time()
            pose = self.subscriber.get() if self.subscriber is not None else None
            if pose is None:
                time.sleep(min(0.2, interval))
                continue

            gz_north, gz_east = self.state.geometry.world_to_ned(pose.x, pose.y)
            if not self._ensure_bootstrapped(pose, gz_north, gz_east):
                time.sleep(min(0.2, interval))
                continue

            assert self.ref_gz_ned is not None
            assert self.ref_truth_ned is not None
            north = self.ref_truth_ned[0] + (gz_north - self.ref_gz_ned[0])
            east = self.ref_truth_ned[1] + (gz_east - self.ref_gz_ned[1])
            now = time.time()
            self.state.update_truth(
                {
                    "status": "receiving",
                    "source": "gazebo",
                    "north": north,
                    "east": east,
                    "down": -float(pose.z),
                    "gz_x": pose.x,
                    "gz_y": pose.y,
                    "gz_z": pose.z,
                    "pose_age_sec": max(0.0, now - pose.timestamp),
                    "timestamp": now,
                    "topic": self.topic,
                    "model": self.model,
                    "bootstrap_mode": self.bootstrap_mode,
                }
            )
            elapsed = time.time() - loop_started
            time.sleep(max(0.0, interval - elapsed))

    def _ensure_bootstrapped(self, pose: GazeboPose, gz_north: float, gz_east: float) -> bool:
        if self.ref_gz_ned is not None and self.ref_truth_ned is not None:
            return True

        if self.bootstrap_mode == "map-origin":
            self.ref_gz_ned = (gz_north, gz_east)
            self.ref_truth_ned = (gz_north, gz_east)
            return True

        snapshot = self.state.snapshot()
        seed = snapshot.get("telemetry", {}).get("seed_ned", {})
        if seed.get("north") is None or seed.get("east") is None:
            self.state.update_truth(
                {
                    "status": "waiting_telemetry_seed",
                    "source": "gazebo",
                    "topic": self.topic,
                    "model": self.model,
                    "timestamp": time.time(),
                    "gz_x": pose.x,
                    "gz_y": pose.y,
                    "gz_z": pose.z,
                }
            )
            return False

        self.ref_gz_ned = (gz_north, gz_east)
        self.ref_truth_ned = (float(seed["north"]), float(seed["east"]))
        self.state.update_truth(
            {
                "status": "bootstrapped",
                "source": "gazebo",
                "topic": self.topic,
                "model": self.model,
                "bootstrap_mode": self.bootstrap_mode,
                "bootstrap_seed_source": seed.get("source"),
                "bootstrap_seed_age_sec": seed.get("age_sec"),
                "bootstrap_gz_x": pose.x,
                "bootstrap_gz_y": pose.y,
                "bootstrap_gz_z": pose.z,
                "timestamp": time.time(),
            }
        )
        return True
