from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np


@dataclass
class KalmanPrediction:
    timestamp: float
    north: float
    east: float
    vn: float
    ve: float
    pos_std_m: float


class VisualPositionKalman2D:
    """Small constant-velocity Kalman filter for aligned visual N/E fixes."""

    def __init__(
        self,
        process_noise_pos: float = 0.5,
        process_noise_vel: float = 1.0,
        process_accel_noise: Optional[float] = None,
        meas_noise_pos: float = 4.0,
        initial_pos_std: float = 2.0,
        initial_vel_std: float = 1.0,
    ) -> None:
        self.process_noise_pos = max(1.0e-6, float(process_noise_pos))
        self.process_accel_noise = max(
            1.0e-6,
            float(process_accel_noise if process_accel_noise is not None else process_noise_vel),
        )
        self.process_noise_vel = self.process_accel_noise
        self.meas_noise_pos = max(1.0e-6, float(meas_noise_pos))
        self.initial_pos_std = max(1.0e-6, float(initial_pos_std))
        self.initial_vel_std = max(1.0e-6, float(initial_vel_std))
        self.x: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None
        self.timestamp: Optional[float] = None
        self.update_count = 0

    @property
    def initialized(self) -> bool:
        return self.x is not None and self.P is not None and self.timestamp is not None

    def reset(self, north: float, east: float, timestamp: float) -> Dict[str, Any]:
        self.x = np.array([float(north), float(east), 0.0, 0.0], dtype=np.float64)
        pos_var = self.initial_pos_std * self.initial_pos_std
        vel_var = self.initial_vel_std * self.initial_vel_std
        self.P = np.diag([pos_var, pos_var, vel_var, vel_var]).astype(np.float64)
        self.timestamp = float(timestamp)
        self.update_count = 1
        return {
            "initialized": True,
            "innovation_m": 0.0,
            "north": float(self.x[0]),
            "east": float(self.x[1]),
            "vn": float(self.x[2]),
            "ve": float(self.x[3]),
            "update_count": self.update_count,
        }

    def predict_at(self, timestamp: float) -> Optional[KalmanPrediction]:
        if not self.initialized:
            return None
        assert self.x is not None
        assert self.P is not None
        assert self.timestamp is not None
        x_pred, P_pred = self._predict_arrays(self.x, self.P, max(0.0, float(timestamp) - self.timestamp))
        pos_std = float(np.sqrt(max(P_pred[0, 0], P_pred[1, 1], 0.0)))
        return KalmanPrediction(
            timestamp=float(timestamp),
            north=float(x_pred[0]),
            east=float(x_pred[1]),
            vn=float(x_pred[2]),
            ve=float(x_pred[3]),
            pos_std_m=pos_std,
        )

    def innovation(self, north: float, east: float, timestamp: float) -> Optional[float]:
        pred = self.predict_at(timestamp)
        if pred is None:
            return None
        return float(np.hypot(float(north) - pred.north, float(east) - pred.east))

    def update(self, north: float, east: float, timestamp: float) -> Dict[str, Any]:
        timestamp = float(timestamp)
        if not self.initialized:
            return self.reset(north, east, timestamp)

        assert self.x is not None
        assert self.P is not None
        assert self.timestamp is not None
        x_pred, P_pred = self._predict_arrays(self.x, self.P, max(0.0, timestamp - self.timestamp))
        z = np.array([float(north), float(east)], dtype=np.float64)
        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )
        R = np.eye(2, dtype=np.float64) * (self.meas_noise_pos * self.meas_noise_pos)
        y = z - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        self.x = x_pred + K @ y
        I = np.eye(4, dtype=np.float64)
        self.P = (I - K @ H) @ P_pred
        self.timestamp = timestamp
        self.update_count += 1
        return {
            "initialized": True,
            "innovation_m": float(np.hypot(y[0], y[1])),
            "north": float(self.x[0]),
            "east": float(self.x[1]),
            "vn": float(self.x[2]),
            "ve": float(self.x[3]),
            "update_count": self.update_count,
        }

    def snapshot(self, timestamp: Optional[float] = None) -> Dict[str, Any]:
        if not self.initialized:
            return {"initialized": False}
        if timestamp is None:
            timestamp = self.timestamp
        pred = self.predict_at(float(timestamp))
        if pred is None:
            return {"initialized": False}
        assert self.timestamp is not None
        return {
            "initialized": True,
            "timestamp": self.timestamp,
            "prediction_timestamp": pred.timestamp,
            "north": pred.north,
            "east": pred.east,
            "vn": pred.vn,
            "ve": pred.ve,
            "pos_std_m": pred.pos_std_m,
            "update_count": self.update_count,
            "process_noise_pos": self.process_noise_pos,
            "process_noise_vel": self.process_noise_vel,
            "process_accel_noise_mps2": self.process_accel_noise,
            "meas_noise_pos": self.meas_noise_pos,
            "initial_pos_std_m": self.initial_pos_std,
            "initial_vel_std_mps": self.initial_vel_std,
        }

    def _predict_arrays(
        self,
        x: np.ndarray,
        P: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        dt = max(0.0, float(dt))
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        q = self.process_accel_noise * self.process_accel_noise
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        q_pos = 0.25 * dt4 * q
        q_cross = 0.5 * dt3 * q
        q_vel = dt2 * q
        Q = np.array(
            [
                [q_pos, 0.0, q_cross, 0.0],
                [0.0, q_pos, 0.0, q_cross],
                [q_cross, 0.0, q_vel, 0.0],
                [0.0, q_cross, 0.0, q_vel],
            ],
            dtype=np.float64,
        )
        x_pred = F @ x
        P_pred = F @ P @ F.T + Q
        return x_pred, P_pred
