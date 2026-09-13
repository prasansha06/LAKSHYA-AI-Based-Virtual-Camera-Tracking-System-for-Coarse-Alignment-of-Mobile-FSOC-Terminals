"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracker/kalman.py
--------------------------------------------------------------------------------
Purpose:
    Beacon position prediction and tracker state management using a 4D Linear
    Kalman Filter. Implements:
    - State vector: [x, y, vx, vy]^T (position and velocity).
    - Measurement vector: [z_x, z_y]^T from centroid detector.
    - Process noise covariance Q and measurement noise covariance R.
    - State prediction during target occlusion (holds position for 10+ frames).
    - Uses filterpy.kalman.KalmanFilter with complete NumPy matrix fallback.

Inputs:
    measured_pos (tuple or None): (x, y) measured centroid in viewport pixels.
    dt (float): Elapsed time step in seconds (default 1/30 s).

Outputs:
    estimated_pos (tuple): (x, y) optimal filtered/predicted beacon location.
    velocity (tuple): (vx, vy) estimated instantaneous velocity.
    is_predicted (bool): True if update was performed without measurement.
================================================================================
"""

import numpy as np
from typing import Tuple, Optional

try:
    from filterpy.kalman import KalmanFilter as FilterPyKF
    HAS_FILTERPY = True
except ImportError:
    HAS_FILTERPY = False

class KalmanTracker:
    """
    4D Kalman Filter for continuous target tracking and occlusion prediction.
    """

    def __init__(self, initial_x: float = 320.0, initial_y: float = 240.0, dt: float = 1.0 / 30.0):
        self.dt = dt
        self.consecutive_lost_frames = 0

        # State vector [x, y, vx, vy]
        self.x = np.array([initial_x, initial_y, 0.0, 0.0], dtype=np.float64)

        # State Transition Matrix F:
        # [1, 0, dt, 0]
        # [0, 1, 0, dt]
        # [0, 0, 1,  0]
        # [0, 0, 0,  1]
        self.F = np.array([
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Measurement Matrix H (2x4)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=np.float64)

        # State Covariance Matrix P (4x4)
        self.P = np.diag([10.0, 10.0, 50.0, 50.0]).astype(np.float64)

        # Process Noise Covariance Q (4x4)
        q_pos = 0.5
        q_vel = 2.0
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel]).astype(np.float64)

        # Measurement Noise Covariance R (2x2)
        r_val = 4.0
        self.R = np.diag([r_val, r_val]).astype(np.float64)

        # Identity Matrix I
        self.I = np.eye(4, dtype=np.float64)

        # Initialize FilterPy instance if available
        self.fk = None
        if HAS_FILTERPY:
            self._init_filterpy(initial_x, initial_y)

    def _init_filterpy(self, ix: float, iy: float) -> None:
        """Configures filterpy KalmanFilter instance."""
        self.fk = FilterPyKF(dim_x=4, dim_z=2)
        self.fk.x = np.array([ix, iy, 0.0, 0.0])
        self.fk.F = self.F.copy()
        self.fk.H = self.H.copy()
        self.fk.P = self.P.copy()
        self.fk.Q = self.Q.copy()
        self.fk.R = self.R.copy()

    def reset(self, initial_x: float = 320.0, initial_y: float = 240.0) -> None:
        """Reset state vector and covariances."""
        self.x = np.array([initial_x, initial_y, 0.0, 0.0], dtype=np.float64)
        self.P = np.diag([10.0, 10.0, 50.0, 50.0]).astype(np.float64)
        self.consecutive_lost_frames = 0
        if self.fk is not None:
            self._init_filterpy(initial_x, initial_y)

    def set_noise_matrices(self, q_scale: float, r_scale: float) -> None:
        """Tune process noise Q and measurement noise R matrices."""
        q_scale = max(0.01, float(q_scale))
        r_scale = max(0.01, float(r_scale))
        self.Q = np.diag([q_scale * 0.25, q_scale * 0.25, q_scale, q_scale])
        self.R = np.diag([r_scale, r_scale])
        if self.fk is not None:
            self.fk.Q = self.Q.copy()
            self.fk.R = self.R.copy()

    def predict(self, dt: float = None) -> Tuple[float, float]:
        """
        Executes Kalman time-update / prediction step.
        x_pred = F * x
        P_pred = F * P * F^T + Q
        """
        if dt is not None and dt > 0:
            self.dt = dt
            self.F[0, 2] = self.dt
            self.F[1, 3] = self.dt
            if self.fk is not None:
                self.fk.F = self.F.copy()

        if self.fk is not None:
            self.fk.predict()
            self.x = self.fk.x.copy()
            self.P = self.fk.P.copy()
        else:
            self.x = self.F @ self.x
            self.P = self.F @ self.P @ self.F.T + self.Q

        return float(self.x[0]), float(self.x[1])

    def update(self, measurement: Optional[Tuple[float, float]]) -> Tuple[Tuple[float, float], Tuple[float, float], bool]:
        """
        Executes Kalman measurement update step.
        If measurement is None (occlusion/target loss), holds position via prediction.
        Returns: ((x, y), (vx, vy), is_predicted)
        """
        if measurement is None:
            self.consecutive_lost_frames += 1
            return (float(self.x[0]), float(self.x[1])), (float(self.x[2]), float(self.x[3])), True

        self.consecutive_lost_frames = 0
        z = np.array([measurement[0], measurement[1]], dtype=np.float64)

        if self.fk is not None:
            self.fk.update(z)
            self.x = self.fk.x.copy()
            self.P = self.fk.P.copy()
        else:
            # Innovation: y = z - H * x
            y = z - (self.H @ self.x)
            # Innovation covariance: S = H * P * H^T + R
            S = self.H @ self.P @ self.H.T + self.R
            # Kalman Gain: K = P * H^T * inv(S)
            K = self.P @ self.H.T @ np.linalg.inv(S)
            # State update
            self.x = self.x + K @ y
            # Covariance update: P = (I - K * H) * P
            self.P = (self.I - K @ self.H) @ self.P

        return (float(self.x[0]), float(self.x[1])), (float(self.x[2]), float(self.x[3])), False
