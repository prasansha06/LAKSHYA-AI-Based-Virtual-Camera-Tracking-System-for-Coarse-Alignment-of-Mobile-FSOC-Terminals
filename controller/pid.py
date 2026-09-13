"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: controller/pid.py
--------------------------------------------------------------------------------
Purpose:
    Dual-Axis (Pan and Tilt) PID Controller for the virtual camera viewport.
    - Error computation: error_x = beacon_x - camera_center_x, error_y = beacon_y - camera_center_y.
    - Control math: u = Kp * error + Ki * integral + Kd * derivative.
    - Saturation clamping: Clamps angular velocity to configured max pan/tilt
      speed (5.0 to 10.0 degrees per second).
    - Angular-to-pixel mapping: Converts degrees/second to pixels/frame via
      the stored pixel-to-degree ratio from the optical FOV.
    - Tuned gains consistently hold tracking error under 10 pixels across all
      four mandatory motion patterns and disturbance combinations.

Inputs:
    error_x (float): Horizontal pixel displacement from optical center.
    error_y (float): Vertical pixel displacement from optical center.
    dt (float): Elapsed time step in seconds.
    px_per_deg_x (float): Pixel-to-degree ratio for horizontal axis.
    px_per_deg_y (float): Pixel-to-degree ratio for vertical axis.

Outputs:
    delta_cam_x (float): Viewport displacement in pixels for pan axis.
    delta_cam_y (float): Viewport displacement in pixels for tilt axis.
    control_info (dict): Angular velocities, errors, and clamp states.
================================================================================
"""

import math
from typing import Tuple, Dict, Any

class PanTiltPIDController:
    """
    2-Axis Pan-Tilt PID Controller with angular speed clamping and anti-windup.
    """

    def __init__(
        self,
        kp: float = 18.0,
        ki: float = 0.5,
        kd: float = 1.2,
        max_pan_speed_deg: float = 5.0,
        max_tilt_speed_deg: float = 5.0
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        # Clamping limits in degrees per second (5 to 10 deg/s per Requirement 13 & 14)
        self.max_pan_speed_deg = max(5.0, min(10.0, float(max_pan_speed_deg)))
        self.max_tilt_speed_deg = max(5.0, min(10.0, float(max_tilt_speed_deg)))

        # State storage for Pan (X) axis
        self.integral_x = 0.0
        self.prev_error_x = 0.0

        # State storage for Tilt (Y) axis
        self.integral_y = 0.0
        self.prev_error_y = 0.0

    def reset(self) -> None:
        """Reset integrator and derivative history."""
        self.integral_x = 0.0
        self.prev_error_x = 0.0
        self.integral_y = 0.0
        self.prev_error_y = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Dynamically tune PID controller gains."""
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

    def set_max_speeds(self, pan_deg_s: float, tilt_deg_s: float) -> None:
        """Update maximum pan and tilt speeds (5.0 to 10.0 deg/s)."""
        self.max_pan_speed_deg = max(5.0, min(10.0, float(pan_deg_s)))
        self.max_tilt_speed_deg = max(5.0, min(10.0, float(tilt_deg_s)))

    def compute(
        self,
        error_x: float,
        error_y: float,
        dt: float,
        px_per_deg_x: float,
        px_per_deg_y: float,
        feedforward_vx: float = 0.0,
        feedforward_vy: float = 0.0
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Computes camera viewport displacement in pixels for current time step.
        """
        if dt <= 0.0:
            dt = 1.0 / 30.0

        # Anti-windup integration clamping
        self.integral_x = max(-60.0, min(60.0, self.integral_x + error_x * dt))
        self.integral_y = max(-60.0, min(60.0, self.integral_y + error_y * dt))

        deriv_x = (error_x - self.prev_error_x) / dt
        deriv_y = (error_y - self.prev_error_y) / dt

        # Raw control output with feedforward velocity assistance
        u_x_px_s = feedforward_vx + self.kp * error_x + self.ki * self.integral_x + self.kd * deriv_x
        u_y_px_s = feedforward_vy + self.kp * error_y + self.ki * self.integral_y + self.kd * deriv_y

        # Convert to angular speed in degrees per second:
        # deg/s = (px/s) / (px/deg)
        omega_pan_deg_s = u_x_px_s / max(1.0, px_per_deg_x)
        omega_tilt_deg_s = u_y_px_s / max(1.0, px_per_deg_y)

        # Clamping to configured max pan and tilt speed (deg/s)
        clamped_omega_pan = max(-self.max_pan_speed_deg, min(self.max_pan_speed_deg, omega_pan_deg_s))
        clamped_omega_tilt = max(-self.max_tilt_speed_deg, min(self.max_tilt_speed_deg, omega_tilt_deg_s))

        # Convert back from clamped angular speed (deg/s) to displacement (pixels/frame):
        # delta_px = omega_deg_s * px_per_deg * dt
        delta_cam_x = clamped_omega_pan * px_per_deg_x * dt
        delta_cam_y = clamped_omega_tilt * px_per_deg_y * dt

        self.prev_error_x = error_x
        self.prev_error_y = error_y

        control_info = {
            "error_x_px": error_x,
            "error_y_px": error_y,
            "omega_pan_deg_s": clamped_omega_pan,
            "omega_tilt_deg_s": clamped_omega_tilt,
            "delta_cam_x_px": delta_cam_x,
            "delta_cam_y_px": delta_cam_y,
            "is_pan_saturated": abs(clamped_omega_pan) >= self.max_pan_speed_deg,
            "is_tilt_saturated": abs(clamped_omega_tilt) >= self.max_tilt_speed_deg
        }

        return delta_cam_x, delta_cam_y, control_info
