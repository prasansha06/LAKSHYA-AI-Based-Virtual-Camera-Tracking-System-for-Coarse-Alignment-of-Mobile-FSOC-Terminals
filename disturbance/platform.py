"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: disturbance/platform.py
--------------------------------------------------------------------------------
Purpose:
    Simulates high-frequency camera mechanical vibrations (jitter) and base
    platform translational motion (aircraft/satellite drift). Implements:
    - Camera Jitter: Up to ±20 pixels per frame (user defined maximum).
    - Platform Motion: Up to ±20 pixels per frame maximum (Linear mandatory,
      circular and random modes optional).

Inputs:
    dt (float): Elapsed time per simulation frame.

Outputs:
    jitter (tuple): (dx, dy) instantaneous camera displacement in pixels.
    platform_disp (tuple): (dx, dy) base platform displacement in pixels.
================================================================================
"""

import math
import random
from typing import Tuple

class PlatformDynamics:
    """
    Simulates physical motion disturbances of the tracking platform and sensor gimbal.
    """

    def __init__(self):
        # Camera Jitter (±20 pixels per frame maximum, default 5 px)
        self.max_camera_jitter = 5.0

        # Platform Motion (±20 pixels per frame maximum, default linear)
        self.platform_mode = "linear"  # linear, circular, random, none
        self.platform_phase = 0.0
        self.max_platform_speed = 18.0  # pixels per frame

    def set_max_jitter(self, jitter: float) -> None:
        """Set maximum camera jitter amplitude in pixels (0 to 20 px)."""
        self.max_camera_jitter = max(0.0, min(20.0, float(jitter)))

    def set_platform_mode(self, mode: str) -> None:
        """Set platform motion mode: 'linear', 'circular', 'random', 'none'."""
        valid_modes = ["linear", "circular", "random", "none"]
        if mode.lower() in valid_modes:
            self.platform_mode = mode.lower()

    def get_camera_jitter(self) -> Tuple[float, float]:
        """
        Computes random instantaneous camera jitter displacement up to ±20 px.
        Applied directly to the camera viewport window.
        """
        if self.max_camera_jitter <= 0.0:
            return 0.0, 0.0
        jx = random.uniform(-self.max_camera_jitter, self.max_camera_jitter)
        jy = random.uniform(-self.max_camera_jitter, self.max_camera_jitter)
        return jx, jy

    def update_platform_motion(self, dt: float) -> Tuple[float, float]:
        """
        Computes platform motion displacement up to ±20 pixels per frame.
        Applied to the beacon global coordinate position.
        """
        if self.platform_mode == "none":
            return 0.0, 0.0

        if self.platform_mode == "linear":
            self.platform_phase += dt * 2.5
            # Oscillatory linear perturbation clamped to ±20 pixels
            dx = math.sin(self.platform_phase) * self.max_platform_speed
            dy = math.cos(self.platform_phase) * (self.max_platform_speed * 0.6)
            return dx, dy

        elif self.platform_mode == "circular":
            self.platform_phase += dt * 3.0
            dx = math.cos(self.platform_phase) * 19.5
            dy = math.sin(self.platform_phase) * 19.5
            return dx, dy

        elif self.platform_mode == "random":
            dx = random.uniform(-20.0, 20.0)
            dy = random.uniform(-20.0, 20.0)
            return dx, dy

        return 0.0, 0.0
