"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: sim/motion.py
--------------------------------------------------------------------------------
Purpose:
    Generates deterministic and stochastic trajectories for the mobile optical
    beacon within the 2000x2000 virtual space. Implements all four mandatory
    patterns (Straight Line, Circular, Figure of 8, Random) and two optional
    patterns (Spiral, Sinusoidal).

Inputs:
    pattern_type (str): Name of motion pattern ('straight', 'circular', 'figure8', 'random', 'spiral', 'sinusoidal').
    canvas_size (tuple): (width, height) of the virtual environment in pixels (default 2000x2000).
    dt (float): Elapsed time step in seconds.

Outputs:
    (x, y): Updated 2D beacon coordinates in global pixel coordinates.
================================================================================
"""

import math
import random
from typing import Tuple

class MotionPatternGenerator:
    """
    Trajectory engine providing realistic motion profiles for mobile FSOC terminals
    (satellites, UAVs, and optical ground terminals).
    """

    def __init__(self, canvas_width: int = 2000, canvas_height: int = 2000):
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.cx = canvas_width / 2.0
        self.cy = canvas_height / 2.0
        
        self.pattern = "figure8"
        self.time = 0.0

        # Straight-line parameters
        self.pos_x = self.cx - 200.0
        self.pos_y = self.cy - 150.0
        self.vel_x = 55.0  # px/s
        self.vel_y = 40.0  # px/s

        # Circular parameters
        self.radius = 260.0
        self.angular_speed = 0.42  # rad/s

        # Figure of 8 parameters
        self.fig8_a = 480.0
        self.fig8_speed = 0.40

        # Random walk parameters
        self.rw_vx = 35.0
        self.rw_vy = 25.0

    def set_pattern(self, pattern: str) -> None:
        """Switch active pattern at runtime without restarting simulation."""
        valid_patterns = ["straight", "circular", "figure8", "random", "spiral", "sinusoidal"]
        if pattern.lower() in valid_patterns:
            self.pattern = pattern.lower()
            self.time = 0.0
            if self.pattern in ["circular", "figure8"]:
                self.pos_x = self.cx
                self.pos_y = self.cy
            elif self.pattern == "random":
                self.pos_x = self.cx
                self.pos_y = self.cy
                self.rw_vx = 35.0
                self.rw_vy = 25.0
            elif self.pattern == "straight":
                self.pos_x = self.cx - 200.0
                self.pos_y = self.cy - 150.0
                self.vel_x = 55.0
                self.vel_y = 40.0

    def set_initial_position(self, x: float, y: float) -> None:
        """Configure initial beacon starting location."""
        self.pos_x = max(50.0, min(self.canvas_width - 50.0, float(x)))
        self.pos_y = max(50.0, min(self.canvas_height - 50.0, float(y)))

    def randomize_position(self) -> Tuple[float, float]:
        """Place beacon at a random location within reachable camera margins."""
        margin_x = 350.0
        margin_y = 300.0
        self.pos_x = random.uniform(margin_x, self.canvas_width - margin_x)
        self.pos_y = random.uniform(margin_y, self.canvas_height - margin_y)
        return self.pos_x, self.pos_y

    def update(self, dt: float) -> Tuple[float, float]:
        """Compute the next beacon position based on elapsed time dt."""
        self.time += dt

        # Fully steerable viewport boundary padding (viewport is 640x480 on 2000x2000 canvas)
        pad_x = 340.0
        pad_y = 260.0

        if self.pattern == "straight":
            self.pos_x += self.vel_x * dt
            self.pos_y += self.vel_y * dt

            # Bounding wall reflections within camera steerable envelope
            if self.pos_x < pad_x:
                self.pos_x = pad_x
                self.vel_x = abs(self.vel_x)
            elif self.pos_x > self.canvas_width - pad_x:
                self.pos_x = self.canvas_width - pad_x
                self.vel_x = -abs(self.vel_x)

            if self.pos_y < pad_y:
                self.pos_y = pad_y
                self.vel_y = abs(self.vel_y)
            elif self.pos_y > self.canvas_height - pad_y:
                self.pos_y = self.canvas_height - pad_y
                self.vel_y = -abs(self.vel_y)

        elif self.pattern == "circular":
            self.pos_x = self.cx + self.radius * math.sin(self.angular_speed * self.time)
            self.pos_y = self.cy + self.radius * (1.0 - math.cos(self.angular_speed * self.time))

        elif self.pattern == "figure8":
            # Lemniscate of Gerono: x = A * sin(t), y = (A * 0.6) * sin(2t) / 2
            t = self.fig8_speed * self.time
            self.pos_x = self.cx + self.fig8_a * math.sin(t)
            self.pos_y = self.cy + (self.fig8_a * 0.6) * math.sin(2.0 * t) / 2.0

        elif self.pattern == "random":
            # Smooth Brownian walk
            if random.random() < 0.08:
                self.rw_vx += random.uniform(-60.0, 60.0)
                self.rw_vy += random.uniform(-60.0, 60.0)
                max_v = 110.0
                self.rw_vx = max(-max_v, min(max_v, self.rw_vx))
                self.rw_vy = max(-max_v, min(max_v, self.rw_vy))

            self.pos_x += self.rw_vx * dt
            self.pos_y += self.rw_vy * dt

            # Bounding wall reflections within camera steerable envelope
            if self.pos_x < pad_x:
                self.pos_x = pad_x
                self.rw_vx = abs(self.rw_vx)
            elif self.pos_x > self.canvas_width - pad_x:
                self.pos_x = self.canvas_width - pad_x
                self.rw_vx = -abs(self.rw_vx)
            if self.pos_y < pad_y:
                self.pos_y = pad_y
                self.rw_vy = abs(self.rw_vy)
            elif self.pos_y > self.canvas_height - pad_y:
                self.pos_y = self.canvas_height - pad_y
                self.rw_vy = -abs(self.rw_vy)

        elif self.pattern == "spiral":
            r = (60.0 + (self.time * 30.0) % 500.0)
            theta = self.time * 0.8
            self.pos_x = self.cx + r * math.cos(theta)
            self.pos_y = self.cy + r * math.sin(theta)

        elif self.pattern == "sinusoidal":
            self.pos_x = self.cx + 550.0 * math.sin(self.time * 0.35)
            self.pos_y = self.cy + 250.0 * math.sin(self.time * 1.5)

        return self.pos_x, self.pos_y
