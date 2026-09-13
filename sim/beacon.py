"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: sim/beacon.py
--------------------------------------------------------------------------------
Purpose:
    Defines the optical beacon (laser transmitter terminal) characteristics.
    Supports user-defined shapes (Square default, Circle, Cross), user-defined
    sizes (5 to 20 pixels), and arbitrary coordinate positioning.

Inputs:
    shape (str): 'square', 'circle', or 'cross'.
    size (int): 5 to 20 pixels (default 10).
    intensity (int): 0 to 255 grayscale brightness (default 255).

Outputs:
    Renders beacon footprint onto a NumPy grayscale matrix or Pygame surface.
================================================================================
"""

import numpy as np
from typing import Tuple

class Beacon:
    """
    Simulated optical beacon point source for coarse PAT tracking.
    """

    def __init__(self, shape: str = "square", size: int = 10, intensity: int = 255):
        self.shape = shape.lower()
        self.size = max(5, min(20, int(size)))
        self.intensity = int(intensity)

    def set_shape(self, shape: str) -> None:
        """Update beacon geometry at runtime: square, circle, cross."""
        if shape.lower() in ["square", "circle", "cross"]:
            self.shape = shape.lower()

    def set_size(self, size: int) -> None:
        """Update beacon size between 5 and 20 pixels."""
        self.size = max(5, min(20, int(size)))

    def render_to_numpy(self, frame: np.ndarray, x: float, y: float, intensity_factor: float = 1.0) -> None:
        """
        Draws the beacon onto a 2D single-channel NumPy array (Monochrome FPA frame).
        In-place modification with sub-pixel boundary clamping.
        """
        h, w = frame.shape[:2]
        s = self.size
        hs = s // 2
        ix = int(round(x))
        iy = int(round(y))

        # Check if beacon is completely outside viewport
        if ix + hs < 0 or ix - hs >= w or iy + hs < 0 or iy - hs >= h:
            return

        lum = int(min(255, max(0, self.intensity * intensity_factor)))

        if self.shape == "square":
            y1 = max(0, iy - hs)
            y2 = min(h, iy - hs + s)
            x1 = max(0, ix - hs)
            x2 = min(w, ix - hs + s)
            frame[y1:y2, x1:x2] = lum

        elif self.shape == "circle":
            r_sq = (s / 2.0) ** 2
            for dy in range(-hs, hs + 1):
                for dx in range(-hs, hs + 1):
                    if dx * dx + dy * dy <= r_sq:
                        px = ix + dx
                        py = iy + dy
                        if 0 <= px < w and 0 <= py < h:
                            frame[py, px] = lum

        elif self.shape == "cross":
            thickness = max(1, s // 4)
            ht = thickness // 2
            # Horizontal bar
            y1 = max(0, iy - ht)
            y2 = min(h, iy - ht + thickness)
            x1 = max(0, ix - hs)
            x2 = min(w, ix - hs + s)
            frame[y1:y2, x1:x2] = lum
            # Vertical bar
            y1_v = max(0, iy - hs)
            y2_v = min(h, iy - hs + s)
            x1_v = max(0, ix - ht)
            x2_v = min(w, ix - ht + thickness)
            frame[y1_v:y2_v, x1_v:x2_v] = lum
