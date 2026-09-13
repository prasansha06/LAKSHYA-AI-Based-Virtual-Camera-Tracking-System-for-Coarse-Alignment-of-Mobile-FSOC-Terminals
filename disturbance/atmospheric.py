"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: disturbance/atmospheric.py
--------------------------------------------------------------------------------
Purpose:
    Simulates atmospheric optical transmission effects on free space optical
    communication links. Implements all five mandatory atmospheric modes:
    - Clear: Baseline transmission (no effect).
    - Haze: Reduces optical contrast by 30%.
    - Fog: Adds white overlay at 40% opacity.
    - Rain: Adds synthetic vertical rain streaks.
    - Low Light: Reduces image brightness by 50%.

Inputs:
    frame (np.ndarray): 2D uint8 grayscale image.
    mode (str): 'clear', 'haze', 'fog', 'rain', 'lowlight'.

Outputs:
    attenuated_frame (np.ndarray): 2D uint8 image with atmospheric disturbances.
================================================================================
"""

import numpy as np
import random
from typing import List, Dict

class AtmosphericDisturbance:
    """
    Simulates weather and atmospheric attenuation on FSOC laser beam channels.
    """

    def __init__(self, viewport_width: int = 640, viewport_height: int = 480):
        self.mode = "clear"
        self.viewport_width = viewport_width
        self.viewport_height = viewport_height

        # Initialize rain particle system
        self.rain_drops: List[Dict[str, float]] = []
        for _ in range(80):
            self.rain_drops.append({
                "x": random.uniform(0, viewport_width),
                "y": random.uniform(0, viewport_height),
                "length": random.uniform(10.0, 25.0),
                "speed": random.uniform(15.0, 30.0),
                "intensity": random.uniform(180, 240)
            })

    def set_mode(self, mode: str) -> None:
        """Select atmospheric disturbance mode at runtime."""
        valid_modes = ["clear", "haze", "fog", "rain", "lowlight"]
        if mode.lower() in valid_modes:
            self.mode = mode.lower()

    def get_intensity_factor(self) -> float:
        """Returns relative beacon brightness multiplier for current atmosphere."""
        if self.mode == "haze":
            return 0.70  # 30% contrast reduction
        elif self.mode == "lowlight":
            return 0.50  # 50% brightness reduction
        return 1.0

    def apply(self, frame: np.ndarray, dt: float = 1.0 / 30.0) -> np.ndarray:
        """
        Applies atmospheric disturbance to the input frame.
        """
        if self.mode == "clear":
            return frame.copy()

        h, w = frame.shape[:2]
        out = frame.astype(np.float32)

        # 1. Haze: Reduces contrast by 30% (scale around mean and shift towards gray)
        if self.mode == "haze":
            mean_val = np.mean(out)
            out = (out - mean_val) * 0.70 + mean_val

        # 2. Fog: Adds white overlay at 40% opacity (0.60 * image + 0.40 * 255)
        elif self.mode == "fog":
            white_overlay = 255.0
            out = out * 0.60 + white_overlay * 0.40

        # 3. Low Light: Reduces brightness by 50%
        elif self.mode == "lowlight":
            out = out * 0.50

        # 4. Rain: Adds dynamic vertical streaks simulating precipitation
        elif self.mode == "rain":
            # Baseline slight contrast drop
            out = out * 0.85
            # Draw rain streaks
            for drop in self.rain_drops:
                drop["y"] += drop["speed"]
                if drop["y"] > h:
                    drop["y"] = -drop["length"]
                    drop["x"] = random.uniform(0, w)

                x_idx = int(drop["x"])
                if 0 <= x_idx < w:
                    y_start = max(0, int(drop["y"]))
                    y_end = min(h, int(drop["y"] + drop["length"]))
                    if y_start < y_end:
                        out[y_start:y_end, x_idx] = np.maximum(out[y_start:y_end, x_idx], drop["intensity"])

        return np.clip(out, 0.0, 255.0).astype(np.uint8)
