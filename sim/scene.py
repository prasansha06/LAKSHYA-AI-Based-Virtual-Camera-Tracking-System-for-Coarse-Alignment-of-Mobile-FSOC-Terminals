"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: sim/scene.py
--------------------------------------------------------------------------------
Purpose:
    Virtual scene manager and Monochrome Focal Plane Array (FPA) camera simulator.
    Renders the 2000x2000 pixel global scene, manages the 640x480 movable virtual
    camera viewport, computes angular pixel-to-degree ratios from FOV (4° x 3°),
    and generates single-channel 8-bit grayscale FPA sensor frames at >= 30 Hz.

Inputs:
    canvas_size (tuple): (width, height) of global virtual environment.
    viewport_size (tuple): (width, height) of virtual camera resolution.
    fov_degrees (tuple): (horizontal, vertical) field of view in degrees.

Outputs:
    grayscale_fpa_frame (np.ndarray): 640x480 uint8 Monochrome FPA camera image.
    ground_truth (dict): Global and relative beacon ground truth coordinates.
================================================================================
"""

import numpy as np
from typing import Tuple, Dict, Any
from .beacon import Beacon
from .motion import MotionPatternGenerator

class VirtualScene:
    """
    Virtual Space and Movable Camera Viewport Simulator.
    """

    def __init__(
        self,
        canvas_width: int = 2000,
        canvas_height: int = 2000,
        viewport_width: int = 640,
        viewport_height: int = 480,
        fov_width_deg: float = 4.0,
        fov_height_deg: float = 3.0
    ):
        self.canvas_width = max(2000, int(canvas_width))
        self.canvas_height = max(2000, int(canvas_height))
        self.viewport_width = int(viewport_width)
        self.viewport_height = int(viewport_height)
        
        self.fov_width_deg = float(fov_width_deg)
        self.fov_height_deg = float(fov_height_deg)
        self.compute_fov_ratio()

        # Initial Camera Position: Center of screen (Requirement 6)
        self.cam_x = self.canvas_width / 2.0
        self.cam_y = self.canvas_height / 2.0

        # Subsystems
        self.beacon = Beacon(shape="square", size=10)
        self.motion = MotionPatternGenerator(self.canvas_width, self.canvas_height)

        # Baseline dark sensor current
        self.sensor_baseline = 12

    def compute_fov_ratio(self) -> None:
        """
        Computes pixel-to-degree conversion ratio (Requirement 4).
        pixels_per_degree = viewport_width / fov_width_degrees
        """
        self.pixels_per_deg_x = self.viewport_width / self.fov_width_deg
        self.pixels_per_deg_y = self.viewport_height / self.fov_height_deg

    def set_fov(self, fov_w: float, fov_h: float) -> None:
        """Dynamically update camera Field of View."""
        self.fov_width_deg = max(0.5, float(fov_w))
        self.fov_height_deg = max(0.5, float(fov_h))
        self.compute_fov_ratio()

    def set_camera_position(self, x: float, y: float) -> None:
        """Reposition movable virtual PTZ camera, bounded by canvas margins."""
        half_w = self.viewport_width / 2.0
        half_h = self.viewport_height / 2.0
        self.cam_x = max(half_w, min(self.canvas_width - half_w, float(x)))
        self.cam_y = max(half_h, min(self.canvas_height - half_h, float(y)))

    def move_camera(self, delta_x: float, delta_y: float) -> None:
        """Displace camera position by given pixel delta."""
        self.set_camera_position(self.cam_x + delta_x, self.cam_y + delta_y)

    def render_frame(
        self,
        platform_displacement: Tuple[float, float] = (0.0, 0.0),
        camera_jitter: Tuple[float, float] = (0.0, 0.0),
        intensity_factor: float = 1.0,
        baseline_override: int = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Renders the active 640x480 Monochrome Focal Plane Array (FPA) viewport frame.
        Applies platform displacement to the beacon and jitter to the camera viewport.
        """
        w, h = self.viewport_width, self.viewport_height
        baseline = self.sensor_baseline if baseline_override is None else baseline_override
        
        # Initialize single-channel 8-bit grayscale array (Monochrome FPA sensor)
        frame = np.full((h, w), baseline, dtype=np.uint8)

        # Effective positions with disturbances
        effective_cam_x = self.cam_x + camera_jitter[0]
        effective_cam_y = self.cam_y + camera_jitter[1]

        effective_target_x = self.motion.pos_x + platform_displacement[0]
        effective_target_y = self.motion.pos_y + platform_displacement[1]

        # Convert to viewport-relative coordinates
        rel_x = effective_target_x - (effective_cam_x - w / 2.0)
        rel_y = effective_target_y - (effective_cam_y - h / 2.0)

        # Draw beacon spot onto sensor matrix
        self.beacon.render_to_numpy(frame, rel_x, rel_y, intensity_factor=intensity_factor)

        ground_truth = {
            "target_global": (effective_target_x, effective_target_y),
            "camera_global": (effective_cam_x, effective_cam_y),
            "target_viewport": (rel_x, rel_y),
            "is_inside_fov": (0 <= rel_x < w and 0 <= rel_y < h),
            "fov_deg": (self.fov_width_deg, self.fov_height_deg),
            "px_per_deg": (self.pixels_per_deg_x, self.pixels_per_deg_y)
        }

        return frame, ground_truth
