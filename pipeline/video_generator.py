"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: pipeline/video_generator.py
--------------------------------------------------------------------------------
Purpose:
    Generates synthetic .mp4 test videos covering all combinations of motion
    patterns, atmospheric modes, and noise types (Day 6 requirement, Page 11).
    Used to validate and benchmark the VideoPipelineTracker prior to ISRO evaluation.

Inputs:
    output_path (str): Destination .mp4 file path.
    motion_pattern (str): 'straight', 'circular', 'figure8', 'random'.
    atmospheric_mode (str): 'clear', 'haze', 'fog', 'rain', 'lowlight'.
    duration_sec (float): Video clip length in seconds (default 5.0 s).
    fps (float): Frame rate (default 30.0 FPS).

Outputs:
    Saves an encoded .mp4 video file with simulated optical beacon dynamics.
================================================================================
"""

import os
import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from sim.scene import VirtualScene
from disturbance.noise import NoiseGenerator
from disturbance.atmospheric import AtmosphericDisturbance
from disturbance.platform import PlatformDynamics
from tracker.detector import CentroidDetector
from tracker.kalman import KalmanTracker
from controller.pid import PanTiltPIDController

class SyntheticVideoGenerator:
    """
    Synthesizes test .mp4 videos for Benchmark Stage 2 validation.
    """

    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height

    def generate_video(
        self,
        output_path: str,
        motion_pattern: str = "figure8",
        atmospheric_mode: str = "clear",
        enable_noise: bool = False,
        std_dev_px: float = 5.0,
        duration_sec: float = 4.0,
        fps: float = 30.0
    ) -> str:
        """
        Generates and saves a synthetic .mp4 video clip.
        """
        if not HAS_CV2:
            raise ImportError("OpenCV (cv2) is required to generate synthetic .mp4 videos.")

        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

        scene = VirtualScene(
            canvas_width=2000,
            canvas_height=2000,
            viewport_width=self.width,
            viewport_height=self.height
        )
        scene.motion.set_pattern(motion_pattern)

        noise_gen = NoiseGenerator()
        if enable_noise:
            noise_gen.set_noise_types(salt_pepper=True, gaussian=True, poisson=False)
            noise_gen.set_std_dev(std_dev_px)

        atm_gen = AtmosphericDisturbance(self.width, self.height)
        atm_gen.set_mode(atmospheric_mode)

        platform = PlatformDynamics()
        platform.set_platform_mode("linear")

        # FourCC codec for MP4
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (self.width, self.height), isColor=False)

        total_frames = int(duration_sec * fps)
        dt = 1.0 / fps

        detector = CentroidDetector()
        kalman = KalmanTracker(self.width / 2.0, self.height / 2.0)
        from controller.pid import PanTiltPIDController
        scene.set_camera_position(scene.motion.pos_x, scene.motion.pos_y)
        pid = PanTiltPIDController(kp=18.0, ki=0.5, kd=1.2)

        prev_target_world_x = scene.cam_x
        prev_target_world_y = scene.cam_y
        target_world_vx = 0.0
        target_world_vy = 0.0

        for _ in range(total_frames):
            # Update motion
            scene.motion.update(dt)
            p_disp = platform.update_platform_motion(dt)
            c_jit = platform.get_camera_jitter()

            # Render frame
            raw_frame, gt = scene.render_frame(
                platform_displacement=p_disp,
                camera_jitter=c_jit,
                intensity_factor=atm_gen.get_intensity_factor()
            )

            # Apply atmospheric disturbance & noise
            atm_frame = atm_gen.apply(raw_frame, dt)
            final_frame = noise_gen.apply(atm_frame)

            out.write(final_frame)

            # Move camera so the recorded video represents realistic tracking feed
            det, _ = detector.detect(final_frame)
            kalman.predict(dt)
            if det is not None:
                pos, vel, _ = kalman.update(det)
            else:
                pos, vel, _ = kalman.update(None)

            err_x = pos[0] - self.width / 2.0
            err_y = pos[1] - self.height / 2.0

            target_world_x = scene.cam_x + err_x
            target_world_y = scene.cam_y + err_y
            raw_vx = (target_world_x - prev_target_world_x) / max(0.0001, dt)
            raw_vy = (target_world_y - prev_target_world_y) / max(0.0001, dt)
            alpha = 0.75
            target_world_vx = alpha * target_world_vx + (1.0 - alpha) * raw_vx
            target_world_vy = alpha * target_world_vy + (1.0 - alpha) * raw_vy
            prev_target_world_x = target_world_x
            prev_target_world_y = target_world_y

            dx, dy, _ = pid.compute(
                err_x, err_y, dt,
                scene.pixels_per_deg_x, scene.pixels_per_deg_y,
                feedforward_vx=target_world_vx, feedforward_vy=target_world_vy
            )
            scene.move_camera(dx, dy)

        out.release()
        return output_path
