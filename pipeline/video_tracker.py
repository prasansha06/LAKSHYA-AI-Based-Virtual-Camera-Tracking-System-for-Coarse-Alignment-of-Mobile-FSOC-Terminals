"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: pipeline/video_tracker.py
--------------------------------------------------------------------------------
Purpose:
    Executes the critical Benchmark Performance 2 evaluation protocol (30% marks).
    Processes arbitrary external .mp4 video files supplied by ISRO evaluators:
    - Bypasses the virtual PTZ camera entirely.
    - Decodes video frames using OpenCV VideoCapture.
    - Applies Centroid Detection on every frame.
    - Updates 4D Kalman Filter for smooth trajectory estimation & loss prediction.
    - Measures instantaneous error relative to optical center or reference.
    - Computes Root Mean Square Error (RMSE) across all frames.
    - Automatically exports performance log to CSV and JSON on completion.

Inputs:
    video_path (str): Path to the input .mp4 video file.
    output_log_path (str, optional): Target destination for exported report.

Outputs:
    results (dict): RMSE, average error, max error, lock retention rate, FPS,
                    acquisition time, re-acquisition time, and frame log table.
================================================================================
"""

import time
import math
import os
from typing import Dict, Any, Optional

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from tracker.detector import CentroidDetector
from tracker.kalman import KalmanTracker
from tracker.state_machine import TrackerStateMachine
from logs.logger import PerformanceLogger

class VideoPipelineTracker:
    """
    Dedicated video file tracking pipeline for ISRO Benchmark Stage 2 evaluation.
    """

    def __init__(self):
        self.detector = CentroidDetector()
        self.kalman = KalmanTracker()
        self.state_machine = TrackerStateMachine()
        self.logger = PerformanceLogger()

    def process_video(self, video_path: str, output_csv: Optional[str] = None) -> Dict[str, Any]:
        """
        Processes an .mp4 video file frame-by-frame and evaluates all mandatory metrics.
        """
        if not os.path.exists(video_path):
            raise FileNotFoundError(f"Video file not found at: {video_path}")

        if not HAS_CV2:
            raise ImportError("OpenCV (cv2) is required to process .mp4 video files.")

        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video file: {video_path}")

        fps_nominal = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        center_x, center_y = w / 2.0, h / 2.0

        self.kalman.reset(center_x, center_y)
        self.state_machine.start_session(0.0)
        self.logger.start_session(mode="benchmark2_video", config={"file": video_path, "resolution": f"{w}x{h}"})

        start_time = time.time()
        frame_idx = 0
        squared_errors_sum = 0.0
        total_error_sum = 0.0
        max_error = 0.0

        while True:
            t_frame_start = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            frame_idx += 1
            current_time = frame_idx / fps_nominal

            # Convert to grayscale if colour
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            # 1. Centroid Detection
            detection, det_info = self.detector.detect(gray)

            # 2. Kalman Prediction and Update
            self.kalman.predict(1.0 / fps_nominal)
            if detection is not None:
                track_pos, vel, is_pred = self.kalman.update(detection)
                meas_x, meas_y = detection
            else:
                track_pos, vel, is_pred = self.kalman.update(None)
                meas_x, meas_y = None, None

            # 3. State Machine Update
            state = self.state_machine.update(detection is not None, current_time)

            # 4. Error Calculation (displacement from center reticle in pixels)
            err = math.hypot(track_pos[0] - center_x, track_pos[1] - center_y)
            squared_errors_sum += err * err
            total_error_sum += err
            if err > max_error:
                max_error = err

            # Frame processing time
            proc_time_ms = (time.time() - t_frame_start) * 1000.0

            # Log frame entry
            self.logger.log_frame(
                frame_number=frame_idx,
                timestamp_sec=current_time,
                fps=fps_nominal,
                centroid_error_px=err,
                lock_state=state,
                target_x=track_pos[0],
                target_y=track_pos[1],
                camera_x=center_x,
                camera_y=center_y,
                control_vx=0.0,
                control_vy=0.0,
                processing_time_ms=proc_time_ms
            )

        cap.release()
        total_elapsed = time.time() - start_time
        measured_fps = frame_idx / max(0.001, total_elapsed)

        # Compute summary metrics (Requirement 44: RMSE)
        n_frames = max(1, frame_idx)
        rmse = math.sqrt(squared_errors_sum / n_frames)
        avg_error = total_error_sum / n_frames
        timing_metrics = self.state_machine.get_metrics()

        summary = {
            "video_path": video_path,
            "total_frames": frame_idx,
            "duration_sec": round(n_frames / fps_nominal, 2),
            "measured_fps": round(measured_fps, 1),
            "avg_error_px": round(avg_error, 2),
            "max_error_px": round(max_error, 2),
            "rmse_px": round(rmse, 2),
            "lock_retention_rate_pct": timing_metrics["lock_retention_rate_pct"],
            "target_loss_rate_pct": timing_metrics["target_loss_rate_pct"],
            "acquisition_time_s": timing_metrics["acquisition_time_s"],
            "reacquisition_time_s": timing_metrics["reacquisition_time_s"],
            "pass_targets": {
                "acquisition": timing_metrics["acquisition_pass"],
                "reacquisition": timing_metrics["reacquisition_pass"],
                "tracking_error": (avg_error <= 10.0),
                "target_loss": timing_metrics["loss_rate_pass"],
                "speed": (measured_fps >= 20.0)
            }
        }

        # Auto export performance log
        if output_csv is None:
            base = os.path.splitext(os.path.basename(video_path))[0]
            output_csv = f"logs/Benchmark2_{base}_results.csv"

        self.logger.export_csv(output_csv)
        json_path = output_csv.replace(".csv", ".json")
        self.logger.export_json(json_path, extra_summary=summary)

        return summary
