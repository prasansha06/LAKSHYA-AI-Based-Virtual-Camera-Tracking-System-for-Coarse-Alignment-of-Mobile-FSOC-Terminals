"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: logs/logger.py
--------------------------------------------------------------------------------
Purpose:
    Auto Performance Log Export Module generating CSV and JSON session reports.
    Implements all required reporting fields (Requirement 38 & 44):
    - Simulation duration (seconds)
    - Average processing speed (FPS)
    - Acquisition time (seconds)
    - Average tracking error (pixels)
    - Maximum tracking error (pixels)
    - Root Mean Square Error - RMSE (pixels)
    - Lock retention rate (%)
    - Frame processing time (milliseconds)
    - Full frame-by-frame telemetry rows

Inputs:
    Frame telemetry records during simulation or video tracking.

Outputs:
    Structured CSV file and JSON metadata document.
================================================================================
"""

import csv
import json
import math
import os
import time
from typing import List, Dict, Any, Optional

class PerformanceLogger:
    """
    Session telemetry recorder and automated CSV/JSON performance report generator.
    """

    def __init__(self):
        self.mode = "simulation"
        self.config: Dict[str, Any] = {}
        self.frames: List[Dict[str, Any]] = []
        self.start_time: float = 0.0
        self.session_duration_s: float = 0.0

    def start_session(self, mode: str = "simulation", config: Optional[Dict[str, Any]] = None) -> None:
        """Starts a new logging session."""
        self.mode = mode
        self.config = config or {}
        self.frames = []
        self.start_time = time.time()
        self.session_duration_s = 0.0

    def log_frame(
        self,
        frame_number: int,
        timestamp_sec: float,
        fps: float,
        centroid_error_px: float,
        lock_state: str,
        target_x: float,
        target_y: float,
        camera_x: float,
        camera_y: float,
        control_vx: float = 0.0,
        control_vy: float = 0.0,
        processing_time_ms: float = 0.0
    ) -> None:
        """Records a single frame telemetry snapshot."""
        record = {
            "frame": int(frame_number),
            "timestamp_s": round(float(timestamp_sec), 3),
            "fps": round(float(fps), 1),
            "centroid_error_px": round(float(centroid_error_px), 2),
            "lock_state": str(lock_state),
            "target_x": round(float(target_x), 1),
            "target_y": round(float(target_y), 1),
            "camera_x": round(float(camera_x), 1),
            "camera_y": round(float(camera_y), 1),
            "control_vx": round(float(control_vx), 2),
            "control_vy": round(float(control_vy), 2),
            "processing_time_ms": round(float(processing_time_ms), 2)
        }
        self.frames.append(record)

    def compute_summary(self, acquisition_time_s: float = 0.0, reacquisition_time_s: float = 0.0) -> Dict[str, Any]:
        """
        Computes the mandatory performance targets and statistical metrics.
        Computes RMSE explicitly: sqrt(mean(squared_errors))
        """
        n = len(self.frames)
        if n == 0:
            return {
                "duration_s": 0.0,
                "total_frames": 0,
                "average_fps": 0.0,
                "average_error_px": 0.0,
                "max_error_px": 0.0,
                "rmse_px": 0.0,
                "lock_retention_rate_pct": 0.0,
                "target_loss_rate_pct": 100.0,
                "avg_processing_time_ms": 0.0,
                "acquisition_time_s": 0.0,
                "reacquisition_time_s": 0.0
            }

        duration = self.frames[-1]["timestamp_s"] - self.frames[0]["timestamp_s"]
        if duration <= 0:
            duration = time.time() - self.start_time

        errors = [f["centroid_error_px"] for f in self.frames]
        fps_vals = [f["fps"] for f in self.frames]
        proc_times = [f["processing_time_ms"] for f in self.frames]

        # RMSE Computation (Requirement 44)
        squared_errors = [e * e for e in errors]
        rmse = math.sqrt(sum(squared_errors) / n)

        # Lock Retention Rate & Loss Rate
        lost_count = sum(1 for f in self.frames if f["lock_state"] == "lost")
        loss_rate = (lost_count / n) * 100.0
        retention_rate = max(0.0, 100.0 - loss_rate)

        return {
            "duration_s": round(duration, 2),
            "total_frames": n,
            "average_fps": round(sum(fps_vals) / n, 1),
            "average_error_px": round(sum(errors) / n, 2),
            "max_error_px": round(max(errors), 2),
            "rmse_px": round(rmse, 2),
            "lock_retention_rate_pct": round(retention_rate, 2),
            "target_loss_rate_pct": round(loss_rate, 2),
            "avg_processing_time_ms": round(sum(proc_times) / n, 2),
            "acquisition_time_s": round(acquisition_time_s, 3),
            "reacquisition_time_s": round(reacquisition_time_s, 3),
            "target_compliance": {
                "acquisition_time_pass": (acquisition_time_s <= 2.0),
                "tracking_error_pass": (round(sum(errors) / n, 2) <= 10.0),
                "target_loss_rate_pass": (loss_rate < 5.0),
                "reacquisition_time_pass": (reacquisition_time_s <= 1.0),
                "processing_speed_pass": (round(sum(fps_vals) / n, 1) >= 20.0)
            }
        }

    def export_csv(self, filepath: str, acquisition_time_s: float = 0.0, reacquisition_time_s: float = 0.0) -> str:
        """
        Exports all frame telemetry rows to a CSV file.
        Includes summary metadata header comments.
        """
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        summary = self.compute_summary(acquisition_time_s, reacquisition_time_s)

        with open(filepath, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            # ISRO Header block
            writer.writerow(["# ISRO SIH 2026 - Problem Statement 26169 Performance Log"])
            writer.writerow(["# Duration (s)", summary["duration_s"]])
            writer.writerow(["# Average FPS", summary["average_fps"]])
            writer.writerow(["# Average Tracking Error (px)", summary["average_error_px"]])
            writer.writerow(["# Maximum Tracking Error (px)", summary["max_error_px"]])
            writer.writerow(["# RMSE (px)", summary["rmse_px"]])
            writer.writerow(["# Lock Retention Rate (%)", summary["lock_retention_rate_pct"]])
            writer.writerow(["# Acquisition Time (s)", summary["acquisition_time_s"]])
            writer.writerow(["# Re-acquisition Time (s)", summary["reacquisition_time_s"]])
            writer.writerow([])

            # Column Headers
            headers = [
                "Frame", "Timestamp_s", "FPS", "Centroid_Error_px",
                "Lock_State", "Target_X", "Target_Y", "Camera_X", "Camera_Y",
                "Control_Vx", "Control_Vy", "Processing_Time_ms"
            ]
            writer.writerow(headers)

            for frame in self.frames:
                writer.writerow([
                    frame["frame"],
                    frame["timestamp_s"],
                    frame["fps"],
                    frame["centroid_error_px"],
                    frame["lock_state"],
                    frame["target_x"],
                    frame["target_y"],
                    frame["camera_x"],
                    frame["camera_y"],
                    frame["control_vx"],
                    frame["control_vy"],
                    frame["processing_time_ms"]
                ])

        return filepath

    def export_json(self, filepath: str, acquisition_time_s: float = 0.0, reacquisition_time_s: float = 0.0, extra_summary: Optional[Dict[str, Any]] = None) -> str:
        """Exports session summary and frame logs to a JSON document."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        summary = self.compute_summary(acquisition_time_s, reacquisition_time_s)
        if extra_summary:
            summary.update(extra_summary)

        data = {
            "metadata": {
                "organization": "Indian Space Research Organisation (ISRO)",
                "event": "Smart India Hackathon 2026",
                "problem_statement_id": "26169",
                "title": "AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals",
                "export_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "mode": self.mode
            },
            "configuration": self.config,
            "performance_summary": summary,
            "frames": self.frames
        }

        with open(filepath, mode="w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        return filepath
