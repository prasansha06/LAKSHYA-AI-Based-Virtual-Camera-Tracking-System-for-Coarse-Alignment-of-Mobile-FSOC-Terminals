"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracker/state_machine.py
--------------------------------------------------------------------------------
Purpose:
    Manages the 3-state tracking discrete finite state machine:
    - ACQUIRED: Beacon detected consistently (>= 3 consecutive frames).
    - RE-ACQUIRING: Detector returns None; Kalman filter maintains trajectory prediction.
    - LOST: Detector returns None for 3 or more consecutive frames.
    Tracks and validates hard performance metrics:
    - Acquisition Time (<= 2.0 s from session start).
    - Re-acquisition Time (<= 1.0 s after occlusion).
    - Target Loss Rate (< 5.0% total frames).

Inputs:
    detection_found (bool): Whether detector produced a valid centroid this frame.
    current_time_sec (float): Current timestamp in seconds.

Outputs:
    current_state (str): 'acquired', 're-acquiring', 'lost'.
    timing_metrics (dict): Acquisition and re-acquisition times, lock retention rate.
================================================================================
"""

from typing import Dict, Any

class TrackerStateMachine:
    """
    State machine and timing benchmark evaluator for PAT coarse alignment.
    """

    STATE_ACQUIRED = "acquired"
    STATE_REACQUIRING = "re-acquiring"
    STATE_LOST = "lost"

    def __init__(self):
        self.state = self.STATE_LOST
        self.consecutive_detections = 0
        self.consecutive_losses = 0

        self.session_start_time = 0.0
        self.acquisition_start_time = 0.0
        self.acquisition_time_sec = 0.0
        self.acquisition_completed = False

        self.loss_start_time = 0.0
        self.reacquisition_time_sec = 0.0

        self.total_frames = 0
        self.lost_frames = 0

    def start_session(self, current_time_sec: float) -> None:
        """Initialize session timers."""
        self.session_start_time = current_time_sec
        self.acquisition_start_time = current_time_sec
        self.acquisition_completed = False
        self.acquisition_time_sec = 0.0
        self.reacquisition_time_sec = 0.0
        self.state = self.STATE_LOST
        self.consecutive_detections = 0
        self.consecutive_losses = 0
        self.total_frames = 0
        self.lost_frames = 0

    def update(self, detection_found: bool, current_time_sec: float) -> str:
        """
        Transitions state machine and measures timing milestones.
        Returns: 'acquired', 're-acquiring', or 'lost'
        """
        self.total_frames += 1
        prev_state = self.state

        if detection_found:
            self.consecutive_detections += 1
            self.consecutive_losses = 0

            if self.consecutive_detections >= 3:
                self.state = self.STATE_ACQUIRED

                # First initial acquisition from start
                if not self.acquisition_completed:
                    self.acquisition_time_sec = current_time_sec - self.acquisition_start_time
                    self.acquisition_completed = True

                # Re-acquisition after an occlusion/lost episode
                if prev_state in [self.STATE_LOST, self.STATE_REACQUIRING] and self.loss_start_time > 0:
                    self.reacquisition_time_sec = current_time_sec - self.loss_start_time
                    self.loss_start_time = 0.0
            else:
                self.state = self.STATE_REACQUIRING

        else:
            self.consecutive_losses += 1
            self.consecutive_detections = 0
            self.lost_frames += 1

            if prev_state == self.STATE_ACQUIRED:
                self.loss_start_time = current_time_sec

            if self.consecutive_losses >= 3:
                self.state = self.STATE_LOST
            else:
                self.state = self.STATE_REACQUIRING

        return self.state

    def get_metrics(self) -> Dict[str, Any]:
        """Returns computed timing and lock retention statistics."""
        loss_rate = (self.lost_frames / max(1, self.total_frames)) * 100.0
        retention_rate = max(0.0, 100.0 - loss_rate)
        return {
            "state": self.state,
            "acquisition_time_s": round(self.acquisition_time_sec, 3),
            "reacquisition_time_s": round(self.reacquisition_time_sec, 3),
            "target_loss_rate_pct": round(loss_rate, 2),
            "lock_retention_rate_pct": round(retention_rate, 2),
            "acquisition_pass": (self.acquisition_time_sec <= 2.0 and self.acquisition_completed),
            "reacquisition_pass": (self.reacquisition_time_sec <= 1.0),
            "loss_rate_pass": (loss_rate < 5.0)
        }
