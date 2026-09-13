"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracker/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for computer vision beacon detection, Kalman filter state
    estimation, and tracker state machine management.

Inputs:
    None.

Outputs:
    Exported classes: CentroidDetector, KalmanTracker, TrackerStateMachine.
================================================================================
"""

from .detector import CentroidDetector
from .kalman import KalmanTracker
from .state_machine import TrackerStateMachine

__all__ = ["CentroidDetector", "KalmanTracker", "TrackerStateMachine"]
