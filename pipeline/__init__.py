"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: pipeline/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for Benchmark 2 external video file evaluation pipeline
    and synthetic video generation suite.

Inputs:
    None.

Outputs:
    Exported classes: VideoPipelineTracker, SyntheticVideoGenerator.
================================================================================
"""

from .video_tracker import VideoPipelineTracker
from .video_generator import SyntheticVideoGenerator

__all__ = ["VideoPipelineTracker", "SyntheticVideoGenerator"]
