"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: sim/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for the virtual simulation engine. Exports the VirtualScene,
    Beacon, and MotionPatternGenerator classes.

Inputs:
    None.

Outputs:
    Exported classes: VirtualScene, Beacon, MotionPatternGenerator.
================================================================================
"""

from .scene import VirtualScene
from .beacon import Beacon
from .motion import MotionPatternGenerator

__all__ = ["VirtualScene", "Beacon", "MotionPatternGenerator"]
