"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: controller/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for the virtual camera PTZ servo control system.
    Exports the PanTiltPIDController class.

Inputs:
    None.

Outputs:
    Exported class: PanTiltPIDController.
================================================================================
"""

from .pid import PanTiltPIDController

__all__ = ["PanTiltPIDController"]
