"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: logs/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for telemetry logging and automated performance report
    generation (CSV and JSON). Exports the PerformanceLogger class.

Inputs:
    None.

Outputs:
    Exported class: PerformanceLogger.
================================================================================
"""

from .logger import PerformanceLogger

__all__ = ["PerformanceLogger"]
