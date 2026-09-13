"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: backend/__init__.py
--------------------------------------------------------------------------------
Purpose:
    Package initializer for backend intelligence and automated analysis.
    Exports the ClaudePerformanceAdvisor class.

Inputs:
    None.

Outputs:
    Exported class: ClaudePerformanceAdvisor.
================================================================================
"""

from .claude_advisor import ClaudePerformanceAdvisor

__all__ = ["ClaudePerformanceAdvisor"]
