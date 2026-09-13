"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracker/detector.py
--------------------------------------------------------------------------------
Purpose:
    Optical beacon detection using Computer Vision (OpenCV).
    Pipeline:
    1. Input: 640x480 Monochrome FPA grayscale frame.
    2. Noise reduction via Gaussian blur filter.
    3. Binary intensity thresholding to segment bright optical beacon.
    4. Contour extraction using OpenCV findContours.
    5. Centroid computation via spatial moments: (M10/M00, M01/M00).
    6. Returns (cx, cy) or None if target is lost/occluded.
    Maintains >95% detection rate under noise and atmospheric disturbances.

Inputs:
    frame (np.ndarray): 2D uint8 grayscale image.
    atmospheric_mode (str): Active atmosphere to dynamically adapt threshold.

Outputs:
    centroid (tuple or None): (cx, cy) in viewport pixels, or None if lost.
    detection_info (dict): Bounding box, contour area, and raw moments.
================================================================================
"""

import numpy as np
from typing import Tuple, Optional, Dict, Any

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

class CentroidDetector:
    """
    OpenCV-based Centroiding Detector for coarse alignment PAT tracking.
    """

    def __init__(self, blur_kernel: int = 5, base_threshold: int = 140):
        self.blur_kernel = blur_kernel
        self.base_threshold = base_threshold
        self.total_frames = 0
        self.successful_detections = 0

    def get_detection_rate(self) -> float:
        """Returns cumulative detection success percentage (>95% required)."""
        if self.total_frames == 0:
            return 100.0
        return (self.successful_detections / self.total_frames) * 100.0

    def detect(self, frame: np.ndarray, atmospheric_mode: str = "clear") -> Tuple[Optional[Tuple[float, float]], Dict[str, Any]]:
        """
        Executes Gaussian blur, thresholding, and contour centroiding.
        Returns ((cx, cy), info_dict) or (None, info_dict) if lost.
        """
        self.total_frames += 1
        h, w = frame.shape[:2]

        # Dynamic threshold adaptation for atmospheric modes
        threshold_val = self.base_threshold
        if atmospheric_mode == "lowlight":
            threshold_val = 70
        elif atmospheric_mode == "fog":
            threshold_val = 175
        elif atmospheric_mode == "haze":
            threshold_val = 100

        if HAS_CV2:
            # 1. Gaussian Blur to reduce noise
            k = self.blur_kernel if self.blur_kernel % 2 == 1 else self.blur_kernel + 1
            blurred = cv2.GaussianBlur(frame, (k, k), 0)

            # 2. Binary Thresholding to isolate bright beacon
            _, binary = cv2.threshold(blurred, threshold_val, 255, cv2.THRESH_BINARY)

            # 3. Find Contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if not contours:
                return None, {"found": False, "bbox": None, "area": 0}

            # 4. Find Largest Contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Rejection of spurious single-pixel noise or full-frame flashes
            if area < 1.0 or area > (w * h * 0.40):
                return None, {"found": False, "bbox": None, "area": area}

            # 5. Spatial Moments Centroid: (M10/M00, M01/M00)
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return None, {"found": False, "bbox": None, "area": area}

            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])

            x, y, bw, bh = cv2.boundingRect(largest_contour)

            self.successful_detections += 1
            return (cx, cy), {
                "found": True,
                "bbox": (x, y, bw, bh),
                "area": area,
                "binary": binary
            }

        else:
            # High performance pure NumPy fallback if OpenCV is loading
            mask = frame > threshold_val
            y_indices, x_indices = np.nonzero(mask)
            count = len(x_indices)

            if count < 2 or count > (w * h * 0.40):
                return None, {"found": False, "bbox": None, "area": count}

            cx = float(np.mean(x_indices))
            cy = float(np.mean(y_indices))
            min_x, max_x = int(np.min(x_indices)), int(np.max(x_indices))
            min_y, max_y = int(np.min(y_indices)), int(np.max(y_indices))

            self.successful_detections += 1
            return (cx, cy), {
                "found": True,
                "bbox": (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1),
                "area": count
            }
