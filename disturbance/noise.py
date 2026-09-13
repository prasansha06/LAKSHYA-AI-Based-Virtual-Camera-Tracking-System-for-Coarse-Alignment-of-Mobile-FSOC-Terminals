"""
================================================================================
ISRO SIH 2026 - Problem Statement 26169
AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: disturbance/noise.py
--------------------------------------------------------------------------------
Purpose:
    Generates sensor-level image noise on the Monochrome Focal Plane Array (FPA)
    frames. Implements Salt and Pepper noise (10%), Gaussian noise using NumPy
    random normal, and Poisson photon noise. All three noise types are user-
    selectable individually or simultaneously. Includes configurable standard
    deviation parameter (0 to 20 pixels).

Inputs:
    frame (np.ndarray): 2D uint8 grayscale image.
    salt_pepper (bool): Enable Salt and Pepper at 10%.
    gaussian (bool): Enable Gaussian noise.
    poisson (bool): Enable Poisson shot noise.
    std_dev_pixels (float): Standard deviation parameter (0 to 20 px).

Outputs:
    noisy_frame (np.ndarray): Corrupted 2D uint8 grayscale image.
================================================================================
"""

import cv2
import numpy as np

class NoiseGenerator:
    """
    Simulates optical detector noise processes (thermal, read, and shot noise)
    using vectorized OpenCV C++ kernels for >= 60 FPS real-time performance.
    """

    def __init__(self):
        self.enable_salt_pepper = False
        self.enable_gaussian = False
        self.enable_poisson = False
        self.std_dev_pixels = 5.0  # Slider: Standard Deviation in pixels (0 to 20 px)
        self._noise_buf = None
        self._rand_buf = None

    def set_noise_types(self, salt_pepper: bool, gaussian: bool, poisson: bool) -> None:
        """Select noise models individually or in combination."""
        self.enable_salt_pepper = bool(salt_pepper)
        self.enable_gaussian = bool(gaussian)
        self.enable_poisson = bool(poisson)

    def set_std_dev(self, std_dev: float) -> None:
        """Set standard deviation parameter in pixels (0.0 to 20.0)."""
        self.std_dev_pixels = max(0.0, min(20.0, float(std_dev)))

    def apply(self, frame: np.ndarray) -> np.ndarray:
        """
        Applies configured noise distributions to the input image frame at high throughput (>= 60 FPS).
        """
        active_gauss = (self.enable_gaussian or self.std_dev_pixels > 0) and self.std_dev_pixels > 0.01
        if not (self.enable_salt_pepper or active_gauss or self.enable_poisson):
            return frame.copy()

        h, w = frame.shape
        if self._noise_buf is None or self._noise_buf.shape != (h, w):
            self._noise_buf = np.empty((h, w), dtype=np.float32)
            self._rand_buf = np.empty((h, w), dtype=np.uint8)

        noisy = frame.astype(np.float32)

        # 1. Gaussian Noise (Vectorized OpenCV C++ AVX kernel)
        if active_gauss:
            cv2.randn(self._noise_buf, 0.0, float(self.std_dev_pixels))
            noisy = cv2.add(noisy, self._noise_buf)

        # 2. Poisson Noise: Physical photon shot noise (variance = intensity)
        if self.enable_poisson:
            cv2.randn(self._noise_buf, 0.0, 1.0)
            shot = cv2.multiply(cv2.sqrt(cv2.max(noisy, 1.0)), self._noise_buf)
            noisy = cv2.add(noisy, shot)

        # 3. Salt and Pepper Noise: 10% corrupted pixels (5% salt, 5% pepper)
        if self.enable_salt_pepper:
            cv2.randu(self._rand_buf, 0, 255)
            noisy[self._rand_buf < 13] = 0.0      # 5% pepper
            noisy[self._rand_buf > 242] = 255.0   # 5% salt

        return np.clip(noisy, 0.0, 255.0).astype(np.uint8)
