# AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

**Smart India Hackathon (SIH) 2026 | Problem Statement ID: 26169**  
**Organization:** Indian Space Research Organisation (ISRO)  
**Theme:** Smart Automation and Space Technology  

---

## Overview

This repository contains the **Frontend GUI and Real-Time Evaluation Dashboard** for ISRO Problem Statement 26169. Free Space Optical Communication (FSOC) requires solving the Pointing, Acquisition, and Tracking (PAT) problem for narrow laser beams. Coarse alignment maintains the remote terminal beacon within the camera Field of View (FOV).

Per the ISRO technical specification, the frontend is built using modern web technologies to handle:
1. **Real-time telemetry and KPI tracking** against mandatory ISRO targets.
2. **Dual-viewport visualization**: 640x480 Monochrome Focal Plane Array (FPA) camera view alongside a 2000x2000 global space radar minimap.
3. **Full parameter configuration panel** for dynamic runtime hot-reloading (shapes, motion patterns, noise, atmospheric modes, FOV, and speeds).
4. **Benchmark 2 Video Pipeline** (.mp4 video tracking bypassing the virtual PTZ camera).
5. **Claude AI Intelligence Layer** for real-time parameter tuning suggestions and auto-generated technical reports.
6. **Auto Performance Log Export** generating CSV, JSON, and printable evaluation summaries.

---

## Directory Structure

```
fsoc_tracking_system/
├── frontend/
│   ├── index.html            # Main GUI Dashboard (HTML5 + Tailwind CSS)
│   ├── simulation_engine.js  # Pure JS Physics, Kalman Filter, PID, & CV Engine
│   ├── app.js                # UI Controller, Metrics Engine, & Event Handlers
│   └── bridge_client.js      # WebSocket Bridge Client for Python backend
├── bridge_server.py          # Python WebSocket Server for Pygame/OpenCV integration
└── README.md                 # Documentation & User Manual
```

---

## Mandatory ISRO Performance Targets

The dashboard continuously verifies compliance with all 5 mandatory targets:

| Metric | Target Value | Dashboard Badge | Consequence if Missed |
|---|---|---|---|
| **Acquisition Time** | $\le 2.0$ seconds | `PASS (≤2s)` | Fails Benchmark Stages 1 & 2 |
| **Tracking Error** | $\le 10$ pixels | `PASS (≤10px)` | Fails Benchmark Stages 1 & 2 |
| **Target Loss Rate** | $< 5.0\%$ | `PASS (<5%)` | Fails Benchmark Stages 1 & 2 |
| **Re-acquisition Time** | $\le 1.0$ second | `PASS (≤1s)` | Fails Benchmark Stages 1 & 2 |
| **Processing Speed** | $\ge 20$ FPS | `PASS (≥20)` | Fails all benchmark stages |
| **Session RMSE** | Reported in real time | `ISRO B2 Metric` | Evaluated in Benchmark Stage 2 |

---

## How to Run

### Mode 1: Autonomous Browser Mode (Instant Demo)
No Python libraries or compilation needed. Simply double-click or open `frontend/index.html` in any modern web browser (Chrome, Edge, Firefox). The built-in simulation engine will start running immediately at 30+ FPS.

### Mode 2: Connected Python Backend Mode
To stream telemetry directly from your Python simulation (`pygame`, `opencv`, `filterpy`, `scipy`):
1. Install requirements:
   ```bash
   pip install websockets
   ```
2. Start the bridge server:
   ```bash
   python bridge_server.py
   ```
3. Open `frontend/index.html` in your browser. The dashboard will automatically link to `ws://localhost:8765`.

---

## Features & Controls

### 1. Dual Viewport Display
- **Monochrome FPA Viewport (640x480 px)**: Displays single-channel grayscale sensor frames, simulated atmospheric attenuation (Haze, Fog, Rain, Low Light), sensor noise (Gaussian, Poisson, Salt & Pepper), optical center crosshair reticle, centroid bounding box, Kalman filter prediction ring, and PID error vector.
- **Global Field Radar (2000x2000 px)**: Overhead radar minimap showing the full arena, moving platform, target beacon trail, and the virtual PTZ camera viewport rectangle moving to track the target.

### 2. Real-Time Error Plot
- Plots Centroiding Error over the last 100 frames.
- Features a prominent **red dashed line at 10 pixels** to monitor the hard target threshold.

### 3. Parameter Configuration Panel
- **Target Parameters**:
  - Target Shape: `Square` (default), `Circle`, `Cross`.
  - Target Size: Slider from `5` to `20` pixels (default `10 px`).
  - Target Initial Location: Numerical X/Y coordinates + `Randomize` button.
- **Motion Patterns**:
  - `Straight Line` (with wall bouncing), `Circular`, `Figure of 8` (Lemniscate), `Random Walk` (all mandatory), plus `Spiral` and `Sinusoidal`.
- **Atmospheric Disturbances**:
  - `Clear` (baseline)
  - `Haze` (30% contrast reduction)
  - `Fog` (40% white opacity overlay)
  - `Rain` (animated vertical streaks)
  - `Low Light` (50% brightness reduction)
- **Sensor Noise & Disturbances**:
  - `Salt & Pepper` (10%), `Gaussian`, and `Poisson` toggles (combinable).
  - **Standard Deviation in pixels**: Slider strictly labelled as specified, range `0` to `20` px.
  - **Max Camera Jitter**: `±0` to `±20` px/frame.
  - **Platform Motion**: `Linear` (±20 px max, mandatory), `Circular`, `Random`.
- **Camera & PTZ Constraints**:
  - Camera FOV: $4^\circ \times 3^\circ$ default, with real-time computation of the pixel-to-degree ratio ($\text{px/deg} = \text{width} / \text{FOV width}^\circ$).
  - Max Pan Speed: `5` to `10` deg/s slider.
  - Max Tilt Speed: `5` to `10` deg/s slider.
  - PID Gains: $K_p$, $K_i$, and $K_d$ sliders.

### 4. Benchmark 2 Video File Pipeline
Switch to the **Benchmark 2 (.mp4)** tab in the header:
- Drag-and-drop or select an unseen `.mp4` video supplied by ISRO evaluators.
- The pipeline bypasses the PTZ camera, runs centroid detection + Kalman filtering directly on the video frames, and reports RMSE, Lock Retention Rate, and FPS.

### 5. Claude AI Tuning Advisor (Day 8 Integration)
Click **Run Diagnostic Analysis** in the AI panel. Claude evaluates the telemetry log, flags error spikes caused by severe disturbance scenarios, and outputs actionable PID/Kalman tuning recommendations alongside an auto-generated narrative paragraph for the technical report.

### 6. Export Options
- **CSV Log**: Detailed frame-by-frame log (timestamps, error, camera coords, target coords, PID control signals).
- **JSON Data**: Comprehensive session summary and metadata for programmatic evaluation.
- **Print / PDF**: Clean, print-formatted evaluation report.
