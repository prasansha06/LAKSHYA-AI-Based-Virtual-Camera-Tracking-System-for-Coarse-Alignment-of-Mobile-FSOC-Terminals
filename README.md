# AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals



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
│
├── main.py                            # Master system entry point (CLI launcher & orchestrator)
├── bridge_server.py                   # AsyncIO WebSocket telemetry server (ws://127.0.0.1:8765)
├── build_executable.py                # Standalone PyInstaller packaging & automated post-build FPS test
├── ISRO_FSOC_Tracker.spec             # PyInstaller build specification & asset bundling config
├── requirements.txt                   # Production dependencies (OpenCV, Pygame, NumPy, SciPy, FilterPy, etc.)
├── README.md                          # Master project documentation & quickstart guide
│
├── sim/                               # 1. Physical Simulation Engine & Optics
│   ├── __init__.py                    # Module export definitions & header documentation
│   ├── scene.py                       # 2000x2000 arena, 640x480 Monochrome FPA viewport, 160 px/deg optical mapping
│   ├── beacon.py                      # Target beacon geometry (Square, Circle, Cross; 5 to 20 px sizing)
│   └── motion.py                      # Trajectory generators (Straight, Circular, Figure-8, Random Walk, Spiral, Sinusoidal)
│
├── disturbance/                       # 2. Multi-Physical Environmental Disturbance Engine
│   ├── __init__.py
│   ├── noise.py                       # Vectorized OpenCV C++ SIMD noise (Gaussian σ=0-20px, Poisson shot noise, Salt & Pepper 10%)
│   ├── atmospheric.py                 # Atmospheric attenuation models (Clear, Haze -30%, Fog +40%, Rain streaks, Low Light -50%)
│   └── platform.py                    # High-frequency camera jitter (±20 px/frame) & Linear platform motion (±20 px/frame)
│
├── tracker/                           # 3. Computer Vision, State Estimation & FSM
│   ├── __init__.py
│   ├── detector.py                    # Spatial Moments centroiding (M10/M00, M01/M00) & dynamic atmospheric thresholding
│   ├── kalman.py                      # 4D Linear Kalman Filter ([x, y, vx, vy]^T) with 10+ frame occlusion-hold predictor
│   └── state_machine.py               # 3-State FSM (ACQUIRED, RE-ACQUIRING, LOST) with acquisition & re-acquisition timers
│
├── controller/                        # 4. Closed-Loop PTZ Servo Control
│   ├── __init__.py
│   └── pid.py                         # Dual-axis Pan/Tilt PID with World Velocity Feedforward & angular rate clamping (5-10 deg/s)
│
├── pipeline/                          # 5. Benchmark 2 Video File Evaluation Pipeline
│   ├── __init__.py
│   ├── video_tracker.py               # Ingests external .mp4 videos with PTZ bypass, computes RMSE & lock retention
│   └── video_generator.py             # Synthesizes test .mp4 videos across motion and weather profiles
│
├── backend/                           # 6. Artificial Intelligence & Analytics Layer
│   ├── __init__.py
│   └── claude_advisor.py              # Automated log analyzer, PID damping diagnostics, and report narrative generator
│
├── frontend/                          # 7. Mission Control Dashboard (Web GUI)
│   ├── index.html                     # Responsive glassmorphic space command-center UI (Tailwind CSS)
│   ├── app.js                         # Telemetry bindings, live KPI status badges, and 100-frame error plotting
│   ├── simulation_engine.js           # Client-side simulation engine with 16,384-sample Float32Array Gaussian LUT
│   └── bridge_client.js               # Bi-directional WebSocket client streaming live telemetry from Python
│
├── tests/                             # 8. Verification & Quality Assurance Suite
│   ├── __init__.py
│   └── test_all_requirements.py       # Full automated audit verifying all 44 requirements from the ISRO specification
│
├── docs/                              # 9. Formal Documentation & Presentation Assets
│   ├── TECHNICAL_REPORT.md            # Comprehensive 15-page Technical Report covering all 8 ISRO sections
│   ├── USER_MANUAL.md                 # Complete User Manual, GUI operating instructions & troubleshooting
│   └── PRESENTATION_STUDY_GUIDE.md    # Master viva defense questions, math breakdown & slide speaker notes
│
├── logs/                              # 10. Telemetry Data & Audit Exports
│   ├── logger.py                      # Performance logging engine calculating RMSE, loss rate, and FPS
│   ├── simulation_session_log.csv     # Real-time frame-by-frame telemetry log (CSV export)
│   ├── simulation_session_log.json    # Machine-readable performance summary (JSON export)
│   ├── Benchmark1_straight_log.csv    # Benchmark Stage 1 audit log (Straight line scenario)
│   ├── Benchmark1_circular_log.csv    # Benchmark Stage 1 audit log (Circular orbit scenario)
│   ├── Benchmark1_figure8_log.csv     # Benchmark Stage 1 audit log (Figure-of-8 scenario)
│   └── Benchmark1_random_log.csv      # Benchmark Stage 1 audit log (Brownian random walk scenario)
│
├── test_videos/                       # 11. Benchmark 2 Video Test Feeds (.mp4)
│   ├── test_figure8_clear.mp4         # Synthetic Figure-of-8 test video under clear conditions
│   ├── test_circular_fog.mp4          # Synthetic Circular motion test video under fog attenuation
│   └── test_random_noise.mp4          # Synthetic Random walk test video under Gaussian/Poisson noise
│
└── dist/                              # 12. Packaged Standalone Release (ISRO Deliverable)
    └── ISRO_FSOC_Tracker/
        └── ISRO_FSOC_Tracker.exe      # Self-contained, zero-dependency executable ready for offline deployment

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
