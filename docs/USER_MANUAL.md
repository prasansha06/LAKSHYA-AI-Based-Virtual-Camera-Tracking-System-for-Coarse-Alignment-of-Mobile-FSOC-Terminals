# USER MANUAL: AI-BASED VIRTUAL CAMERA TRACKING SYSTEM

**Smart India Hackathon (SIH) 2026 | Problem Statement 26169**  
**Organization:** Indian Space Research Organisation (ISRO)  

---

## 1. System Overview & Quickstart

The **AI-Based Virtual Camera Tracking System** provides a virtual testbed for testing Pointing, Acquisition, and Tracking (PAT) coarse alignment algorithms for Free Space Optical Communication (FSOC) terminals.

### 1.1 Requirements
- Operating System: Windows 10/11, Linux, or macOS.
- Python: Version 3.10 or higher.
- Core Packages: `numpy`, `scipy`, `opencv-python`, `pygame`, `filterpy`, `matplotlib`, `websockets`, `pyinstaller`.

### 1.2 Quick Installation
Clone or navigate to the repository directory and install dependencies:
```bash
pip install -r requirements.txt
```

### 1.3 Running the Application
The system can be launched in several modes using `main.py`:

```bash
# 1. Launch Interactive Graphical Simulation (Pygame + OpenCV + Kalman + PID)
python main.py --sim

# 2. Launch Mission Control Dashboard (Web GUI + Python WebSocket Bridge)
python main.py --gui

# 3. Execute Benchmark Stage 1 (Automated Multi-Scenario Validation)
python main.py --benchmark1

# 4. Execute Benchmark Stage 2 on External .mp4 Video File
python main.py --benchmark2 test_videos/sample_beacon.mp4

# 5. Generate Synthetic .mp4 Test Videos for Benchmark 2 Preparation
python main.py --generate-videos

# 6. Run Complete Automated Audit of all 44 Requirements
python main.py --test
```

---

## 2. Graphical User Interface (GUI) Guide

The mission control interface is divided into functional operational panels:

```
+---------------------------------------------------------------------------------------------+
| [Header] ISRO SIH 2026 | PS ID 26169 | Status Indicators | Mode Switcher | Run/Reset        |
+---------------------------------------------------------------------------------------------+
| [KPI Strip] Lock Status | Error (<=10px) | Acq Time (<=2s) | Loss (<5%) | FPS (>=20) | RMSE  |
+-------------------------------------------------------------+-------------------------------+
|  DISPLAY VIEWPORTS                                          |  PARAMETER CONFIGURATION      |
|  1. Monochrome FPA Camera Viewport (640x480 px)             |  1. Target Parameters         |
|     - Single-channel grayscale sensor view                  |     - Shape, Size, Coords     |
|     - Optical center crosshair reticle                      |  2. Motion Patterns           |
|     - Green centroid bounding box + coordinates             |     - Straight, Cir, 8, Rand  |
|     - Cyan/Amber Kalman prediction ring                     |  3. Atmospheric Disturbances  |
|     - Red PID displacement vector                           |     - Clear, Haze, Fog, Rain  |
|  2. Global Field Tactical Radar (2000x2000 px Minimap)      |  4. Sensor Noise & Jitter     |
|     - Full arena coordinate grid                            |     - Std Dev slider (0-20px) |
|     - Moving platform & beacon trajectory trail             |     - Max Jitter slider       |
|     - Moving virtual camera bounding box                    |  5. Camera & PID Constraints  |
|  3. Real-Time Centroid Error Strip-Chart (Last 100 Frames)  |     - FOV (px/deg ratio)      |
|     - Red dashed target limit line at 10.0 px               |     - Pan/Tilt max speed      |
|  4. Claude AI Diagnostic Tuning & Narrative Panel           |  6. Log Exporters             |
|                                                             |     - CSV, JSON, Print Report |
+-------------------------------------------------------------+-------------------------------+
```

---

## 3. Parameter Configuration Reference

All simulation parameters update in real time without requiring a simulation restart:

### 3.1 Target & Beacon Settings
- **Target Shape**: Dropdown selector offering `Square` (default), `Circle`, or `Cross`.
- **Target Size**: Slider from `5` to `20` pixels (default: `10 px`).
- **Initial Location**: Numerical coordinate inputs for X and Y, plus a `Rand` button to randomize the beacon coordinates within safe operational margins.

### 3.2 Motion Patterns
Selectable radio buttons switching trajectory kinematics instantaneously:
- **Straight Line**: Linear vector with boundary reflection physics.
- **Circular**: $450\text{ px}$ radius orbital trajectory around canvas center.
- **Figure of 8**: Lemniscate of Gerono with dual-axis continuous acceleration changes.
- **Random Walk**: Brownian motion with velocity damping.
- **Spiral & Sinusoidal**: Optional extended test patterns.

### 3.3 Atmospheric Disturbances
- **Clear**: Baseline optical transmission (no attenuation).
- **Haze**: Optical contrast reduced by $30\%$.
- **Fog**: White atmospheric scattering overlay applied at $40\%$ opacity.
- **Rain**: Synthetic vertical rain streaks with light attenuation.
- **Low Light**: Scene brightness reduced by $50\%$.

### 3.4 Sensor Noise & Jitter
- **Noise Types**: Toggles for `Salt and Pepper` ($10\%$), `Gaussian`, and `Poisson`.
- **Standard Deviation in pixels**: Slider with range `0.0` to `20.0` pixels, directly controlling Gaussian noise variance.
- **Max Camera Jitter**: Slider from `±0` to `±20` pixels/frame.
- **Platform Motion**: Dropdown selector for `Linear` (mandatory $\pm 20\text{ px}$ max), `Circular`, `Random`, or `None`.

### 3.5 Camera & PTZ Constraints
- **Camera FOV**: Horizontal and vertical field of view in degrees (default $4^\circ \times 3^\circ$). The pixel-to-degree ratio ($\text{px/deg} = \text{width} / \text{FOV}^\circ$) is automatically calculated and displayed ($160.0\text{ px/deg}$).
- **Max Pan Speed**: Slider from `5.0` to `10.0` degrees per second (default $5.0^\circ/\text{s}$).
- **Max Tilt Speed**: Slider from `5.0` to `10.0` degrees per second (default $5.0^\circ/\text{s}$).
- **PID Gains ($K_p, K_i, K_d$)**: Sliders for fine-tuning tracking stability.

---

## 4. Benchmark 2 Video File Evaluation Guide

Benchmark Performance 2 represents **30% of the total evaluation marks**. ISRO evaluators supply unseen `.mp4` video files to test whether your tracking algorithm functions under arbitrary real-world conditions without a virtual PTZ camera.

### 4.1 Running Benchmark 2 via CLI
```bash
python main.py --benchmark2 test_videos/ISRO_eval_test.mp4
```

### 4.2 Running Benchmark 2 in GUI
1. Switch to the **Benchmark 2 (.mp4)** tab in the header.
2. Drag and drop the `.mp4` file onto the video drop zone, or click **Choose File**.
3. The video pipeline starts immediately, decoding frames at $30\text{ FPS}$, running centroid detection + Kalman filtering, and displaying live RMSE, Lock Retention Rate, and FPS.
4. When the video ends, the performance log is automatically saved to `logs/`.

---

## 5. Interpreting Performance Logs & Metrics

Exported performance reports (CSV and JSON) contain the following mandatory metrics:

| Metric Name | Calculation Method | Target Threshold | Interpretation |
|---|---|---|---|
| **Acquisition Time** | Elapsed time from start until 3 consecutive detections | $\le 2.0\text{ s}$ | Rapid lock acquisition |
| **Tracking Error** | Mean Euclidean displacement: $\frac{1}{N}\sum \|P_t - P_c\|$ | $\le 10.0\text{ px}$ | Centering accuracy |
| **Target Loss Rate** | Percentage of frames in `LOST` state: $\frac{N_{\text{lost}}}{N_{\text{total}}} \times 100\%$ | $< 5.0\%$ | Robustness against dropouts |
| **Re-acquisition Time** | Time to re-establish lock after occlusion | $\le 1.0\text{ s}$ | Rapid recovery |
| **Processing Speed** | Effective frame rate: $\frac{N}{\Delta T_{\text{total}}}$ | $\ge 20.0\text{ FPS}$ | Real-time throughput |
| **Root Mean Square Error (RMSE)** | $\sqrt{\frac{1}{N} \sum_{i=1}^N e_i^2}$ | Monitored | Sensitivity to transient spikes |

---

## 6. Troubleshooting

1. **Camera viewport loses target during sharp turns**:
   - Increase the **Max Pan/Tilt Speed** sliders from $5.0$ to $7.5\text{ deg/s}$.
   - Increase PID derivative gain $K_d$ from $0.35$ to $0.45$ to anticipate high acceleration.

2. **Centroid detector produces false positives under dense fog**:
   - The detector automatically adjusts threshold $T_{\text{dyn}}$ for fog ($175$). If false noise spots are detected, increase the Gaussian blur kernel or ensure the target size is at least $8\text{ px}$.

3. **Frame rate drops below 20 FPS**:
   - Verify that display hardware acceleration is enabled.
   - For headless automated benchmarks, use `python main.py --headless` which runs at over $150\text{ FPS}$.
