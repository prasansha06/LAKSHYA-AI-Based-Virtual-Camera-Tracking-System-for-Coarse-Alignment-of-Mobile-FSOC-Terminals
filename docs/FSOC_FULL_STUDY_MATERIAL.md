# Comprehensive Study Material & Presentation Guide
## ISRO SIH 2026 Problem Statement 26169
### AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

---

## 1. Executive Summary & Problem Context

### What is Free Space Optical Communication (FSOC)?
Free Space Optical Communication (FSOC) transmits data by modulating near-infrared laser beams (typically $850\text{ nm}$ or $1550\text{ nm}$) through the atmosphere or outer space.

| Feature | Conventional RF (Radio Frequency) | FSOC (Optical / Laser) |
|---|---|---|
| **Data Rate** | Megabits to low Gigabits ($< 1\text{ Gbps}$) | Multi-Gigabits to Terabits ($10\text{–}100+\text{ Gbps}$) |
| **Beam Divergence** | Wide beam cone (degrees) | Extremely narrow beam (microradians to milliradians) |
| **Spectrum Licensing** | Strictly regulated, congested RF bands | Unlicensed, unlimited optical spectrum |
| **Security & Interference** | Susceptible to jamming & eavesdropping | Immune to RF jamming & EMI; tap-proof |
| **Terminal Weight & Power** | Heavy antenna dishes, high power | Compact optical apertures, low power SWaP |

### The Pointing, Acquisition, and Tracking (PAT) Dilemma
Because optical laser beams have beam divergence angles in the order of **microradians** ($1\text{ mrad} \approx 0.057^\circ$), even microscopic vibrations of a moving satellite, drone, ship, or ground terminal cause the laser beam to miss the receiver completely.

To solve this, optical communication architectures split alignment into two stages:
1. **Coarse Alignment (This Project)**:
   - Uses a wide Field-of-View (FOV) virtual Pan-Tilt camera ($4^\circ \times 3^\circ$, $640\times 480\text{ px}$) to locate an optical beacon across a large angular domain ($2000\times 2000\text{ px}$ canvas).
   - Steers the gimbaled optical assembly to center the beacon within $\le 10\text{ pixels}$ of the optical boresight.
   - Operates under severe platform vibrations, camera jitter, sensor noise, and weather degradation.
2. **Fine Alignment (Subsequent Hardware Stage)**:
   - Once the coarse tracker brings the beacon within $\le 10\text{ pixels}$, high-bandwidth Fast Steering Mirrors (FSMs) or piezo-actuated quad-cell sensors take over to achieve sub-microradian coupling into a single-mode optical fiber core ($9\text{ \mu m}$ diameter).

---

## 2. End-to-End System Architecture

```
+----------------------------------------------------------------------------------------------------+
|                         ISRO FSOC COARSE ALIGNMENT SYSTEM ARCHITECTURE                             |
+----------------------------------------------------------------------------------------------------+
|                                                                                                    |
|  [2000x2000 Global Arena]               [Disturbance & Noise Injectors]                            |
|  - Target Beacon Generator   ---------> - Salt & Pepper (10%), Gaussian, Poisson                   |
|  - 4 Motion Patterns Dynamics           - Camera Jitter (±20px), Platform Motion (±20px)           |
|                                         - Atmospheric (Clear, Haze, Fog, Rain, Low Light)          |
|                                                              |                                     |
|                                                              v                                     |
|                                         +--------------------------------------------+             |
|                                         | 640x480 Monochrome FPA Viewport (4° x 3°)  |             |
|                                         | Pixel-to-degree ratio: 160 px/deg          |             |
|                                         +--------------------------------------------+             |
|                                                              |                                     |
|   [Benchmark 2 .mp4 Video File] ----------------------------+ (Optical Frame Matrix)              |
|                                                              |                                     |
|                                                              v                                     |
|                                         +--------------------------------------------+             |
|                                         | Computer Vision Centroid Detector (OpenCV) |             |
|                                         | Gaussian Blur -> Binary Threshold -> (M10) |             |
|                                         +--------------------------------------------+             |
|                                                              | (Centroid X, Y or None)             |
|                                                              v                                     |
|                                         +--------------------------------------------+             |
|                                         | 4D Linear Kalman Filter Tracker & FSM      |             |
|                                         | State: [x, y, vx, vy]^T | Covariance: P    |             |
|                                         | Occlusion Prediction (10+ frames)          |             |
|                                         +--------------------------------------------+             |
|                                                              | (Filtered State / Prediction)       |
|                                                              v                                     |
|                                         +--------------------------------------------+             |
|                                         | Dual-Axis Pan/Tilt PID Controller          |             |
|                                         | World-Space Target Velocity Feedforward    |             |
|                                         | Clamped to 5.0 - 10.0 deg/s via px/deg     |             |
|                                         +--------------------------------------------+             |
|                                                              |                                     |
|                                                              v (Displacement Vector Δcam)          |
|                                         +--------------------------------------------+             |
|                                         | Movable Virtual Camera Repositioning Loop  |             |
|                                         +--------------------------------------------+             |
|                                                              |                                     |
|                                                              v                                     |
|                                         +--------------------------------------------+             |
|                                         | Telemetry Logging & Performance Dashboard  |             |
|                                         | - Real-time error strip-chart (last 100 f) |             |
|                                         | - Session RMSE: sqrt(mean(error^2))        |             |
|                                         | - Auto-export to CSV & JSON reports        |             |
|                                         | - Claude AI Adaptive Tuning Advisor        |             |
|                                         +--------------------------------------------+             |
+----------------------------------------------------------------------------------------------------+
```

---

## 3. Mathematical Formulations & Component Deep Dive

### 3.1 Optics & Focal Plane Array (FPA) Geometry
- **Canvas Dimension**: $2000 \times 2000\text{ pixels}$.
- **Camera Resolution**: $640 \times 480\text{ pixels}$ (8-bit grayscale, Monochrome FPA sensor).
- **Field of View (FOV)**: $4.0^\circ\text{ horizontal} \times 3.0^\circ\text{ vertical}$.
- **Pixel-to-Degree Ratio**:
  $$\text{Ratio}_x = \frac{W_{\text{viewport}}}{\text{FOV}_x} = \frac{640\text{ px}}{4.0^\circ} = 160.0\text{ px/deg}$$
  $$\text{Ratio}_y = \frac{H_{\text{viewport}}}{\text{FOV}_y} = \frac{480\text{ px}}{3.0^\circ} = 160.0\text{ px/deg}$$
- **Angular-to-Pixel Speed Clamping**:
  $$\text{Max Speed (px/s)} = \text{Max Speed (deg/s)} \times 160.0\text{ px/deg}$$
  For $5.0^\circ\text{/s}$ max speed:
  $$V_{\max} = 5.0 \times 160.0 = 800.0\text{ px/s}$$

---

### 3.2 Target Motion Dynamics
1. **Straight Line**:
   $$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v} \cdot \Delta t$$
   With elastic reflection at canvas boundary pads ($340\text{ px}, 260\text{ px}$).
2. **Circular**:
   $$x(t) = x_c + R \cos(\omega t), \quad y(t) = y_c + R \sin(\omega t)$$
   ($R = 450\text{ px}, \omega = 0.55\text{ rad/s}$).
3. **Figure of 8 (Lemniscate of Gerono)**:
   $$x(t) = x_c + a \sin(\omega t), \quad y(t) = y_c + \frac{b}{2} \sin(2\omega t)$$
   ($a = 500\text{ px}, b = 300\text{ px}, \omega = 0.45\text{ rad/s}$).
4. **Random Walk (Brownian Motion)**:
   $$\mathbf{v}(t + \Delta t) = \mathbf{v}(t) + \mathcal{N}(0, \sigma_v^2)$$
   $$\mathbf{p}(t + \Delta t) = \mathbf{p}(t) + \mathbf{v}(t + \Delta t) \cdot \Delta t$$

---

### 3.3 Environmental Disturbances & Atmospheric Modeling
1. **Camera Jitter**:
   $$\Delta_{\text{jitter}} \sim \mathcal{U}(-20\text{ px}, +20\text{ px})$$
2. **Linear Platform Motion**:
   $$\Delta_{\text{platform}}(t) = A_p \sin(\omega_p t) \quad (A_p \le 20\text{ px/frame})$$
3. **Sensor Noise**:
   - **Gaussian Noise**: Thermal noise / Johnson-Nyquist noise in sensor electronics:
     $$I_{\text{noisy}}(x, y) = \text{clip}\left(I(x, y) + \mathcal{N}(0, \sigma^2), 0, 255\right)$$
   - **Poisson Shot Noise**: Quantum photon arrival statistics modeled via variance scaling:
     $$I_{\text{noisy}} = \text{clip}\left(I + \sqrt{I} \cdot \mathcal{N}(0, 1), 0, 255\right)$$
   - **Salt & Pepper Noise**: Bit flips and dead/hot pixels:
     $$P(I = 255) = \frac{p}{2}, \quad P(I = 0) = \frac{p}{2}, \quad P(\text{unchanged}) = 1 - p$$
4. **Atmospheric Weather Modes**:
   - **Clear**: Baseline contrast ($100\%$).
   - **Haze**: Aerosol scattering ($30\%$ contrast attenuation):
     $$I_{\text{haze}} = 0.70 \cdot I_{\text{beacon}} + 0.30 \cdot I_{\text{ambient}}$$
   - **Fog**: Mie scattering overlay ($40\%$ additive luminance veil):
     $$I_{\text{fog}} = I + 0.40 \times 255$$
   - **Rain**: Moving vertical streak particles ($70$ particles, speed $15\text{–}25\text{ px/frame}$).
   - **Low Light**: Sensor dark current regime ($50\%$ irradiance drop):
     $$I_{\text{lowlight}} = 0.50 \cdot I$$

---

### 3.4 Computer Vision Centroid Detection
Instead of computationally heavy deep neural networks, we utilize **Spatial Moments Centroid Estimation**:
1. **Pre-filtering**: $3 \times 3$ Gaussian blur kernel ($\sigma = 1.0$) to suppress high-frequency salt & pepper noise.
2. **Dynamic Binary Thresholding**:
   $$\mathcal{B}(x, y) = \begin{cases} 255 & \text{if } I(x, y) > T_{\text{mode}} \\ 0 & \text{otherwise} \end{cases}$$
   where $T_{\text{clear}} = 140$, $T_{\text{haze}} = 100$, $T_{\text{fog}} = 175$, $T_{\text{lowlight}} = 70$.
3. **Contour Extraction & Spatial Moments**:
   $$M_{pq} = \sum_x \sum_y x^p y^q \mathcal{B}(x, y)$$
   - Zeroth moment (Area): $M_{00} = \sum_x \sum_y \mathcal{B}(x, y)$
   - First moments: $M_{10} = \sum x \mathcal{B}(x, y), \quad M_{01} = \sum y \mathcal{B}(x, y)$
   - **Sub-pixel Centroid**:
     $$c_x = \frac{M_{10}}{M_{00}}, \quad c_y = \frac{M_{01}}{M_{00}}$$

---

### 3.5 4D Linear Kalman Filter State Estimator
To filter jitter, bridge momentary sensor dropouts, and estimate target velocity, we use a 4D Linear Kalman Filter:
- **State Vector**:
  $$\mathbf{x} = \begin{bmatrix} x \\ y \\ v_x \\ v_y \end{bmatrix}$$
- **State Transition Matrix ($F$)**:
  $$F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
- **Measurement Matrix ($H$)**:
  $$H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$
- **Prediction Equations**:
  $$\hat{\mathbf{x}}_k^- = F \hat{\mathbf{x}}_{k-1}$$
  $$P_k^- = F P_{k-1} F^T + Q$$
- **Update Equations (when beacon detected)**:
  $$\mathbf{y}_k = \mathbf{z}_k - H \hat{\mathbf{x}}_k^- \quad (\text{Innovation})$$
  $$S_k = H P_k^- H^T + R \quad (\text{Innovation Covariance})$$
  $$K_k = P_k^- H^T S_k^{-1} \quad (\text{Kalman Gain})$$
  $$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \mathbf{y}_k$$
  $$P_k = (I - K_k H) P_k^-$$
- **Occlusion Handling (when beacon occluded)**:
  $$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^-, \quad P_k = P_k^-$$
  Allows the camera to continue tracking target inertia across $10+$ consecutive frames of complete visual loss.

---

### 3.6 Discrete 3-State Finite State Machine (FSM)
- **States**:
  - `LOST`: Target not in FOV or unacquired.
  - `RE-ACQUIRING`: Lost target re-enters FOV or transient detection ($< 3$ consecutive frames).
  - `ACQUIRED`: Locked on target ($\ge 3$ consecutive detections).
- **Hysteresis Thresholding**:
  - $3$ consecutive detections required to enter `ACQUIRED`.
  - $3$ consecutive misses required to enter `LOST`.
  - Eliminates flickering during noisy frame transitions.

---

### 3.7 Dual-Axis Pan/Tilt PID Controller with World Velocity Feedforward
- **Tracking Error Vector**:
  $$e_x = x_{\text{track}} - 320.0, \quad e_y = y_{\text{track}} - 240.0$$
  $$\text{Centroid Error} = \sqrt{e_x^2 + e_y^2}$$

- **World-Space Velocity Feedforward**:
  $$X_{\text{world}}(t) = X_{\text{cam}}(t) + e_x(t), \quad Y_{\text{world}}(t) = Y_{\text{cam}}(t) + e_y(t)$$
  $$V_{x, \text{world}}(t) = \alpha V_{x, \text{world}}(t - \Delta t) + (1 - \alpha) \frac{X_{\text{world}}(t) - X_{\text{world}}(t - \Delta t)}{\Delta t} \quad (\alpha = 0.75)$$
  $$V_{y, \text{world}}(t) = \alpha V_{y, \text{world}}(t - \Delta t) + (1 - \alpha) \frac{Y_{\text{world}}(t) - Y_{\text{world}}(t - \Delta t)}{\Delta t}$$

- **Dual-Axis Control Law**:
  $$u_x(t) = V_{x, \text{world}}(t) + K_p e_x(t) + K_i \int e_x(\tau) d\tau + K_d \frac{de_x(t)}{dt}$$
  $$u_y(t) = V_{y, \text{world}}(t) + K_p e_y(t) + K_i \int e_y(\tau) d\tau + K_d \frac{de_y(t)}{dt}$$

- **Tuned Gains**:
  $$K_p = 18.0, \quad K_i = 0.5, \quad K_d = 1.2$$
  - Anti-windup clamping on integral: $[-60, +60]$.
  - Angular rate clamping: $\omega_{\text{pan}} \le 5.0^\circ\text{/s}$, $\omega_{\text{tilt}} \le 5.0^\circ\text{/s}$.

---

## 4. Why We Chose These Specific Technologies ("Why did I use any of these?")

### Why Spatial Moments Centroid instead of Deep Learning (YOLO / CNN)?
1. **Computational Budget & Hard FPS Target**:
   - ISRO Requirement 20 demands processing speed $\ge 20\text{ FPS}$ ($50\text{ ms}$ max latency).
   - Deep learning models (YOLOv8, MobileNet) require $30\text{–}80\text{ ms}$ on CPU per frame and have substantial memory footprints.
   - Spatial moments centroiding computes in **$< 0.5\text{ ms}$** on CPU ($> 800\text{ FPS}$ headless audit).
2. **Nature of the Beacon Spot**:
   - An optical laser beacon on a monochrome FPA is not a complex semantic object (like a car or person); it is an active emitter characterized by a high-intensity Gaussian irradiance distribution.
   - Heavy semantic feature extraction adds zero value while introducing non-deterministic neural inference latencies.
3. **Sub-Pixel Precision**:
   - Spatial moments $M_{10}/M_{00}$ naturally yield floating-point sub-pixel centroids, critical for keeping tracking errors under $2\text{ pixels}$.
4. **Zero GPU Dependency**:
   - Spacecraft flight computers and embedded ground terminal DSPs rarely have high-power consumer GPUs. CPU-friendly spatial moments can be compiled directly onto radiation-hardened FPGA or DSP chips.

---

### Why 4D Linear Kalman Filter instead of Particle Filter or UKF?
1. **Physical Match with Kinematics**:
   - At high sampling rates ($30\text{–}60\text{ Hz}$), the time step $\Delta t \le 33\text{ ms}$ is sufficiently small that target trajectory between consecutive frames is linear. A constant-velocity kinematic model ($[x, y, v_x, v_y]^T$) fits the physics.
2. **Computational Determinism**:
   - The linear Kalman filter involves only $4 \times 4$ matrix operations with closed-form matrix inversions. It takes $< 0.05\text{ ms}$ per step.
   - Particle filters require evaluating hundreds of particles, creating frame-rate jitter.
3. **BLUE Optimality**:
   - Under Gaussian thermal sensor noise, the Kalman filter is mathematically proven to be the **Best Linear Unbiased Estimator (BLUE)**.
4. **Occlusion Bridging**:
   - During heavy fog, low light, or rain obstruction where detection fails, the Kalman filter’s prediction step continues dead reckoning based on target momentum, allowing the camera to maintain trajectory.

---

### Why PID with World-Space Velocity Feedforward instead of Plain PID?
1. **Eliminating the Fundamental Velocity Lag**:
   - A standard PID controller produces control effort proportional to error. For a moving target, the camera must lag behind the target to generate the forward drive speed ($e_{ss} = V / K_p$).
   - Adding **World-Space Velocity Feedforward** passes the target's physical ground speed directly to the servo drive. The feedforward term handles the movement, while PID feedback only cleans up residual errors ($e \approx 0$).
2. **Decoupling Tracking Speed from Jitter Sensitivity**:
   - Without feedforward, reducing tracking error requires cranking $K_p$ to extreme values ($K_p > 50$), which amplifies camera jitter and noise, causing high-frequency mechanical oscillation.
   - Feedforward keeps $K_p$ moderate ($18.0$) while achieving average errors of $< 1.9\text{ pixels}$.

---

### Why Vectorized OpenCV Kernels & Float32Array LUT?
1. **NumPy Inhomogeneity Elimination**:
   - Profiling showed scalar loops and inhomogeneous Poisson sampling in NumPy took $108\text{ ms}$ per frame ($8.5\text{ FPS}$).
   - Replacing them with C++ vectorized `cv2.randn()` and variance scaling reduced noise latency to $2.0\text{ ms}$ ($61.7\text{ FPS}$).
2. **Eliminating Box-Muller Loops in Browser**:
   - In JavaScript, calculating Box-Muller transforms ($307,200$ calls to `Math.log` & `Math.cos` per frame) choked the browser at $11.5\text{ FPS}$.
   - Precomputing a 16,384-sample Float32Array LUT reduced frame times from $87\text{ ms}$ to $10.5\text{ ms}$ ($95.2\text{ FPS}$ raw capability, locked to $60\text{ FPS}$).

---

### Why Dual-Layer Architecture (Mission Control Web Dashboard + Python Desktop)?
1. **Modern Mission Control Experience**:
   - Web frontend provides responsive glassmorphic UI, real-time KPI status badges, interactive 100-frame error plots, and parameter sliders.
2. **Rigorous Offline Evaluation**:
   - Python backend handles native headless benchmarking, batch testing, CSV/JSON exporting, and standalone packaging via PyInstaller.
3. **WebSocket Bridge**:
   - Bi-directional bridge allows the browser frontend to directly monitor or drive the real-time Python simulation engine via lightweight JSON telemetry packets.

---

## 5. Verification Against All 5 Mandatory ISRO Targets

| Performance Metric | Required Target | Our Achieved Value | Margin of Safety | Status |
|---|---|---|---|---|
| **Acquisition Time** | $\le 2.0\text{ s}$ | **$0.10\text{ s}$** | $+1.90\text{ s}$ ($95\%$ headroom) | **PASS** |
| **Tracking Error** | $\le 10.0\text{ px}$ | **$1.87\text{ px}$ avg** ($2.18\text{ px}$ RMSE) | $+8.13\text{ px}$ ($81\%$ margin) | **PASS** |
| **Target Loss Rate** | $< 5.0\%$ | **$0.00\%$** | $+5.00\%$ ($100\%$ lock retention) | **PASS** |
| **Re-acquisition Time** | $\le 1.0\text{ s}$ | **$0.00\text{–}0.10\text{ s}$** | $+0.90\text{ s}$ ($90\%$ headroom) | **PASS** |
| **Processing Speed** | $\ge 20.0\text{ FPS}$ | **$30.0\text{ FPS}$** (Packaged) / **$848.0\text{ FPS}$** (Audit) | $+10.0\text{–}828.0\text{ FPS}$ | **PASS** |
| **Benchmark 1 (4 Scenarios)** | All Pass | **4 / 4 PASSED** (Straight, Cir, Fig8, Rand) | Avg error $< 1.95\text{ px}$ | **PASS** |
| **Benchmark 2 (Video .mp4)** | RMSE $\le 10.0\text{ px}$ | **$2.49\text{ px}$ RMSE** at $349.1\text{ FPS}$ | $+7.51\text{ px}$ margin | **PASS** |
| **Full Requirements Audit** | 44 / 44 | **44 / 44 Verified** | $0$ gaps remaining | **PASS** |

---

## 6. Viva / Panel Defense Q&A (Top 15 Anticipated Questions)

### Q1: What is the primary purpose of this project?
> **Answer**: To develop a high-fidelity software testbed simulating coarse Pointing, Acquisition, and Tracking (PAT) for Free Space Optical Communication (FSOC). The software simulates a mobile terminal beacon across a $2000\times 2000$ canvas and steers a $640\times 480$ virtual camera to keep the beacon within $\le 10\text{ pixels}$ of the optical center under severe platform jitter, atmospheric disturbances, and sensor noise.

### Q2: Why did you not use deep learning (YOLO or CNNs) for beacon detection?
> **Answer**: Three reasons: 
> 1. Laser beacons on an 8-bit monochrome FPA are high-intensity optical spots, not complex semantic objects.
> 2. Spatial moments ($M_{10}/M_{00}$) compute in $< 0.5\text{ ms}$ on CPU, whereas YOLO requires $30\text{–}80\text{ ms}$, threatening the mandatory $\ge 20\text{ FPS}$ constraint.
> 3. Spatial moments inherently provide sub-pixel floating-point centroids with zero GPU dependency, enabling direct deployment on embedded spaceflight DSPs.

### Q3: How does your system achieve tracking errors of under 2 pixels when the beacon moves rapidly?
> **Answer**: By implementing **World-Space Target Velocity Feedforward**. In a standard PID controller, steady-state error is proportional to speed ($e_{ss} = V / K_p$). By calculating the target's physical velocity in global coordinates and feeding it directly to the servo drive, the feedforward term provides the exact velocity needed to match target translation. The PID feedback loop ($K_p=18.0, K_i=0.5, K_d=1.2$) only needs to correct for sensor noise and jitter, dropping average tracking error from $9.8\text{ px}$ to $1.86\text{ px}$.

### Q4: How do you handle target loss during heavy fog, rain, or sensor noise?
> **Answer**: We use a **4D Linear Kalman Filter with an occlusion-hold predictor**. When the centroid detector reports `None` due to cloud/fog attenuation, the Kalman filter holds its measurement update and propagates the state transition forward using the target's estimated velocity ($\hat{\mathbf{x}}_k = F \hat{\mathbf{x}}_{k-1}$). This allows the camera to track through $10+$ frames of complete obstruction.

### Q5: What is the role of the 3-state Finite State Machine?
> **Answer**: The FSM enforces discrete operational transitions: `LOST`, `RE-ACQUIRING`, and `ACQUIRED`. It uses a 3-frame hysteresis threshold to prevent spurious state toggling during transient noise, and precisely logs Acquisition Time ($\le 2\text{ s}$) and Re-acquisition Time ($\le 1\text{ s}$) per ISRO specifications.

### Q6: How did you optimize processing speed to exceed 20 FPS?
> **Answer**: We identified and eliminated two major bottlenecks:
> 1. Replaced scalar NumPy noise loops with vectorized OpenCV C++ routines (`cv2.randn`, `cv2.randu`) and variance-scaled photon shot noise, accelerating Python noise generation from $108\text{ ms}$ ($8.5\text{ FPS}$) to $2.0\text{ ms}$ ($61.7\text{ FPS}$).
> 2. Precomputed a 16,384-sample Float32Array Gaussian LUT in the browser dashboard to eliminate Box-Muller transcendental function overhead, reducing JS frame time from $87\text{ ms}$ to $10.5\text{ ms}$ ($95.2\text{ FPS}$).

### Q7: What is the difference between Benchmark 1 and Benchmark 2?
> **Answer**: 
> - **Benchmark 1 (Closed-Loop PTZ)**: Evaluates the interactive PTZ camera tracking all 4 mandatory motion patterns (Straight, Circular, Figure-8, Random) on the $2000\times 2000$ canvas with active disturbances.
> - **Benchmark 2 (External Video Pipeline)**: Ingests external, pre-recorded `.mp4` video files where the camera is fixed, evaluating detector and Kalman tracker RMSE independently of the PTZ gimbal.

### Q8: What does the pixel-to-degree ratio represent?
> **Answer**: The optical mapping between sensor pixels and angular mechanical motion. For a $640\times 480$ sensor with a $4^\circ \times 3^\circ$ FOV:
> $$\text{Ratio} = \frac{640\text{ px}}{4^\circ} = \frac{480\text{ px}}{3^\circ} = 160.0\text{ px/deg}$$
> This allows converting angular speed limits ($5\text{–}10^\circ\text{/s}$) into pixel displacement limits ($800\text{–}1600\text{ px/s}$).

### Q9: How do you prevent PID integrator windup during fast target maneuvers?
> **Answer**: We clamp the integral accumulator to $[-60, +60]\text{ pixels}$ in Python and $[-60, +60]$ in JavaScript. This prevents saturation when the target accelerates or bounces off arena boundaries, eliminating overshoot.

### Q10: How is atmospheric disturbance modeled?
> **Answer**: We implement five distinct optical transmission regimes: Clear ($100\%$ transmission), Haze ($30\%$ contrast reduction via background blending), Fog ($40\%$ additive scattering veil), Rain (dynamic streak particle system), and Low Light ($50\%$ irradiance drop into the sensor dark current threshold).

### Q11: How do you evaluate the random motion pattern without getting trapped in canvas corners?
> **Answer**: We established camera steerable reflection boundaries at $X \in [340, 1660]$ and $Y \in [260, 1740]$. When the beacon approaches these limits, its velocity vector reverses elastically, ensuring the beacon always remains within the steerable field of view of the pan-tilt camera.

### Q12: Why is the standalone executable (.exe) required?
> **Answer**: ISRO Requirement 34 requires a standalone deployable executable that runs without requiring a Python development environment or manual dependency installation. We configured PyInstaller with automated post-build verification to package the runtime into `dist/ISRO_FSOC_Tracker/ISRO_FSOC_Tracker.exe`.

### Q13: What does the Claude AI Advisor do?
> **Answer**: It is an intelligence backend that inspects exported CSV/JSON telemetry summaries, analyzes PID damping ratios and Kalman innovation residuals, generates engineering tuning suggestions, and auto-generates technical performance narratives for mission logs.

### Q14: How is Root Mean Square Error (RMSE) computed?
> **Answer**: 
> $$\text{RMSE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \left( (x_i - x_{\text{center}})^2 + (y_i - y_{\text{center}})^2 \right)}$$
> It represents the cumulative quadratic tracking penalty, penalizing large transient displacements more heavily than mean absolute error.

### Q15: If deployed on real ISRO hardware tomorrow, what changes?
> **Answer**: The software architecture remains identical:
> 1. The virtual camera frame capture is replaced by the camera's GenICam/USB3/GigE SDK buffer.
> 2. The `dx, dy` PID outputs are sent over RS-422/CAN bus to physical pan/tilt stepper/brushless gimbal motor drivers.
> 3. The detector, Kalman filter, and PID controller algorithms run directly on the onboard payload computer.

---

## 7. Recommended Presentation Slide Outline (10-Minute Talk)

- **Slide 1: Title & Overview**
  - Project Title: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals.
  - Problem Statement ID: 26169 | Smart India Hackathon 2026.
  - Team Name & Roles.
- **Slide 2: The Problem — Why FSOC Needs Coarse Tracking**
  - Laser divergence is microradians wide.
  - Mobile platforms (satellites, UAVs, ships) vibrate and drift.
  - Coarse tracking ($\le 10\text{ px}$) is the vital bridge between initial acquisition and fine fiber coupling.
- **Slide 3: System Architecture & End-to-End Pipeline**
  - High-level block diagram: $2000\times 2000$ Arena $\rightarrow$ FPA Viewport $\rightarrow$ Noise $\rightarrow$ Moments Centroid $\rightarrow$ 4D Kalman $\rightarrow$ Feedforward PID $\rightarrow$ PTZ Steer.
- **Slide 4: Computer Vision & State Estimation**
  - Spatial Moments ($M_{10}/M_{00}$) for sub-millisecond, sub-pixel centroiding.
  - 4D Kalman Filter for jitter rejection and $10+$ frame occlusion prediction.
- **Slide 5: Control Engineering Breakthrough — World Velocity Feedforward**
  - The problem: Steady-state lag $e_{ss} = V / K_p$.
  - The breakthrough: Feedforward target velocity in world space.
  - Results: Error dropped from $9.8\text{ px}$ to $1.86\text{ px}$ ($5\times$ improvement).
- **Slide 6: Environmental Disturbances & Real-Time Performance**
  - 5 Atmospheric modes (Clear, Haze, Fog, Rain, Low Light) + Jitter + Noise.
  - OpenCV C++ vectorization & Float32Array LUT: $8.5\text{ FPS} \rightarrow 61.7\text{ FPS}$ (Python) and $95.2\text{ FPS}$ (Browser).
- **Slide 7: Verification Matrix — 100% Target Pass**
  - Table of all 5 mandatory targets showing huge safety margins.
  - Benchmark 1: 4/4 scenarios passed.
  - Benchmark 2: $2.49\text{ px}$ RMSE on test video.
- **Slide 8: Mission Control Dashboard & Live Demo**
  - Showcase Web GUI, telemetry strip, real-time error plot, and standalone `.exe`.
- **Slide 9: Deliverables & Conclusion**
  - 44/44 requirements validated.
  - Modular, fully commented codebase + 15-page Technical Report + User Manual + Standalone Executable.
  - Future roadmap: Integration with hardware gimbal and FSM.
- **Slide 10: Q&A**
  - Open for questions from the jury.
