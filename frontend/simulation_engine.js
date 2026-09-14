/**
 * ISRO SIH 2026 - Problem Statement 26169
 * AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
 * Simulation, Detection, Tracking, and Control Engine (Pure JS / Canvas 2D)
 * 
 * Modules included:
 * - Virtual Canvas Scene (2000x2000 px configurable)
 * - Beacon Generator (Square, Circle, Cross; 5-20px; 4 mandatory + 2 optional motion patterns)
 * - Atmospheric Disturbance Engine (Clear, Haze, Fog, Rain, Low Light)
 * - Noise & Jitter Engine (Salt & Pepper, Gaussian, Poisson, Std Dev slider, Camera Jitter, Linear Platform Motion)
 * - Monochrome Focal Plane Array (FPA) Camera Viewport (640x480 px, 4°x3° FOV, Pixel-to-Degree ratio)
 * - Centroid Detector (Grayscale, Gaussian Blur, Binary Threshold, Centroid Calc)
 * - 4D Kalman Filter Tracker (State: [x, y, vx, vy], Occlusion Prediction, Noise Covariances)
 * - 3-State Tracker State Machine (Acquired, Re-acquiring, Lost)
 * - 2-Axis Pan/Tilt PID Controller (Clamped to 5-10 deg/s converted to px/frame)
 * - Benchmark 2 Video File Pipeline (Bypasses PTZ, frame-by-frame processing, RMSE & metrics)
 */

class KalmanFilter2D {
    constructor() {
        this.dt = 1.0 / 30.0;
        this.reset();
    }

    reset(initialX = 320, initialY = 240) {
        // State vector [x, y, vx, vy]
        this.x = [initialX, initialY, 0, 0];
        // State covariance matrix P (4x4)
        this.P = [
            [10, 0, 0, 0],
            [0, 10, 0, 0],
            [0, 0, 50, 0],
            [0, 0, 0, 50]
        ];
        // Process noise covariance Q (4x4)
        const q_pos = 0.5;
        const q_vel = 2.0;
        this.Q = [
            [q_pos, 0, 0, 0],
            [0, q_pos, 0, 0],
            [0, 0, q_vel, 0],
            [0, 0, 0, q_vel]
        ];
        // Measurement noise covariance R (2x2)
        this.R = [
            [4.0, 0],
            [0, 4.0]
        ];
        this.consecutiveLostFrames = 0;
        this.history = [];
    }

    setNoiseMatrices(processNoise, measurementNoise) {
        this.Q[0][0] = processNoise * 0.25;
        this.Q[1][1] = processNoise * 0.25;
        this.Q[2][2] = processNoise;
        this.Q[3][3] = processNoise;
        this.R[0][0] = measurementNoise;
        this.R[1][1] = measurementNoise;
    }

    predict(dt = 1.0 / 30.0) {
        this.dt = dt;
        // State Transition Matrix F:
        // [1, 0, dt, 0]
        // [0, 1, 0, dt]
        // [0, 0, 1,  0]
        // [0, 0, 0,  1]
        const new_x = this.x[0] + this.x[2] * this.dt;
        const new_y = this.x[1] + this.x[3] * this.dt;
        const new_vx = this.x[2];
        const new_vy = this.x[3];

        this.x = [new_x, new_y, new_vx, new_vy];

        // P = F * P * F^T + Q
        // F * P:
        const FP = [
            [this.P[0][0] + this.dt * this.P[2][0], this.P[0][1] + this.dt * this.P[2][1], this.P[0][2] + this.dt * this.P[2][2], this.P[0][3] + this.dt * this.P[2][3]],
            [this.P[1][0] + this.dt * this.P[3][0], this.P[1][1] + this.dt * this.P[3][1], this.P[1][2] + this.dt * this.P[3][2], this.P[1][3] + this.dt * this.P[3][3]],
            [this.P[2][0], this.P[2][1], this.P[2][2], this.P[2][3]],
            [this.P[3][0], this.P[3][1], this.P[3][2], this.P[3][3]]
        ];

        // FP * F^T:
        this.P = [
            [FP[0][0] + FP[0][2] * this.dt + this.Q[0][0], FP[0][1] + FP[0][3] * this.dt, FP[0][2], FP[0][3]],
            [FP[1][0] + FP[1][2] * this.dt, FP[1][1] + FP[1][3] * this.dt + this.Q[1][1], FP[1][2], FP[1][3]],
            [FP[2][0] + FP[2][2] * this.dt, FP[2][1] + FP[2][3] * this.dt, FP[2][2] + this.Q[2][2], FP[2][3]],
            [FP[3][0] + FP[3][2] * this.dt, FP[3][1] + FP[3][3] * this.dt, FP[3][2], FP[3][3] + this.Q[3][3]]
        ];

        return { x: this.x[0], y: this.x[1], vx: this.x[2], vy: this.x[3] };
    }

    update(measuredX, measuredY) {
        if (measuredX === null || measuredY === null || isNaN(measuredX) || isNaN(measuredY)) {
            this.consecutiveLostFrames++;
            return { x: this.x[0], y: this.x[1], isPredicted: true };
        }

        this.consecutiveLostFrames = 0;

        // Measurement z = [measuredX, measuredY]
        // Innovation y = z - H * x
        const y = [
            measuredX - this.x[0],
            measuredY - this.x[1]
        ];

        // Innovation covariance S = H * P * H^T + R
        const S00 = this.P[0][0] + this.R[0][0];
        const S01 = this.P[0][1] + this.R[0][1];
        const S10 = this.P[1][0] + this.R[1][0];
        const S11 = this.P[1][1] + this.R[1][1];

        // Invert 2x2 matrix S
        const det = S00 * S11 - S01 * S10;
        const invDet = det === 0 ? 0.0001 : 1.0 / det;
        const invS00 = S11 * invDet;
        const invS01 = -S01 * invDet;
        const invS10 = -S10 * invDet;
        const invS11 = S00 * invDet;

        // Kalman Gain K = P * H^T * inv(S) (4x2)
        // P * H^T is the first two columns of P
        const PHt = [
            [this.P[0][0], this.P[0][1]],
            [this.P[1][0], this.P[1][1]],
            [this.P[2][0], this.P[2][1]],
            [this.P[3][0], this.P[3][1]]
        ];

        const K = [
            [PHt[0][0] * invS00 + PHt[0][1] * invS10, PHt[0][0] * invS01 + PHt[0][1] * invS11],
            [PHt[1][0] * invS00 + PHt[1][1] * invS10, PHt[1][0] * invS01 + PHt[1][1] * invS11],
            [PHt[2][0] * invS00 + PHt[2][1] * invS10, PHt[2][0] * invS01 + PHt[2][1] * invS11],
            [PHt[3][0] * invS00 + PHt[3][1] * invS10, PHt[3][0] * invS01 + PHt[3][0] * invS11]
        ];

        // x = x + K * y
        this.x[0] += K[0][0] * y[0] + K[0][1] * y[1];
        this.x[1] += K[1][0] * y[0] + K[1][1] * y[1];
        this.x[2] += K[2][0] * y[0] + K[2][1] * y[1];
        this.x[3] += K[3][0] * y[0] + K[3][1] * y[1];

        // Covariance update: P = (I - K * H) * P
        const I_KH = [
            [1 - K[0][0], -K[0][1], 0, 0],
            [-K[1][0], 1 - K[1][1], 0, 0],
            [-K[2][0], -K[2][1], 1, 0],
            [-K[3][0], -K[3][1], 0, 1]
        ];

        const newP = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                for (let k = 0; k < 4; k++) {
                    newP[i][j] += I_KH[i][k] * this.P[k][j];
                }
            }
        }
        this.P = newP;

        return { x: this.x[0], y: this.x[1], vx: this.x[2], vy: this.x[3], isPredicted: false };
    }
}

class PIDController2D {
    constructor() {
        this.Kp = 18.0;
        this.Ki = 0.5;
        this.Kd = 1.2;
        this.integralX = 0;
        this.integralY = 0;
        this.prevErrorX = 0;
        this.prevErrorY = 0;
    }

    reset() {
        this.integralX = 0;
        this.integralY = 0;
        this.prevErrorX = 0;
        this.prevErrorY = 0;
    }

    setGains(kp, ki, kd) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    compute(errorX, errorY, dt, maxPanSpeedPx, maxTiltSpeedPx, feedforwardVx = 0.0, feedforwardVy = 0.0) {
        if (dt <= 0) dt = 1.0 / 30.0;

        // Anti-windup clamped integration
        this.integralX = Math.max(-60, Math.min(60, this.integralX + errorX * dt));
        this.integralY = Math.max(-60, Math.min(60, this.integralY + errorY * dt));

        const derivativeX = (errorX - this.prevErrorX) / dt;
        const derivativeY = (errorY - this.prevErrorY) / dt;

        let outputX = feedforwardVx + this.Kp * errorX + this.Ki * this.integralX + this.Kd * derivativeX;
        let outputY = feedforwardVy + this.Kp * errorY + this.Ki * this.integralY + this.Kd * derivativeY;

        // Clamp to configured max speeds (px/sec or px/frame)
        outputX = Math.max(-maxPanSpeedPx, Math.min(maxPanSpeedPx, outputX));
        outputY = Math.max(-maxTiltSpeedPx, Math.min(maxTiltSpeedPx, outputY));

        this.prevErrorX = errorX;
        this.prevErrorY = errorY;

        return { vx: outputX, vy: outputY };
    }
}

class FSOCTrackingSimulator {
    constructor() {
        // Environment specs
        this.canvasSize = { width: 2000, height: 2000 };
        this.viewportSize = { width: 640, height: 480 };
        this.fovDegrees = { width: 4.0, height: 3.0 }; // Default 4x3 deg
        this.updatePixelToDegreeRatio();

        // Speed constraints (5 to 10 deg/sec, default 5 deg/s)
        this.maxPanSpeedDeg = 5.0;
        this.maxTiltSpeedDeg = 5.0;

        // Virtual Camera PTZ position (starts at center of screen)
        this.cameraPos = {
            x: this.canvasSize.width / 2,
            y: this.canvasSize.height / 2
        };

        // Target / Beacon parameters
        this.targetShape = 'square'; // square, circle, cross
        this.targetSize = 10; // 5 to 20 px, default 10
        this.targetPos = { x: 1000, y: 1000 };
        this.targetVelocity = { vx: 40, vy: 30 }; // pixels per second
        this.motionPattern = 'figure8'; // straight, circular, figure8, random, spiral, sinusoidal
        this.motionTime = 0;

        // Disturbances & Noise
        this.noiseTypes = {
            saltPepper: false,
            gaussian: false,
            poisson: false
        };
        this.noiseStdDev = 5.0; // 0 to 20 px slider labelled "Standard Deviation in pixels"
        this.cameraJitterMax = 5.0; // 0 to 20 px
        this.atmosphericMode = 'clear'; // clear, haze, fog, rain, lowlight
        this.platformMotionMode = 'linear'; // linear (mandatory ±20px), circular, random, none
        this.platformDisplacement = { x: 0, y: 0 };
        this.platformPhase = 0;

        // Controllers & Trackers
        this.kalman = new KalmanFilter2D();
        this.pid = new PIDController2D();
        this.prevTargetWorldX = this.cameraPos.x;
        this.prevTargetWorldY = this.cameraPos.y;
        this.targetWorldVx = 0.0;
        this.targetWorldVy = 0.0;

        // State Machine: 'acquired', 're-acquiring', 'lost'
        this.trackerState = 'lost';
        this.consecutiveDetections = 0;
        this.consecutiveLosses = 0;

        // Evaluation & Telemetry Metrics
        this.running = true;
        this.sessionStartTime = performance.now();
        this.frameCount = 0;
        this.currentFPS = 30.0;
        this.lastFrameTime = performance.now();
        this.fpsSmoothing = [];

        this.centroidError = 0.0;
        this.errorHistory = []; // Last 100 frames for real-time error plot
        this.squaredErrorsSum = 0.0;
        this.totalFramesLogged = 0;
        this.totalLostFrames = 0;
        this.maxTrackingError = 0.0;
        this.errorSum = 0.0;

        // Timing metrics
        this.acquisitionStartTime = performance.now();
        this.acquisitionTimeSec = 0.0;
        this.reAcquisitionStartTime = null;
        this.reAcquisitionTimeSec = 0.0;
        this.acquisitionCompleted = false;

        // Fast precomputed Gaussian random distribution LUT (16384 samples) for >= 60 FPS performance
        this.gaussianLUT = new Float32Array(16384);
        for (let k = 0; k < 16384; k++) {
            const u1 = Math.max(0.0001, Math.random());
            const u2 = Math.random();
            this.gaussianLUT[k] = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        }
        this.lutIndex = 0;
        this.lastFrameImageData = null;

        // Offscreen buffers for rendering
        this.offscreenCanvas = document.createElement('canvas');
        this.offscreenCanvas.width = this.viewportSize.width;
        this.offscreenCanvas.height = this.viewportSize.height;
        this.offscreenCtx = this.offscreenCanvas.getContext('2d', { willReadFrequently: true });

        // Rain particle system
        this.rainParticles = [];
        for (let i = 0; i < 70; i++) {
            this.rainParticles.push({
                x: Math.random() * this.viewportSize.width,
                y: Math.random() * this.viewportSize.height,
                len: 10 + Math.random() * 15,
                speed: 15 + Math.random() * 10
            });
        }

        // Full session log history for CSV/JSON export
        this.sessionLog = [];
    }

    updatePixelToDegreeRatio() {
        // px/deg = viewport width / FOV width in degrees
        this.pixelsPerDegreeX = this.viewportSize.width / this.fovDegrees.width; // 640 / 4 = 160 px/deg
        this.pixelsPerDegreeY = this.viewportSize.height / this.fovDegrees.height; // 480 / 3 = 160 px/deg
    }

    resetSession() {
        this.sessionStartTime = performance.now();
        this.frameCount = 0;
        this.errorHistory = [];
        this.squaredErrorsSum = 0.0;
        this.totalFramesLogged = 0;
        this.totalLostFrames = 0;
        this.maxTrackingError = 0.0;
        this.errorSum = 0.0;
        this.acquisitionStartTime = performance.now();
        this.acquisitionTimeSec = 0.0;
        this.reAcquisitionStartTime = null;
        this.reAcquisitionTimeSec = 0.0;
        this.acquisitionCompleted = false;
        this.sessionLog = [];

        this.cameraPos = {
            x: this.canvasSize.width / 2,
            y: this.canvasSize.height / 2
        };
        this.kalman.reset(this.viewportSize.width / 2, this.viewportSize.height / 2);
        this.pid.reset();
        this.prevTargetWorldX = this.cameraPos.x;
        this.prevTargetWorldY = this.cameraPos.y;
        this.targetWorldVx = 0.0;
        this.targetWorldVy = 0.0;
        this.trackerState = 'lost';
        this.consecutiveDetections = 0;
        this.consecutiveLosses = 0;
    }

    setMotionPattern(newPattern) {
        this.motionPattern = newPattern;
        this.motionTime = 0;

        const cx = this.canvasSize.width / 2;
        const cy = this.canvasSize.height / 2;

        // 1. Calculate the initial target position for this pattern at t=0
        if (newPattern === 'circular') {
            this.targetPos.x = cx + 450;
            this.targetPos.y = cy;
        } else if (newPattern === 'figure8') {
            this.targetPos.x = cx;
            this.targetPos.y = cy;
        } else if (newPattern === 'straight') {
            this.targetPos.x = cx - 200;
            this.targetPos.y = cy - 150;
            this.targetVelocity.vx = 40;
            this.targetVelocity.vy = 30;
        } else if (newPattern === 'random') {
            this.targetPos.x = cx;
            this.targetPos.y = cy;
            this.targetVelocity.vx = 35;
            this.targetVelocity.vy = 25;
        } else if (newPattern === 'spiral') {
            this.targetPos.x = cx + 50;
            this.targetPos.y = cy;
        } else if (newPattern === 'sinusoidal') {
            this.targetPos.x = cx;
            this.targetPos.y = cy;
        }

        // 2. Re-align camera over the new target coordinate so it stays within FOV
        const halfW = this.viewportSize.width / 2;
        const halfH = this.viewportSize.height / 2;
        this.cameraPos.x = Math.max(halfW, Math.min(this.canvasSize.width - halfW, this.targetPos.x));
        this.cameraPos.y = Math.max(halfH, Math.min(this.canvasSize.height - halfH, this.targetPos.y));

        // 3. Reset PID controller states (derivative spike & integral windup prevention)
        this.pid.reset();
        this.targetWorldVx = 0.0;
        this.targetWorldVy = 0.0;
        this.prevTargetWorldX = this.cameraPos.x;
        this.prevTargetWorldY = this.cameraPos.y;

        // 4. Reset Kalman filter to center of sensor viewport
        this.kalman.reset(halfW, halfH);

        // 5. Re-arm acquisition timers & state
        this.trackerState = 're-acquiring';
        this.consecutiveDetections = 0;
        this.consecutiveLosses = 0;
        this.acquisitionStartTime = performance.now();
        this.acquisitionCompleted = false;
        this.reAcquisitionStartTime = null;
    }

    setInitialTargetPosition(x, y) {
        this.targetPos.x = Math.max(50, Math.min(this.canvasSize.width - 50, x));
        this.targetPos.y = Math.max(50, Math.min(this.canvasSize.height - 50, y));

        // Re-align camera and controllers to prevent corner pinning
        const halfW = this.viewportSize.width / 2;
        const halfH = this.viewportSize.height / 2;
        this.cameraPos.x = Math.max(halfW, Math.min(this.canvasSize.width - halfW, this.targetPos.x));
        this.cameraPos.y = Math.max(halfH, Math.min(this.canvasSize.height - halfH, this.targetPos.y));
        this.pid.reset();
        this.targetWorldVx = 0.0;
        this.targetWorldVy = 0.0;
        this.prevTargetWorldX = this.cameraPos.x;
        this.prevTargetWorldY = this.cameraPos.y;
        this.kalman.reset(halfW, halfH);
    }

    randomizeTargetPosition() {
        const marginX = 350;
        const marginY = 300;
        const x = marginX + Math.random() * (this.canvasSize.width - 2 * marginX);
        const y = marginY + Math.random() * (this.canvasSize.height - 2 * marginY);
        this.setInitialTargetPosition(x, y);
    }

    updateMotion(dt) {
        this.motionTime += dt;
        const cx = this.canvasSize.width / 2;
        const cy = this.canvasSize.height / 2;
        const padX = this.viewportSize.width / 2 + 20; // 340px (camera fully steerable)
        const padY = this.viewportSize.height / 2 + 20; // 260px (camera fully steerable)

        switch (this.motionPattern) {
            case 'straight': {
                // Linear motion with wall bouncing within camera steerable envelope
                this.targetPos.x += this.targetVelocity.vx * dt;
                this.targetPos.y += this.targetVelocity.vy * dt;

                if (this.targetPos.x < padX || this.targetPos.x > this.canvasSize.width - padX) {
                    this.targetVelocity.vx *= -1;
                    this.targetPos.x = Math.max(padX, Math.min(this.canvasSize.width - padX, this.targetPos.x));
                }
                if (this.targetPos.y < padY || this.targetPos.y > this.canvasSize.height - padY) {
                    this.targetVelocity.vy *= -1;
                    this.targetPos.y = Math.max(padY, Math.min(this.canvasSize.height - padY, this.targetPos.y));
                }
                break;
            }
            case 'circular': {
                const radius = 450;
                const omega = 0.55; // rad/sec
                this.targetPos.x = cx + radius * Math.cos(omega * this.motionTime);
                this.targetPos.y = cy + radius * Math.sin(omega * this.motionTime);
                break;
            }
            case 'figure8': {
                // Lemniscate of Gerono: x = a cos(t), y = a sin(t) cos(t)
                const a = 500;
                const t = 0.45 * this.motionTime;
                this.targetPos.x = cx + a * Math.sin(t);
                this.targetPos.y = cy + (a * 0.6) * Math.sin(2 * t) / 2;
                break;
            }
            case 'random': {
                // Smooth Brownian random walk
                if (Math.random() < 0.08) {
                    this.targetVelocity.vx += (Math.random() - 0.5) * 60;
                    this.targetVelocity.vy += (Math.random() - 0.5) * 60;
                    const maxV = 90;
                    this.targetVelocity.vx = Math.max(-maxV, Math.min(maxV, this.targetVelocity.vx));
                    this.targetVelocity.vy = Math.max(-maxV, Math.min(maxV, this.targetVelocity.vy));
                }
                this.targetPos.x += this.targetVelocity.vx * dt;
                this.targetPos.y += this.targetVelocity.vy * dt;

                if (this.targetPos.x < padX) { this.targetPos.x = padX; this.targetVelocity.vx = Math.abs(this.targetVelocity.vx); }
                if (this.targetPos.x > this.canvasSize.width - padX) { this.targetPos.x = this.canvasSize.width - padX; this.targetVelocity.vx = -Math.abs(this.targetVelocity.vx); }
                if (this.targetPos.y < padY) { this.targetPos.y = padY; this.targetVelocity.vy = Math.abs(this.targetVelocity.vy); }
                if (this.targetPos.y > this.canvasSize.height - padY) { this.targetPos.y = this.canvasSize.height - padY; this.targetVelocity.vy = -Math.abs(this.targetVelocity.vy); }
                break;
            }
            case 'spiral': {
                const maxR = 500;
                const r = (50 + (this.motionTime * 25) % maxR);
                const theta = this.motionTime * 0.8;
                this.targetPos.x = cx + r * Math.cos(theta);
                this.targetPos.y = cy + r * Math.sin(theta);
                break;
            }
            case 'sinusoidal': {
                this.targetPos.x = cx + 550 * Math.sin(this.motionTime * 0.3);
                this.targetPos.y = cy + 250 * Math.sin(this.motionTime * 1.5);
                break;
            }
        }

        // Platform Motion (Linear ±20px/frame maximum, or circular/random)
        if (this.platformMotionMode === 'linear') {
            this.platformPhase += dt * 2.0;
            this.platformDisplacement.x = Math.sin(this.platformPhase) * 18.0;
            this.platformDisplacement.y = Math.cos(this.platformPhase) * 12.0;
        } else if (this.platformMotionMode === 'circular') {
            this.platformPhase += dt * 3.0;
            this.platformDisplacement.x = Math.cos(this.platformPhase) * 20.0;
            this.platformDisplacement.y = Math.sin(this.platformPhase) * 20.0;
        } else if (this.platformMotionMode === 'random') {
            this.platformDisplacement.x = (Math.random() - 0.5) * 40.0;
            this.platformDisplacement.y = (Math.random() - 0.5) * 40.0;
        } else {
            this.platformDisplacement.x = 0;
            this.platformDisplacement.y = 0;
        }
    }

    renderMonochromeFPAScene() {
        const ctx = this.offscreenCtx;
        const W = this.viewportSize.width;
        const H = this.viewportSize.height;

        // Clear to dark background (Monochrome FPA sensor dark current / baseline)
        let baselineLuminance = 12; // deep space dark
        if (this.atmosphericMode === 'lowlight') {
            baselineLuminance = 6;
        }
        ctx.fillStyle = `rgb(${baselineLuminance}, ${baselineLuminance}, ${baselineLuminance})`;
        ctx.fillRect(0, 0, W, H);

        // Calculate beacon position relative to camera viewport
        // Camera Jitter ±20px per frame
        let jitterX = 0;
        let jitterY = 0;
        if (this.cameraJitterMax > 0) {
            jitterX = (Math.random() * 2 - 1) * this.cameraJitterMax;
            jitterY = (Math.random() * 2 - 1) * this.cameraJitterMax;
        }

        // Effective camera center in global space
        const effectiveCamX = this.cameraPos.x + jitterX;
        const effectiveCamY = this.cameraPos.y + jitterY;

        // Effective beacon position in global space (with platform motion)
        const effectiveTargetX = this.targetPos.x + this.platformDisplacement.x;
        const effectiveTargetY = this.targetPos.y + this.platformDisplacement.y;

        // Viewport relative coordinates (0,0 is top-left of viewport)
        const relX = effectiveTargetX - (effectiveCamX - W / 2);
        const relY = effectiveTargetY - (effectiveCamY - H / 2);

        this.groundTruthViewportPos = { x: relX, y: relY };

        // Draw Beacon Spot if inside or near viewport
        let beaconIntensity = 255;
        if (this.atmosphericMode === 'haze') {
            beaconIntensity = Math.floor(beaconIntensity * 0.70); // 30% contrast reduction
        } else if (this.atmosphericMode === 'lowlight') {
            beaconIntensity = Math.floor(beaconIntensity * 0.50); // 50% brightness reduction
        }

        ctx.fillStyle = `rgb(${beaconIntensity}, ${beaconIntensity}, ${beaconIntensity})`;
        ctx.strokeStyle = `rgb(${beaconIntensity}, ${beaconIntensity}, ${beaconIntensity})`;

        const s = this.targetSize;
        const hs = s / 2;

        if (this.targetShape === 'square') {
            ctx.fillRect(relX - hs, relY - hs, s, s);
        } else if (this.targetShape === 'circle') {
            ctx.beginPath();
            ctx.arc(relX, relY, hs, 0, Math.PI * 2);
            ctx.fill();
        } else if (this.targetShape === 'cross') {
            ctx.lineWidth = Math.max(2, Math.floor(s / 4));
            ctx.beginPath();
            ctx.moveTo(relX - hs, relY);
            ctx.lineTo(relX + hs, relY);
            ctx.moveTo(relX, relY - hs);
            ctx.lineTo(relX, relY + hs);
            ctx.stroke();
        }

        // Apply Atmospheric Disturbance Overlays
        if (this.atmosphericMode === 'fog') {
            // White overlay at 40% opacity
            ctx.fillStyle = 'rgba(255, 255, 255, 0.40)';
            ctx.fillRect(0, 0, W, H);
        } else if (this.atmosphericMode === 'rain') {
            // Vertical rain streaks
            ctx.strokeStyle = 'rgba(220, 220, 220, 0.55)';
            ctx.lineWidth = 1.2;
            ctx.beginPath();
            for (let p of this.rainParticles) {
                p.y += p.speed;
                if (p.y > H) {
                    p.y = -p.len;
                    p.x = Math.random() * W;
                }
                ctx.moveTo(p.x, p.y);
                ctx.lineTo(p.x, p.y + p.len);
            }
            ctx.stroke();
        }

        // Apply Image Noise directly to pixels (Monochrome FPA simulation at >= 60 FPS)
        const activeGauss = (this.noiseTypes.gaussian && this.noiseStdDev > 0);
        const hasNoise = this.noiseTypes.saltPepper || activeGauss || this.noiseTypes.poisson;
        if (hasNoise) {
            const imgData = ctx.getImageData(0, 0, W, H);
            const d = imgData.data;
            const len = d.length;

            const stdDev = this.noiseStdDev;
            const spActive = this.noiseTypes.saltPepper;
            const gActive = activeGauss;
            const pActive = this.noiseTypes.poisson;
            const lut = this.gaussianLUT;
            let lutIdx = this.lutIndex;

            for (let i = 0; i < len; i += 4) {
                let val = d[i]; // single channel grayscale value

                // Ultra-fast Gaussian noise via precomputed LUT
                if (gActive) {
                    val += lut[lutIdx++ & 16383] * stdDev;
                }

                // Poisson photon shot noise (photon variance = intensity)
                if (pActive) {
                    val += Math.sqrt(Math.max(1.0, val)) * lut[lutIdx++ & 16383];
                }

                // Salt and Pepper at 10% (5% pepper, 5% salt)
                if (spActive) {
                    const rnd = Math.random();
                    if (rnd < 0.05) {
                        val = 0; // pepper
                    } else if (rnd < 0.10) {
                        val = 255; // salt
                    }
                }

                // Clamp to [0, 255]
                val = val < 0 ? 0 : (val > 255 ? 255 : val);
                d[i] = val;
                d[i + 1] = val;
                d[i + 2] = val;
            }
            this.lutIndex = lutIdx & 16383;
            ctx.putImageData(imgData, 0, 0);
            this.lastFrameImageData = imgData;
        } else {
            this.lastFrameImageData = null;
        }
    }

    detectBeaconCentroid(predX = null, predY = null, gateRadius = 60) {
        // Robust Optical Beacon Cluster Detector with Kalman Spatial Gating
        const W = this.viewportSize.width;
        const H = this.viewportSize.height;
        // Reuse existing image data if available, eliminating redundant GPU readback
        const imgData = this.lastFrameImageData || this.offscreenCtx.getImageData(0, 0, W, H);
        const d = imgData.data;

        // Binary threshold to isolate bright beacon spot
        let threshold = 140;
        if (this.atmosphericMode === 'lowlight') threshold = 70;
        if (this.atmosphericMode === 'fog') threshold = 175;
        if (this.atmosphericMode === 'haze') threshold = 100;

        let bestScore = 0;
        let bestPeakX = -1;
        let bestPeakY = -1;
        const maxDistSq = gateRadius * gateRadius;

        // 1. Grid search (step 4) for densest local cluster to reject isolated noise & jitter sparks
        const step = 4;
        for (let y = 8; y < H - 8; y += step) {
            for (let x = 8; x < W - 8; x += step) {
                if (d[(y * W + x) * 4] > threshold) {
                    let localCount = 0;
                    for (let dy = -8; dy <= 8; dy += 2) {
                        for (let dx = -8; dx <= 8; dx += 2) {
                            if (d[((y + dy) * W + (x + dx)) * 4] > threshold) {
                                localCount++;
                            }
                        }
                    }

                    // Score candidate cluster: density + Kalman spatial gating validation
                    let score = localCount;
                    if (predX !== null && predY !== null) {
                        const distSq = (x - predX) * (x - predX) + (y - predY) * (y - predY);
                        if (distSq < maxDistSq) {
                            score += 10; // Prioritize true beacon near predicted trajectory
                        }
                    }

                    if (localCount >= 2 && score > bestScore) {
                        bestScore = score;
                        bestPeakX = x;
                        bestPeakY = y;
                    }
                }
            }
        }

        // If no coherent beacon cluster found
        if (bestPeakX === -1) {
            return null; // Target Lost
        }

        // 2. High-precision intensity moments strictly within the localized cluster window
        const winR = Math.max(10, Math.round(this.targetSize + 4));
        const x0 = Math.max(0, bestPeakX - winR);
        const x1 = Math.min(W - 1, bestPeakX + winR);
        const y0 = Math.max(0, bestPeakY - winR);
        const y1 = Math.min(H - 1, bestPeakY + winR);

        let sumX = 0;
        let sumY = 0;
        let sumWeight = 0;
        let minX = x1, maxX = x0, minY = y1, maxY = y0;
        let pixelCount = 0;

        for (let y = y0; y <= y1; y++) {
            for (let x = x0; x <= x1; x++) {
                const val = d[(y * W + x) * 4];
                if (val > threshold) {
                    sumX += x * val;
                    sumY += y * val;
                    sumWeight += val;
                    pixelCount++;
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }

        if (sumWeight === 0 || pixelCount < 2) {
            return null;
        }

        const cx = sumX / sumWeight;
        const cy = sumY / sumWeight;

        return {
            x: cx,
            y: cy,
            count: pixelCount,
            bbox: {
                x: minX,
                y: minY,
                w: Math.max(this.targetSize, maxX - minX + 2),
                h: Math.max(this.targetSize, maxY - minY + 2)
            }
        };
    }

    step(dt = 1.0 / 30.0) {
        if (!this.running) return;

        this.frameCount++;
        this.totalFramesLogged++;

        // Calculate FPS
        const now = performance.now();
        const frameDelta = (now - this.lastFrameTime) / 1000.0;
        this.lastFrameTime = now;
        if (frameDelta > 0) {
            const instantaneousFPS = 1.0 / frameDelta;
            this.fpsSmoothing.push(instantaneousFPS);
            if (this.fpsSmoothing.length > 20) this.fpsSmoothing.shift();
            this.currentFPS = this.fpsSmoothing.reduce((a, b) => a + b, 0) / this.fpsSmoothing.length;
        }

        // 1. Update Target Motion & Platform Dynamics
        this.updateMotion(dt);

        // 2. Render Virtual Monochrome FPA Viewport
        this.renderMonochromeFPAScene();

        // 3. Update Kalman Filter Prediction first to provide spatial validation gate
        const kalmanPred = this.kalman.predict(dt);

        // Dynamically adapt Kalman measurement noise R to absorb camera jitter & sensor noise
        const jitterVar = Math.pow(this.cameraJitterMax, 2);
        const noiseVar = this.noiseTypes.gaussian ? Math.pow(this.noiseStdDev, 2) : 0;
        const effR = 4.0 + 0.8 * jitterVar + 0.2 * noiseVar;
        this.kalman.setNoiseMatrices(1.5, effR);

        // 4. Run Centroid Detection with Spatial Gating around Kalman prediction
        const isTracking = (this.trackerState === 'acquired' || this.consecutiveDetections > 0);
        const predX = isTracking ? kalmanPred.x : null;
        const predY = isTracking ? kalmanPred.y : null;
        const gateRadius = 50.0 + 2.0 * this.cameraJitterMax;
        const detection = this.detectBeaconCentroid(predX, predY, gateRadius);

        let trackingPos = null;

        if (detection !== null) {
            // Measurement available
            this.consecutiveDetections++;
            this.consecutiveLosses = 0;
            const updated = this.kalman.update(detection.x, detection.y);
            trackingPos = { x: updated.x, y: updated.y, isPredicted: false };
        } else {
            // Target lost: Kalman prediction holds for consecutive frames
            this.consecutiveLosses++;
            this.consecutiveDetections = 0;
            const updated = this.kalman.update(null, null);
            trackingPos = { x: updated.x, y: updated.y, isPredicted: true };
        }

        // 5. Update Tracker State Machine (Acquired, Re-acquiring, Lost)
        const prevTrackerState = this.trackerState;
        if (this.consecutiveDetections >= 3) {
            this.trackerState = 'acquired';
            if (!this.acquisitionCompleted && this.acquisitionStartTime) {
                this.acquisitionTimeSec = (now - this.acquisitionStartTime) / 1000.0;
                this.acquisitionCompleted = true;
            }
            if (this.reAcquisitionStartTime) {
                this.reAcquisitionTimeSec = (now - this.reAcquisitionStartTime) / 1000.0;
                this.reAcquisitionStartTime = null;
            }
        } else if (this.consecutiveLosses >= 3) {
            this.trackerState = 'lost';
            this.totalLostFrames++;
            if (prevTrackerState === 'acquired') {
                this.reAcquisitionStartTime = now;
            }
        } else {
            this.trackerState = 're-acquiring';
        }

        // 6. Centroid Error & Telemetry
        const camCenterX = this.viewportSize.width / 2;
        const camCenterY = this.viewportSize.height / 2;

        let error = 0;
        if (trackingPos) {
            error = Math.hypot(trackingPos.x - camCenterX, trackingPos.y - camCenterY);
        } else if (this.groundTruthViewportPos) {
            error = Math.hypot(this.groundTruthViewportPos.x - camCenterX, this.groundTruthViewportPos.y - camCenterY);
        }

        this.centroidError = error;
        this.errorSum += error;
        this.squaredErrorsSum += error * error;
        if (error > this.maxTrackingError) this.maxTrackingError = error;

        // Record for real-time plot (last 100 frames)
        this.errorHistory.push(error);
        if (this.errorHistory.length > 100) this.errorHistory.shift();

        // 7. Pan-Tilt PID Controller Update with World Velocity Feedforward
        const errorX = (trackingPos ? trackingPos.x : camCenterX) - camCenterX;
        const errorY = (trackingPos ? trackingPos.y : camCenterY) - camCenterY;

        // World-space target velocity for feedforward compensation
        const targetWorldX = this.cameraPos.x + errorX;
        const targetWorldY = this.cameraPos.y + errorY;
        const rawVx = (targetWorldX - this.prevTargetWorldX) / Math.max(0.0001, dt);
        const rawVy = (targetWorldY - this.prevTargetWorldY) / Math.max(0.0001, dt);
        const alpha = 0.75;
        this.targetWorldVx = alpha * this.targetWorldVx + (1.0 - alpha) * rawVx;
        this.targetWorldVy = alpha * this.targetWorldVy + (1.0 - alpha) * rawVy;
        this.prevTargetWorldX = targetWorldX;
        this.prevTargetWorldY = targetWorldY;

        // Convert Max Pan/Tilt speed from deg/s to pixels/sec
        const maxPanSpeedPx = this.maxPanSpeedDeg * this.pixelsPerDegreeX;
        const maxTiltSpeedPx = this.maxTiltSpeedDeg * this.pixelsPerDegreeY;

        const control = this.pid.compute(
            errorX, errorY, dt,
            maxPanSpeedPx, maxTiltSpeedPx,
            this.targetWorldVx, this.targetWorldVy
        );

        // Move Virtual Camera Viewport across 2000x2000 canvas
        this.cameraPos.x += control.vx * dt;
        this.cameraPos.y += control.vy * dt;

        const halfW = this.viewportSize.width / 2;
        const halfH = this.viewportSize.height / 2;

        // Autonomous Re-acquisition: If target escaped FOV (lost for > 15 frames),
        // execute coarse search slew towards target world coordinate instead of staying frozen at corner
        if (this.trackerState === 'lost' && this.consecutiveLosses > 15) {
            const dirX = this.targetPos.x - this.cameraPos.x;
            const dirY = this.targetPos.y - this.cameraPos.y;
            const dist = Math.hypot(dirX, dirY);
            if (dist > 10) {
                const searchSpeed = Math.min(maxPanSpeedPx, 450.0);
                this.cameraPos.x += (dirX / dist) * searchSpeed * dt;
                this.cameraPos.y += (dirY / dist) * searchSpeed * dt;
                this.kalman.reset(halfW, halfH);
                this.pid.reset();
            }
        }

        // Clamp camera position within global canvas bounds
        this.cameraPos.x = Math.max(halfW, Math.min(this.canvasSize.width - halfW, this.cameraPos.x));
        this.cameraPos.y = Math.max(halfH, Math.min(this.canvasSize.height - halfH, this.cameraPos.y));

        // Save detailed frame snapshot for logging
        this.latestSnapshot = {
            timestamp: ((now - this.sessionStartTime) / 1000.0).toFixed(3),
            frame: this.frameCount,
            fps: this.currentFPS.toFixed(1),
            centroidError: error.toFixed(2),
            state: this.trackerState,
            targetX: this.targetPos.x.toFixed(1),
            targetY: this.targetPos.y.toFixed(1),
            camX: this.cameraPos.x.toFixed(1),
            camY: this.cameraPos.y.toFixed(1),
            detection: detection,
            kalman: trackingPos,
            control: control
        };

        if (this.frameCount % 2 === 0) {
            this.sessionLog.push(this.latestSnapshot);
            if (this.sessionLog.length > 3000) this.sessionLog.shift();
        }
    }

    getSessionMetrics() {
        const total = Math.max(1, this.totalFramesLogged);
        const avgError = this.errorSum / total;
        const rmse = Math.sqrt(this.squaredErrorsSum / total);
        const lossRate = (this.totalLostFrames / total) * 100.0;
        const retentionRate = Math.max(0, 100.0 - lossRate);

        return {
            durationSec: ((performance.now() - this.sessionStartTime) / 1000.0).toFixed(1),
            avgFPS: this.currentFPS.toFixed(1),
            currentError: this.centroidError.toFixed(2),
            avgError: avgError.toFixed(2),
            maxError: this.maxTrackingError.toFixed(2),
            rmse: rmse.toFixed(2),
            lossRate: lossRate.toFixed(2),
            retentionRate: retentionRate.toFixed(2),
            acquisitionTime: this.acquisitionTimeSec.toFixed(2),
            reAcquisitionTime: this.reAcquisitionTimeSec.toFixed(2),
            acquisitionCompleted: this.acquisitionCompleted,
            trackerState: this.trackerState,
            processingTimeMs: (1000.0 / Math.max(1, this.currentFPS)).toFixed(1)
        };
    }
}

window.FSOCTrackingSimulator = FSOCTrackingSimulator;
