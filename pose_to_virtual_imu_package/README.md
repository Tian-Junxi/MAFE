# Pose-to-Virtual-IMU

> 📄 [中文版 (Chinese)](README_zh.md)

This package performs a key preprocessing step in the MAFE/FMAFE framework:

```
ROS Pose / Odometry → Virtual IMU / Reference IMU → Subtract from real IMU → IMU Vibration Residual
```

In other words, it derives the **ideal angular velocity and specific force** that an IMU *should* measure from a low/mid-frequency pose trajectory, then subtracts these from the real IMU measurements. This suppresses the platform's true motion components, preserving the high-frequency narrowband vibration caused by motors, propellers, and airframe structure — which are then used by subsequent modules (GPR, KL/SKL, AR-FFRLS, KF, etc.) for frequency estimation.

The package contains two implementations:

```
pose_to_virtual_imu_package/
├── README.md                # English (this file)
├── README_zh.md             # Chinese version
├── matlab/                  # MATLAB offline debugging
│   ├── pose_to_virtual_imu_from_csv_2016a.m
│   └── run_pose_to_virtual_imu_example.m
└── ros_pose_to_virtual_imu/ # ROS C++ online node
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   └── pose_to_virtual_imu.yaml
    ├── launch/
    │   └── pose_to_virtual_imu.launch
    └── src/
        └── pose_to_virtual_imu_node.cpp
```

- `matlab/pose_to_virtual_imu_from_csv_2016a.m`: Offline debugging, compatible with MATLAB 2016a+;
- `matlab/run_pose_to_virtual_imu_example.m`: MATLAB usage example;
- `ros_pose_to_virtual_imu/`: ROS C++ online node, subscribes to `/odom` or `/pose`, publishes `/virtual_imu_from_pose` in real time.

---

## 1. Why "Pose-to-IMU"?

A real IMU measurement can be roughly expressed as:

```
measured_gyro = true_body_angular_rate + gyro_bias + vibration_gyro + white_noise
measured_acc  = true_specific_force     + acc_bias  + vibration_acc  + white_noise
```

MAFE focuses on narrowband vibration frequencies induced by motor–airframe coupling. Direct spectral analysis of raw IMU data contains both true maneuvering motion and vibration noise. To highlight the high-frequency vibration components, we derive a **low-frequency motion reference IMU** from the pose trajectory:

```
virtual_gyro = angular rate derived from pose trajectory
virtual_acc  = specific force derived from pose trajectory
```

Then construct the residual:

```
gyro_residual = measured_gyro - virtual_gyro
acc_residual  = measured_acc  - virtual_acc
```

Subsequent GPR/AR-FFRLS/KF modules operate on these residuals for dominant frequency estimation.

---

## 2. Coordinate Frames & Conventions

This code assumes the common ROS ENU / z-up world frame:

```
world frame: x-East, y-North, z-Up
body frame : UAV body frame / IMU body frame
```

Input pose quaternion convention:

```
q_wb: body → world
R_wb: body → world rotation matrix
```

Position is:

```
p_wb: body origin position expressed in world frame
```

World-frame gravity vector:

```
g_w = [0, 0, -9.80665]ᵀ
```

Core formula for deriving IMU from pose:

```
ω_b = angular velocity expressed in body frame
f_b = R_wbᵀ · (a_w − g_w)
```

where:
- `ω_b` corresponds to gyroscope angular velocity (rad/s);
- `f_b` corresponds to accelerometer specific force (m/s²);
- `a_w` is translational acceleration in world frame, from the second derivative of position.

At rest, with body aligned to world frame:

```
a_w = [0, 0, 0]ᵀ
f_b = R_wbᵀ · ([0, 0, 0]ᵀ − [0, 0, −9.80665]ᵀ)
    = [0, 0, 9.80665]ᵀ
```

Thus a resting virtual accelerometer reads ~+9.81 m/s² on the z-axis — this is normal.

---

## 3. Why Not Simple Finite Differences?

The most straightforward approach:

```
v_k   = (p_k − p_{k−1}) / dt
a_k   = (v_k − v_{k−1}) / dt
ω_k   = Log(R_{k−1}ᵀ R_k) / dt
```

is highly sensitive to pose noise:

1. Acceleration requires the second derivative of position — noise is amplified quadratically;
2. Adjacent quaternion differencing suffers from pose jumps, timestamp jitter, and quaternion sign flips;
3. ROS topic intervals are not perfectly uniform, introducing spurious spikes from naive differencing;
4. LIO/VIO outputs occasionally exhibit local jumps, producing large virtual IMU spikes.

This package therefore uses **sliding-window local fitting** rather than simple finite differences.

---

## 4. Optimization Method

### 4.1 Position: Local Polynomial Fitting

For each window center time $t_k$, take surrounding pose samples and construct relative time:

$$\tau_i = t_i - t_k$$

Fit each position axis independently:

$$p(\tau) = c_0 + c_1\tau + c_2\tau^2 + c_3\tau^3$$

At window center $\tau = 0$:

```
position     = c₀
velocity     = c₁
acceleration = 2·c₂
```

Hence:

```
v_w = c₁
a_w = 2·c₂
```

- `poly_order = 2`: smoother, suitable for hovering, steady-state, higher-noise poses;
- `poly_order = 3`: more flexible for trajectory tracking and stronger acceleration/deceleration scenarios.

### 4.2 Attitude: Local Lie Algebra Fitting

Quaternions cannot be differenced element-wise, and Euler angles are not recommended due to singularities. This code performs local fitting on the SO(3) Lie algebra.

Let the window-center attitude be $R_c$ and the $i$-th attitude in the window be $R_i$. The relative rotation is:

$$R_{\text{rel},i} = R_c^{\mathsf T} R_i$$

Map it to a rotation vector:

$$\boldsymbol{\phi}_i = \operatorname{Log}(R_{\text{rel},i})$$

Then fit each axis:

$$\boldsymbol{\phi}(\tau) = \mathbf{d}_0 + \mathbf{d}_1\tau + \mathbf{d}_2\tau^2 + \dots$$

The angular velocity at center is:

$$\boldsymbol{\omega}_b = \left.\frac{d\boldsymbol{\phi}}{d\tau}\right|_{\tau=0} = \mathbf{d}_1$$

This avoids Euler-angle singularities and quaternion sign-flip issues.

### 4.3 Quaternion Sign Continuity

Quaternions $q$ and $-q$ represent the same rotation. If the sign flips in a ROS pose sequence, computing relative rotation directly produces spurious large angular velocities. The code enforces continuity when reading and receiving poses:

```
if dot(q_k, q_{k−1}) < 0:
    q_k = −q_k
```

### 4.4 Post-Derivative Low-Pass Filtering and Clipping

Even with local fitting, occasional pose jumps can produce virtual IMU spikes. The ROS C++ version provides:

```yaml
gyro_lowpass_tau: 0.03
acc_lowpass_tau: 0.03
enable_clipping: true
max_gyro_rad_s: 20.0
max_acc_m_s2: 80.0
```

The MATLAB offline version provides:

```matlab
cfg.postSmoothWindow = 3;
cfg.enableClipping = true;
cfg.maxGyroRadS = 20.0;
cfg.maxAccMS2 = 80.0;
```

These suppress non-physical spikes — do not set them too aggressively, or real maneuver variations may be attenuated.

---

## 5. MATLAB Offline Usage

### 5.1 Prepare `pose.csv`

Recommended CSV format:

```
stamp,px,py,pz,qx,qy,qz,qw
0.000,0.0,0.0,0.0,0.0,0.0,0.0,1.0
0.005,0.0,0.0,0.0,0.0,0.0,0.0,1.0
...
```

| Field | Description |
|-------|-------------|
| `stamp` | Timestamp in seconds; also supports ROS epoch seconds or nanoseconds |
| `px, py, pz` | World-frame position (m) |
| `qx, qy, qz, qw` | Quaternion (body → world) |

If your CSV has no header, the first 8 columns default to:

```
time, px, py, pz, qx, qy, qz, qw
```

### 5.2 MATLAB Example

Navigate to the `matlab/` folder, place `pose.csv` there, then run:

```matlab
run_pose_to_virtual_imu_example
```

Or call the main function directly:

```matlab
cfg = struct();
cfg.fsOut = 200;
cfg.windowSize = 21;
cfg.polyOrder = 3;
cfg.gravity = 9.80665;
cfg.resampleMethod = 'pchip';
cfg.postSmoothWindow = 3;
cfg.enableClipping = true;
cfg.maxGyroRadS = 20.0;
cfg.maxAccMS2 = 80.0;
cfg.makePlots = true;

out = pose_to_virtual_imu_from_csv_2016a('pose.csv', 'output_virtual_imu', cfg);
```

### 5.3 MATLAB Output Files

```
output_virtual_imu/
├── virtual_imu_from_pose.csv
├── virtual_imu_summary.csv
├── virtual_gyro.png
├── virtual_acc.png
├── velocity_world.png
└── acceleration_world.png
```

---

## 6. ROS C++ Online Node

### 6.1 Build

```bash
cd ~/catkin_ws/src
cp -r pose_to_virtual_imu_package/ros_pose_to_virtual_imu .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 6.2 Run

```bash
roslaunch ros_pose_to_virtual_imu pose_to_virtual_imu.launch
```

The node subscribes to `/odom` (or a custom topic configured in the YAML file) and publishes `/virtual_imu_from_pose` with `sensor_msgs/Imu` messages.
