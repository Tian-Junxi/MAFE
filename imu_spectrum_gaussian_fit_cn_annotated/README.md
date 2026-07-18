# imu_spectrum_gaussian_fit

> 📄 [中文版 (Chinese)](README_zh.md)

A ROS1 / Catkin C++ package for extracting vibration noise spectral peak parameters from 6-axis IMU time-domain data, using Gaussian peak fitting as described in the paper.

## 1. Overview

**Input CSV:**
```
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
```

Defaults:
- Columns 1–3: gyroscope triaxial measurements;
- Columns 4–6: accelerometer triaxial measurements;
- Sampling rate: 200 Hz (configurable in `config/fit_params.yaml`).

The program processes gyroscope and accelerometer data separately:

```
Triaxial time-domain data
  → De-mean
  → Hann windowing
  → FFTW single-sided spectrum
  → Triaxial magnitude synthesis
  → Detect 4 candidate spectral peaks
  → Local Gaussian function fitting
  → Output dominant frequency, variance, left/right bounds
  → Plot spectrum & fitting curves
```

Corresponding paper formula:

$$S(f) = A \exp\left(-\frac{(f - \mu)^2}{2\sigma_f^2}\right)$$

After fitting:

$$f_d = \mu, \quad f_L = \mu - \beta\sigma_f, \quad f_R = \mu + \beta\sigma_f$$

Default $\beta = 2$.

## 2. Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-std-msgs \
  libeigen3-dev libfftw3-dev libceres-dev
```

| Dependency | Purpose |
|-----------|---------|
| FFTW3 | FFT spectrum computation |
| Ceres Solver | Nonlinear least-squares Gaussian peak fitting |
| Eigen | Matrix computation (required by Ceres) |
| roscpp | ROS1 C++ node |

## 3. Build

```bash
cd ~/catkin_ws/src
cp -r imu_spectrum_gaussian_fit .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. Quick Start

The package includes a sample IMU file:

```
examples/example_imu_200hz_6cols.csv
```

Run:

```bash
roslaunch imu_spectrum_gaussian_fit fit_imu_gaussian.launch \
  csv_path:=$(rospack find imu_spectrum_gaussian_fit)/examples/example_imu_200hz_6cols.csv \
  output_dir:=/tmp/imu_spectrum_gaussian_fit
```

## 5. Output Files

The following files are generated under `output_dir`:

```
gyro_peak_params.csv
gyro_spectrum_fit.csv
gyro_spectrum_fit.svg

acc_peak_params.csv
acc_spectrum_fit.csv
acc_spectrum_fit.svg
```

### `*_peak_params.csv`

| Field | Description |
|-------|-------------|
| `peak_id` | Peak index |
| `valid` | Validity flag |
| `A` | Gaussian amplitude |
| `fd_mu_hz` | Dominant frequency (Gaussian center $\mu$) |
| `sigma_hz` | Gaussian standard deviation |
| `variance_hz2` | Frequency variance ($\sigma^2$) |
| `f_left_hz` | Left boundary |
| `f_right_hz` | Right boundary |
| `rmse` | Local fitting residual |

### `*_spectrum_fit.svg`

SVG plot containing:
- FFT single-sided spectrum;
- Individual Gaussian peak fitting curves;
- Superposed Gaussian peak curve;
- Dominant frequency markers;
- Left and right frequency boundaries.

## 6. Parameters

Parameter file: `config/fit_params.yaml`

```yaml
sample_rate_hz: 200.0
num_peaks: 4
min_freq_hz: 2.0
max_freq_hz: 100.0
fit_half_window_hz: 5.0
beta: 2.0
```

If your IMU sampling rate differs from 200 Hz, modify:

```yaml
sample_rate_hz: <your_rate>
max_freq_hz: <not exceeding sample_rate_hz/2>
```

## 7. Integration with GPR Model

This package extracts:

```
Dominant frequency:  fd_mu_hz
Frequency variance: variance_hz2
```

These can be paired with motor speeds to construct:

```
motor_speed, fd_mu_hz, variance_hz2
```

for training the shared GPR model, achieving:

```
4 motor speeds → 4 predicted dominant frequencies + 4 predicted variances
```

## 8. Code Annotations

This version includes detailed Chinese annotations in the main source files, particularly:

- `fft_spectrum.hpp`: FFT, single-sided spectrum, Hann window, triaxial magnitude synthesis;
- `gaussian_peak_fitter.hpp`: Peak detection, Gaussian fitting, Ceres optimization, frequency/variance/boundary calculation;
- `imu_gaussian_fit_node.cpp`: ROS parameters, processing pipeline, and output files.

See also: `docs/code_annotation_zh.md`
