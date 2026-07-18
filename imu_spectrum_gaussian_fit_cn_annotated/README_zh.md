# imu_spectrum_gaussian_fit

ROS1 / catkin C++ 包，用于从 IMU 六列时域数据中提取振动噪声谱峰参数，并按照论文中的高斯拟合方法输出主频和方差。

## 1. 功能概述

输入 CSV：

```text
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z
```

默认：

- 第 1–3 列为陀螺仪三轴测量；
- 第 4–6 列为加速度计三轴测量；
- 采样率默认 200 Hz，可在 `config/fit_params.yaml` 中修改。

程序会分别对陀螺仪和加速度计执行：

```text
三轴时域数据
  -> 去均值
  -> Hann 加窗
  -> FFTW 计算单边频谱
  -> 三轴频谱幅值合成
  -> 检测 4 个候选谱峰
  -> 局部高斯函数拟合
  -> 输出主频、方差、左右边界
  -> 绘制频谱和拟合曲线
```

对应论文公式：

```text
S(f) = A exp(-(f-mu)^2 / (2 sigma_f^2))
```

拟合后：

```text
fd = mu
fL = mu - beta * sigma_f
fR = mu + beta * sigma_f
```

默认 `beta=2`。

## 2. 依赖安装

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-std-msgs \
  libeigen3-dev libfftw3-dev libceres-dev
```

依赖说明：

- `FFTW3`：FFT 频谱计算；
- `Ceres Solver`：高斯峰非线性最小二乘拟合；
- `Eigen`：Ceres 和矩阵计算依赖；
- `roscpp`：ROS1 C++ 节点。

## 3. 编译

```bash
cd ~/catkin_ws/src
cp -r imu_spectrum_gaussian_fit .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 4. 使用示例

包内自带一个示例 IMU 文件：

```text
examples/example_imu_200hz_6cols.csv
```

运行：

```bash
roslaunch imu_spectrum_gaussian_fit fit_imu_gaussian.launch \
  csv_path:=$(rospack find imu_spectrum_gaussian_fit)/examples/example_imu_200hz_6cols.csv \
  output_dir:=/tmp/imu_spectrum_gaussian_fit
```

## 5. 输出文件

运行后会生成：

```text
/tmp/imu_spectrum_gaussian_fit/gyro_peak_params.csv
/tmp/imu_spectrum_gaussian_fit/gyro_spectrum_fit.csv
/tmp/imu_spectrum_gaussian_fit/gyro_spectrum_fit.svg

/tmp/imu_spectrum_gaussian_fit/acc_peak_params.csv
/tmp/imu_spectrum_gaussian_fit/acc_spectrum_fit.csv
/tmp/imu_spectrum_gaussian_fit/acc_spectrum_fit.svg
```

### `*_peak_params.csv`

字段：

```text
peak_id, valid, A, fd_mu_hz, sigma_hz, variance_hz2, f_left_hz, f_right_hz, rmse
```

含义：

- `fd_mu_hz`：主频，等于高斯中心 `mu`；
- `sigma_hz`：高斯标准差；
- `variance_hz2`：频率方差，即 `sigma_hz^2`；
- `f_left_hz`：左边界；
- `f_right_hz`：右边界；
- `rmse`：局部拟合残差。

### `*_spectrum_fit.svg`

SVG 图中包含：

- FFT 单边频谱；
- 每个高斯峰的拟合曲线；
- 高斯峰叠加曲线；
- 主频位置；
- 左右频率边界。

## 6. 参数说明

参数文件：

```text
config/fit_params.yaml
```

常用参数：

```yaml
sample_rate_hz: 200.0
num_peaks: 4
min_freq_hz: 2.0
max_freq_hz: 100.0
fit_half_window_hz: 5.0
beta: 2.0
```

如果你的 IMU 采样率不是 200 Hz，只需要修改：

```yaml
sample_rate_hz: 你的采样率
max_freq_hz: 不超过 sample_rate_hz/2
```

## 7. 与后续 GPR 模型衔接

本包负责提取：

```text
主频 fd_mu_hz
主频方差 variance_hz2
```

后续可与电机转速配对，构造：

```text
motor_speed, fd_mu_hz, variance_hz2
```

用于训练共享 GPR 模型，实现：

```text
4 个电机转速 -> 4 个预测主频 + 4 个预测方差
```

## 8. 代码注释

本版本已经在主要源码文件中加入了较详细中文注释，尤其是：

- `fft_spectrum.hpp`：解释 FFT、单边谱、Hann 窗、三轴频谱合成；
- `gaussian_peak_fitter.hpp`：解释谱峰检测、高斯拟合、Ceres 优化、主频/方差/边界计算；
- `imu_gaussian_fit_node.cpp`：解释 ROS 参数、处理流程和输出文件。

更多说明见：

```text
docs/代码中文注释说明.md
```
