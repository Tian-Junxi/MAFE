# 示例运行说明

## 1. 安装依赖

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-std-msgs \
  libeigen3-dev libfftw3-dev libceres-dev
```

## 2. 编译

```bash
cd ~/catkin_ws/src
cp -r /path/to/imu_spectrum_gaussian_fit .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 3. 运行示例数据

```bash
roslaunch imu_spectrum_gaussian_fit fit_imu_gaussian.launch \
  csv_path:=$(rospack find imu_spectrum_gaussian_fit)/examples/example_imu_200hz_6cols.csv \
  output_dir:=/tmp/imu_spectrum_gaussian_fit
```

## 4. 查看输出

```bash
ls /tmp/imu_spectrum_gaussian_fit
```

主要输出包括：

```text
gyro_peak_params.csv
gyro_spectrum_fit.csv
gyro_spectrum_fit.svg
acc_peak_params.csv
acc_spectrum_fit.csv
acc_spectrum_fit.svg
```

其中：

- `*_peak_params.csv`：每个高斯峰的主频 `fd_mu_hz`、标准差 `sigma_hz`、方差 `variance_hz2`、左右边界 `f_left_hz/f_right_hz`；
- `*_spectrum_fit.csv`：频率、原始 FFT 幅值谱、每个高斯峰曲线、叠加曲线；
- `*_spectrum_fit.svg`：带高斯拟合曲线的频谱图，可直接用浏览器打开。
