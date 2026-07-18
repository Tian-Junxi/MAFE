# 代码中文注释说明

本 ROS/C++ 包用于实现论文中 **Identification of Vibration-Noise Parameters** 这一部分。

整体流程为：

```text
IMU 六列时域数据
  -> 去均值与加窗
  -> FFT 单边频谱
  -> 三轴频谱合成
  -> 局部谱峰搜索
  -> 高斯函数非线性最小二乘拟合
  -> 输出主频 fd、方差 sigma_f^2、左右边界 fL/fR
  -> 绘制频谱与高斯拟合曲线
```

## 代码文件对应关系

| 文件 | 作用 |
|---|---|
| `csv_reader.hpp` | 读取 CSV，支持配置陀螺仪和加速度计列索引 |
| `fft_spectrum.hpp` | 使用 FFTW 计算单边频谱，并进行三轴频谱合成 |
| `gaussian_peak_fitter.hpp` | 搜索谱峰，使用 Ceres 拟合高斯函数，输出主频和方差 |
| `io_utils.hpp` | 保存峰参数和频谱拟合曲线数据 |
| `svg_plotter.hpp` | 直接生成 SVG 频谱图和高斯拟合曲线 |
| `imu_gaussian_fit_node.cpp` | ROS 节点主程序，串联完整流程 |

## 与论文公式的对应

代码中的高斯拟合函数为：

```text
S(f) = A exp(-(f-mu)^2 / (2 sigma_f^2))
```

输出参数为：

```text
fd = mu
variance = sigma_f^2
fL = mu - beta * sigma_f
fR = mu + beta * sigma_f
```

默认 `beta=2`，对应高斯假设下约 95% 频率区间。

## 与后续 GPR 模型的衔接

本包输出的：

```text
fd_mu_hz, variance_hz2
```

可以和电机转速配对，作为后续共享 GPR 模型的训练标签：

```text
motor_speed -> dominant_frequency + frequency_variance
```

也就是说，本包负责从 IMU 时域数据中提取频谱参数；之前的 GPR 包负责学习“电机转速—主频”的非线性关系。
