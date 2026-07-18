# gpr_shared_frequency_model

这是一个 ROS1 / catkin 风格的 C++ 包，用于根据实验数据库训练“四电机共享 GPR 主频模型”，并在 ROS 中根据给定四个电机转速预测对应的振动噪声主频和方差。

核心输入来自 Excel/CSV 数据库：

- 不同推力、不同姿态角工况；
- 四个电机转速 `m1,m2,m3,m4`；
- 由 `FFT + Gaussian peak fitting` 得到的 1~4 个振动噪声主频；
- 每个主频对应的高斯拟合方差。

核心输出：

```text
output_dir/gyro_gpr_model.gpr
output_dir/acc_gpr_model.gpr
```

这两个 `.gpr` 文件就是训练好的模型文件，后续预测节点会加载它们。

---

## 1. 方法说明

本包实现的是：

```text
平均转速先验 + 迭代重匹配 + 四电机共享 GPR
```

也就是说，四个电机不分别训练四个模型，而是共享一个等效单电机模型：

```math
f_d = g(m)
```

其中 `m` 是任意一个电机转速，`f_d` 是该转速对应的振动主频。

对于输入四个电机转速：

```text
[m1, m2, m3, m4]
```

预测节点会对同一个 GPR 模型调用四次，输出：

```text
[mean1, var1, mean2, var2, mean3, var3, mean4, var4]
```

其中 `mean_i` 是第 `i` 个电机的预测主频，`var_i` 是预测方差。

---

## 2. 编译

```bash
cd ~/catkin_ws/src
cp -r gpr_shared_frequency_model .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

依赖：

```bash
sudo apt-get install ros-${ROS_DISTRO}-roscpp ros-${ROS_DISTRO}-std-msgs libeigen3-dev libyaml-cpp-dev
```

---

## 3. 用当前示例数据库训练

包内已经包含一个由你上传的 Excel 转换得到的示例 CSV：

```text
examples/updated_database_from_uploaded_excel.csv
```

直接训练：

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch gpr_shared_frequency_model train_shared_gpr.launch
```

默认会使用：

```text
examples/updated_database_from_uploaded_excel.csv
config/train_params_excel_current.yaml
```

并把模型输出到：

```text
~/catkin_ws/gpr_shared_frequency_model_result/
```

输出文件包括：

```text
gyro_gpr_model.gpr
acc_gpr_model.gpr
gyro_iter_stats.csv
acc_iter_stats.csv
gyro_final_matched_samples.csv
acc_final_matched_samples.csv
```

其中：

- `gyro_gpr_model.gpr`：陀螺仪共享 GPR 主频模型；
- `acc_gpr_model.gpr`：加速度计共享 GPR 主频模型；
- `*_iter_stats.csv`：每轮迭代的 R²、RMSE、匹配变化行数；
- `*_final_matched_samples.csv`：最终展开后的 `(motor_speed, matched_frequency)` 训练样本。

---

## 4. 使用自己的 Excel 训练

假设你的 Excel 在：

```text
/home/tjx/data/new_database.xlsx
```

可以运行：

```bash
cd ~/catkin_ws
source devel/setup.bash

bash $(rospack find gpr_shared_frequency_model)/scripts/train_from_xlsx.sh \
  /home/tjx/data/new_database.xlsx \
  ./gpr_shared_frequency_model_result \
  $(rospack find gpr_shared_frequency_model)/config/train_params_excel_current.yaml
```

训练完成后模型在：

```text
~/catkin_ws/gpr_shared_frequency_model_result/gyro_gpr_model.gpr
~/catkin_ws/gpr_shared_frequency_model_result/acc_gpr_model.gpr
```

---

## 5. 当前 Excel 列号配置

`config/train_params_excel_current.yaml` 对应如下 Excel 布局：

```text
A: 推力或角度标签
C-F: 电机1, 电机2, 电机3, 电机4
H-J: 陀螺仪主频方差1-3
L-N: 陀螺仪主频1-3
Q-S: 加速度计主频方差1-3
U-W: 加速度计主频1-3
```

对应 0-based 列号为：

```yaml
motor_speed_cols: [2, 3, 4, 5]

gyro:
  variance_cols:  [7, 8, 9]
  frequency_cols: [11, 12, 13]

acc:
  variance_cols:  [16, 17, 18]
  frequency_cols: [20, 21, 22]
```

如果后续你增加到 4 个主频，只需要修改 YAML 中的 `variance_cols` 和 `frequency_cols`，不需要修改 C++ 代码。

---

## 6. 文件结构

```text
include/gpr_shared_frequency_model/gpr1d_matern.hpp
  一维 Matern 3/2 GPR 模型，包含训练、预测、保存、加载。

include/gpr_shared_frequency_model/csv_utils.hpp
  CSV 读取和空值处理工具。

src/train_shared_gpr_node.cpp
  训练节点：读取 CSV，迭代重匹配，训练共享 GPR，保存 .gpr 模型。

src/predict_shared_gpr_node.cpp
  预测节点：加载 .gpr 模型，订阅 /motor_speeds，发布主频均值和方差。

scripts/xlsx_to_csv.py
  无第三方依赖的 Excel 转 CSV 工具，保留空列位置。

scripts/train_from_xlsx.sh
  Excel 一键转换并训练模型。

config/train_params_excel_current.yaml
  当前 Excel 数据库的列号配置。

config/train_params_excel_4peak_template.yaml
  后续 4 主频版本模板。
```

---

## 7. 训练/预测耗时统计

### 训练耗时

训练节点会在终端输出每个传感器、每一轮迭代的耗时，单位为 ms，例如：

```text
gyro iter 1: changed_rows=..., R2=..., RMSE=..., match_time=... ms, gpr_train_time=... ms, eval_time=... ms, iter_total_time=... ms
[Timing] gyro total_time=... ms, total_matching_time=... ms, total_gpr_training_time=... ms, total_eval_time=... ms, save_time=... ms
Training finished. total_elapsed_time=... ms
```

其中：

- `match_time`：当前轮隐变量谱峰重匹配耗时；
- `gpr_train_time`：当前轮 GPR 拟合训练耗时，包括核矩阵构建、超参数搜索、Cholesky 分解等；
- `eval_time`：在匹配样本上计算 R² 和 RMSE 的耗时；
- `iter_total_time`：当前迭代总耗时；
- `total_gpr_training_time`：该传感器所有 GPR 训练轮次的累计耗时。

### 预测耗时

预测节点收到 `/motor_speeds` 后，会在终端输出每次预测耗时：

```text
[Timing] GPR prediction: gyro=... ms, acc=... ms, callback_total=... ms, input_speeds=[m1, m2, m3, m4]
```

如果输入频率较高，不想每次都打印，可以设置日志节流：

```bash
roslaunch gpr_shared_frequency_model predict_shared_gpr.launch \
  output_dir:=~/catkin_ws/output \
  timing_log_throttle_sec:=1.0
```

如果完全不打印预测耗时：

```bash
roslaunch gpr_shared_frequency_model predict_shared_gpr.launch \
  output_dir:=~/catkin_ws/output \
  print_timing:=false
```

### 输出目录

训练和预测默认使用：

```text
~/catkin_ws/output
```

训练时修改输出目录：

```bash
roslaunch gpr_shared_frequency_model train_shared_gpr.launch \
  output_dir:=/home/tjx/my_gpr_output
```

预测时读取同一目录：

```bash
roslaunch gpr_shared_frequency_model predict_shared_gpr.launch \
  output_dir:=/home/tjx/my_gpr_output
```
