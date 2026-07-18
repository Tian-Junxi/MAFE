# ROS 位姿反解虚拟 IMU 代码包（详细注释版）

本代码包用于 MAFE/FMAFE 方法中的一个关键预处理环节：

```text
ROS 位姿 / 里程计  ->  虚拟 IMU / 参考 IMU  ->  与真实 IMU 相减  ->  IMU 振动 residual
```

也就是说，代码从低频或中频位姿轨迹中反推出理想情况下 IMU 应该测到的角速度和比力，然后与真实 IMU 测量相减。这样可以尽量去掉平台真实运动成分，保留由电机、桨叶、机架结构引起的高频窄带振动成分，用于后续的 GPR、KL/SKL、AR-FFRLS、KF 等频率估计模块。

代码包包含两部分：

```text
pose_to_virtual_imu_package/
├── README_中文.md
├── matlab/
│   ├── pose_to_virtual_imu_from_csv_2016a.m
│   └── run_pose_to_virtual_imu_example.m
└── ros_pose_to_virtual_imu/
    ├── package.xml
    ├── CMakeLists.txt
    ├── config/
    │   └── pose_to_virtual_imu.yaml
    ├── launch/
    │   └── pose_to_virtual_imu.launch
    └── src/
        └── pose_to_virtual_imu_node.cpp
```

其中：

- `matlab/pose_to_virtual_imu_from_csv_2016a.m` 是离线调试版，兼容 MATLAB 2016a；
- `matlab/run_pose_to_virtual_imu_example.m` 是 MATLAB 调用示例；
- `ros_pose_to_virtual_imu` 是 ROS C++ 在线节点，可订阅 `/odom` 或 `/pose`，实时发布 `/virtual_imu_from_pose`。

---

## 1. 为什么需要“从位姿反解 IMU”

真实 IMU 测量可以粗略写成：

```text
measured_gyro = true_body_angular_rate + gyro_bias + vibration_gyro + white_noise
measured_acc  = true_specific_force     + acc_bias  + vibration_acc  + white_noise
```

MAFE 关注的是电机与机架耦合产生的窄带振动频率。如果直接对原始 IMU 做频谱分析，里面既包含真实机动运动，也包含振动噪声。为了突出高频振动分量，可以用位姿轨迹反推出一个“低频运动对应的参考 IMU”：

```text
virtual_gyro = angular rate derived from pose trajectory
virtual_acc  = specific force derived from pose trajectory
```

然后构造 residual：

```text
gyro_residual = measured_gyro - virtual_gyro
acc_residual  = measured_acc  - virtual_acc
```

后续 GPR/AR-FFRLS/KF 主要对 residual 进行主频估计。

---

## 2. 坐标系和物理量约定

本代码默认采用 ROS 中常见的 ENU / z-up 世界系：

```text
world frame: x-East, y-North, z-Up
body frame : UAV body frame / IMU body frame
```

输入位姿中的四元数约定为：

```text
q_wb: body -> world
R_wb: body -> world rotation matrix
```

位置为：

```text
p_wb: body origin position expressed in world frame
```

世界系重力向量为：

```text
g_w = [0, 0, -9.80665]^T
```

从位姿反解 IMU 的核心公式为：

```text
omega_b = angular velocity expressed in body frame
f_b     = R_wb^T * (a_w - g_w)
```

其中：

- `omega_b` 对应陀螺仪角速度，单位 rad/s；
- `f_b` 对应加速度计测到的比力，单位 m/s^2；
- `a_w` 是世界系下的平动加速度，由位置二阶导数得到。

静止时如果机体姿态与世界系对齐，则：

```text
a_w = [0,0,0]^T
f_b = R_wb^T * ([0,0,0]^T - [0,0,-9.80665]^T)
    = [0,0,9.80665]^T
```

因此静止时虚拟加速度计 z 轴约为 +9.81 m/s^2，这是正常现象。

---

## 3. 为什么不用简单差分

最直接的做法是：

```text
v_k = (p_k - p_{k-1}) / dt
a_k = (v_k - v_{k-1}) / dt
omega_k = Log(R_{k-1}^T R_k) / dt
```

但这个做法对位姿噪声非常敏感，尤其是：

1. 加速度需要位置二阶导数，噪声会被二次放大；
2. 四元数相邻差分会受位姿跳变、时间戳抖动、四元数符号翻转影响；
3. ROS 话题时间间隔不完全均匀，直接差分会引入额外毛刺；
4. LIO/VIO 输出位姿有时会存在局部小跳变，直接差分会产生极大的虚拟 IMU 尖峰。

因此本代码采用滑窗局部拟合而不是简单差分。

---

## 4. 本代码采用的优化方法

### 4.1 位置局部多项式拟合

对每一个窗口中心时刻 `t_k`，取前后若干个位姿样本，构造相对时间：

```text
tau_i = t_i - t_k
```

对位置每个轴分别拟合：

```text
p(tau) = c0 + c1*tau + c2*tau^2 + c3*tau^3
```

在窗口中心 `tau=0` 处：

```text
position     = c0
velocity     = c1
acceleration = 2*c2
```

因此代码输出：

```text
v_w = c1
a_w = 2*c2
```

`poly_order=2` 更平滑，适合悬停、稳态、位姿噪声较大；`poly_order=3` 对机动变化更灵活，适合轨迹跟踪、加减速较明显的场景。

### 4.2 姿态李代数局部拟合

四元数不能直接逐元素差分，也不建议转欧拉角差分。代码在 SO(3) 李代数上做局部拟合。

设窗口中心姿态为 `R_c`，窗口内第 `i` 个姿态为 `R_i`，相对旋转为：

```text
R_rel_i = R_c^T * R_i
```

将其映射到旋转向量：

```text
phi_i = Log(R_rel_i)
```

再对每个轴拟合：

```text
phi(tau) = d0 + d1*tau + d2*tau^2 + ...
```

中心处的角速度为：

```text
omega_b = d phi / dt |_{tau=0} = d1
```

这种方式避免了欧拉角奇异和四元数符号跳变问题。

### 4.3 四元数符号连续化

四元数 `q` 和 `-q` 表示同一个姿态。如果 ROS 位姿序列中符号突然翻转，直接计算相对旋转会产生虚假的大角速度。代码在读取和在线接收位姿时都会执行：

```text
if dot(q_k, q_{k-1}) < 0:
    q_k = -q_k
```

### 4.4 导数后低通和限幅

即使用局部拟合，位姿偶发跳变仍可能产生虚拟 IMU 尖峰。ROS C++ 版提供：

```yaml
gyro_lowpass_tau: 0.03
acc_lowpass_tau: 0.03
enable_clipping: true
max_gyro_rad_s: 20.0
max_acc_m_s2: 80.0
```

MATLAB 离线版提供：

```matlab
cfg.postSmoothWindow = 3;
cfg.enableClipping = true;
cfg.maxGyroRadS = 20.0;
cfg.maxAccMS2 = 80.0;
```

这些参数主要用于抑制非物理尖峰，不建议设置得过强，否则可能抹掉真实机动变化。

---

## 5. MATLAB 离线版使用方法

### 5.1 准备 pose.csv

推荐 CSV 格式：

```text
stamp,px,py,pz,qx,qy,qz,qw
0.000,0.0,0.0,0.0,0.0,0.0,0.0,1.0
0.005,0.0,0.0,0.0,0.0,0.0,0.0,1.0
...
```

字段说明：

| 字段 | 含义 |
|---|---|
| `stamp` | 时间戳，单位秒；也支持 ROS epoch 秒或纳秒 |
| `px,py,pz` | 世界系位置，单位 m |
| `qx,qy,qz,qw` | 四元数，body -> world |

如果你的 CSV 无表头，则默认前 8 列为：

```text
time, px, py, pz, qx, qy, qz, qw
```

### 5.2 MATLAB 运行示例

进入 `matlab/` 文件夹，将 `pose.csv` 放到当前目录，然后运行：

```matlab
run_pose_to_virtual_imu_example
```

或者直接调用主函数：

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

### 5.3 MATLAB 输出文件

运行后会生成：

```text
output_virtual_imu/
├── virtual_imu_from_pose.csv
├── virtual_imu_summary.csv
├── virtual_gyro.png
├── virtual_acc.png
├── velocity_world.png
└── acceleration_world.png
```

其中 `virtual_imu_from_pose.csv` 字段为：

```text
t,px,py,pz,qx,qy,qz,qw,
vx_w,vy_w,vz_w,ax_w,ay_w,az_w,
gyro_x_b,gyro_y_b,gyro_z_b,
acc_x_b,acc_y_b,acc_z_b
```

后续可将真实 IMU 插值到同一个 `t`，并构造：

```matlab
gyro_residual = measured_gyro - virtual_gyro;
acc_residual  = measured_acc  - virtual_acc;
```

---

## 6. ROS C++ 在线版使用方法

### 6.1 放入 catkin_ws 并编译

```bash
cd ~/catkin_ws/src
cp -r /path/to/pose_to_virtual_imu_package/ros_pose_to_virtual_imu .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果你的 catkin 工作空间不是 `~/catkin_ws`，把路径改成自己的即可。

### 6.2 修改配置文件

配置文件位于：

```text
ros_pose_to_virtual_imu/config/pose_to_virtual_imu.yaml
```

常用配置：

```yaml
pose_type: "odom"
pose_topic: "/odom"
output_imu_topic: "/virtual_imu_from_pose"

save_csv: true
output_csv_path: "$(env HOME)/catkin_ws/output/virtual_imu_from_pose.csv"

window_size: 21
poly_order: 3
gravity: 9.80665

gyro_lowpass_tau: 0.03
acc_lowpass_tau: 0.03

enable_clipping: true
max_gyro_rad_s: 20.0
max_acc_m_s2: 80.0

max_pose_gap: 0.2
publish_orientation: true
```

`pose_type` 可选：

```text
odom         -> 订阅 nav_msgs/Odometry
pose_stamped -> 订阅 geometry_msgs/PoseStamped
```

### 6.3 启动节点

```bash
roslaunch ros_pose_to_virtual_imu pose_to_virtual_imu.launch
```

启动后节点会输出：

```text
/virtual_imu_from_pose
```

消息类型为：

```text
sensor_msgs/Imu
```

其中：

```text
angular_velocity      = virtual gyro, rad/s
linear_acceleration   = virtual accelerometer specific force, m/s^2
orientation           = input pose orientation, optional
```

### 6.4 rosbag 回放测试

如果已有 rosbag：

```bash
roscore
rosparam set /use_sim_time true
roslaunch ros_pose_to_virtual_imu pose_to_virtual_imu.launch
rosbag play your_data.bag --clock
```

查看输出频率：

```bash
rostopic hz /virtual_imu_from_pose
```

查看数值：

```bash
rostopic echo /virtual_imu_from_pose
```

录制结果：

```bash
rosbag record /imu/data /odom /virtual_imu_from_pose
```

### 6.5 在线输出 CSV

如果 `save_csv: true`，节点会写出：

```text
~/catkin_ws/output/virtual_imu_from_pose.csv
```

字段包括：

```text
stamp,t,px,py,pz,qx,qy,qz,qw,
vx_w,vy_w,vz_w,ax_w,ay_w,az_w,
gyro_x_b,gyro_y_b,gyro_z_b,
acc_x_b,acc_y_b,acc_z_b
```

---

## 7. 参数如何调

### 7.1 如果虚拟 IMU 波动很大

优先尝试：

```yaml
window_size: 31
poly_order: 2
gyro_lowpass_tau: 0.05
acc_lowpass_tau: 0.06
```

MATLAB：

```matlab
cfg.windowSize = 31;
cfg.polyOrder = 2;
cfg.postSmoothWindow = 5;
```

适合：悬停、低速轨迹、位姿噪声较大、LIO/VIO 输出有轻微抖动。

### 7.2 如果动态响应太慢

尝试：

```yaml
window_size: 11
poly_order: 3
gyro_lowpass_tau: 0.01
acc_lowpass_tau: 0.02
```

适合：急转弯、加减速、8 字轨迹、动态转速阶跃实验。

### 7.3 如果加速度有很多尖峰

原因通常是位置二阶导对 pose 跳变敏感。可以：

```yaml
enable_clipping: true
max_acc_m_s2: 50.0
acc_lowpass_tau: 0.08
window_size: 31
```

同时检查原始 pose 是否有跳变。

### 7.4 如果角速度偶发尖峰

通常与四元数跳变、时间戳不均匀、姿态噪声有关。可以：

```yaml
gyro_lowpass_tau: 0.05
window_size: 21
poly_order: 2
```

也要确认输入四元数确实表示 body -> world。

---

## 8. 用于 MAFE residual 的推荐流程

### 8.1 在线流程

```text
/odom or /pose
    -> ros_pose_to_virtual_imu_node
    -> /virtual_imu_from_pose

/imu/data and /virtual_imu_from_pose
    -> 时间同步 / 插值
    -> residual_gyro, residual_acc
    -> GPR/KL/AR-FFRLS/KF 主频估计
```

### 8.2 离线流程

```text
rosbag -> pose.csv
pose.csv -> MATLAB virtual_imu_from_pose.csv
imu.csv + virtual_imu_from_pose.csv -> residual.csv
residual.csv -> MAFE frequency estimation
```

### 8.3 residual 构造注意事项

真实 IMU 和虚拟 IMU 的坐标系必须一致。如果真实 IMU 安装坐标与 body frame 不一致，需要先进行外参变换：

```text
omega_body = R_bi * omega_imu
acc_body   = R_bi * acc_imu
```

其中 `R_bi` 表示 IMU frame -> body frame。否则 residual 中会混入坐标系误差。

真实 IMU 和虚拟 IMU 时间也必须对齐。若存在固定时间偏移，需要先做时间标定或通过互相关估计时间延迟。

---

## 9. 常见问题

### Q1：为什么虚拟 IMU 输出频率低于真实 IMU？

ROS 在线版输出频率基本等于输入 pose 频率，因为它只能在新的 pose 到来时更新。如果 pose 是 50 Hz，虚拟 IMU 也是约 50 Hz。若后续 residual 需要 200 Hz，可以离线使用 MATLAB 版重采样，或在线另加插值模块。

### Q2：为什么 MATLAB 版可以输出 200 Hz？

MATLAB 版先将 pose 插值到均匀时间轴，再做局部拟合。因此可以设置 `cfg.fsOut=200`。但这并不代表凭空增加真实信息，只是为了方便与真实 IMU 对齐。

### Q3：为什么窗口越大越平滑但延迟越大？

输出时间戳是滑窗中心。若窗口为 21 个 pose 样本，理论延迟约为 10 个 pose 采样周期。对于 50 Hz pose，延迟约为 0.2 s。

### Q4：为什么 residual 低频分量仍然很大？

可能原因：

1. 真实 IMU 和 pose 时间不同步；
2. IMU frame 与 body frame 外参不一致；
3. pose 不是 IMU/body 位姿，而是 LiDAR/camera 位姿；
4. 位姿本身尺度或坐标系方向有误；
5. bias 未去除，尤其是加速度计 bias。

### Q5：静止时 acc_z 是 +9.81 还是 -9.81？

在 ROS ENU 且机体 z 轴朝上的约定下，静止时加速度计比力为 +9.81 m/s^2。如果你的 IMU 静止读数是 -9.81，说明 IMU 轴向或坐标系定义不同，需要做坐标转换。

---

## 10. 建议的调试顺序

1. 先画原始 pose 的 `px,py,pz,qx,qy,qz,qw`，确认没有明显跳变；
2. 运行 MATLAB 版，查看 `velocity_world.png` 和 `acceleration_world.png`；
3. 静止/悬停段检查 `virtual_acc.png` 是否接近重力方向；
4. 检查 `virtual_gyro.png` 是否存在不合理尖峰；
5. 用真实 IMU 构造 residual，看低频运动成分是否明显下降；
6. 再进入 MAFE 主频估计模块。

推荐先用 MATLAB 离线版确定参数，再把同样的 `window_size / poly_order / lowpass_tau` 写入 ROS C++ 配置。

