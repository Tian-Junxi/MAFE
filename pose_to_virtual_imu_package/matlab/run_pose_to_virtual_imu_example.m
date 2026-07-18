% RUN_POSE_TO_VIRTUAL_IMU_EXAMPLE
% -------------------------------------------------------------------------
% MATLAB 2016a 兼容示例脚本。
%
% 功能：
%   读取 pose.csv，调用 pose_to_virtual_imu_from_csv_2016a.m，生成虚拟 IMU。
%
% 使用步骤：
%   1) 将 ROS 位姿数据整理成 pose.csv；
%   2) CSV 推荐表头：stamp,px,py,pz,qx,qy,qz,qw；
%   3) 修改下面 poseCsv 为你的 CSV 路径；
%   4) 运行本脚本；
%   5) 在 output_virtual_imu/ 中查看 CSV 和调试图。
%
% 注意：
%   qx,qy,qz,qw 必须表示 body -> world。
%   若你的 pose 是 LiDAR/camera frame，需要先用外参转换到 body/IMU frame。
% -------------------------------------------------------------------------

clear; clc;

% 输入位姿 CSV 路径。
% 默认当前目录下 pose.csv，也可以写绝对路径，例如：
% poseCsv = '/home/xxx/data/pose.csv';
poseCsv = fullfile(pwd, 'pose.csv');

% 输出目录。程序会自动创建。
outputDir = fullfile(pwd, 'output_virtual_imu');

% -------------------------
% 参数配置
% -------------------------
cfg = struct();

% 输出虚拟 IMU 频率。
% 如果要和真实 IMU residual 对齐，建议设置为真实 IMU 频率，例如 200/500/1000 Hz。
% 如果不想重采样，可以设置 cfg.fsOut = [] 或 cfg.fsOut <= 0。
cfg.fsOut = 200;

% 滑动窗口长度，必须为奇数。
% 11~15：响应快，但导数更抖；
% 21：折中；
% 31~41：更平滑，适合悬停/稳态，但延迟更大。
cfg.windowSize = 21;

% 局部拟合阶数。
% 2：更稳健，适合位姿噪声大或悬停；
% 3：能更好拟合加减速和转弯，但对噪声稍敏感。
cfg.polyOrder = 3;

% 重力加速度。ROS ENU/z-up 下，函数内部使用 g^w=[0;0;-gravity]。
cfg.gravity = 9.80665;

% 位置插值方法。
% 'linear'：最稳但不够平滑；
% 'pchip' ：推荐，较平滑且不易过冲；
% 'spline'：更平滑但可能过冲。
cfg.resampleMethod = 'pchip';

% 对导数结果进行简单移动平均。
% 1 表示关闭；3/5 可压制轻微尖峰。
cfg.postSmoothWindow = 3;

% 是否限幅，防止 pose 偶发跳变导致虚拟 IMU 极大值。
cfg.enableClipping = true;
cfg.maxGyroRadS = 20.0;  % rad/s
cfg.maxAccMS2 = 80.0;    % m/s^2

% 是否输出调试图。
cfg.makePlots = true;

% -------------------------
% 运行主函数
% -------------------------
out = pose_to_virtual_imu_from_csv_2016a(poseCsv, outputDir, cfg);

fprintf('\n虚拟 IMU 已生成：\n%s\n', out.outputCsv);
fprintf('摘要文件：\n%s\n', out.summaryCsv);

% out 结构体中也直接包含数组，便于后续脚本继续处理：
% out.t       : 输出时间，s
% out.gyro_b  : 虚拟陀螺仪，rad/s
% out.acc_b   : 虚拟加速度计比力，m/s^2
% out.v_w     : 世界系速度，m/s
% out.a_w     : 世界系加速度，m/s^2
