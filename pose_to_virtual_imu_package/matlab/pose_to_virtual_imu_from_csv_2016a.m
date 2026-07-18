function out = pose_to_virtual_imu_from_csv_2016a(poseCsv, outputDir, cfg)
%POSE_TO_VIRTUAL_IMU_FROM_CSV_2016A
% 从 ROS 位姿 / 里程计 CSV 反解“虚拟 IMU / 参考 IMU”。
%
% -------------------------------------------------------------------------
% 1. 这个函数在 MAFE/FMAFE 中的位置
% -------------------------------------------------------------------------
% MAFE 的 residual 构造需要先从位姿轨迹反推出低频运动对应的 IMU 量：
%
%   pose / odometry  --->  virtual gyro, virtual acc
%   real IMU         --->  measured gyro, measured acc
%
% 然后构造：
%
%   gyro_residual = measured_gyro - virtual_gyro
%   acc_residual  = measured_acc  - virtual_acc
%
% 后续 GPR / KL-SKL / AR-FFRLS / KF 主要对 residual 做主频估计。
%
% -------------------------------------------------------------------------
% 2. 为什么不用简单差分
% -------------------------------------------------------------------------
% 如果直接使用：
%
%   v = diff(p) / dt
%   a = diff(v) / dt
%   omega = Log(R_k^T R_{k+1}) / dt
%
% 位置二阶导和姿态差分会显著放大位姿噪声，尤其 LIO/VIO 位姿在实际飞行中
% 往往存在微小抖动、时间戳不均匀、局部跳变等问题。因此本函数采用：
%
%   位置：滑窗局部多项式拟合，然后取一阶 / 二阶导数；
%   姿态：四元数符号连续化 + SO(3) 李代数局部拟合，然后取一阶导数；
%   输出：可选移动平均与限幅，抑制偶发非物理尖峰。
%
% -------------------------------------------------------------------------
% 3. 输入 CSV 格式
% -------------------------------------------------------------------------
% 推荐带表头：
%
%   stamp,px,py,pz,qx,qy,qz,qw
%
% 或无表头前 8 列：
%
%   time, px, py, pz, qx, qy, qz, qw
%
% 其中 q=[qx,qy,qz,qw] 表示 body -> world 的四元数。
% 时间戳可为相对秒、ROS epoch 秒或纳秒，函数内部会转为相对秒。
%
% -------------------------------------------------------------------------
% 4. 坐标系约定
% -------------------------------------------------------------------------
% 世界系采用 ROS 常用 ENU / z-up：
%
%   g^w = [0, 0, -9.80665]^T
%
% 输入四元数 q_wb 表示 body -> world。加速度计测到的是比力：
%
%   f^b = R_wb^T * (a^w - g^w)
%
% 静止时，如果机体 z 轴向上，则 virtual acc 的 z 轴约为 +9.81 m/s^2。
%
% -------------------------------------------------------------------------
% 5. 典型调用
% -------------------------------------------------------------------------
%   cfg = struct();
%   cfg.fsOut = 200;
%   cfg.windowSize = 21;
%   cfg.polyOrder = 3;
%   cfg.gravity = 9.80665;
%   cfg.resampleMethod = 'pchip';
%   cfg.postSmoothWindow = 3;
%   cfg.enableClipping = true;
%   cfg.maxGyroRadS = 20.0;
%   cfg.maxAccMS2 = 80.0;
%   cfg.makePlots = true;
%   out = pose_to_virtual_imu_from_csv_2016a('pose.csv','output_virtual_imu',cfg);
%
% -------------------------------------------------------------------------
% 6. 输出
% -------------------------------------------------------------------------
%   outputDir/virtual_imu_from_pose.csv
%   outputDir/virtual_imu_summary.csv
%   outputDir/virtual_gyro.png
%   outputDir/virtual_acc.png
%   outputDir/velocity_world.png
%   outputDir/acceleration_world.png
%
% Author: ChatGPT for MAFE/FMAFE debugging workflow

if nargin < 1 || isempty(poseCsv)
    poseCsv = 'pose.csv';
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(pwd, 'output_virtual_imu');
end
if nargin < 3
    cfg = struct();
end

cfg = setDefaultCfg(cfg);
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% -------------------------------------------------------------------------
% Step 1. 读取位姿 CSV
% -------------------------------------------------------------------------
% tRaw: 原始时间戳。
% pRaw: 世界系位置 [px,py,pz]。
% qRaw: 四元数 [qx,qy,qz,qw]，表示 body -> world。
[tRaw, pRaw, qRaw] = readPoseCsvCompat(poseCsv);

% 清洗数据：去掉 NaN/Inf，排序，去重，并把 ROS epoch / 纳秒时间转为相对秒。
[tRaw, pRaw, qRaw] = sanitizePose(tRaw, pRaw, qRaw);

% 四元数符号连续化。
% 注意 q 和 -q 表示同一个姿态，但如果序列中突然从 q 跳到 -q，
% Log(q_c^{-1}q_j) 会出现虚假的大角度，导致 gyro 尖峰。
qRaw = enforceQuatContinuity(qRaw);

% -------------------------------------------------------------------------
% Step 2. 重采样到均匀时间轴
% -------------------------------------------------------------------------
% MATLAB 离线版允许输出虚拟 IMU 到指定频率 fsOut，便于和真实 IMU 对齐。
% 例如 pose 为 50 Hz，真实 IMU 为 200 Hz，可以设置 cfg.fsOut=200。
% 需要注意：插值不会增加真实信息，只是为了统一时间轴。
dtPose = median(diff(tRaw));
fsPose = 1.0 / dtPose;

if isempty(cfg.fsOut) || cfg.fsOut <= 0
    fsOut = fsPose;
else
    fsOut = cfg.fsOut;
end

dtOut = 1.0 / fsOut;
tUniform = (tRaw(1):dtOut:tRaw(end))';

% 位置插值。
% pchip 通常比 spline 更不容易过冲，比 linear 更平滑。
pUniform = zeros(length(tUniform), 3);
for j = 1:3
    pUniform(:,j) = interp1(tRaw, pRaw(:,j), tUniform, cfg.resampleMethod, 'extrap');
end

% 四元数插值。
% 为了兼容 MATLAB 2016a，这里采用逐元素线性插值 + 归一化。
% 在高频小间隔位姿中该做法足够稳定；若 pose 很稀疏且姿态变化剧烈，
% 可以进一步替换为 slerp 插值。
qUniform = zeros(length(tUniform), 4);
for j = 1:4
    qUniform(:,j) = interp1(tRaw, qRaw(:,j), tUniform, 'linear', 'extrap');
end
qUniform = normalizeQuatArray(qUniform);
qUniform = enforceQuatContinuity(qUniform);

% -------------------------------------------------------------------------
% Step 3. 滑窗局部多项式拟合反解速度、加速度、角速度
% -------------------------------------------------------------------------
% validIdx 是去掉窗口两端后可计算虚拟 IMU 的中心样本索引。
% vW/aW 是世界系速度/加速度；gyroB 是机体系角速度。
[validIdx, vW, aW, gyroB] = localPolyInverseImu(tUniform, pUniform, qUniform, cfg);

tOut = tUniform(validIdx);
pOut = pUniform(validIdx,:);
qOut = qUniform(validIdx,:);

% -------------------------------------------------------------------------
% Step 4. 世界系加速度转换为机体系比力
% -------------------------------------------------------------------------
% 加速度计测量的是 specific force，而不是世界系加速度本身。
% ROS ENU 下 gW=[0,0,-g]，因此静止时：
%   f_b = R_wb^T * (0 - gW) ≈ [0,0,+9.81]^T
% 如果你的真实 IMU 静止读数相反，需要检查坐标轴定义和外参。
gW = [0; 0; -cfg.gravity];
accB = zeros(length(tOut), 3);
for k = 1:length(tOut)
    % qOut 存储格式为 [qx,qy,qz,qw]，旋转矩阵函数需要 [w,x,y,z]。
    Rwb = quatToRotmWXYZ([qOut(k,4), qOut(k,1), qOut(k,2), qOut(k,3)]);
    fb = Rwb' * (aW(k,:)' - gW);
    accB(k,:) = fb';
end

% -------------------------------------------------------------------------
% Step 5. 可选后处理：移动平均 + 限幅
% -------------------------------------------------------------------------
% postSmoothWindow 是非常轻量的后平滑，主要用于压制局部导数毛刺。
% 不建议过大，否则会抹掉真实运动变化。
if cfg.postSmoothWindow > 1
    gyroB = movingAverageCols(gyroB, cfg.postSmoothWindow);
    accB  = movingAverageCols(accB,  cfg.postSmoothWindow);
    vW    = movingAverageCols(vW,    cfg.postSmoothWindow);
    aW    = movingAverageCols(aW,    cfg.postSmoothWindow);
end

% 限幅只用于抑制位姿异常造成的非物理尖峰。
% 若后续要精确分析极端机动，可以关闭 enableClipping。
if cfg.enableClipping
    gyroB = clipMatrix(gyroB, -cfg.maxGyroRadS, cfg.maxGyroRadS);
    accB  = clipMatrix(accB,  -cfg.maxAccMS2,   cfg.maxAccMS2);
end

% -------------------------------------------------------------------------
% Step 6. 写出 CSV、摘要和调试图
% -------------------------------------------------------------------------
outCsv = fullfile(outputDir, 'virtual_imu_from_pose.csv');
writeVirtualImuCsv(outCsv, tOut, pOut, qOut, vW, aW, gyroB, accB);

summaryCsv = fullfile(outputDir, 'virtual_imu_summary.csv');
writeSummaryCsv(summaryCsv, poseCsv, fsPose, fsOut, cfg, length(tRaw), length(tOut));

if cfg.makePlots
    makeDebugPlots(outputDir, tOut, gyroB, accB, vW, aW);
end

% 返回结构体，便于脚本中继续构造 residual。
out = struct();
out.t = tOut;
out.p_w = pOut;
out.q_xyzw = qOut;
out.v_w = vW;
out.a_w = aW;
out.gyro_b = gyroB;
out.acc_b = accB;
out.outputCsv = outCsv;
out.summaryCsv = summaryCsv;

fprintf('Virtual IMU generated: %s\n', outCsv);
fprintf('Pose fs = %.3f Hz, virtual IMU fs = %.3f Hz, samples = %d\n', fsPose, fsOut, length(tOut));
end

% =========================================================================
% 默认配置
% =========================================================================
function cfg = setDefaultCfg(cfg)
% 该函数集中设置参数默认值，避免主函数中反复判断字段是否存在。
%
% fsOut:
%   离线输出虚拟 IMU 频率。若设置为空或 <=0，则保持原始 pose 频率。
%
% windowSize:
%   滑窗长度，必须为奇数。越大越平滑，但窗口中心输出造成的延迟越大。
%
% polyOrder:
%   多项式阶数。2 更稳健；3 对加减速和转弯更灵活。
%
% postSmoothWindow:
%   对最终 gyro/acc/v/a 做简单移动平均。1 表示关闭。
%
% enableClipping:
%   是否限幅，防止位姿偶发跳变导致极大虚拟 IMU。
if ~isfield(cfg, 'fsOut'), cfg.fsOut = 200; end
if ~isfield(cfg, 'windowSize'), cfg.windowSize = 21; end
if ~isfield(cfg, 'polyOrder'), cfg.polyOrder = 3; end
if ~isfield(cfg, 'gravity'), cfg.gravity = 9.80665; end
if ~isfield(cfg, 'resampleMethod'), cfg.resampleMethod = 'pchip'; end
if ~isfield(cfg, 'postSmoothWindow'), cfg.postSmoothWindow = 3; end
if ~isfield(cfg, 'enableClipping'), cfg.enableClipping = true; end
if ~isfield(cfg, 'maxGyroRadS'), cfg.maxGyroRadS = 20.0; end
if ~isfield(cfg, 'maxAccMS2'), cfg.maxAccMS2 = 80.0; end
if ~isfield(cfg, 'makePlots'), cfg.makePlots = true; end

cfg.windowSize = max(5, round(cfg.windowSize));
if mod(cfg.windowSize,2) == 0
    cfg.windowSize = cfg.windowSize + 1;
end
cfg.polyOrder = max(2, min(round(cfg.polyOrder), cfg.windowSize-2));
end

% =========================================================================
% 读取 CSV：支持有表头和无表头
% =========================================================================
function [t, p, q] = readPoseCsvCompat(csvFile)
if ~exist(csvFile, 'file')
    error('Pose CSV not found: %s', csvFile);
end

try
    % readtable 可以根据表头自动读取，适合从 Python/ROS 转出的规范 CSV。
    T = readtable(csvFile);
    names = lower(T.Properties.VariableNames);
    t = getColumnByNames(T, names, {'stamp','time','timestamp','t'});
    px = getColumnByNames(T, names, {'px','p_x','position_x','x'});
    py = getColumnByNames(T, names, {'py','p_y','position_y','y'});
    pz = getColumnByNames(T, names, {'pz','p_z','position_z','z'});
    qx = getColumnByNames(T, names, {'qx','q_x','orientation_x'});
    qy = getColumnByNames(T, names, {'qy','q_y','orientation_y'});
    qz = getColumnByNames(T, names, {'qz','q_z','orientation_z'});
    qw = getColumnByNames(T, names, {'qw','q_w','orientation_w'});
    p = [px, py, pz];
    q = [qx, qy, qz, qw];
catch
    % 无表头时，按前 8 列读取。该方式适合最简单的 numeric CSV。
    M = csvread(csvFile);
    if size(M,2) < 8
        error('CSV must contain at least 8 columns: time, px, py, pz, qx, qy, qz, qw.');
    end
    t = M(:,1);
    p = M(:,2:4);
    q = M(:,5:8);
end
end

function col = getColumnByNames(T, names, candidates)
idx = [];
for i = 1:length(candidates)
    tmp = find(strcmp(names, lower(candidates{i})), 1);
    if ~isempty(tmp)
        idx = tmp;
        break;
    end
end
if isempty(idx)
    error('Required column not found. Candidates: %s', strjoin(candidates, ', '));
end
col = T{:,idx};
col = double(col(:));
end

% =========================================================================
% 清洗时间戳、排序、去重
% =========================================================================
function [t, p, q] = sanitizePose(t, p, q)
t = double(t(:));
p = double(p);
q = double(q);

valid = isfinite(t) & all(isfinite(p),2) & all(isfinite(q),2);
t = t(valid); p = p(valid,:); q = q(valid,:);

% 如果时间戳量级大于 1e12，通常是纳秒。
if median(abs(t)) > 1e12
    t = t * 1e-9;
end

[t, order] = sort(t);
p = p(order,:);
q = q(order,:);

[t, uniqueIdx] = unique(t, 'stable');
p = p(uniqueIdx,:);
q = q(uniqueIdx,:);

if length(t) < 10
    error('Too few valid pose samples.');
end

% 转相对时间，避免 ROS epoch 秒很大时影响多项式拟合数值稳定性。
t = t - t(1);
q = normalizeQuatArray(q);
end

% =========================================================================
% 滑动窗口局部多项式反解 IMU
% =========================================================================
function [validIdx, vW, aW, gyroB] = localPolyInverseImu(t, p, qxyzw, cfg)
N = length(t);
halfWin = floor(cfg.windowSize/2);
validIdx = (halfWin+1):(N-halfWin);
M = length(validIdx);

vW = zeros(M,3);
aW = zeros(M,3);
gyroB = zeros(M,3);

order = cfg.polyOrder;

for ii = 1:M
    k = validIdx(ii);
    idx = (k-halfWin):(k+halfWin);

    % 以窗口中心为时间零点，能够提高多项式拟合条件数。
    tau = t(idx) - t(k);
    A = polyDesignMatrix(tau, order);

    % -----------------------------
    % 位置拟合
    % -----------------------------
    % A * coef ≈ p，其中 coef=[c0,c1,c2,...]^T。
    % 在 tau=0 处，速度为 c1，加速度为 2*c2。
    % MATLAB 反斜杠会自动做最小二乘求解。
    for ax = 1:3
        coef = A \ p(idx,ax);
        vW(ii,ax) = coef(2);
        aW(ii,ax) = 2.0 * coef(3);
    end

    % -----------------------------
    % 姿态拟合
    % -----------------------------
    % 对窗口中心姿态 qc，计算 q_rel = qc^{-1} * q_j。
    % 其 Log 映射得到局部旋转向量 rotVec。
    % rotVec 对 tau 的一阶导数就是中心时刻机体系角速度。
    qc = [qxyzw(k,4), qxyzw(k,1), qxyzw(k,2), qxyzw(k,3)]; % [w x y z]
    rotVec = zeros(length(idx),3);
    for jj = 1:length(idx)
        qj = [qxyzw(idx(jj),4), qxyzw(idx(jj),1), qxyzw(idx(jj),2), qxyzw(idx(jj),3)];
        qrel = quatMultiplyWXYZ(quatInvWXYZ(qc), qj);
        rotVec(jj,:) = quatLogVecWXYZ(qrel)';
    end
    for ax = 1:3
        coef = A \ rotVec(:,ax);
        gyroB(ii,ax) = coef(2);
    end
end
end

function A = polyDesignMatrix(tau, order)
% 构造 Vandermonde 型设计矩阵：
% A(:,1)=1, A(:,2)=tau, A(:,3)=tau^2, ...
A = zeros(length(tau), order+1);
for j = 0:order
    A(:,j+1) = tau(:).^j;
end
end

% =========================================================================
% 四元数工具，格式 [w x y z]
% =========================================================================
function q = normalizeQuatArray(q)
% 输入 / 输出 q 为 [qx,qy,qz,qw]，逐行归一化。
for i = 1:size(q,1)
    n = norm(q(i,:));
    if n < 1e-12
        q(i,:) = [0 0 0 1];
    else
        q(i,:) = q(i,:) ./ n;
    end
end
end

function q = enforceQuatContinuity(q)
% 保证相邻四元数点积为正，避免 q 与 -q 的符号跳变。
for i = 2:size(q,1)
    if dot(q(i-1,:), q(i,:)) < 0
        q(i,:) = -q(i,:);
    end
end
end

function qi = quatInvWXYZ(q)
q = q(:)';
qi = [q(1), -q(2), -q(3), -q(4)] ./ max(dot(q,q), 1e-12);
end

function q = quatMultiplyWXYZ(a,b)
a = a(:)'; b = b(:)';
w1=a(1); x1=a(2); y1=a(3); z1=a(4);
w2=b(1); x2=b(2); y2=b(3); z2=b(4);
q = [w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
     w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
     w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
     w1*z2 + x1*y2 - y1*x2 + z1*w2];
q = q ./ max(norm(q), 1e-12);
end

function v = quatLogVecWXYZ(q)
% 四元数 Log 映射到旋转向量。
% 对单位四元数 q=[cos(theta/2), u*sin(theta/2)]，返回 theta*u。
q = q(:);
q = q ./ max(norm(q), 1e-12);
if q(1) < 0
    q = -q;
end
vec = q(2:4);
nv = norm(vec);
if nv < 1e-12
    v = 2.0 * vec;
else
    angle = 2.0 * atan2(nv, q(1));
    v = angle * vec / nv;
end
end

function R = quatToRotmWXYZ(q)
% 四元数 [w,x,y,z] 转旋转矩阵。
q = q(:)' ./ max(norm(q), 1e-12);
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w); ...
     2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w); ...
     2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)];
end

% =========================================================================
% 输出和调试
% =========================================================================
function X = movingAverageCols(X, win)
% 对矩阵每列做简单移动平均。用于轻微平滑导数结果。
if win <= 1, return; end
kernel = ones(win,1) / win;
for j = 1:size(X,2)
    X(:,j) = conv(X(:,j), kernel, 'same');
end
end

function X = clipMatrix(X, lo, hi)
% 简单限幅，防止局部异常值污染后续频谱估计。
X(X < lo) = lo;
X(X > hi) = hi;
end

function writeVirtualImuCsv(fileName, t, p, q, vW, aW, gyroB, accB)
fid = fopen(fileName, 'w');
if fid < 0, error('Cannot open output CSV: %s', fileName); end
fprintf(fid, ['t,px,py,pz,qx,qy,qz,qw,' ...
              'vx_w,vy_w,vz_w,ax_w,ay_w,az_w,' ...
              'gyro_x_b,gyro_y_b,gyro_z_b,acc_x_b,acc_y_b,acc_z_b\n']);
for k = 1:length(t)
    fprintf(fid, '%.9f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n', ...
        t(k), p(k,1),p(k,2),p(k,3), q(k,1),q(k,2),q(k,3),q(k,4), ...
        vW(k,1),vW(k,2),vW(k,3), aW(k,1),aW(k,2),aW(k,3), ...
        gyroB(k,1),gyroB(k,2),gyroB(k,3), accB(k,1),accB(k,2),accB(k,3));
end
fclose(fid);
end

function writeSummaryCsv(fileName, poseCsv, fsPose, fsOut, cfg, nRaw, nOut)
fid = fopen(fileName, 'w');
fprintf(fid, 'item,value\n');
fprintf(fid, 'pose_csv,%s\n', poseCsv);
fprintf(fid, 'raw_pose_samples,%d\n', nRaw);
fprintf(fid, 'virtual_imu_samples,%d\n', nOut);
fprintf(fid, 'pose_sampling_frequency_hz,%.9f\n', fsPose);
fprintf(fid, 'virtual_imu_frequency_hz,%.9f\n', fsOut);
fprintf(fid, 'window_size,%d\n', cfg.windowSize);
fprintf(fid, 'poly_order,%d\n', cfg.polyOrder);
fprintf(fid, 'gravity,%.9f\n', cfg.gravity);
fprintf(fid, 'post_smooth_window,%d\n', cfg.postSmoothWindow);
fclose(fid);
end

function makeDebugPlots(outputDir, t, gyroB, accB, vW, aW)
try
    figure('Visible','off');
    plot(t, gyroB); grid on;
    xlabel('Time (s)'); ylabel('Gyro (rad/s)');
    legend('x','y','z'); title('Virtual gyro from pose');
    saveas(gcf, fullfile(outputDir, 'virtual_gyro.png'));
    close(gcf);

    figure('Visible','off');
    plot(t, accB); grid on;
    xlabel('Time (s)'); ylabel('Specific force (m/s^2)');
    legend('x','y','z'); title('Virtual accelerometer from pose');
    saveas(gcf, fullfile(outputDir, 'virtual_acc.png'));
    close(gcf);

    figure('Visible','off');
    plot(t, vW); grid on;
    xlabel('Time (s)'); ylabel('Velocity in world (m/s)');
    legend('x','y','z'); title('Differentiated velocity');
    saveas(gcf, fullfile(outputDir, 'velocity_world.png'));
    close(gcf);

    figure('Visible','off');
    plot(t, aW); grid on;
    xlabel('Time (s)'); ylabel('Acceleration in world (m/s^2)');
    legend('x','y','z'); title('Differentiated acceleration');
    saveas(gcf, fullfile(outputDir, 'acceleration_world.png'));
    close(gcf);
catch ME
    warning('Plot generation failed: %s', ME.message);
end
end
