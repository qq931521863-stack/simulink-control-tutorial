%% ============================================================
% 教程 35：IMU 多传感器融合 — EKF 在姿态估计中的应用
%
% 【为什么要学这课】
%   前面几课都是"纯仿真"——用数学方程模拟非线性系统
%   本课模拟真实的 IMU 场景：
%     - 加速度计测重力方向（长期准，但瞬时噪声大/有运动加速度干扰）
%     - 陀螺仪积分得角度（短期准，但长期漂移）
%     - EKF 融合两者：用陀螺仪做预报，加速度计做更新
%
%   这就是你手机里、无人机里、机器人里每天都在跑的算法！
%
% ┌─────────────────────────────────────────────────────────┐
% │ IMU 姿态估计的物理原理                                    │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   单轴旋转 (绕 y 轴):                                     │
% │     θ = pitch angle (俯仰角)                             │
% │     ω = 陀螺仪测的角速度 (带漂移 b)                      │
% │                                                         │
% │   状态方程:                                              │
% │     θ̇ = ω_m - b + w_θ         (角度预报)               │
% │     ḃ = w_b                     (漂移随机游走)           │
% │                                                         │
% │   状态向量: x = [θ; b]                                  │
% │                                                         │
% │   测量: 加速度计测重力方向                                │
% │     y = sin(θ) + v  → H = [cos(θ), 0]                  │
% │                                                         │
% │   EKF 融合:                                              │
% │     预报: 用陀螺仪积分 → 短期准                           │
% │     更新: 用加速度计校正 → 消除长期漂移                    │
% └─────────────────────────────────────────────────────────┘
%
% 【本课结构】
%   Step 1: MATLAB 预计算运动轨迹
%   Step 2: Simulink IMU 传感器模型 (From Workspace → 噪声/漂移 → 传感器读数)
%   Step 3: MATLAB 实现三种滤波器对比 (纯积分 / 互补 / EKF)
%
% 【本课目标】
%   1. 理解 IMU 传感器融合的基本原理
%   2. 用 EKF 同时估计姿态角和陀螺仪漂移
%   3. 对比"纯积分"vs"互补滤波"vs"EKF"三种方案
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== IMU 仿真参数 =====

Ts = 0.005;             % IMU 采样率 200 Hz
T_final = 30;
N_imu = round(T_final / Ts) + 1;

gyro_noise_std = 0.005;  % 陀螺仪噪声 (rad/s/√Hz)
gyro_bias_0    = 0.02;    % 初始漂移 (rad/s ≈ 1.15 deg/s)
accel_noise_std = 0.05;  % 加速度计噪声 (g)
accel_bias_0    = 0.01;   % 加速度计偏置 (g)

fprintf('============================================\n');
fprintf('  教程 35：IMU 多传感器融合 — EKF 姿态估计\n');
fprintf('============================================\n\n');

fprintf('【IMU 配置】\n');
fprintf('  陀螺仪: σ=%.3f rad/s, bias₀=%.3f rad/s\n', gyro_noise_std, gyro_bias_0);
fprintf('  加速度计: σ=%.3f g, bias₀=%.3f g\n', accel_noise_std, accel_bias_0);
fprintf('  采样率: %d Hz, 时长: %d s\n\n', 1/Ts, T_final);

%% ===== 第 1 步：MATLAB 预计算运动轨迹 =====

fprintf('【Step 1: 生成运动轨迹】\n');

t_imu = (0:Ts:T_final)';
theta_true = zeros(N_imu, 1);
omega_true = zeros(N_imu, 1);

for k = 2:N_imu
    tk = t_imu(k);
    if tk < 5
        theta_true(k) = deg2rad(30) * sin(2*pi*0.5*tk);
        omega_true(k) = deg2rad(30) * 2*pi*0.5 * cos(2*pi*0.5*tk);
    elseif tk < 10
        theta_true(k) = theta_true(k-1) * 0.995;
        omega_true(k) = (theta_true(k) - theta_true(k-1)) / Ts;
    elseif tk < 20
        theta_true(k) = deg2rad(8) * sin(2*pi*1.5*(tk-10));
        omega_true(k) = deg2rad(8) * 2*pi*1.5 * cos(2*pi*1.5*(tk-10));
    else
        theta_true(k) = 0;
        omega_true(k) = 0;
    end
end

fprintf('  [OK] %d 个时间点生成\n\n', N_imu);

%% ===== 第 2 步：MATLAB 生成 IMU 传感器数据 =====

fprintf('【Step 2: 生成 IMU 传感器数据】\n');

% 陀螺仪漂移 (随机游走)
bias_true = zeros(N_imu, 1);
bias_true(1) = gyro_bias_0;
for k = 2:N_imu
    bias_true(k) = bias_true(k-1) + 0.0003 * sqrt(Ts) * randn();
end
bias_true = bias_true + 0.0005 * t_imu;

% 传感器读数
gyro_meas = omega_true + bias_true + gyro_noise_std * randn(N_imu, 1) / sqrt(Ts);
accel_meas = sin(theta_true) + accel_bias_0 + accel_noise_std * randn(N_imu, 1);

fprintf('  [OK] %d 个 IMU 采样点生成\n', N_imu);
fprintf('  陀螺仪: σ=%.3f rad/s, bias₀=%.3f, 终值=%.3f\n', ...
    gyro_noise_std, gyro_bias_0, bias_true(end));
fprintf('  加速度计: σ=%.3f g\n\n', accel_noise_std);

%% ===== 第 3 步：Simulink 互补滤波器模型 =====

fprintf('【Step 3: Simulink 互补滤波器】\n');

mdl = 'tutorial35_imu';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 导入预生成的传感器数据
gyro_ts = struct('time', t_imu, 'signals', struct('values', gyro_meas));
accel_ts = struct('time', t_imu, 'signals', struct('values', accel_meas));
assignin('base', 'gyro_ts', gyro_ts);
assignin('base', 'accel_ts', accel_ts);

add_block('simulink/Sources/From Workspace', [mdl '/Gyro_Data'], ...
    'Position', [50, 50, 130, 80]);
set_param([mdl '/Gyro_Data'], 'VariableName', 'gyro_ts');

add_block('simulink/Sources/From Workspace', [mdl '/Accel_Data'], ...
    'Position', [50, 150, 130, 180]);
set_param([mdl '/Accel_Data'], 'VariableName', 'accel_ts');

% 互补滤波: θ̂ = α*(θ̂₋₁ + ω·Ts) + (1-α)*asin(accel)
% 陀螺仪通路 (高速): θ_gyro = 积分(ω)
add_block('simulink/Continuous/Integrator', [mdl '/IntGyro'], ...
    'Position', [230, 45, 280, 75]);
set_param([mdl '/IntGyro'], 'InitialCondition', '0');

add_block('simulink/Math Operations/Gain', [mdl '/Alpha'], ...
    'Position', [350, 45, 390, 75]);
set_param([mdl '/Alpha'], 'Gain', '0.98');

% 加速度计通路 (低速): θ_accel = asin(accel), 但 asin 范围有限
% 对于小角度: θ ≈ accel (简化处理)
add_block('simulink/Math Operations/Gain', [mdl '/OneMinusAlpha'], ...
    'Position', [350, 150, 390, 180]);
set_param([mdl '/OneMinusAlpha'], 'Gain', '0.02');

% 融合
add_block('simulink/Math Operations/Sum', [mdl '/SumFusion'], ...
    'Position', [460, 70, 490, 150]);
set_param([mdl '/SumFusion'], 'Inputs', '|++', 'IconShape', 'round');

% 反馈: θ̂ → 积分器输入 (-θ̂ 让积分器变成一阶低通)
% 实际上积分器出来的直接是 θ_gyro, 不需要反馈
% Let me fix: 互补滤波结构 = α·∫ω + (1-α)·θ_accel
% 简化: ∫ω 出来 → ×α → sum
%       asin(accel) → ×(1-α) → sum

add_block('simulink/Sinks/To Workspace', [mdl '/ws_theta_cf'], ...
    'Position', [560, 90, 610, 115]);
set_param([mdl '/ws_theta_cf'], 'VariableName', 'theta_cf_sim');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_time_mdl'], ...
    'Position', [560, 160, 610, 185]);
set_param([mdl '/ws_time_mdl'], 'VariableName', 't_mdl');

% 连线
add_line(mdl, 'Gyro_Data/1', 'IntGyro/1');
add_line(mdl, 'IntGyro/1', 'Alpha/1');
add_line(mdl, 'Alpha/1', 'SumFusion/1');
add_line(mdl, 'Accel_Data/1', 'OneMinusAlpha/1');
add_line(mdl, 'OneMinusAlpha/1', 'SumFusion/2');
add_line(mdl, 'SumFusion/1', 'ws_theta_cf/1');
add_line(mdl, 'SumFusion/1', 'ws_time_mdl/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

set_param(mdl, 'StopTime', num2str(T_final));
simOut = sim(mdl);
close_system(mdl, 0);

fprintf('  [OK] 模型已保存到 models/%s.slx\n', mdl);
fprintf('  说明: Simulink 展示互补滤波结构 (α=0.98)\n');
fprintf('        EKF 在 MATLAB 中实现，因为需要每步重算 Jacobian\n\n');

%% ===== 第 4 步：纯陀螺仪积分 (对比基线) =====

fprintf('【Step 4: 纯陀螺仪积分】\n');

theta_pure = zeros(N_imu, 1);
for k = 2:N_imu
    theta_pure(k) = theta_pure(k-1) + Ts * gyro_meas(k-1);
end
rms_pure = rms(theta_pure - theta_true);
fprintf('  RMSE = %.2f deg\n\n', rad2deg(rms_pure));

%% ===== 第 5 步：MATLAB 互补滤波 =====

fprintf('【Step 5: MATLAB 互补滤波 (α=0.98)】\n');

alpha = 0.98;
theta_cf = zeros(N_imu, 1);
theta_cf(1) = asin(clip(accel_meas(1), -1, 1));
for k = 2:N_imu
    theta_gyro = theta_cf(k-1) + Ts * gyro_meas(k-1);
    theta_acc  = asin(clip(accel_meas(k), -1, 1));
    theta_cf(k) = alpha * theta_gyro + (1-alpha) * theta_acc;
end
rms_cf = rms(theta_cf - theta_true);
fprintf('  RMSE = %.2f deg\n\n', rad2deg(rms_cf));

%% ===== 第 6 步：EKF =====

fprintf('【Step 6: EKF (状态=[θ; b])】\n');

Q_ekf = diag([0.05^2, 0.001^2]);
R_ekf = 0.05^2;

theta_ekf = zeros(N_imu, 1);
bias_ekf  = zeros(N_imu, 1);
x_hat = [asin(clip(accel_meas(1), -1, 1)); 0];
P = diag([0.2^2, 0.05^2]);

for k = 2:N_imu
    dt = Ts;

    % 预报
    x_pred = [x_hat(1) + dt*(gyro_meas(k-1) - x_hat(2));  x_hat(2)];
    F = [1, -dt; 0, 1];
    P_pred = F * P * F' + Q_ekf * dt;

    % 更新
    H = [cos(x_pred(1)), 0];
    S = H * P_pred * H' + R_ekf;
    K = P_pred * H' / S;
    innovation = accel_meas(k) - sin(x_pred(1));
    x_hat = x_pred + K * innovation;
    P = (eye(2) - K * H) * P_pred;

    theta_ekf(k) = x_hat(1);
    bias_ekf(k)  = x_hat(2);
end
rms_ekf = rms(theta_ekf - theta_true);
fprintf('  RMSE = %.2f deg\n', rad2deg(rms_ekf));
fprintf('  漂移估计终值: %.4f rad/s (真实: %.4f)\n\n', bias_ekf(end), bias_true(end));

%% ===== 第 6 步：绘图 =====

figure('Name', 't35: IMU 姿态估计 — 三种方案对比', ...
    'Position', [50, 50, 1100, 950]);

subplot(4,1,1);
plot(t_imu, rad2deg(theta_true), 'k', 'LineWidth', 2); hold on;
plot(t_imu, rad2deg(theta_pure), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
plot(t_imu, rad2deg(theta_cf), 'b-', 'LineWidth', 1.2);
plot(t_imu, rad2deg(theta_ekf), 'r-', 'LineWidth', 1.5); hold off;
legend('真实 θ', '纯积分', '互补滤波', 'EKF', 'Location', 'southeast');
title(sprintf('姿态角估计 — 纯积分(漂移%.1f°)、互补(α=%.2f)、EKF', ...
    rad2deg(max(abs(theta_pure - theta_true))), alpha));
ylabel('Pitch (deg)'); grid on;

subplot(4,1,2);
plot(t_imu, rad2deg(bias_ekf), 'r-', 'LineWidth', 1.5);
title('陀螺仪漂移 b — EKF 在线估计（真实漂移约 0.02 rad/s 起）');
ylabel('Bias (deg/s)'); grid on;

subplot(4,1,3);
plot(t_imu, rad2deg(theta_pure - theta_true), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5); hold on;
plot(t_imu, rad2deg(theta_cf - theta_true), 'b-', 'LineWidth', 1);
plot(t_imu, rad2deg(theta_ekf - theta_true), 'r-', 'LineWidth', 1.5); hold off;
legend('纯积分误差', '互补滤波误差', 'EKF 误差', 'Location', 'southeast');
title(sprintf('估计误差 — Pure=%.1f°, CF=%.1f°, EKF=%.1f°', ...
    rad2deg(rms_pure), rad2deg(rms_cf), rad2deg(rms_ekf)));
ylabel('Error (deg)'); grid on;

subplot(4,1,4);
plot(t_imu, rad2deg(asin(clip(accel_meas, -1, 1))), 'Color', [0.7 0.7 0.7], ...
    'LineWidth', 0.3); hold on;
plot(t_imu, rad2deg(theta_true), 'k', 'LineWidth', 1.2);
plot(t_imu, rad2deg(theta_ekf), 'r-', 'LineWidth', 1.5); hold off;
legend('加速度计 (原始)', '真实 θ', 'EKF 输出', 'Location', 'southeast');
title('加速度计原始数据 vs EKF 滤波输出');
ylabel('Pitch (deg)'); xlabel('时间 (s)'); grid on;

sgtitle('教程 35：IMU 多传感器融合 — EKF 姿态估计');

%% ===== 第 7 步：总结 =====

fprintf('\n========================================\n');
fprintf('  教程 35 完成！\n');
fprintf('========================================\n\n');

fprintf('【三种方案对比】\n');
fprintf('  纯积分:    RMSE = %.1f°  (漂移无限积累)\n', rad2deg(rms_pure));
fprintf('  互补滤波:  RMSE = %.1f°  (简单实用，消费级无人机标配)\n', rad2deg(rms_cf));
fprintf('  EKF:       RMSE = %.1f°  (最优 + 在线估计漂移)\n\n', rad2deg(rms_ekf));

fprintf('【EKF vs 互补滤波】\n');
fprintf('  1. EKF 增益 K 是动态的——cos(θ) 项自动调节信噪比\n');
fprintf('  2. EKF 同时估计漂移 b——互补滤波做不到\n');
fprintf('  3. 协方差 P 给出"现在估计有多不确定"——安全关键应用必需\n\n');

fprintf('【这其实就是你手机里在跑的东西】\n');
fprintf('  手机横竖屏切换 → 6轴 IMU + EKF/互补\n');
fprintf('  无人机飞控 (PX4/ArduPilot) → EKF2/EKF3\n');
fprintf('  VR 头显 tracking → IMU + 视觉 + EKF\n');
fprintf('  自动驾驶定位 → GNSS + IMU + EKF\n\n');

fprintf('  恭喜！Kalman → EKF → UKF → IMU 融合 链路完成。\n\n');

%% ===== 辅助函数 =====
function y = clip(x, lo, hi)
    y = min(max(x, lo), hi);
end
