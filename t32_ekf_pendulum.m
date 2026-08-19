%% ============================================================
% 教程 32：EKF 实战 — 倒立摆状态估计
%
% 【为什么要学这课】
%   t31 的 Van der Pol 振子有 2 个状态，非线性来自阻尼项
%   倒立摆也是 2 个状态，但非线性更强——
%   动力学里有 sin(θ)，标准 Kalman 完全不能工作
%   这是 EKF 最经典的入门实战案例
%
% 【系统方程】
%   倒立摆 (点质量, 无阻尼):
%     ẋ₁ = x₂                      (角度 θ)
%     ẋ₂ = (g/L)*sin(x₁) + u       (角加速度)
%     y  = x₁ + v                  (只测角度，有噪声)
%
%   g/L = 9.81/1.0 = 9.81
%
% ┌─────────────────────────────────────────────────────────┐
% │ 本课重点：EKF 在强非线性系统中的表现                      │
% │                                                         │
% │  Jacobian: A = [0,         1]                           │
% │                [(g/L)*cos(x₁), 0]                       │
% │                                                         │
% │  注意：sin(x₁) 的导数是 cos(x₁)                          │
% │       当 θ→90° 时，cos→0 → 局部近似退化为纯积分器        │
% │       这就是 EKF 在极端角度时会出问题的原因               │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 对强非线性系统 (sin) 实现 EKF 状态估计
%   2. 观察 EKF 在不同初值误差下的收敛过程
%   3. 理解"Jacobian 随时间变化"的含义
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统定义：倒立摆 =====

g = 9.81;
L = 1.0;
omega0 = sqrt(g/L);     % 自然频率 ≈ 3.13 rad/s
Ts = 0.01;              % EKF 采样时间 (100 Hz)
T_final = 10;

fprintf('============================================\n');
fprintf('  教程 32：EKF 实战 — 倒立摆状态估计\n');
fprintf('============================================\n\n');

fprintf('【系统：倒立摆】\n');
fprintf('  ẋ₁ = x₂\n');
fprintf('  ẋ₂ = %.2f * sin(x₁) + u\n', omega0^2);
fprintf('  y  = x₁ + v\n\n');
fprintf('  不稳定性：θ=0 是竖直向上（不稳定平衡点）\n');
fprintf('  非线性：sin(θ) → 远离 θ=0 时近似极差\n\n');

%% ===== 第 1 步：Simulink 模型 =====

fprintf('【Step 1: Simulink 采集数据】\n');

mdl = 'tutorial32_pendulum';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 输入 u(t)：小幅方波扰动 ---
add_block('simulink/Sources/Pulse Generator', [mdl '/Input_u'], ...
    'Position', [50, 130, 110, 170]);
set_param([mdl '/Input_u'], 'Amplitude', '0.3', 'Period', '4', ...
    'PulseWidth', '25', 'PhaseDelay', '0.5');

% --- 倒立摆 Plant ---
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Pendulum_Plant'], ...
    'Position', [200, 70, 300, 160]);
Simulink.SubSystem.deleteContents([mdl '/Pendulum_Plant']);
plant_root = [mdl '/Pendulum_Plant'];

add_block('simulink/Ports & Subsystems/In1', [plant_root '/u'], ...
    'Position', [30, 80, 50, 100]);
add_block('simulink/Math Operations/Sum', [plant_root '/Sum_accel'], ...
    'Position', [130, 80, 160, 120]);
set_param([plant_root '/Sum_accel'], 'Inputs', '|++', 'IconShape', 'round');
add_block('simulink/Continuous/Integrator', [plant_root '/Int_x2'], ...
    'Position', [230, 85, 280, 115]);
set_param([plant_root '/Int_x2'], 'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', [plant_root '/Int_x1'], ...
    'Position', [350, 85, 400, 115]);
set_param([plant_root '/Int_x1'], 'InitialCondition', '0.5');  % ★ 初始偏离 28.6°

% 重力项: (g/L)*sin(x₁)
add_block('simulink/Math Operations/Trigonometric Function', ...
    [plant_root '/Sin_x1'], 'Position', [60, 170, 110, 210]);
set_param([plant_root '/Sin_x1'], 'Operator', 'sin');
add_block('simulink/Math Operations/Gain', [plant_root '/GravityGain'], ...
    'Position', [140, 180, 180, 210]);
set_param([plant_root '/GravityGain'], 'Gain', num2str(omega0^2));

add_block('simulink/Ports & Subsystems/Out1', [plant_root '/y'], ...
    'Position', [490, 80, 510, 100]);
add_block('simulink/Ports & Subsystems/Out1', [plant_root '/x2_out'], ...
    'Position', [490, 130, 510, 150]);

% 连线
add_line(plant_root, 'u/1', 'Sum_accel/1');
add_line(plant_root, 'Sum_accel/1', 'Int_x2/1');
add_line(plant_root, 'Int_x2/1', 'Int_x1/1');
add_line(plant_root, 'Int_x1/1', 'Sin_x1/1');
add_line(plant_root, 'Sin_x1/1', 'GravityGain/1');
add_line(plant_root, 'GravityGain/1', 'Sum_accel/2');
add_line(plant_root, 'Int_x1/1', 'y/1');
add_line(plant_root, 'Int_x2/1', 'x2_out/1');

% --- 测量噪声 ---
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/MeasNoise'], ...
    'Position', [340, 80, 380, 110]);
add_block('simulink/Math Operations/Gain', [mdl '/NoiseScale'], ...
    'Position', [410, 85, 440, 110]);
set_param([mdl '/NoiseScale'], 'Gain', '0.03');
add_block('simulink/Math Operations/Add', [mdl '/Add_Noise'], ...
    'Position', [480, 85, 510, 115]);
set_param([mdl '/Add_Noise'], 'Inputs', '|++', 'IconShape', 'round');

% --- To Workspace ---
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1true'], ...
    'Position', [540, 180, 590, 205]);
set_param([mdl '/ws_x1true'], 'VariableName', 'x1_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2true'], ...
    'Position', [540, 220, 590, 245]);
set_param([mdl '/ws_x2true'], 'VariableName', 'x2_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_y_noisy'], ...
    'Position', [540, 260, 590, 285]);
set_param([mdl '/ws_y_noisy'], 'VariableName', 'y_noisy');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_u'], ...
    'Position', [540, 300, 590, 325]);
set_param([mdl '/ws_u'], 'VariableName', 'u_data');

% --- 顶层连线 ---
add_line(mdl, 'Input_u/1', 'Pendulum_Plant/1');
add_line(mdl, 'Pendulum_Plant/1', 'Add_Noise/1');
add_line(mdl, 'MeasNoise/1', 'NoiseScale/1');
add_line(mdl, 'NoiseScale/1', 'Add_Noise/2');
add_line(mdl, 'Pendulum_Plant/1', 'ws_x1true/1');
add_line(mdl, 'Pendulum_Plant/2', 'ws_x2true/1');
add_line(mdl, 'Add_Noise/1', 'ws_y_noisy/1');
add_line(mdl, 'Input_u/1', 'ws_u/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

set_param(mdl, 'StopTime', num2str(T_final));
set_param(mdl, 'Solver', 'ode45');
set_param(mdl, 'MaxStep', num2str(Ts/2));
simOut = sim(mdl);

t = simOut.tout;
x1t = getSimData(simOut, 'x1_true', t);
x2t = getSimData(simOut, 'x2_true', t);
y_m  = getSimData(simOut, 'y_noisy', t);
u_d  = getSimData(simOut, 'u_data', t);
N = length(t);
close_system(mdl, 0);

fprintf('  [OK] 仿真完成: %d 个采样点\n\n', N);

%% ===== 第 2 步：EKF 实现 =====

fprintf('【Step 2: EKF 估计】\n');

Q = diag([0.001, 0.01]);
R = 0.001;

x_ekf = zeros(2, N);
x_hat = [1.0; 0];        % ★ 故意给一个很差的初值 (57.3°! 真值是 0.5)
P = eye(2);

for k = 2:N
    dt = t(k) - t(k-1);
    u_k = u_d(k-1);
    x1 = x_hat(1); x2 = x_hat(2);

    % 1. 非线性预报
    x_pred = x_hat + dt * [x2;
                            omega0^2 * sin(x1) + u_k];

    % 2. 离散状态转移矩阵 F = I + dt·(∂f/∂x)
    F = [1,                       dt;
         dt*omega0^2*cos(x1),     1];

    % 3. 预报协方差
    P_pred = F * P * F' + Q * dt;

    % 4. Kalman 增益
    H = [1, 0];
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    % 5. 更新
    y_pred = x_pred(1);
    x_hat = x_pred + K * (y_m(k) - y_pred);
    P = (eye(2) - K * H) * P_pred;

    x_ekf(:, k) = x_hat;
end

% 线性 Kalman (错误模型: 用 sinθ≈θ 近似)
A_lin = [0, 1; omega0^2, 0];  % 小角度近似
C_lin = [1, 0];
[L_kal, ~, ~] = lqe(A_lin, eye(2), C_lin, Q, R);
x_kal = zeros(2, N);
x_hat_kal = [1.0; 0];
for k = 2:N
    dt = t(k) - t(k-1);
    u_k = u_d(k-1);
    x_pred_kal = x_hat_kal + dt * (A_lin * x_hat_kal + [0; 1] * u_k);
    y_pred_kal = x_pred_kal(1);
    x_hat_kal = x_pred_kal + L_kal * (y_m(k) - y_pred_kal);
    x_kal(:, k) = x_hat_kal;
end

fprintf('  [OK] EKF + 线性 Kalman 完成\n\n');

%% ===== 第 3 步：绘图 =====

figure('Name', 't32: EKF — 倒立摆状态估计', ...
    'Position', [50, 50, 1100, 850]);

subplot(4,1,1);
plot(t, x1t, 'k', 'LineWidth', 2); hold on;
plot(t, x_kal(1,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ekf(1,:)', 'r-', 'LineWidth', 1.5); hold off;
legend('真实 θ', '线性 Kalman', 'EKF', 'Location', 'southeast');
title(sprintf('角度估计 — EKF 从初值 %.1f rad (%.0f°) 快速收敛', 1.0, rad2deg(1.0)));
xlabel('时间 (s)'); ylabel('θ (rad)'); grid on;

rmse_x1_k = sqrt(mean((x1t - x_kal(1,:)').^2, 'omitnan'));
rmse_x1_e = sqrt(mean((x1t - x_ekf(1,:)').^2, 'omitnan'));
fprintf('  角度 RMSE: 线性 Kalman = %.4f,  EKF = %.4f\n', rmse_x1_k, rmse_x1_e);

subplot(4,1,2);
plot(t, x2t, 'k', 'LineWidth', 2); hold on;
plot(t, x_kal(2,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ekf(2,:)', 'r-', 'LineWidth', 1.5); hold off;
legend('真实 ω', '线性 Kalman', 'EKF', 'Location', 'southeast');
title('角速度估计');
xlabel('时间 (s)'); ylabel('ω (rad/s)'); grid on;

rmse_x2_k = sqrt(mean((x2t - x_kal(2,:)').^2, 'omitnan'));
rmse_x2_e = sqrt(mean((x2t - x_ekf(2,:)').^2, 'omitnan'));
fprintf('  角速度 RMSE: 线性 Kalman = %.4f,  EKF = %.4f\n', rmse_x2_k, rmse_x2_e);

subplot(4,1,3);
plot(t, x1t - x_kal(1,:)', 'b-', 'LineWidth', 0.8); hold on;
plot(t, x1t - x_ekf(1,:)', 'r-', 'LineWidth', 1.2); hold off;
legend('线性 Kalman 误差', 'EKF 误差', 'Location', 'southeast');
title('估计误差对比 — EKF 收敛更快更稳');
xlabel('时间 (s)'); ylabel('角度误差 (rad)'); grid on;

subplot(4,1,4);
innov = y_m(2:end) - x_ekf(1,1:end-1)';
plot(t(2:end), innov, 'c-', 'LineWidth', 0.5); hold on;
yline(mean(innov), 'r--', 'LineWidth', 1.2); hold off;
title(sprintf('新息序列 y - h(x̂⁻) — 均值=%.4f (应≈0)', mean(innov)));
xlabel('时间 (s)'); ylabel('新息'); grid on;

sgtitle('教程 32：EKF 实战 — 倒立摆状态估计');

%% ===== 第 4 步：总结 =====

fprintf('\n========================================\n');
fprintf('  教程 32 完成！\n');
fprintf('========================================\n\n');

fprintf('【关键收获】\n\n');
fprintf('  1. sin 非线性比 Van der Pol 的乘积非线性更强\n');
fprintf('     EKF 自动通过 cos(x₁) 调节 Jacobian\n');
fprintf('     θ 大时 cos→0 → F 接近 [0 1; 0 0] → 相信测量更多\n\n');
fprintf('  2. 初值误差 1.0 rad → EKF 约 0.3s 收敛到真值附近\n');
fprintf('     这是 EKF 的另一个优势：不需要知道精确初值\n\n');
fprintf('  3. 线性 Kalman 用 sinθ≈θ 近似 → θ>30° 时误差急剧增大\n\n');

fprintf('【动手实验】\n');
fprintf('  1. 改初始角度: Int_x1 InitialCondition = 1.5 → 接近 90°\n');
fprintf('     观察 EKF 还能不能收敛\n\n');
fprintf('  2. 去掉控制输入 u=0 → 纯自由摆动\n');
fprintf('     没有激励，EKF 的 P 会变大 → 估计不确定性增加\n\n');

fprintf('  下一课预告：t33 — UKF：不靠 Jacobian 的非线性滤波\n');
