%% 
%% ============================================================
% 教程 31：扩展 Kalman 滤波 (EKF) — 当系统是非线性的时候
%
% 【从 Kalman 到 EKF】
%   标准 Kalman 滤波要求 ẋ = Ax + Bu（线性系统）
%   真实世界呢？大部分系统是非线性的——
%     ẋ₁ = x₂
%     ẋ₂ = -sin(x₁) - 0.3*x₂ + u           (单摆! 有 sin)
%
%   EKF 的思路极其简单：
%     每一步在当前位置 x̂ 处做一阶泰勒展开
%     → 把非线性 f(x) 局部近似成线性的 F_k
%     → 然后跑标准 Kalman
%
% ┌─────────────────────────────────────────────────────────┐
% │ EKF 五步循环（和标准 Kalman 对比）                       │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   Kalman (线性):           EKF (非线性):                 │
% │   ─────────────            ────────────                  │
% │   1. x̂⁻ = A x̂ + B u       1. x̂⁻ = f(x̂, u)  ← 非线性预报  │
% │   2. P⁻ = A P A' + Q      2. F = ∂f/∂x|x̂   ← 局部线性化  │
% │   3. K = P⁻C'/(CP⁻C'+R)   3. P⁻ = F P F' + Q            │
% │   4. x̂ = x̂⁻ + K(y-Cx̂⁻)   4. K = P⁻H'/(HP⁻H'+R)         │
% │   5. P = (I-KC)P⁻         5. x̂ = x̂⁻ + K(y-h(x̂⁻))         │
% │                            6. P = (I-KH)P⁻               │
% │                                                         │
% │   区别就两点：                                           │
% │   1. 预报用非线性 f(x) 而不是 A*x                        │
% │   2. 每步算 Jacobian F (代替 A) 和 H (代替 C)           │
% └─────────────────────────────────────────────────────────┘
%
% 【本课结构】
%   Step 1: Simulink 搭 Van der Pol 非线性 Plant → 采集带噪声的数据
%   Step 2: MATLAB 手写 EKF 五步循环 → 离线处理数据
%   Step 3: 对比 线性 Kalman（错误模型）vs EKF（正确模型）
%
% 【本课目标】
%   1. 理解为什么标准 Kalman 在非线性系统上会发散
%   2. 手写 EKF 五步循环，理解每一行的物理含义
%   3. 理解 Jacobian = 局部线性化的数学本质
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统定义：Van der Pol 振子 =====

% 非线性动力学：
%   ẋ₁ = x₂
%   ẋ₂ = μ(1 - x₁²)x₂ - x₁ + u
%
% 测量：y = x₁ + v （只测位移，有噪声）

mu = 1.5;              % 非线性强度（μ=0 退化为简谐振子）
Ts = 0.02;             % EKF 采样时间 (50 Hz)
T_final = 15;          % 仿真时长

fprintf('============================================\n');
fprintf('  教程 31：扩展 Kalman 滤波 (EKF)\n');
fprintf('============================================\n\n');

fprintf('【系统：Van der Pol 振子】\n');
fprintf('  ẋ₁ = x₂\n');
fprintf('  ẋ₂ = μ(1-x₁²)x₂ - x₁ + u\n');
fprintf('  y  = x₁ + v  (测量噪声 σ_v ≈ 0.05)\n\n');
fprintf('  当 μ=%.1f 时，系统有稳定的极限环\n', mu);
fprintf('  标准 Kalman 假设线性 → 在大振幅时估计误差大\n');
fprintf('  EKF 每步局部线性化 → 正确跟踪非线性动力学\n\n');

%% ===== 第 1 步：Simulink 模型 — 非线性 Plant + 噪声采集 =====

fprintf('【Step 1: Simulink 采集数据】\n');

mdl = 'tutorial31_ekf';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 输入 u(t) ---
add_block('simulink/Sources/Sine Wave', [mdl '/Input_u'], ...
    'Position', [50, 130, 110, 170]);
set_param([mdl '/Input_u'], 'Amplitude', '0.5', 'Frequency', '0.8', 'Bias', '0');

% --- Van der Pol Plant (子系统：用 Integrator + Fcn 搭) ---
% ẋ₂ = μ(1-x₁²)x₂ - x₁ + u  →  Integrator_x2 → Integrator_x1 = x₁
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/VanDerPol_Plant'], ...
    'Position', [200, 70, 300, 160]);
Simulink.SubSystem.deleteContents([mdl '/VanDerPol_Plant']);

plant_root = [mdl '/VanDerPol_Plant'];

add_block('simulink/Ports & Subsystems/In1', [plant_root '/u'], ...
    'Position', [30, 80, 50, 100]);

add_block('simulink/Math Operations/Sum', [plant_root '/Sum_accel'], ...
    'Position', [120, 80, 150, 130]);
set_param([plant_root '/Sum_accel'], 'Inputs', '|++-', 'IconShape', 'round');

add_block('simulink/Continuous/Integrator', [plant_root '/Int_x2'], ...
    'Position', [220, 85, 270, 115]);
set_param([plant_root '/Int_x2'], 'InitialCondition', '0.5');

add_block('simulink/Continuous/Integrator', [plant_root '/Int_x1'], ...
    'Position', [340, 85, 390, 115]);
set_param([plant_root '/Int_x1'], 'InitialCondition', '0.8');

% Mux 将 [x₁; x₂] 合成为向量 → Fcn 块
add_block('simulink/Signal Routing/Mux', [plant_root '/Mux_x1x2'], ...
    'Position', [60, 180, 90, 220]);
set_param([plant_root '/Mux_x1x2'], 'Inputs', '2', 'DisplayOption', 'bar');

% 非线性阻尼: μ*(1 - x₁²)*x₂
add_block('simulink/User-Defined Functions/Fcn', [plant_root '/NonlinearDamping'], ...
    'Position', [140, 180, 220, 220]);
set_param([plant_root '/NonlinearDamping'], 'Expr', sprintf('%.1f*(1-u(1)^2)*u(2)', mu));

% Out1 x 2 (Simulink 自动编号为 Out1, Out2)
add_block('simulink/Ports & Subsystems/Out1', [plant_root '/y'], ...
    'Position', [480, 80, 500, 100]);
add_block('simulink/Ports & Subsystems/Out1', [plant_root '/x2_out'], ...
    'Position', [480, 130, 500, 150]);

% --- Plant 内部连线 ---
add_line(plant_root, 'u/1', 'Sum_accel/1');
add_line(plant_root, 'Sum_accel/1', 'Int_x2/1');
add_line(plant_root, 'Int_x2/1', 'Int_x1/1');

% x₁ → Mux, x₂ → Mux → NonlinearDamping → Sum_accel
add_line(plant_root, 'Int_x1/1', 'Mux_x1x2/1');
add_line(plant_root, 'Int_x2/1', 'Mux_x1x2/2');
add_line(plant_root, 'Mux_x1x2/1', 'NonlinearDamping/1');
add_line(plant_root, 'NonlinearDamping/1', 'Sum_accel/2');

% 恢复力: -x₁ → Sum_accel (负端口)
add_line(plant_root, 'Int_x1/1', 'Sum_accel/3');

% 输出
add_line(plant_root, 'Int_x1/1', 'y/1');
add_line(plant_root, 'Int_x2/1', 'x2_out/1');

% --- 测量噪声 ---
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/MeasNoise'], ...
    'Position', [350, 80, 390, 110]);

add_block('simulink/Math Operations/Gain', [mdl '/NoiseScale'], ...
    'Position', [420, 85, 450, 110]);
set_param([mdl '/NoiseScale'], 'Gain', '0.05');

add_block('simulink/Math Operations/Add', [mdl '/Add_Noise'], ...
    'Position', [490, 85, 520, 115]);
set_param([mdl '/Add_Noise'], 'Inputs', '|++', 'IconShape', 'round');

% --- To Workspace ---
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1true'], ...
    'Position', [550, 180, 600, 205]);
set_param([mdl '/ws_x1true'], 'VariableName', 'x1_true');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2true'], ...
    'Position', [550, 220, 600, 245]);
set_param([mdl '/ws_x2true'], 'VariableName', 'x2_true');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_y_noisy'], ...
    'Position', [550, 260, 600, 285]);
set_param([mdl '/ws_y_noisy'], 'VariableName', 'y_noisy');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_u'], ...
    'Position', [550, 300, 600, 325]);
set_param([mdl '/ws_u'], 'VariableName', 'u_data');

% --- 顶层连线 ---
add_line(mdl, 'Input_u/1', 'VanDerPol_Plant/1');
add_line(mdl, 'VanDerPol_Plant/1', 'Add_Noise/1');
add_line(mdl, 'MeasNoise/1', 'NoiseScale/1');
add_line(mdl, 'NoiseScale/1', 'Add_Noise/2');

% To Workspace
add_line(mdl, 'VanDerPol_Plant/1', 'ws_x1true/1');
add_line(mdl, 'VanDerPol_Plant/2', 'ws_x2true/1');
add_line(mdl, 'Add_Noise/1', 'ws_y_noisy/1');
add_line(mdl, 'Input_u/1', 'ws_u/1');

Simulink.BlockDiagram.arrangeSystem(mdl);

% --- 保存模型 ---
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

% --- 运行仿真 ---
set_param(mdl, 'StopTime', num2str(T_final));
set_param(mdl, 'Solver', 'ode45');
set_param(mdl, 'MaxStep', num2str(Ts/2));
simOut = sim(mdl);

% 提取数据
t = simOut.tout;
x1t = getSimData(simOut, 'x1_true', t);
x2t = getSimData(simOut, 'x2_true', t);
y_m  = getSimData(simOut, 'y_noisy', t);
u_d  = getSimData(simOut, 'u_data', t);

N = length(t);

close_system(mdl, 0);

fprintf('  [OK] 模型已保存到 models/%s.slx\n', mdl);
fprintf('  [OK] 仿真完成: %d 个采样点, Ts=%.2f s\n\n', N, Ts);

%% ===== 第 2 步：MATLAB 实现 EKF 五步循环 =====

fprintf('【Step 2: EKF 离线估计 (for-loop)】\n');

% ---- 噪声参数 ----
Q = diag([0.01, 0.1]);    % 过程噪声协方差
R = 0.005;                % 测量噪声协方差

% ---- EKF 初始化 ----
x_ekf = zeros(2, N);      % 存储估计值
P_ekf = zeros(2, 2, N);   % 存储协方差
x_hat = [0; 0];           % 初始猜测（不知道真实初态 0.8, 0.5）
P = eye(2);               % 初始协方差

% ---- EKF 主循环 ----
for k = 2:N
    dt = t(k) - t(k-1);   % 步长（处理变步长情况）
    u_k = u_d(k-1);
    x1 = x_hat(1);
    x2 = x_hat(2);

    % === 1. 预报 (非线性动力学，前向 Euler) ===
    x_pred = x_hat + dt * [x2;
                            mu*(1 - x1^2)*x2 - x1 + u_k];

    % === 2. 计算离散状态转移矩阵 F = I + dt·(∂f/∂x) ===
    F = [1,                       dt;
         dt*(-2*mu*x1*x2 - 1),    1 + dt*mu*(1 - x1^2)];

    % === 3. 预报协方差 ===
    P_pred = F * P * F' + Q * dt;

    % === 4. 计算 Kalman 增益 ===
    H = [1, 0];            % 测量 Jacobian (线性测量，所以 H=C)
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;

    % === 5. 更新 (用真实测量) ===
    y_pred = x_pred(1);    % h(x) = x₁
    innovation = y_m(k) - y_pred;
    x_hat = x_pred + K * innovation;
    P = (eye(2) - K * H) * P_pred;

    % 存储
    x_ekf(:, k) = x_hat;
    P_ekf(:, :, k) = P;
end

fprintf('  [OK] EKF 循环完成 (%d 步)\n', N);

%% ===== 第 3 步：线性 Kalman 对比 (错误模型) =====

fprintf('\n【Step 3: 线性 Kalman 对比 (故意用错误模型)】\n');

% 用线性近似 A=[0 1; -1 -0.3] 的 Kalman —— 这忽略了 μ 项！
A_lin = [0, 1; -1, -0.3];
C_lin = [1, 0];
G_lin = eye(2);

% 稳态 Kalman 增益
[L_kal, ~, ~] = lqe(A_lin, G_lin, C_lin, Q, R * Ts);

% 线性 Kalman 观测器
x_kal = zeros(2, N);
x_hat_kal = [0; 0];
for k = 2:N
    dt = t(k) - t(k-1);
    u_k = u_d(k-1);

    % 线性预报 (用错误的 A 矩阵!)
    x_pred_kal = x_hat_kal + dt * (A_lin * x_hat_kal + [0; 1] * u_k);

    % 用稳态增益更新
    y_pred_kal = C_lin * x_pred_kal;
    x_hat_kal = x_pred_kal + L_kal * (y_m(k) - y_pred_kal(1));

    x_kal(:, k) = x_hat_kal;
end

fprintf('  [OK] 线性 Kalman 完成\n\n');

%% ===== 第 4 步：绘图分析 =====

figure('Name', 't31: EKF vs 线性 Kalman (Van der Pol 振子)', ...
    'Position', [50, 50, 1100, 900]);

% --- 子图 1：位移估计对比 ---
subplot(4, 1, 1);
plot(t, x1t, 'k', 'LineWidth', 2); hold on;
plot(t, x_kal(1,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ekf(1,:)', 'r-', 'LineWidth', 1.5); hold off;
legend('真实 x₁', '线性 Kalman', 'EKF', 'Location', 'southeast');
title('位移估计 — EKF 紧密跟踪，线性 Kalman 在大振幅时偏离');
xlabel('时间 (s)'); ylabel('位移'); grid on;

rmse_x1_k = sqrt(mean((x1t - x_kal(1,:)').^2, 'omitnan'));
rmse_x1_e = sqrt(mean((x1t - x_ekf(1,:)').^2, 'omitnan'));
fprintf('  位移 RMSE:  线性 Kalman = %.4f,  EKF = %.4f\n', rmse_x1_k, rmse_x1_e);

% --- 子图 2：速度估计对比 ---
subplot(4, 1, 2);
plot(t, x2t, 'k', 'LineWidth', 2); hold on;
plot(t, x_kal(2,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ekf(2,:)', 'r-', 'LineWidth', 1.5); hold off;
legend('真实 x₂', '线性 Kalman', 'EKF', 'Location', 'southeast');
title('速度估计 — 线性 Kalman 相位滞后严重，EKF 正确跟踪极限环');
xlabel('时间 (s)'); ylabel('速度'); grid on;

rmse_x2_k = sqrt(mean((x2t - x_kal(2,:)').^2, 'omitnan'));
rmse_x2_e = sqrt(mean((x2t - x_ekf(2,:)').^2, 'omitnan'));
fprintf('  速度 RMSE:  线性 Kalman = %.4f,  EKF = %.4f\n', rmse_x2_k, rmse_x2_e);

% --- 子图 3：估计误差对比 ---
subplot(4, 1, 3);
plot(t, x1t - x_kal(1,:)', 'b-', 'LineWidth', 0.8); hold on;
plot(t, x1t - x_ekf(1,:)', 'r-', 'LineWidth', 1.2); hold off;
legend('线性 Kalman 误差', 'EKF 误差', 'Location', 'southeast');
title(sprintf('估计误差 e₁ = x₁ - x̂₁ — EKF 改善 %.0f%%', ...
    100*(rmse_x1_k - rmse_x1_e)/rmse_x1_k));
xlabel('时间 (s)'); ylabel('位移误差'); grid on;

% --- 子图 4：滤波效果 ---
subplot(4, 1, 4);
plot(t, y_m, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5); hold on;
plot(t, x1t, 'k', 'LineWidth', 2);
plot(t, x_ekf(1,:)', 'r-', 'LineWidth', 1.5); hold off;
legend('含噪测量 y', '真实 x₁', 'EKF 估计', 'Location', 'southeast');
title(sprintf('滤波效果 — σ_v=%.2f 噪声下 EKF 恢复真实信号', sqrt(R)));
xlabel('时间 (s)'); ylabel('幅值'); grid on;

sgtitle('教程 31：扩展 Kalman 滤波 — 在非线性系统中估计状态');

%% ===== 第 5 步：理论总结 =====

fprintf('\n========================================\n');
fprintf('  教程 31 完成！\n');
fprintf('========================================\n\n');

fprintf('【理论总结】\n\n');

fprintf('  1. EKF = Kalman + 局部线性化\n');
fprintf('     标准 Kalman 用固定的 A, C 矩阵\n');
fprintf('     EKF 每步重新算 F=∂f/∂x, H=∂h/∂x\n');
fprintf('     本质：在 x̂ 处把非线性函数"掰直"了再算\n\n');

fprintf('  2. 你的结果说明了什么\n');
fprintf('     Van der Pol 振子 (μ=%.1f) 有稳定的极限环\n', mu);
fprintf('     线性 Kalman 用 A=[0 1; -1 -0.3] 近似 → 在大振幅时误差明显\n');
fprintf('     EKF 每步局部线性化 → 全程正确跟踪\n');
fprintf('     x₁ RMSE: Kalman %.4f → EKF %.4f (改善 %.0f%%)\n', ...
    rmse_x1_k, rmse_x1_e, 100*(rmse_x1_k - rmse_x1_e)/rmse_x1_k);
fprintf('     x₂ RMSE: Kalman %.4f → EKF %.4f (改善 %.0f%%)\n\n', ...
    rmse_x2_k, rmse_x2_e, 100*(rmse_x2_k - rmse_x2_e)/rmse_x2_k);

fprintf('  3. EKF 五步循环回顾\n');
fprintf('     打开本脚本，搜索 "EKF 主循环"\n');
fprintf('     看到 for k=2:N ... 这一段\n');
fprintf('     ├─ 1. 非线性预报：x_pred = x + dt*f(x,u)\n');
fprintf('     ├─ 2. 算 Jacobian：F = ∂f/∂x|x̂\n');
fprintf('     ├─ 3. 预报协方差：P_pred = F*P*F'' + Q*dt\n');
fprintf('     ├─ 4. 算 Kalman 增益：K = P_pred*H''/(H*P_pred*H''+R)\n');
fprintf('     └─ 5. 更新：x̂ = x_pred + K*(y - h(x_pred))\n\n');

fprintf('  4. EKF 的局限\n');
fprintf('     - 需要解析计算 Jacobian（复杂系统很痛苦）\n');
fprintf('     - 一阶泰勒展开 ≈ 强非线性时精度不够\n');
fprintf('     - P 矩阵可能因线性化误差而发散\n');
fprintf('     - 解决方案: UKF (t33) — 不用 Jacobian，用采样点！\n\n');

fprintf('  5. 工程应用\n');
fprintf('     - GPS/IMU 融合 — 非线性测量方程\n');
fprintf('     - 机器人 SLAM — 非线性运动+观测模型\n');
fprintf('     - 弹道跟踪 — 非线性弹道方程\n');
fprintf('     - 电池 SOC 估计 — 非线性电化学模型\n\n');

fprintf('【动手实验】\n\n');
fprintf('  1. 改 μ 值看效果\n');
fprintf('     μ=0.2: 接近线性 → 两个滤波器差不多\n');
fprintf('     μ=3.0: 强非线性 → 线性 Kalman 误差爆炸，EKF 依然坚挺\n\n');
fprintf('  2. 增大 R (传感器更差)\n');
fprintf('     R=0.05 → EKF 更信任模型，估计更平滑但跟踪变慢\n');
fprintf('     R=0.001 → EKF 几乎直接抄测量值\n\n');
fprintf('  3. 找到本脚本的 EKF 主循环 (for k=2:N)\n');
fprintf('     逐行对照讲义中的五步方程\n');
fprintf('     把 F 的计算改成手算结果，验证理解\n\n');

fprintf('  下一课预告：t32 — EKF 实战：倒立摆状态估计\n');
fprintf('  更多非线性 + 处理离散化精度\n');
