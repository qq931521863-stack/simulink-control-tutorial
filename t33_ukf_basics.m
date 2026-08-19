%% ============================================================
% 教程 33：无迹 Kalman 滤波 (UKF) — 不靠 Jacobian 的非线性滤波
%
% 【为什么要学这课】
%   EKF 做了个一阶泰勒展开 ≈ 把非线性函数在一点处"掰直"
%   问题：强非线性时，一阶不够 → 需要二阶甚至更高
%
%   UKF 换了一个完全不同的思路：
%     不线性化函数 → 而是用一组"采样点"（Sigma Points）
%     把这组点通过非线性函数"扔过去"，看它们变成了什么分布
%     → 从变换后的点云直接算出均值和协方差
%
% ┌─────────────────────────────────────────────────────────┐
% │ UKF 的核心直觉                                          │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   EKF: 把函数掰直 → 走一步 → 误差是线性化误差           │
% │   UKF: 在均值周围撒 2n+1 个点 → 全部扔过非线性函数      │
% │         → 看这些点组成的"云"变成什么形状                 │
% │         → 不需要 Jacobian！                              │
% │                                                         │
% │   类比：EKF 是"用一个向量做侦察"，UKF 是"派一小队人"     │
% │         小队的分布比一个人的猜测更准确                   │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ UKF 算法流程                                            │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   已知: x̂(t), P(t)                                      │
% │                                                         │
% │   Step 1: 生成 Sigma 点                                 │
% │     X₀ = x̂                                               │
% │     Xᵢ = x̂ + (√((n+λ)P))ᵢ     (i=1..n)                  │
% │     Xᵢ₊ₙ = x̂ - (√((n+λ)P))ᵢ   (i=1..n)                  │
% │     一共 2n+1 个点                                       │
% │                                                         │
% │   Step 2: 预报 — 每个 Sigma 点通过 f(x)                  │
% │     Yᵢ = f(Xᵢ)  for i=0..2n                             │
% │     x̂⁻ = Σ Wᵢᵐ Yᵢ     (加权平均 = 预报均值)             │
% │     P⁻ = Σ Wᵢᶜ (Yᵢ - x̂⁻)(Yᵢ - x̂⁻)' + Q                 │
% │                                                         │
% │   Step 3: 更新 — 每个预报点通过 h(x)                     │
% │     Zᵢ = h(Yᵢ)  for i=0..2n                             │
% │     ẑ = Σ Wᵢᵐ Zᵢ     (预测测量均值)                     │
% │     P_zz = Σ Wᵢᶜ (Zᵢ - ẑ)(Zᵢ - ẑ)' + R                 │
% │     P_xz = Σ Wᵢᶜ (Yᵢ - x̂⁻)(Zᵢ - ẑ)'                    │
% │     K = P_xz / P_zz                                      │
% │     x̂ = x̂⁻ + K(y - ẑ)                                   │
% │     P = P⁻ - K P_zz K'                                   │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 理解 UKF 的"采样点替代线性化"思想
%   2. 手写 UKF 完整流程（Sigma 点生成 + Unscented 变换）
%   3. 和 EKF 对比：UKF 精度更高，但计算量更大
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统定义（和 t31 一样：Van der Pol 振子）=====

mu = 2.0;              % 比 t31 更强非线性 (μ=2.0 vs 1.5)
Ts = 0.02; T_final = 15;

fprintf('============================================\n');
fprintf('  教程 33：无迹 Kalman 滤波 (UKF)\n');
fprintf('============================================\n\n');

fprintf('【系统：Van der Pol 振子 (μ=%.1f, 强非线性)】\n', mu);
fprintf('  UKF 不需要 Jacobian → 强非线性时比 EKF 更准\n');
fprintf('  代价：每步要算 2n+1 = %d 个点\n\n', 2*2+1);

%% ===== 第 1 步：Simulink 采集数据（和 t31 一样的 Plant）=====

fprintf('【Step 1: Simulink 采集数据】\n');

mdl = 'tutorial33_ukf';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

add_block('simulink/Sources/Sine Wave', [mdl '/Input_u'], ...
    'Position', [50, 130, 110, 170]);
set_param([mdl '/Input_u'], 'Amplitude', '0.5', 'Frequency', '0.8', 'Bias', '0');

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/VanDerPol_Plant'], ...
    'Position', [200, 70, 300, 160]);
Simulink.SubSystem.deleteContents([mdl '/VanDerPol_Plant']);
plant_root = [mdl '/VanDerPol_Plant'];

add_block('simulink/Ports & Subsystems/In1', [plant_root '/u'], 'Position', [30,80,50,100]);
add_block('simulink/Math Operations/Sum', [plant_root '/Sum_accel'], ...
    'Position', [120,80,150,130]);
set_param([plant_root '/Sum_accel'], 'Inputs', '|++-', 'IconShape', 'round');
add_block('simulink/Continuous/Integrator', [plant_root '/Int_x2'], ...
    'Position', [220,85,270,115]);
set_param([plant_root '/Int_x2'], 'InitialCondition', '0.5');
add_block('simulink/Continuous/Integrator', [plant_root '/Int_x1'], ...
    'Position', [340,85,390,115]);
set_param([plant_root '/Int_x1'], 'InitialCondition', '0.8');

add_block('simulink/Signal Routing/Mux', [plant_root '/Mux_x1x2'], ...
    'Position', [60,180,90,220]);
set_param([plant_root '/Mux_x1x2'], 'Inputs', '2', 'DisplayOption', 'bar');
add_block('simulink/User-Defined Functions/Fcn', [plant_root '/NonlinearDamping'], ...
    'Position', [140,180,220,220]);
set_param([plant_root '/NonlinearDamping'], 'Expr', sprintf('%.1f*(1-u(1)^2)*u(2)', mu));

add_block('simulink/Ports & Subsystems/Out1', [plant_root '/y'], ...
    'Position', [480,80,500,100]);
add_block('simulink/Ports & Subsystems/Out1', [plant_root '/x2_out'], ...
    'Position', [480,130,500,150]);

add_line(plant_root, 'u/1', 'Sum_accel/1');
add_line(plant_root, 'Sum_accel/1', 'Int_x2/1');
add_line(plant_root, 'Int_x2/1', 'Int_x1/1');
add_line(plant_root, 'Int_x1/1', 'Mux_x1x2/1');
add_line(plant_root, 'Int_x2/1', 'Mux_x1x2/2');
add_line(plant_root, 'Mux_x1x2/1', 'NonlinearDamping/1');
add_line(plant_root, 'NonlinearDamping/1', 'Sum_accel/2');
add_line(plant_root, 'Int_x1/1', 'Sum_accel/3');
add_line(plant_root, 'Int_x1/1', 'y/1');
add_line(plant_root, 'Int_x2/1', 'x2_out/1');

% 噪声
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/MeasNoise'], ...
    'Position', [340,80,380,110]);
add_block('simulink/Math Operations/Gain', [mdl '/NoiseScale'], ...
    'Position', [410,85,440,110]);
set_param([mdl '/NoiseScale'], 'Gain', '0.05');
add_block('simulink/Math Operations/Add', [mdl '/Add_Noise'], ...
    'Position', [480,85,510,115]);
set_param([mdl '/Add_Noise'], 'Inputs', '|++', 'IconShape', 'round');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1true'], ...
    'Position', [540,180,590,205]);
set_param([mdl '/ws_x1true'], 'VariableName', 'x1_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2true'], ...
    'Position', [540,220,590,245]);
set_param([mdl '/ws_x2true'], 'VariableName', 'x2_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_y_noisy'], ...
    'Position', [540,260,590,285]);
set_param([mdl '/ws_y_noisy'], 'VariableName', 'y_noisy');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_u'], ...
    'Position', [540,300,590,325]);
set_param([mdl '/ws_u'], 'VariableName', 'u_data');

add_line(mdl, 'Input_u/1', 'VanDerPol_Plant/1');
add_line(mdl, 'VanDerPol_Plant/1', 'Add_Noise/1');
add_line(mdl, 'MeasNoise/1', 'NoiseScale/1');
add_line(mdl, 'NoiseScale/1', 'Add_Noise/2');
add_line(mdl, 'VanDerPol_Plant/1', 'ws_x1true/1');
add_line(mdl, 'VanDerPol_Plant/2', 'ws_x2true/1');
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

%% ===== 第 2 步：UKF 实现 =====

fprintf('【Step 2: UKF 估计】\n');

% UKF 参数
n = 2;                           % 状态维度
alpha = 1e-3;                    % Sigma 点散布参数 (小→靠近均值)
beta = 2;                        % 高斯分布最优值
kappa = 0;                       % 辅助缩放参数
lambda = alpha^2 * (n + kappa) - n;

% 权值
Wm = zeros(2*n+1, 1);            % 均值权值
Wc = zeros(2*n+1, 1);            % 协方差权值
Wm(1) = lambda / (n + lambda);
Wc(1) = Wm(1) + (1 - alpha^2 + beta);
for i = 2:(2*n+1)
    Wm(i) = 1 / (2*(n + lambda));
    Wc(i) = 1 / (2*(n + lambda));
end

% 噪声参数
Q = diag([0.01, 0.1]);
R = 0.005;

% UKF 状态初始化
x_ukf = zeros(n, N);
x_hat = [0; 0];
P = eye(n);

for k = 2:N
    dt = t(k) - t(k-1);
    u_k = u_d(k-1);

    % === 1. 生成 Sigma 点 ===
    L = sqrt(n + lambda);
    sqrtP = L * chol(P, 'lower');  % Cholesky: P = sqrtP * sqrtP'
    X = [x_hat, x_hat + sqrtP, x_hat - sqrtP];  % [X₀, X₁..Xₙ, Xₙ₊₁..X₂ₙ]: n × (2n+1)

    % === 2. 预报: 每个 Sigma 点通过 f(x) ===
    Y = zeros(n, 2*n+1);
    for i = 1:(2*n+1)
        x1 = X(1, i); x2 = X(2, i);
        Y(:, i) = X(:, i) + dt * [x2; mu*(1-x1^2)*x2 - x1 + u_k];
    end
    % 加权平均
    x_pred = Y * Wm;
    % 加权协方差
    P_pred = Q * dt;
    for i = 1:(2*n+1)
        e = Y(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (e * e');
    end

    % === 3. 更新: 每个预报点通过 h(x) ===
    Z = Y(1, :);  % h(x) = x₁ (线性测量)
    z_pred = Z * Wm;
    P_zz = R;
    P_xz = zeros(n, 1);
    for i = 1:(2*n+1)
        e_z = Z(i) - z_pred;
        e_x = Y(:, i) - x_pred;
        P_zz = P_zz + Wc(i) * e_z^2;
        P_xz = P_xz + Wc(i) * e_x * e_z;
    end

    % === 4. Kalman 增益 + 更新 ===
    K = P_xz / P_zz;
    x_hat = x_pred + K * (y_m(k) - z_pred);
    P = P_pred - K * P_zz * K';

    x_ukf(:, k) = x_hat;
end

fprintf('  [OK] UKF 循环完成 (%d 步)\n', N);

%% ===== 第 3 步：EKF 对比 =====

fprintf('\n【Step 3: EKF 对比】\n');

x_ekf = zeros(2, N);
x_hat_ekf = [0; 0];
P_ekf = eye(2);
for k = 2:N
    dt = t(k) - t(k-1);
    u_k = u_d(k-1);
    x1 = x_hat_ekf(1); x2 = x_hat_ekf(2);

    x_pred = x_hat_ekf + dt * [x2; mu*(1-x1^2)*x2 - x1 + u_k];
    F = [1, dt; dt*(-2*mu*x1*x2 - 1), 1 + dt*mu*(1-x1^2)];
    P_pred = F * P_ekf * F' + Q * dt;
    H = [1, 0];
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    y_pred = x_pred(1);
    x_hat_ekf = x_pred + K * (y_m(k) - y_pred);
    P_ekf = (eye(2) - K * H) * P_pred;
    x_ekf(:, k) = x_hat_ekf;
end

fprintf('  [OK] EKF 完成\n\n');

%% ===== 第 4 步：绘图 =====

figure('Name', 't33: UKF vs EKF', 'Position', [50, 50, 1100, 900]);

subplot(4,1,1);
plot(t, x1t, 'k', 'LineWidth', 2); hold on;
plot(t, x_ekf(1,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ukf(1,:)', 'g-', 'LineWidth', 1.5); hold off;
legend('真实 x₁', 'EKF', 'UKF', 'Location', 'southeast');
title('位移估计 — UKF vs EKF (μ=2.0 强非线性)');
xlabel('时间 (s)'); ylabel('位移'); grid on;

rmse_e = sqrt(mean((x1t - x_ekf(1,:)').^2, 'omitnan'));
rmse_u = sqrt(mean((x1t - x_ukf(1,:)').^2, 'omitnan'));
fprintf('  x₁ RMSE:  EKF = %.4f,  UKF = %.4f\n', rmse_e, rmse_u);

subplot(4,1,2);
plot(t, x2t, 'k', 'LineWidth', 2); hold on;
plot(t, x_ekf(2,:)', 'b--', 'LineWidth', 1.2);
plot(t, x_ukf(2,:)', 'g-', 'LineWidth', 1.5); hold off;
legend('真实 x₂', 'EKF', 'UKF', 'Location', 'southeast');
title('速度估计');
xlabel('时间 (s)'); ylabel('速度'); grid on;

rmse_e2 = sqrt(mean((x2t - x_ekf(2,:)').^2, 'omitnan'));
rmse_u2 = sqrt(mean((x2t - x_ukf(2,:)').^2, 'omitnan'));
fprintf('  x₂ RMSE:  EKF = %.4f,  UKF = %.4f\n', rmse_e2, rmse_u2);

subplot(4,1,3);
plot(t, x1t - x_ekf(1,:)', 'b-', 'LineWidth', 0.8); hold on;
plot(t, x1t - x_ukf(1,:)', 'g-', 'LineWidth', 1.2); hold off;
legend('EKF 误差', 'UKF 误差', 'Location', 'southeast');
title(sprintf('x₁ 误差对比 — UKF 改善 %.1f%%', ...
    max(0, 100*(rmse_e - rmse_u)/rmse_e)));
xlabel('时间 (s)'); ylabel('位移误差'); grid on;

% Sigma 点可视化 (最后一步)
subplot(4,1,4);
L = sqrt(n + lambda);
sqrtP = L * chol(P, 'lower');
X_last = [x_ukf(:,end), x_ukf(:,end) + sqrtP, x_ukf(:,end) - sqrtP];
plot(X_last(1,:), X_last(2,:), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g'); hold on;
plot(x_ukf(1,end), x_ukf(2,end), 'r*', 'MarkerSize', 12);
ellipse(x_ukf(1,end), x_ukf(2,end), P(1,1), P(1,2), P(2,2)); hold off;
title(sprintf('最后一步的 2n+1=%d 个 Sigma 点(绿) + 均值(红*) + P椭圆', 2*n+1));
xlabel('x₁'); ylabel('x₂'); grid on; axis equal;

sgtitle('教程 33：UKF — 无迹 Kalman 滤波 vs EKF');

%% ===== 第 5 步：总结 =====

fprintf('\n========================================\n');
fprintf('  教程 33 完成！\n');
fprintf('========================================\n\n');

fprintf('【关键收获】\n\n');
fprintf('  1. UKF 不需要 Jacobian — 避免了求导的麻烦和线性化误差\n');
fprintf('     Sigma 点直接"穿"过非线性函数，保留了高阶信息\n\n');
fprintf('  2. 计算量：UKF 每步算 %d 次 f(x) vs EKF 算 1 次 + Jacobian\n', 2*n+1);
fprintf('     对 2 阶系统差异不大，但对 10+ 状态系统 EKF 更快\n\n');
fprintf('  3. UKF 的精度优势在强非线性时才明显\n');
fprintf('     μ=1.5: EKF 和 UKF 差不多\n');
fprintf('     μ=2.0: UKF 开始显示优势\n');
fprintf('     μ=3.0: UKF 明显优于 EKF\n\n');

fprintf('  4. 工程选型建议\n');
fprintf('     - 弱非线性 + 可求导 → EKF（更快）\n');
fprintf('     - 强非线性 → UKF（更准）\n');
fprintf('     - 高维系统 → EKF（UKF 的 2n+1 代价太大）\n');
fprintf('     - 不能求导 → UKF（唯一选择）\n\n');

fprintf('【动手实验】\n');
fprintf('  1. 调大 μ=3.0 → 看 UKF vs EKF 差距\n');
fprintf('  2. 改 alpha=0.1 → Sigma 点散得更开 → 捕捉更大范围的非线性\n');
fprintf('  3. 比较 2n+1=5 个点 vs Monte Carlo 1000 个点 → 理解 UKF 的效率\n\n');

fprintf('  下一课预告：t34 — EKF vs UKF 系统对比\n');
fprintf('  同一个非线性系统，两个滤波器，全面 RMSE 对比\n');

%% ===== 辅助：绘制误差椭圆 =====
function ellipse(mx, my, Pxx, Pxy, Pyy)
    theta = linspace(0, 2*pi, 100);
    P_mat = [Pxx, Pxy; Pxy, Pyy];
    [V, D] = eig(P_mat);
    s = 2;  % 2σ 椭圆
    a = s*sqrt(D(1,1)); b = s*sqrt(D(2,2));
    xy = V * [a*cos(theta); b*sin(theta)];
    plot(xy(1,:)+mx, xy(2,:)+my, 'r--', 'LineWidth', 1);
end
