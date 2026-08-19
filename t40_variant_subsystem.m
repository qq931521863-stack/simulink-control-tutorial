%% ============================================================
% 教程 40：Variant Subsystem — 一个模型塞下多套方案
%
% 【为什么要学这课】
%   你设计了三种控制器——PID、LQR、SMC。想对比效果？
%   常规做法：复制三个模型，每个换一个控制器 → 改一处要同步三处 → 灾难。
%
%   Variant Subsystem 让你在一个模型里放三套控制方案，
%   通过一个变量切换——不用复制粘贴，不用手动改连线。
%
% ┌─────────────────────────────────────────────────────────┐
% │ Variant Subsystem 结构                                  │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   Variant Subsystem (ctrl_type == 1 → PID)              │
% │                    (ctrl_type == 2 → LQR)               │
% │                    (ctrl_type == 3 → SMC)               │
% │                                                         │
% │   输入 e → [PID block] ← ctrl_type=1 时激活              │
% │         → [LQR block] ← ctrl_type=2 时激活              │
% │         → [SMC block] ← ctrl_type=3 时激活              │
% │         → 输出 u                                        │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 用 Variant Subsystem 在一个模型里放 3 种控制器
%   2. 通过 workspace 变量切换激活哪个
%   3. 一键对比三种方案的效果
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 40：Variant Subsystem\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：搭 Plant + Variant 控制器 =====

mdl = 'tutorial40_variant';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 二阶 Plant (和 t05 一样)
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], ...
    'Position', [400, 60, 500, 110]);
set_param([mdl '/Plant'], 'Numerator', '[1]', 'Denominator', '[1 0.5 1]');

add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [30, 70, 90, 100]);
set_param([mdl '/Setpoint'], 'Time', '0.5', 'After', '1');

add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Position', [120, 65, 150, 95]);
set_param([mdl '/Error'], 'Inputs', '|+-');

% Variant Subsystem
add_block('simulink/Ports & Subsystems/Variant Subsystem', ...
    [mdl '/Controller'], 'Position', [220, 40, 320, 130]);

% 设置 Variant 条件 (在 Variant Subsystem 内部用 Variant choices)
% 先把内部的空 Subsystem 换成三个带条件的 Subsystem
sub_root = [mdl '/Controller'];

% 删除默认的 variant choice（保留顶层 In1/Out1 作为接口端口）
delete_block([sub_root '/Subsystem']);

% PID 方案
add_block('simulink/Ports & Subsystems/Subsystem', [sub_root '/PID'], ...
    'Position', [120, 20, 200, 100]);
Simulink.SubSystem.deleteContents([sub_root '/PID']);
pid_root = [sub_root '/PID'];
add_block('simulink/Ports & Subsystems/In1', [pid_root '/In1'], 'Position', [30,50,50,70]);
add_block('simulink/Math Operations/Gain', [pid_root '/Kp'], 'Position', [100,50,140,80]);
set_param([pid_root '/Kp'], 'Gain', '3');
add_block('simulink/Ports & Subsystems/Out1', [pid_root '/Out1'], 'Position', [230,50,250,70]);
add_line(pid_root, 'In1/1', 'Kp/1');
add_line(pid_root, 'Kp/1', 'Out1/1');
set_param([sub_root '/PID'], 'VariantControl', 'ctrl_type == 1');

% LQR 方案 (简化: 放大增益)
add_block('simulink/Ports & Subsystems/Subsystem', [sub_root '/LQR'], ...
    'Position', [120, 120, 200, 200]);
Simulink.SubSystem.deleteContents([sub_root '/LQR']);
lqr_root = [sub_root '/LQR'];
add_block('simulink/Ports & Subsystems/In1', [lqr_root '/In1'], 'Position', [30,50,50,70]);
add_block('simulink/Math Operations/Gain', [lqr_root '/K_lqr'], 'Position', [100,50,140,80]);
set_param([lqr_root '/K_lqr'], 'Gain', '5');
add_block('simulink/Ports & Subsystems/Out1', [lqr_root '/Out1'], 'Position', [230,50,250,70]);
add_line(lqr_root, 'In1/1', 'K_lqr/1');
add_line(lqr_root, 'K_lqr/1', 'Out1/1');
set_param([sub_root '/LQR'], 'VariantControl', 'ctrl_type == 2');

% SMC 方案 (简化: 饱和)
add_block('simulink/Ports & Subsystems/Subsystem', [sub_root '/SMC'], ...
    'Position', [250, 20, 330, 100]);
Simulink.SubSystem.deleteContents([sub_root '/SMC']);
smc_root = [sub_root '/SMC'];
add_block('simulink/Ports & Subsystems/In1', [smc_root '/In1'], 'Position', [30,50,50,70]);
add_block('simulink/Math Operations/Gain', [smc_root '/K_smc'], 'Position', [100,50,140,80]);
set_param([smc_root '/K_smc'], 'Gain', '8');
add_block('simulink/Discontinuities/Saturation', [smc_root '/Sat'], ...
    'Position', [190,50,230,80]);
set_param([smc_root '/Sat'], 'UpperLimit', '5', 'LowerLimit', '-5');
add_block('simulink/Ports & Subsystems/Out1', [smc_root '/Out1'], 'Position', [310,50,330,70]);
add_line(smc_root, 'In1/1', 'K_smc/1');
add_line(smc_root, 'K_smc/1', 'Sat/1');
add_line(smc_root, 'Sat/1', 'Out1/1');
set_param([sub_root '/SMC'], 'VariantControl', 'ctrl_type == 3');

% Variant Subsystem 接口端口自动路由（In1/Out1 同名匹配），无需内部连线

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_y'], ...
    'Position', [550, 60, 600, 85]);
set_param([mdl '/ws_y'], 'VariableName', 'y_data');

% 顶层连线
add_line(mdl, 'Setpoint/1', 'Error/1');
add_line(mdl, 'Error/1', 'Controller/1');
add_line(mdl, 'Controller/1', 'Plant/1');
add_line(mdl, 'Plant/1', 'Error/2');
add_line(mdl, 'Plant/1', 'ws_y/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 2 步：三种方案对比 =====

fprintf('【Step 2: 依次运行三种控制器】\n\n');

ctrl_types = [1, 2, 3];
ctrl_names = {'PID', 'LQR', 'SMC'};
K_gains  = {'3', '5', '8'};
colors = {'b', 'r', 'g'};

figure('Name', 't40: Variant Subsystem — 三种控制器对比', ...
    'Position', [50, 50, 900, 600]);

for ct = 1:3
    assignin('base', 'ctrl_type', ctrl_types(ct));
    set_param(mdl, 'StopTime', '5');
    simOut = sim(mdl);
    t = simOut.tout;
    y = getSimData(simOut, 'y_data', t);

    subplot(2,2,ct);
    plot(t, y, colors{ct}, 'LineWidth', 2); hold on;
    yline(1, 'k--');
    title(sprintf('%s (ctrl_type=%d, K=%s)', ctrl_names{ct}, ctrl_types(ct), ...
        K_gains{ct}));
    xlabel('Time (s)'); ylabel('Output'); grid on;

    fprintf('  %s: 稳态误差 = %.3f\n', ctrl_names{ct}, abs(1 - y(end)));
end

% 合并对比
subplot(2,2,4);
for ct = 1:3
    assignin('base', 'ctrl_type', ctrl_types(ct));
    simOut = sim(mdl);
    t = simOut.tout;
    y = getSimData(simOut, 'y_data', t);
    plot(t, y, colors{ct}, 'LineWidth', 1.5); hold on;
end
yline(1, 'k--');
legend(ctrl_names, 'Location', 'southeast');
title('三种控制器叠加对比');
xlabel('Time (s)'); ylabel('Output'); grid on;

close_system(mdl, 0);

sgtitle('教程 40：Variant Subsystem — 一键切换三套控制方案');

fprintf('\n========================================\n');
fprintf('  教程 40 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Variant Subsystem = 可控的选择器\n');
fprintf('     ctrl_type=1 → PID, =2 → LQR, =3 → SMC\n');
fprintf('     → 不用改模型，只改变量就切换\n\n');
fprintf('  2. VariantControl 条件\n');
fprintf('     set_param(block, ''VariantControl'', ''ctrl_type == 1'')\n');
fprintf('     → 当条件为 true 时该 variant 被激活\n\n');
fprintf('  3. 工程用法\n');
fprintf('     - A/B 测试不同控制方案\n');
fprintf('     - 同一个模型适配不同硬件配置\n');
fprintf('     - 仿真用简化版, 部署用精确版\n');
