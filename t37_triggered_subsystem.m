%% ============================================================
% 教程 37：触发与使能子系统 — 让控制器只在需要时运行
%
% 【为什么要学这课】
%   你之前搭的控制器是连续运行的——每个仿真步长都计算一次。
%   真实嵌入式系统不是这样的：
%     - 控制算法每 10ms 跑一次（定时中断）
%     - 数据采集只在收到触发信号时执行
%     - 某些安全模块只在使能条件下才运行
%
%   Simulink 的 Triggered Subsystem 和 Enabled Subsystem
%   就是模拟这些"条件执行"行为的。
%
% ┌─────────────────────────────────────────────────────────┐
% │ 三种条件执行对比                                         │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │ Enabled:    enable=1 → 运行, enable=0 → 保持输出不变    │
% │ Triggered:  触发沿(上升/下降/任意)来 → 执行一次         │
% │ Function-Call: 外部调用 → 执行一次                       │
% │                                                         │
% │ 本课同时演示三种：                                       │
% │   - Enabled: 控制器只在使能时计算                        │
% │   - Triggered (上升沿): 模拟 ADC 采样保持                │
% │   - Triggered (函数调用): 模拟 10ms 定时中断             │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 理解 Enabled vs Triggered vs 连续执行的区别
%   2. 用 Triggered Subsystem 实现离散采样保持
%   3. 用 Enabled Subsystem 实现条件激活的控制器
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 37：触发与使能子系统\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：搭对比模型 =====

mdl = 'tutorial37_triggered';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 共享的输入信号：正弦波
add_block('simulink/Sources/Sine Wave', [mdl '/Signal'], ...
    'Position', [30, 100, 90, 130]);
set_param([mdl '/Signal'], 'Amplitude', '1', 'Frequency', '2');

% 触发信号：10 Hz 方波 (模拟定时中断)
add_block('simulink/Sources/Pulse Generator', [mdl '/Trigger_10Hz'], ...
    'Position', [30, 40, 100, 70]);
set_param([mdl '/Trigger_10Hz'], 'Amplitude', '1', 'Period', '0.1', ...
    'PulseWidth', '50', 'PhaseDelay', '0');

% 使能信号：前半段开，后半段关
add_block('simulink/Sources/Step', [mdl '/Enable_sig'], ...
    'Position', [30, 180, 90, 210]);
set_param([mdl '/Enable_sig'], 'Time', '2', 'Before', '1', 'After', '0');

% ===== 1. 连续系统 (基准对比) =====
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Cont_Filter'], ...
    'Position', [200, 90, 280, 140]);
set_param([mdl '/Cont_Filter'], 'Numerator', '[1]', 'Denominator', '[0.1 1]');

% ===== 2. Enabled Subsystem =====
add_block('simulink/Ports & Subsystems/Enabled Subsystem', ...
    [mdl '/Enabled_Filter'], 'Position', [200, 250, 280, 310]);
Simulink.SubSystem.deleteContents([mdl '/Enabled_Filter']);
ef = [mdl '/Enabled_Filter'];

add_block('simulink/Ports & Subsystems/In1', [ef '/in'], 'Position', [30,50,50,70]);
add_block('simulink/Ports & Subsystems/Enable', [ef '/Enable'], 'Position', [30,130,50,150]);
add_block('simulink/Continuous/Transfer Fcn', [ef '/TF'], ...
    'Position', [150,40,250,90]);
set_param([ef '/TF'], 'Numerator', '[1]', 'Denominator', '[0.1 1]');
add_block('simulink/Ports & Subsystems/Out1', [ef '/out'], 'Position', [350,50,370,70]);
add_line(ef, 'in/1', 'TF/1');
add_line(ef, 'TF/1', 'out/1');

% Enabled Subsystem 参数: 使能时重置状态

% ===== 3. Triggered Subsystem (上升沿, 采样保持) =====
add_block('simulink/Ports & Subsystems/Triggered Subsystem', ...
    [mdl '/Triggered_Sample'], 'Position', [450, 40, 530, 100]);
Simulink.SubSystem.deleteContents([mdl '/Triggered_Sample']);
ts = [mdl '/Triggered_Sample'];

add_block('simulink/Ports & Subsystems/In1', [ts '/in'], 'Position', [30,50,50,70]);
add_block('simulink/Ports & Subsystems/Trigger', [ts '/Trigger'], 'Position', [30,130,50,150]);
add_block('simulink/Ports & Subsystems/Out1', [ts '/out'], 'Position', [150,50,170,70]);
add_line(ts, 'in/1', 'out/1');
set_param([ts '/Trigger'], 'TriggerType', 'rising');

% ===== To Workspace =====
add_block('simulink/Sinks/To Workspace', [mdl '/ws_cont'], ...
    'Position', [650, 90, 700, 115]);
set_param([mdl '/ws_cont'], 'VariableName', 'cont_out');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_en'], ...
    'Position', [650, 250, 700, 275]);
set_param([mdl '/ws_en'], 'VariableName', 'en_out');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_trig'], ...
    'Position', [650, 40, 700, 65]);
set_param([mdl '/ws_trig'], 'VariableName', 'trig_out');

% 顶层连线
add_line(mdl, 'Signal/1', 'Cont_Filter/1');
add_line(mdl, 'Signal/1', 'Enabled_Filter/1');
add_line(mdl, 'Signal/1', 'Triggered_Sample/1');
add_line(mdl, 'Enable_sig/1', 'Enabled_Filter/Enable');
add_line(mdl, 'Trigger_10Hz/1', 'Triggered_Sample/Trigger');

add_line(mdl, 'Cont_Filter/1', 'ws_cont/1');
add_line(mdl, 'Enabled_Filter/1', 'ws_en/1');
add_line(mdl, 'Triggered_Sample/1', 'ws_trig/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 2 步：仿真 + 绘图 =====

set_param(mdl, 'StopTime', '4');
set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', '0.001');
simOut = sim(mdl);

t = simOut.tout;
y_cont = getSimData(simOut, 'cont_out', t);
y_en   = getSimData(simOut, 'en_out', t);
y_trig = getSimData(simOut, 'trig_out', t);
close_system(mdl, 0);

figure('Name', 't37: 触发与使能子系统', 'Position', [50, 50, 1100, 750]);

subplot(3,1,1);
plot(t, y_cont, 'b', 'LineWidth', 1.5); hold on;
plot(t, sin(2*t), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.8); hold off;
legend('连续滤波输出', '原始信号', 'Location', 'southeast');
title('连续滤波器 — 每个仿真步都执行');
ylabel('Amplitude'); grid on;

subplot(3,1,2);
plot(t, y_en, 'g', 'LineWidth', 1.5);
title('Enabled Subsystem — enable=1 时运行, enable=0 时输出保持 (2s 后关闭)');
ylabel('Amplitude'); grid on;
% 标注使能状态
hold on; yl = ylim;
patch([2 4 4 2], [yl(1) yl(1) yl(2) yl(2)], 'r', 'FaceAlpha', 0.08, 'EdgeColor', 'none');
text(3, yl(2)*0.9, 'DISABLED', 'Color', 'r', 'HorizontalAlignment', 'center');
hold off;

subplot(3,1,3);
stairs(t, y_trig, 'r', 'LineWidth', 1.5); hold on;
plot(t, sin(2*t), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5); hold off;
legend('Triggered 采样 (10Hz)', '原始信号', 'Location', 'southeast');
title('Triggered Subsystem — 只在上升沿采样保持, 10Hz 采样率');
ylabel('Amplitude'); xlabel('Time (s)'); grid on;

sgtitle('教程 37：触发与使能子系统 — 条件执行');

fprintf('\n========================================\n');
fprintf('  教程 37 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Enabled: 使能=0时输出保持 (或复位)\n');
fprintf('     → 条件激活: "只在电池电压正常时才运行加热器"\n\n');
fprintf('  2. Triggered: 只在触发沿执行一次\n');
fprintf('     → 离散采样: "ADC 每 10ms 触发一次采样保持"\n\n');
fprintf('  3. 对比连续: 性能无差异, 但条件执行省计算量\n');
fprintf('     → 嵌入式: 10Hz 采样完全够用, 没必要 1kHz 算\n\n');
fprintf('  4. Outport 的 Output when disabled: reset / held\n');
fprintf('     → held: 保持上一个值 = 零阶保持 (ZOH)\n');
fprintf('     → reset: 输出 0/初始值\n');
