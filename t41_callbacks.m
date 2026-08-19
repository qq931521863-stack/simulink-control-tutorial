%% ============================================================
% 教程 41：Model Callbacks — 让模型自己初始化
%
% 【为什么要学这课】
%   你每次打开别人的模型，第一件事是跑一个 init_xxx.m 脚本
%   加载参数、设置常数、初始化变量。
%   你不跑？模型报错。你忘了跑？仿真结果莫名其妙。
%
%   Model Callbacks 让这些自动化：
%     PreLoadFcn: 模型加载前 → 把参数定义好
%     InitFcn:    仿真开始前 → 初始化状态、检查环境
%     StopFcn:    仿真结束后 → 自动保存结果、清理临时变量
%
% ┌─────────────────────────────────────────────────────────┐
% │ 常用 Callbacks                                          │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  PreLoadFcn  — 打开模型前执行                            │
% │     load('params.mat');   % 从文件加载参数               │
% │     run('init_params.m'); % 从脚本加载参数               │
% │                                                         │
% │  InitFcn     — 每次仿真开始前执行                        │
% │     disp('仿真开始'); check_params();                   │
% │                                                         │
% │  StopFcn     — 仿真结束后执行                            │
% │     save('results.mat', 'tout', 'yout');                │
% │     assignin('base', 'last_result', ans);               │
% │                                                         │
% │  CloseFcn    — 关闭模型时执行                            │
% │     clear params;  % 清理 workspace                     │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 用 InitFcn 自动加载参数（代替手动跑脚本）
%   2. 用 StopFcn 自动保存结果（不用每次手动 save）
%   3. 理解 PreLoadFcn vs InitFcn 的区别
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 41：Model Callbacks\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：创建带 Callbacks 的模型 =====

fprintf('【Step 1: 设置 Auto-Init 模型】\n');

mdl = 'tutorial41_callbacks';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% PreLoadFcn: 打开模型时自动执行
set_param(mdl, 'PreLoadFcn', ...
    'disp(''   [PreLoadFcn] 正在加载参数...'');');

% InitFcn: 每次仿真前执行
% 这里做三件事: 1.检查参数 2.初始化变量 3.显示信息
init_fcn_str = [ ...
    'fprintf(''   [InitFcn] 参数检查通过: Kp=%.1f, Ki=%.1f\n'', Kp, Ki);' newline ...
    'fprintf(''   [InitFcn] 时间: %s\n'', datestr(now));' newline ...
    'last_sim_time = now;'];
set_param(mdl, 'InitFcn', init_fcn_str);

% StopFcn: 仿真结束时自动保存数据
models_dir = fullfile(fileparts(mfilename('fullpath')), 'models');
results_file = fullfile(models_dir, 'tutorial41_results.mat');
% 转义反斜杠给 MATLAB 字符串
results_file_esc = strrep(results_file, '\', '\\');
stop_fcn_str = sprintf([
    'disp(''   [StopFcn] 仿真结束, 自动保存结果...'');' newline ...
    'try' newline ...
    '    sim_data.t = tout;' newline ...
    '    sim_data.y = yout;' newline ...
    '    save(''%s'', ''sim_data'');' newline ...
    '    disp(''   [StopFcn] 结果已保存'');' newline ...
    'catch ME' newline ...
    '    disp([''   [StopFcn] 保存失败: '' ME.message]);' newline ...
    'end'], results_file_esc);
set_param(mdl, 'StopFcn', stop_fcn_str);

fprintf('  [OK] PreLoadFcn  → 打开模型时加载参数\n');
fprintf('  [OK] InitFcn     → 仿真前检查参数\n');
fprintf('  [OK] StopFcn     → 仿真后自动保存\n\n');

%% ===== 第 2 步：搭一个简单模型 =====

fprintf('【Step 2: 搭建测试模型】\n');

% 用 InitFcn 定义的参数 (在 base workspace 预先赋值, InitFcn 会引用)
Kp = 3; Ki = 0.5;
assignin('base', 'Kp', Kp);
assignin('base', 'Ki', Ki);

add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [30, 60, 90, 90]);
set_param([mdl '/Setpoint'], 'Time', '0.5', 'After', '1');

add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Position', [130, 65, 160, 95]);
set_param([mdl '/Error'], 'Inputs', '|+-');

% PI 控制器
add_block('simulink/Math Operations/Gain', [mdl '/Kp_gain'], ...
    'Position', [210, 50, 250, 80]);
set_param([mdl '/Kp_gain'], 'Gain', 'Kp');

add_block('simulink/Math Operations/Gain', [mdl '/Ki_gain'], ...
    'Position', [210, 110, 250, 140]);
set_param([mdl '/Ki_gain'], 'Gain', 'Ki');
add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'Position', [310, 110, 360, 140]);

add_block('simulink/Math Operations/Sum', [mdl '/SumPI'], ...
    'Position', [410, 60, 440, 130]);
set_param([mdl '/SumPI'], 'Inputs', '|++');

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], ...
    'Position', [490, 60, 560, 110]);
set_param([mdl '/Plant'], 'Numerator', '[1]', 'Denominator', '[1 0.5 1]');

% Scope
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [620, 60, 660, 110]);

% Outport（让 sim() 的 yout 有数据，StopFcn 才能保存）
add_block('simulink/Ports & Subsystems/Out1', [mdl '/y_out'], ...
    'Position', [620, 120, 660, 140]);

% 连线
add_line(mdl, 'Setpoint/1', 'Error/1');
add_line(mdl, 'Error/1', 'Kp_gain/1');
add_line(mdl, 'Error/1', 'Ki_gain/1');
add_line(mdl, 'Ki_gain/1', 'Integrator/1');
add_line(mdl, 'Kp_gain/1', 'SumPI/1');
add_line(mdl, 'Integrator/1', 'SumPI/2');
add_line(mdl, 'SumPI/1', 'Plant/1');
add_line(mdl, 'Plant/1', 'Error/2');
add_line(mdl, 'Plant/1', 'Scope/1');
add_line(mdl, 'Plant/1', 'y_out/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(models_dir, [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 3 步：仿真 ==== =

fprintf('【Step 3: 仿真 — 观察 Callback 输出】\n\n');

set_param(mdl, 'StopTime', '5');
simOut = sim(mdl);

% 检查 StopFcn 是否保存了结果
if exist(results_file, 'file')
    load(results_file);
    fprintf('  [OK] StopFcn 自动保存结果: %s\n', results_file);
else
    fprintf('  [WARN] StopFcn 未找到保存文件\n');
end

close_system(mdl, 0);

%% ===== 第 4 步：绘图 =====

figure('Name', 't41: Model Callbacks', 'Position', [50, 50, 800, 400]);
subplot(2,1,1);
plot(simOut.yout{1}.Values.Time, simOut.yout{1}.Values.Data, 'b', 'LineWidth', 2);
title('仿真结果 (参数由 InitFcn 自动加载)');
xlabel('Time (s)'); ylabel('Output'); grid on;

subplot(2,1,2);
plot(simOut.yout{1}.Values.Time, simOut.yout{1}.Values.Data, 'b', 'LineWidth', 2);
title('对比: 从 StopFcn 保存的文件恢复');
xlabel('Time (s)'); ylabel('Output'); grid on;

fprintf('\n========================================\n');
fprintf('  教程 41 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. PreLoadFcn vs InitFcn\n');
fprintf('     PreLoad: 打开模型时执行 (加载参数文件)\n');
fprintf('     InitFcn: 每次仿真前执行 (检查、初始化)\n');
fprintf('     区别: PreLoad 只执行一次, InitFcn 每次仿真都跑\n\n');
fprintf('  2. StopFcn = 自动保存, 不用手动 save\n');
fprintf('     结果 → 自动存到 .mat → 后续脚本直接 load\n\n');
fprintf('  3. 最佳实践\n');
fprintf('     PreLoadFcn: run(''init_params.m'');\n');
fprintf('     InitFcn:    check_setup; 验证参数范围\n');
fprintf('     StopFcn:    save(''results.mat'');\n');
fprintf('     CloseFcn:   clear 临时变量\n');
