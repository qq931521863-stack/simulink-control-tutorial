%% ============================================================
% 教程 39：Model Referencing — 大模型拆成小积木
%
% 【为什么要学这课】
%   你的 t30 主动悬挂模型有 30 个模块、47 根线——一个人还管得过来。
%   但工业级的汽车控制器模型呢？5000 个模块，10 个团队同时改。
%   如果把所有东西塞进一个 .slx，打开都要 5 分钟，合并冲突是灾难。
%
%   Model Referencing 就是解药：
%     主模型引用子模型 (.slx)，子模型独立开发、独立测试、独立仿真。
%     和 Subsystem 的区别：Subsystem 在主文件里存，Model Ref 是独立文件。
%
% ┌─────────────────────────────────────────────────────────┐
% │ Subsystem vs Model Reference                            │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   Subsystem:      一个 .slx 装所有内容                   │
% │     ✓ 简单、直接                                        │
% │     ✗ 多人不能同时改一个文件                             │
% │     ✗ 模型大了编译慢                                    │
% │                                                         │
% │   Model Reference: 多个 .slx 互相引用                    │
% │     ✓ 独立开发、独立测试、独立版本管理                   │
% │     ✓ 只编译改过的部分 (增量编译)                        │
% │     ✓ 可以设置不同的仿真模式 (Normal/Accelerator/SIL)    │
% │     ✗ 设置稍复杂                                        │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 把一个大模型拆成 Plant + Controller 两个独立 .slx
%   2. 用 Model block 引用子模型
%   3. 理解模型参数传递
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 39：Model Referencing\n');
fprintf('============================================\n\n');

models_dir = fullfile(fileparts(mfilename('fullpath')), 'models');
addpath(models_dir);

%% ===== 第 1 步：创建子模型 Plant =====

fprintf('【Step 1: 创建 Plant 子模型】\n');

plant_mdl = 'tutorial39_plant';
if bdIsLoaded(plant_mdl), close_system(plant_mdl, 1); end
new_system(plant_mdl, 'Model');

add_block('simulink/Ports & Subsystems/In1', [plant_mdl '/u'], ...
    'Position', [30, 50, 50, 70]);
add_block('simulink/Continuous/Transfer Fcn', [plant_mdl '/Plant_TF'], ...
    'Position', [120, 40, 200, 90]);
set_param([plant_mdl '/Plant_TF'], 'Numerator', '[K]', 'Denominator', '[tau 1]');
add_block('simulink/Ports & Subsystems/Out1', [plant_mdl '/y'], ...
    'Position', [280, 50, 300, 70]);

add_block('simulink/Sinks/To Workspace', [plant_mdl '/ws_y'], ...
    'Position', [360, 50, 410, 75]);
set_param([plant_mdl '/ws_y'], 'VariableName', 'plant_y_sub');

% 模型参数 (通过 Model Workspace 或 mask 传递)
% 使用 Model Explorer 或者在 InitFcn 中定义
K_val   = 2;
tau_val = 0.5;
set_param(plant_mdl, 'InitFcn', sprintf('K = %.1f; tau = %.1f;', K_val, tau_val));

add_line(plant_mdl, 'u/1', 'Plant_TF/1');
add_line(plant_mdl, 'Plant_TF/1', 'y/1');
add_line(plant_mdl, 'Plant_TF/1', 'ws_y/1');

Simulink.BlockDiagram.arrangeSystem(plant_mdl);
save_system(plant_mdl, fullfile(models_dir, [plant_mdl '.slx']));

fprintf('  [OK] %s.slx 已保存 (参数 K=%.1f, tau=%.1f)\n\n', plant_mdl, K_val, tau_val);

%% ===== 第 2 步：创建子模型 Controller =====

fprintf('【Step 2: 创建 Controller 子模型】\n');

ctrl_mdl = 'tutorial39_controller';
if bdIsLoaded(ctrl_mdl), close_system(ctrl_mdl, 1); end
new_system(ctrl_mdl, 'Model');

add_block('simulink/Ports & Subsystems/In1', [ctrl_mdl '/ref'], ...
    'Position', [30, 30, 50, 50]);
add_block('simulink/Ports & Subsystems/In1', [ctrl_mdl '/y_fb'], ...
    'Position', [30, 90, 50, 110]);
add_block('simulink/Math Operations/Sum', [ctrl_mdl '/Err'], ...
    'Position', [120, 50, 150, 80]);
set_param([ctrl_mdl '/Err'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [ctrl_mdl '/Kp'], ...
    'Position', [200, 50, 240, 80]);
set_param([ctrl_mdl '/Kp'], 'Gain', 'Kp_gain');
add_block('simulink/Ports & Subsystems/Out1', [ctrl_mdl '/u'], ...
    'Position', [310, 50, 330, 70]);

set_param(ctrl_mdl, 'InitFcn', 'Kp_gain = 3;');

add_line(ctrl_mdl, 'ref/1', 'Err/1');
add_line(ctrl_mdl, 'y_fb/1', 'Err/2');
add_line(ctrl_mdl, 'Err/1', 'Kp/1');
add_line(ctrl_mdl, 'Kp/1', 'u/1');

Simulink.BlockDiagram.arrangeSystem(ctrl_mdl);
save_system(ctrl_mdl, fullfile(models_dir, [ctrl_mdl '.slx']));

fprintf('  [OK] %s.slx 已保存 (Kp=3)\n\n', ctrl_mdl);

%% ===== 第 3 步：创建主模型 (引用子模型) =====

fprintf('【Step 3: 创建主模型 (引用 Plant + Controller)】\n');

top_mdl = 'tutorial39_top';
if bdIsLoaded(top_mdl), close_system(top_mdl, 1); end
new_system(top_mdl, 'Model');
open_system(top_mdl);

% Model block 引用 Controller
add_block('simulink/Ports & Subsystems/Model', [top_mdl '/Controller_Ref'], ...
    'Position', [200, 50, 280, 110]);
set_param([top_mdl '/Controller_Ref'], 'ModelName', ctrl_mdl);

% Model block 引用 Plant
add_block('simulink/Ports & Subsystems/Model', [top_mdl '/Plant_Ref'], ...
    'Position', [380, 50, 460, 110]);
set_param([top_mdl '/Plant_Ref'], 'ModelName', plant_mdl);

% 参考输入
add_block('simulink/Sources/Step', [top_mdl '/Setpoint'], ...
    'Position', [30, 60, 90, 90]);
set_param([top_mdl '/Setpoint'], 'Time', '0.5', 'After', '1');

% Scope
add_block('simulink/Sinks/Scope', [top_mdl '/Scope'], ...
    'Position', [550, 60, 600, 100]);

% Outport（把 Plant 输出引出，用 simOut.yout 取数据）
add_block('simulink/Ports & Subsystems/Out1', [top_mdl '/y_out'], ...
    'Position', [550, 110, 600, 130]);

% 连线
add_line(top_mdl, 'Setpoint/1', 'Controller_Ref/1');
add_line(top_mdl, 'Controller_Ref/1', 'Plant_Ref/1');
add_line(top_mdl, 'Plant_Ref/1', 'Controller_Ref/2');
add_line(top_mdl, 'Plant_Ref/1', 'Scope/1');
add_line(top_mdl, 'Plant_Ref/1', 'y_out/1');

Simulink.BlockDiagram.arrangeSystem(top_mdl);
save_system(top_mdl, fullfile(models_dir, [top_mdl '.slx']));

%% ===== 第 4 步：仿真 =====

set_param(top_mdl, 'StopTime', '5');
simOut = sim(top_mdl);
t = simOut.tout;

% 从 simOut.yout 取 Plant 输出数据
plant_y_ws = simOut.yout{1}.Values;
close_system(top_mdl, 0);
close_system(plant_mdl, 0);

figure('Name', 't39: Model Referencing', 'Position', [50, 50, 800, 400]);
plot(plant_y_ws.Time, plant_y_ws.Data, 'b', 'LineWidth', 2);
title('Model Referencing 仿真结果');
xlabel('Time (s)'); ylabel('Output'); grid on;
legend('Plant Output (来自被引用的子模型)', 'Location', 'southeast');

fprintf('  [OK] 主模型 %s.slx 仿真完成\n\n', top_mdl);

fprintf('========================================\n');
fprintf('  教程 39 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Model block = 引用另一个 .slx 文件\n');
fprintf('     tutorial39_plant.slx   → 独立的 Plant 模型\n');
fprintf('     tutorial39_controller  → 独立的 Controller 模型\n');
fprintf('     tutorial39_top         → 主模型, 引用上面两个\n\n');
fprintf('  2. 每个子模型有自己的 InitFcn\n');
fprintf('     Plant: K=2, tau=0.5\n');
fprintf('     Controller: Kp=3\n');
fprintf('     → 参数隔离, 互不污染\n\n');
fprintf('  3. vs Subsystem\n');
fprintf('     Subsystem: 方便但单体\n');
fprintf('     Model Ref: 独立文件, 独立版本, 独立测试\n\n');
fprintf('  4. 工业级: 一个项目 50+ 个 Model Reference 是常态\n');
fprintf('     每个引用可以设不同的仿真模式 (Normal/Accel/SIL)\n');
