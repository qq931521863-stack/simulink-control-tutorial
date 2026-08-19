%% ============================================================
% 教程 42：S-Function — 当你需要的模块 Simulink 没有
%
% 【为什么要学这课】
%   Simulink 提供了几百个内置模块。但总有不够用的时候——
%     - 一个特殊的非线性函数，用 Fcn 块写不了
%     - 一个自定义的数值积分算法
%     - 一段现成的 C 代码想嵌入 Simulink
%
%   S-Function 就是答案——用 MATLAB 或 C 代码写一个自定义模块，
%   有输入、输出、状态、导数，和内置模块完全一样地使用。
%
% ┌─────────────────────────────────────────────────────────┐
% │ Level-2 MATLAB S-Function                               │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   setup() → 定义端口数、采样时间、参数                   │
% │   Outputs() → 每步执行: 根据输入计算输出                 │
% │   Derivatives() → (可选) 连续状态导数                   │
% │   Update() → (可选) 离散状态更新                        │
% │                                                         │
% │   本课写一个"死区"模块 (Dead Zone):                     │
% │     输入在 [-threshold, threshold] 内 → 输出 = 0         │
% │     输入超出范围 → 输出 = 输入                           │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 创建第一个 Level-2 MATLAB S-Function
%   2. 理解 setup / Outputs / Derivatives
%   3. 把 S-Function 当作普通模块使用
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 42：S-Function 基础\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：写 S-Function 代码 =====

fprintf('【Step 1: 定义 Dead-Zone S-Function】\n');

% Level-2 MATLAB S-Function 是一个独立的 .m 文件
% 这里写成内联函数, 实际使用时保存为独立文件
sfcn_dir = fileparts(mfilename('fullpath'));
sfcn_file = fullfile(sfcn_dir, 'deadzone_sfcn.m');

sfcn_code = [ ...
    'function deadzone_sfcn(block)\n' ...
    '%% Level-2 MATLAB S-Function: Dead Zone\n' ...
    '%%   if |u| < threshold → y = 0\n' ...
    '%%   else              → y = u\n\n' ...
    '    setup(block);\n' ...
    'end\n\n' ...
    'function setup(block)\n' ...
    '    %% 输入输出端口数量\n' ...
    '    block.NumInputPorts  = 1;\n' ...
    '    block.NumOutputPorts = 1;\n' ...
    '    block.InputPort(1).Dimensions  = 1;\n' ...
    '    block.OutputPort(1).Dimensions = 1;\n' ...
    '    block.InputPort(1).DirectFeedthrough = true;\n\n' ...
    '    %% 采样时间 (连续: [0 0], 离散: [Ts 0])\n' ...
    '    block.SampleTimes = [0 0];\n\n' ...
    '    %% 参数: 阈值\n' ...
    '    block.NumDialogPrms = 1;\n\n' ...
    '    %% 注册方法\n' ...
    '    block.RegBlockMethod(''Outputs'', @Outputs);\n' ...
    'end\n\n' ...
    'function Outputs(block)\n' ...
    '    threshold = str2double(block.DialogPrm(1).Data);\n' ...
    '    u = block.InputPort(1).Data;\n' ...
    '    if abs(u) < threshold\n' ...
    '        block.OutputPort(1).Data = 0;\n' ...
    '    else\n' ...
    '        block.OutputPort(1).Data = u;\n' ...
    '    end\n' ...
    'end\n'
];

fid = fopen(sfcn_file, 'w');
fprintf(fid, sfcn_code);
fclose(fid);

fprintf('  [OK] S-Function 文件: deadzone_sfcn.m\n\n');

%% ===== 第 2 步：在 Simulink 中测试 =====

fprintf('【Step 2: 搭建测试模型】\n');

mdl = 'tutorial42_sfunction';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 输入: 正弦波 (振幅=2, 覆盖阈值的两侧)
add_block('simulink/Sources/Sine Wave', [mdl '/Input'], ...
    'Position', [30, 60, 90, 90]);
set_param([mdl '/Input'], 'Amplitude', '2', 'Frequency', '1');

% S-Function 块 (引用 deadzone_sfcn)
load_system('simulink');
l2 = find_system('simulink/User-Defined Functions', 'SearchDepth', 1);
l2 = l2(contains(l2, 'Level-2 MATLAB'));
add_block(l2{1}, [mdl '/DeadZone'], 'Position', [180, 45, 260, 105]);
set_param([mdl '/DeadZone'], 'FunctionName', 'deadzone_sfcn');
set_param([mdl '/DeadZone'], 'Parameters', '0.5');

% 对比: 原始信号 vs 死区输出
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [350, 60, 400, 110]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_in'], ...
    'Position', [350, 150, 400, 175]);
set_param([mdl '/ws_in'], 'VariableName', 'sfcn_in');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_out'], ...
    'Position', [350, 200, 400, 225]);
set_param([mdl '/ws_out'], 'VariableName', 'sfcn_out');

% 连线
add_line(mdl, 'Input/1', 'DeadZone/1');
add_line(mdl, 'Input/1', 'Scope/1');
add_line(mdl, 'DeadZone/1', 'Scope/2');
add_line(mdl, 'Input/1', 'ws_in/1');
add_line(mdl, 'DeadZone/1', 'ws_out/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 3 步：仿真 =====

set_param(mdl, 'StopTime', '5');
simOut = sim(mdl);
t = simOut.tout;
u = getSimData(simOut, 'sfcn_in', t);
y = getSimData(simOut, 'sfcn_out', t);
close_system(mdl, 0);

figure('Name', 't42: S-Function — 自定义死区模块', ...
    'Position', [50, 50, 900, 450]);
plot(t, u, 'b', 'LineWidth', 1.5); hold on;
plot(t, y, 'r', 'LineWidth', 2);
yline(0.5, 'k--'); yline(-0.5, 'k--');
legend('输入 (正弦)', '输出 (经死区)', '±threshold', 'Location', 'southeast');
title('S-Function Dead Zone: 阈值内输出=0, 阈值外输出=输入');
xlabel('Time (s)'); ylabel('Amplitude'); grid on;

fprintf('\n========================================\n');
fprintf('  教程 42 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. S-Function = 自定义块的"插件接口"\n');
fprintf('     setup() 定义端口/参数/采样时间\n');
fprintf('     Outputs() 实现计算逻辑\n\n');
fprintf('  2. Level-2 MATLAB S-Function vs Level-1\n');
fprintf('     Level-2: 面向对象, 支持多端口/多维度\n');
fprintf('     Level-1: 简单但受限, 不推荐新项目\n\n');
fprintf('  3. S-Function 可以干什么\n');
fprintf('     - 自定义非线性模块 (本课的死区)\n');
fprintf('     - 嵌入现有 C/Fortran 代码\n');
fprintf('     - 实现特殊数值算法 (自定义积分器)\n');
fprintf('     - 设备驱动接口 (读取硬件数据)\n\n');
fprintf('  4. 位置: deadzone_sfcn.m 复制到工作目录即可用\n');
