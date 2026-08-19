%% ============================================================
% 教程 38：Bus 信号 — 把几十根线捆成一束
%
% 【为什么要学这课】
%   你现在搭的模型，信号一只手数得过来——误差、控制力、输出。
%   但真实项目呢？
%     - 电机有 6 个传感器 (位置、速度、电流×3、温度)
%     - 控制器有 4 个输出 (力矩、模式、状态码、诊断)
%     - 上层监控还要收 20+ 个信号
%
%   如果全部用单线直连，你的模型就是意大利面。
%   Bus 信号就是捆扎带——把相关信号打包成一束，一根线传输。
%
% ┌─────────────────────────────────────────────────────────┐
% │ 本课案例：电机控制系统信号组织                            │
% │                                                         │
% │   MotorBus: {position, speed, current, temp}             │
% │   ControlBus: {torque_cmd, mode, fault_code}             │
% │                                                         │
% │   传感器 → Bus Creator → 一根线 → Bus Selector → 各模块  │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 用 Bus Creator / Bus Selector 打包和拆解信号
%   2. 创建 Bus 对象定义信号结构
%   3. 理解 Bus vs Mux 的本质区别
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 38：Bus 信号\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：定义 Bus 对象 =====

fprintf('【Step 1: 定义 Bus 对象】\n');

% 创建 Bus 类型定义 (在 base workspace)
MotorBus = Simulink.Bus;
MotorBus.Elements(1) = Simulink.BusElement;
MotorBus.Elements(1).Name = 'position';
MotorBus.Elements(1).DataType = 'double';
MotorBus.Elements(2) = Simulink.BusElement;
MotorBus.Elements(2).Name = 'speed';
MotorBus.Elements(2).DataType = 'double';
MotorBus.Elements(3) = Simulink.BusElement;
MotorBus.Elements(3).Name = 'current';
MotorBus.Elements(3).DataType = 'double';
MotorBus.Elements(4) = Simulink.BusElement;
MotorBus.Elements(4).Name = 'temperature';
MotorBus.Elements(4).DataType = 'double';

ControlBus = Simulink.Bus;
ControlBus.Elements(1) = Simulink.BusElement;
ControlBus.Elements(1).Name = 'torque_cmd';
ControlBus.Elements(2) = Simulink.BusElement;
ControlBus.Elements(2).Name = 'mode';
ControlBus.Elements(3) = Simulink.BusElement;
ControlBus.Elements(3).Name = 'fault_code';

assignin('base', 'MotorBus', MotorBus);
assignin('base', 'ControlBus', ControlBus);

fprintf('  [OK] MotorBus: position, speed, current, temperature\n');
fprintf('  [OK] ControlBus: torque_cmd, mode, fault_code\n\n');

%% ===== 第 2 步：搭 Simulink 模型 =====

fprintf('【Step 2: 搭建对比模型 (Bus vs Mux)】\n');

mdl = 'tutorial38_bus';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 生成模拟传感器信号
add_block('simulink/Sources/Sine Wave', [mdl '/Position'], ...
    'Position', [30, 30, 90, 60]);
set_param([mdl '/Position'], 'Amplitude', '1', 'Frequency', '0.5');

add_block('simulink/Sources/Sine Wave', [mdl '/Speed'], ...
    'Position', [30, 90, 90, 120]);
set_param([mdl '/Speed'], 'Amplitude', '3', 'Frequency', '0.5', 'Phase', 'pi/2');

add_block('simulink/Sources/Constant', [mdl '/Current'], ...
    'Position', [30, 150, 90, 175]);
set_param([mdl '/Current'], 'Value', '2.5');

add_block('simulink/Sources/Constant', [mdl '/Temp'], ...
    'Position', [30, 210, 90, 235]);
set_param([mdl '/Temp'], 'Value', '45');

% ===== Bus 方式 =====
add_block('simulink/Signal Routing/Bus Creator', [mdl '/BusCreator'], ...
    'Position', [160, 30, 220, 240]);
set_param([mdl '/BusCreator'], 'Inputs', '4', 'NonVirtualBus', 'off');

add_block('simulink/Signal Routing/Bus Selector', [mdl '/BusSelector'], ...
    'Position', [500, 30, 560, 200]);
set_param([mdl '/BusSelector'], 'OutputSignals', 'position,speed,temperature');

% ===== Mux 方式 (对比) =====
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_signals'], ...
    'Position', [160, 300, 190, 390]);
set_param([mdl '/Mux_signals'], 'Inputs', '4', 'DisplayOption', 'bar');

add_block('simulink/Signal Routing/Demux', [mdl '/Demux_signals'], ...
    'Position', [500, 300, 530, 390]);
set_param([mdl '/Demux_signals'], 'Outputs', '4');

% ===== 控制器子系统 (接收 Bus) =====
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Controller'], ...
    'Position', [300, 480, 400, 560]);
Simulink.SubSystem.deleteContents([mdl '/Controller']);
ctrl = [mdl '/Controller'];

add_block('simulink/Ports & Subsystems/In1', [ctrl '/BusIn'], 'Position', [30,50,50,70]);
add_block('simulink/Signal Routing/Bus Selector', [ctrl '/Sel'], ...
    'Position', [120,30,180,100]);
set_param([ctrl '/Sel'], 'OutputSignals', 'position,speed');
add_block('simulink/Math Operations/Sum', [ctrl '/Sum'], ...
    'Position', [250,50,280,80]);
set_param([ctrl '/Sum'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [ctrl '/Gain'], ...
    'Position', [330,50,370,80]);
set_param([ctrl '/Gain'], 'Gain', '0.5');
add_block('simulink/Ports & Subsystems/Out1', [ctrl '/CtrlOut'], ...
    'Position', [430,50,450,70]);
add_block('simulink/Ports & Subsystems/Out1', [ctrl '/FaultOut'], ...
    'Position', [430,100,450,120]);

add_line(ctrl, 'BusIn/1', 'Sel/1');
add_line(ctrl, 'Sel/1', 'Sum/1');
add_line(ctrl, 'Sel/2', 'Sum/2');
add_line(ctrl, 'Sum/1', 'Gain/1');
add_line(ctrl, 'Gain/1', 'CtrlOut/1');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_pos'], ...
    'Position', [650, 50, 700, 75]);
set_param([mdl '/ws_pos'], 'VariableName', 'pos_out');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_speed'], ...
    'Position', [650, 100, 700, 125]);
set_param([mdl '/ws_speed'], 'VariableName', 'spd_out');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_torque'], ...
    'Position', [650, 480, 700, 505]);
set_param([mdl '/ws_torque'], 'VariableName', 'torque_out');

% 顶层连线
add_line(mdl, 'Position/1', 'BusCreator/1');
add_line(mdl, 'Speed/1', 'BusCreator/2');
add_line(mdl, 'Current/1', 'BusCreator/3');
add_line(mdl, 'Temp/1', 'BusCreator/4');

% 设置信号名（虚拟总线按 SignalName 提取）
set_param(get_param([mdl '/Position'],'PortHandles').Outport(1), 'Name', 'position');
set_param(get_param([mdl '/Speed'],'PortHandles').Outport(1), 'Name', 'speed');
set_param(get_param([mdl '/Current'],'PortHandles').Outport(1), 'Name', 'current');
set_param(get_param([mdl '/Temp'],'PortHandles').Outport(1), 'Name', 'temperature');

add_line(mdl, 'Position/1', 'Mux_signals/1');
add_line(mdl, 'Speed/1', 'Mux_signals/2');
add_line(mdl, 'Current/1', 'Mux_signals/3');
add_line(mdl, 'Temp/1', 'Mux_signals/4');

% Bus → BusSelector → WS
add_line(mdl, 'BusCreator/1', 'BusSelector/1');
add_line(mdl, 'BusSelector/1', 'ws_pos/1');
add_line(mdl, 'BusSelector/2', 'ws_speed/1');

% Bus → Controller
add_line(mdl, 'BusCreator/1', 'Controller/1');
add_line(mdl, 'Controller/1', 'ws_torque/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 3 步：仿真 + 验证 =====

set_param(mdl, 'StopTime', '5');
simOut = sim(mdl);
t = simOut.tout;
pos    = getSimData(simOut, 'pos_out', t);
spd    = getSimData(simOut, 'spd_out', t);
torque = getSimData(simOut, 'torque_out', t);
close_system(mdl, 0);

figure('Name', 't38: Bus 信号', 'Position', [50, 50, 900, 500]);

subplot(2,1,1);
plot(t, pos, 'b', 'LineWidth', 1.5); hold on;
plot(t, spd, 'r', 'LineWidth', 1.5); hold off;
legend('Position (从 Bus 提取)', 'Speed (从 Bus 提取)', 'Location', 'southeast');
title('Bus Selector — 从一束信号中按名字取出需要的');
ylabel('Value'); grid on;

subplot(2,1,2);
plot(t, torque, 'm', 'LineWidth', 1.5);
title('Controller 输出 — 接收 Bus, 内部拆解, 只输出控制量');
ylabel('Torque'); xlabel('Time (s)'); grid on;

sgtitle('教程 38：Bus 信号');

fprintf('\n========================================\n');
fprintf('  教程 38 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Bus = 有名字的结构体, Mux = 无名字的数组\n');
fprintf('     Bus: 按名字访问 signal.position, signal.speed\n');
fprintf('     Mux: 按位置访问 port 1, port 2, port 3\n');
fprintf('     → Bus 不会因为增删信号而全乱了\n\n');
fprintf('  2. Bus Creator/Selector 是核心工具\n');
fprintf('     Creator: 打包 → Selector: 按名提取\n');
fprintf('     不用记"第 3 个端口是电流"——直接选 "current"\n\n');
fprintf('  3. Bus 对象 (Simulink.Bus) 是类型定义\n');
fprintf('     定义了 {position, speed, current, temp}\n');
fprintf('     错误连接会在编译时被检测出来\n\n');
fprintf('  4. 真实项目: 一个 Bus 传几百个信号是常态\n');
fprintf('     没有 Bus → 连线是蜘蛛网 → 根本没法维护\n');
