%% ============================================================
% 教程 36：Stateflow 状态机 — 把控制逻辑画成图
%
% 【为什么要学这课】
%   你写过一个 PID 控制器——输入误差，输出控制力，简单直接。
%   但真实控制器不是这么单纯——
%     - 电机启动时要先预励磁，不能直接给全速
%     - 温度超过阈值要紧急停机
%     - 电池电压低了要切换到节能模式
%
%   这些"if-else + 状态切换"逻辑如果写成 MATLAB Function 块，
%   嵌套三五层就没人看得懂了。
%   Stateflow 就是干这个的——把状态机画成图，一目了然。
%
% ┌─────────────────────────────────────────────────────────┐
% │ 本课案例：电机启停控制器                                  │
% │                                                         │
% │   OFF ──[start_cmd]──▶ STARTING ──[speed>80%]──▶ RUNNING│
% │    ▲                      │                     │       │
% │    └──[estop]─────────────┴──[estop]────────────┘       │
% │                                                         │
% │   OFF:     停止，等待命令                                │
% │   STARTING: 软启动中 (3 秒斜坡)                          │
% │   RUNNING:  正常运行，PID 闭环                           │
% │   任何状态下 estop = 1 → 直接跳回 OFF                    │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 用 Stateflow 画第一个状态机
%   2. 理解 state / transition / condition / action
%   3. Stateflow 输出直接控制 Simulink 模块
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 36：Stateflow 状态机\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：搭 Simulink Plant (电机模型) =====

fprintf('【Step 1: 搭建电机 Plant】\n');

mdl = 'tutorial36_stateflow';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 一阶电机模型: speeḋ = -a*speed + K*u
% 放到子系统里
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Motor_Plant'], ...
    'Position', [350, 60, 450, 120]);
Simulink.SubSystem.deleteContents([mdl '/Motor_Plant']);
mp = [mdl '/Motor_Plant'];

add_block('simulink/Ports & Subsystems/In1', [mp '/voltage'], 'Position', [30,50,50,70]);
add_block('simulink/Math Operations/Gain', [mp '/K_gain'], 'Position', [130,45,170,75]);
set_param([mp '/K_gain'], 'Gain', '30');
add_block('simulink/Math Operations/Sum', [mp '/Sum_dyn'], ...
    'Position', [240,55,270,95]);
set_param([mp '/Sum_dyn'], 'Inputs', '|+-');
add_block('simulink/Continuous/Integrator', [mp '/Integrator'], ...
    'Position', [340,55,390,95]);
set_param([mp '/Integrator'], 'InitialCondition', '0');
add_block('simulink/Math Operations/Gain', [mp '/a_fb'], 'Position', [240,110,270,150]);
set_param([mp '/a_fb'], 'Gain', '2');
add_block('simulink/Ports & Subsystems/Out1', [mp '/speed'], ...
    'Position', [480,55,500,75]);

add_line(mp, 'voltage/1', 'K_gain/1');
add_line(mp, 'K_gain/1', 'Sum_dyn/1');
add_line(mp, 'Sum_dyn/1', 'Integrator/1');
add_line(mp, 'Integrator/1', 'a_fb/1');
add_line(mp, 'a_fb/1', 'Sum_dyn/2');
add_line(mp, 'Integrator/1', 'speed/1');

% --- 参考输入、PID ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [30, 60, 90, 90]);
set_param([mdl '/Setpoint'], 'Time', '0.5', 'Before', '0', 'After', '100');

add_block('simulink/Math Operations/Sum', [mdl '/Error'], ...
    'Position', [120, 65, 150, 95]);
set_param([mdl '/Error'], 'Inputs', '|+-');

add_block('simulink/Math Operations/Gain', [mdl '/Kp'], ...
    'Position', [210, 65, 250, 95]);
set_param([mdl '/Kp'], 'Gain', '0.8');

% 紧急停止信号 (仿真用 Switch)
add_block('simulink/Sources/Constant', [mdl '/estop_const'], ...
    'Position', [30, 160, 80, 185]);
set_param([mdl '/estop_const'], 'Value', '1');

add_block('simulink/Sources/Step', [mdl '/estop_trigger'], ...
    'Position', [120, 160, 180, 185]);
set_param([mdl '/estop_trigger'], 'Time', '8', 'Before', '0', 'After', '1');

% estop 模拟: 8s 时触发紧急停止, 0.5s 后清除
add_block('simulink/Logic and Bit Operations/Logical Operator', ...
    [mdl '/estop_sig'], 'Position', [240, 160, 280, 190]);
set_param([mdl '/estop_sig'], 'Operator', 'AND', 'Inputs', '2');

% --- Stateflow Chart ---
% 用 add_block 创建 Chart（sfnew 是创建新模型，不能用在已存在的模型上）
add_block('sflib/Chart', [mdl '/Chart']);
sc = [mdl '/Chart'];
set_param(sc, 'Position', [350, 180, 550, 380]);

% 获取 Stateflow 根对象并操作
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.Chart', 'Path', [mdl '/Chart']);

% 创建状态
s_off = Stateflow.State(ch);
s_off.Name = 'OFF';
s_off.Position = [30, 120, 120, 50];
s_off.LabelString = sprintf('OFF\nen: out_mode = 0;\n    out_voltage = 0;');

s_start = Stateflow.State(ch);
s_start.Name = 'STARTING';
s_start.Position = [200, 30, 120, 50];
s_start.LabelString = sprintf('STARTING\nen: out_mode = 1;\ndu: out_voltage = min(10, out_voltage + 0.2);');

s_run = Stateflow.State(ch);
s_run.Name = 'RUNNING';
s_run.Position = [200, 150, 120, 50];
s_run.LabelString = sprintf('RUNNING\nen: out_mode = 2;\n    out_voltage = Kp*(setpoint - speed);');

% 默认转移
dt = Stateflow.Transition(ch);
dt.Destination = s_off;
dt.DestinationOClock = 3;
dt.SourceEndPoint = [30, 30];

% OFF → STARTING
t1 = Stateflow.Transition(ch);
t1.Source = s_off;
t1.Destination = s_start;
t1.SourceOClock = 0;
t1.DestinationOClock = 6;
t1.LabelString = '[start_cmd == 1]';

% STARTING → RUNNING
t2 = Stateflow.Transition(ch);
t2.Source = s_start;
t2.Destination = s_run;
t2.SourceOClock = 3;
t2.DestinationOClock = 0;
t2.LabelString = '[speed >= 80]';

% RUNNING → OFF (estop)
t3 = Stateflow.Transition(ch);
t3.Source = s_run;
t3.Destination = s_off;
t3.LabelString = '[estop == 1]';

% STARTING → OFF (estop)
t4 = Stateflow.Transition(ch);
t4.Source = s_start;
t4.Destination = s_off;
t4.LabelString = '[estop == 1]';

% 创建数据端口
d_speed    = Stateflow.Data(ch); d_speed.Name = 'speed';     d_speed.Scope = 'Input';  d_speed.Port = 1;
d_setpoint = Stateflow.Data(ch); d_setpoint.Name = 'setpoint'; d_setpoint.Scope = 'Input'; d_setpoint.Port = 2;
d_start    = Stateflow.Data(ch); d_start.Name = 'start_cmd';  d_start.Scope = 'Input';  d_start.Port = 3;
d_estop    = Stateflow.Data(ch); d_estop.Name = 'estop';      d_estop.Scope = 'Input';  d_estop.Port = 4;
d_mode     = Stateflow.Data(ch); d_mode.Name = 'out_mode';    d_mode.Scope = 'Output';  d_mode.Port = 1;
d_volt     = Stateflow.Data(ch); d_volt.Name = 'out_voltage'; d_volt.Scope = 'Output';  d_volt.Port = 2;
d_Kp_local = Stateflow.Data(ch); d_Kp_local.Name = 'Kp';      d_Kp_local.Scope = 'Constant'; d_Kp_local.Props.InitialValue = '0.8';

% --- 连线 Stateflow ---
add_block('simulink/Sources/Constant', [mdl '/start_cmd'], ...
    'Position', [300, 310, 340, 335]);
set_param([mdl '/start_cmd'], 'Value', '1');

add_line(mdl, 'Motor_Plant/1', 'Chart/1');  % speed
add_line(mdl, 'Setpoint/1', 'Chart/2');     % setpoint
add_line(mdl, 'Setpoint/1', 'Error/1');
add_line(mdl, 'start_cmd/1', 'Chart/3');
add_line(mdl, 'estop_const/1', 'estop_sig/1');
add_line(mdl, 'estop_trigger/1', 'estop_sig/2');
add_line(mdl, 'estop_sig/1', 'Chart/4');

% Stateflow 输出电压 → 电机
add_block('simulink/Signal Routing/Switch', [mdl '/ModeSelect'], ...
    'Position', [600, 50, 640, 90]);
set_param([mdl '/ModeSelect'], 'Criteria', 'u2 ~= 0', 'Threshold', '0');

add_line(mdl, 'Error/1', 'Kp/1');
add_line(mdl, 'Kp/1', 'ModeSelect/3');           % PID 输出 (模式2)
add_line(mdl, 'Chart/2', 'ModeSelect/1');  % Stateflow 电压 (模式0/1)
add_line(mdl, 'Chart/1', 'ModeSelect/2');  % mode 选择信号
add_line(mdl, 'ModeSelect/1', 'Motor_Plant/1');
add_line(mdl, 'Motor_Plant/1', 'Error/2');

% Scope
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [700, 50, 750, 100]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');
add_line(mdl, 'Motor_Plant/1', 'Scope/1');
add_line(mdl, 'Setpoint/1', 'Scope/2');
add_line(mdl, 'Chart/1', 'Scope/3');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_speed'], ...
    'Position', [700, 150, 750, 175]);
set_param([mdl '/ws_speed'], 'VariableName', 'speed_data');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_mode'], ...
    'Position', [700, 200, 750, 225]);
set_param([mdl '/ws_mode'], 'VariableName', 'mode_data');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_volt'], ...
    'Position', [700, 250, 750, 275]);
set_param([mdl '/ws_volt'], 'VariableName', 'volt_data');
add_line(mdl, 'Motor_Plant/1', 'ws_speed/1');
add_line(mdl, 'Chart/1', 'ws_mode/1');
add_line(mdl, 'Chart/2', 'ws_volt/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 2 步：运行仿真 =====

fprintf('【Step 2: 仿真 — 观察状态切换】\n');

set_param(mdl, 'StopTime', '12');
simOut = sim(mdl);
t = simOut.tout;
speed = getSimData(simOut, 'speed_data', t);
mode  = getSimData(simOut, 'mode_data', t);
volt  = getSimData(simOut, 'volt_data', t);

close_system(mdl, 0);
fprintf('  [OK] 模型已保存到 models/%s.slx\n', mdl);

%% ===== 第 3 步：绘图 =====

figure('Name', 't36: Stateflow 电机状态机', 'Position', [50, 50, 1000, 700]);

subplot(3,1,1);
plot(t, speed, 'b', 'LineWidth', 2); hold on;
yline(80, 'r--'); yline(100, 'g--');
legend('电机转速', '启动阈值 80%', '目标 100%', 'Location', 'southeast');
title('电机转速 — 软启动 → 正常运行 → 紧急停机');
ylabel('Speed (%)'); grid on;

subplot(3,1,2);
stairs(t, mode, 'r', 'LineWidth', 2);
set(gca, 'YTick', [0 1 2], 'YTickLabel', {'OFF', 'STARTING', 'RUNNING'});
title('Stateflow 状态 — 模式自动切换');
ylabel('Mode'); ylim([-0.5 2.5]); grid on;

subplot(3,1,3);
plot(t, volt, 'm', 'LineWidth', 1.5);
title('控制电压输出 — STARTING 时缓慢爬升，RUNNING 时 PID 调节');
ylabel('Voltage (V)'); xlabel('Time (s)'); grid on;

sgtitle('教程 36：Stateflow 状态机 — 电机启停控制');

fprintf('\n========================================\n');
fprintf('  教程 36 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Stateflow = 图形化 if-else + 状态切换\n');
fprintf('     OFF → STARTING → RUNNING，条件写在箭头上\n\n');
fprintf('  2. en:/du:/ex: 动作\n');
fprintf('     en: = entry (进入时执行)\n');
fprintf('     du: = during (在状态中每步执行)\n');
fprintf('     ex: = exit (离开时执行)\n\n');
fprintf('  3. 紧急停止逻辑自动覆盖所有状态\n');
fprintf('     不管在 STARTING 还是 RUNNING，estop=1 → OFF\n\n');
fprintf('  4. Stateflow 输出直接驱动 Simulink 模块\n');
fprintf('     数据和事件通过 Input/Output Port 连接\n\n');
fprintf('  真实应用：飞控模式切换、BMS 电池保护、产线工序控制\n');
