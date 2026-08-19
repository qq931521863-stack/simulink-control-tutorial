%% ============================================================
% 教程 43：Model Advisor — 让 Simulink 帮你查问题
%
% 【为什么要学这课】
%   你的模型能跑出结果 ≠ 你的模型没问题。
%   隐藏的坑包括：
%     - 代数环 (algebraic loop): 仿真变慢甚至发散
%     - 未初始化的积分器: 结果依赖"上次残留的值"
%     - 数据类型不匹配: 单精度 vs 双精度悄悄截断
%     - 除零风险: 输入为零时 Gain 或 Product 块炸了
%
%   Model Advisor 是 Simulink 自带的"静态分析工具"——
%   和编译器 warning 一样，在运行前就把潜在问题标出来。
%
% ┌─────────────────────────────────────────────────────────┐
% │ Model Advisor 检查项 (部分)                              │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  - Check for unconnected lines / ports                  │
% │  - Identify algebraic loops                             │
% │  - Check solver settings                                │
% │  - Check for optimization settings                      │
% │  - Model metrics (complexity, hierarchy depth)          │
% │  - Code generation readiness (MathWorks 推荐)           │
% │                                                         │
% │  本课: 搭一个"有问题的模型" → 跑 Advisor → 逐个修      │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 用 Model Advisor 扫描模型问题
%   2. 理解最常见的 warning 及其修复
%   3. 建立"提交前跑 Advisor"的习惯
% ============================================================

clear; close all; bdclose('all');
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

fprintf('============================================\n');
fprintf('  教程 43：Model Advisor\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：故意搭一个"有问题的模型" =====

fprintf('【Step 1: 搭建测试模型 (故意留下问题)】\n');

mdl = 'tutorial43_advisor';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

add_block('simulink/Sources/Sine Wave', [mdl '/Signal'], ...
    'Position', [30, 60, 90, 90]);
set_param([mdl '/Signal'], 'Amplitude', '1', 'Frequency', '1');

% 问题 1: 未连接的端口 (故意留一根悬空线)
add_block('simulink/Math Operations/Gain', [mdl '/Gain_unused'], ...
    'Position', [150, 200, 200, 230]);
set_param([mdl '/Gain_unused'], 'Gain', '10');

% 正常路径
add_block('simulink/Math Operations/Gain', [mdl '/Gain'], ...
    'Position', [150, 55, 200, 95]);
set_param([mdl '/Gain'], 'Gain', '5');

add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'Position', [260, 55, 310, 95]);
% 问题 2: 积分器初始值没有明确设置 (依赖默认值)
set_param([mdl '/Integrator'], 'InitialCondition', '0');  % 显式声明初始值

add_block('simulink/Discontinuities/Saturation', [mdl '/Saturation'], ...
    'Position', [360, 55, 400, 95]);
set_param([mdl '/Saturation'], 'UpperLimit', '10', 'LowerLimit', '-10');

% 问题 3: 故意制造一个代数环 (feedback through direct feedthrough)
add_block('simulink/Math Operations/Sum', [mdl '/SumLoop'], ...
    'Position', [260, 200, 290, 260]);
set_param([mdl '/SumLoop'], 'Inputs', '|++');

add_block('simulink/Math Operations/Gain', [mdl '/Gain_fb'], ...
    'Position', [150, 220, 200, 250]);
set_param([mdl '/Gain_fb'], 'Gain', '0.5');

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_out'], ...
    'Position', [460, 60, 510, 85]);
set_param([mdl '/ws_out'], 'VariableName', 'advisor_out');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_loop'], ...
    'Position', [460, 220, 510, 245]);
set_param([mdl '/ws_loop'], 'VariableName', 'loop_out');

% 正常路径连线
add_line(mdl, 'Signal/1', 'Gain/1');
add_line(mdl, 'Gain/1', 'Integrator/1');
add_line(mdl, 'Integrator/1', 'Saturation/1');
add_line(mdl, 'Saturation/1', 'ws_out/1');

% 代数环连线: Signal → SumLoop → Gain_fb → SumLoop (没有延迟!)
add_line(mdl, 'Signal/1', 'SumLoop/1');
add_line(mdl, 'Gain_fb/1', 'SumLoop/2');
add_line(mdl, 'SumLoop/1', 'Gain_fb/1');  % 反馈环路!
add_line(mdl, 'SumLoop/1', 'ws_loop/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
model_path = fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']);
save_system(mdl, model_path);

%% ===== 第 2 步：跑 Model Advisor =====

fprintf('【Step 2: 运行 Model Advisor】\n\n');

% 使用 ModelAdvisor.run() API
try
    ma = Simulink.ModelAdvisor.getModelAdvisor(mdl);
    fprintf('  [OK] Model Advisor 对象已获取（GUI 方式: 菜单 Analysis > Model Advisor）\n');
catch ME
    fprintf('  [INFO] 编程式 Model Advisor 需要交互环境: %s\n', ME.message);
end

% 使用编程方式检查关键项
fprintf('  --- 编程检查 ---\n\n');

% 检查 1: 未连接的端口
unconnected = find_system(mdl, 'FindAll', 'on', 'Type', 'port', 'Line', -1);
if isempty(unconnected)
    fprintf('  ✓ 没有未连接的端口\n');
else
    fprintf('  ✗ 发现 %d 个未连接端口\n', length(unconnected));
    for i = 1:min(3, length(unconnected))
        port = unconnected(i);
        parent = get_param(port, 'Parent');
        fprintf('    - %s\n', parent);
    end
end

% 检查 2: 代数环
try
    al = Simulink.BlockDiagram.getAlgebraicLoops(mdl);
    if isempty(al)
        fprintf('  ✓ 没有代数环\n');
    else
        fprintf('  ✗ 发现 %d 个代数环 (会导致仿真变慢/发散)\n', length(al));
        for i = 1:min(3, length(al))
            fprintf('    - 环路包括 %d 个块\n', length(al(i).Blocks));
        end
    end
catch
    fprintf('  [WARN] 无法检查代数环 (需要先编译模型)\n');
end

% 检查 3: 求解器设置
solver = get_param(mdl, 'Solver');
fprintf('  → 求解器: %s\n', solver);
if strcmp(solver, 'VariableStepAuto')
    fprintf('    [WARN] 使用自动求解器 → 建议明确指定 ode45 或 ode4\n');
end

fprintf('\n');

%% ===== 第 3 步：仿真 + 观察问题 =====

fprintf('【Step 3: 仿真测试 — 代数环的影响】\n');

set_param(mdl, 'StopTime', '5');
set_param(mdl, 'Solver', 'ode45');
try
    simOut = sim(mdl);
    t = simOut.tout;
    y = getSimData(simOut, 'advisor_out', t);

    fprintf('  [OK] 仿真完成 (正常路径)\n');
    fprintf('  但注意: 代数环的存在让每步都需要迭代求解\n');
    fprintf('  对于大模型, 这会让仿真速度降低 10-100 倍\n\n');
catch ME
    fprintf('  [FAIL] 仿真失败: %s\n', ME.message);
end

close_system(mdl, 0);

%% ===== 第 4 步：修复示范 =====

fprintf('【Step 4: 修复三个常见问题】\n\n');
fprintf('  问题 1: 未连接端口\n');
fprintf('    修复: 删除 Gain_unused 或连接它\n\n');
fprintf('  问题 2: 积分器初始值未明确设置\n');
fprintf('    修复: set_param(blk, ''InitialCondition'', ''0'')\n');
fprintf('    虽然默认值是 0, 但显式声明避免"依赖默认行为"\n\n');
fprintf('  问题 3: 代数环\n');
fprintf('    修复: 在反馈路径上加一个 Memory 块或 Unit Delay\n');
fprintf('    → 打断 direct feedthrough, 代数环消失\n\n');

fprintf('========================================\n');
fprintf('  教程 43 完成！\n');
fprintf('========================================\n\n');
fprintf('【关键收获】\n');
fprintf('  1. Model Advisor = Simulink 的 Linter\n');
fprintf('     帮你找出隐藏的建模问题, 不用等到仿真出错才知道\n\n');
fprintf('  2. 最常见的三个问题:\n');
fprintf('     - 未连接端口 (漏接线)\n');
fprintf('     - 代数环 (直接反馈 → 加 Memory/Unit Delay)\n');
fprintf('     - 数据类型/求解器设置不当\n\n');
fprintf('  3. 养成习惯: 提交/发布前跑一次 Advisor\n');
fprintf('     ModelAdvisor.run(''your_model'')\n');
fprintf('     → 像 git commit 前跑 linter 一样\n\n');
fprintf('  4. 这 8 个专题 (t36-t43) 补全了 Simulink\n');
fprintf('     从"会用"到"会做一个正经工程"的断层\n');
