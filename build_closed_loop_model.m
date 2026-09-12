%% build_closed_loop_model.m
% Builds a NEW model, IntelligentFan_ClosedLoop.slx, that couples:
%   - the existing Fuzzy Logic Controller (myflcc.fis)
%   - a reduced-order (1st-order lag) stand-in for the BLDC drive's
%     step response, so the sim doesn't have to resolve millisecond
%     electrical dynamics for a multi-day run
%   - a Simulink "Building Thermal Plant" subsystem implementing the
%     same RC model used in building_cooling_extension.m
%
% This is a SCAFFOLD: block/port names are set programmatically so it
% should run as-is, but Simulink's API is picky about exact geometry;
% if a line fails to connect, open the model and drag it manually --
% the block list and math are the part that matters.
%
% ---- STEP 0 (do this first, in the ORIGINAL IntelligentFan.slx) ----
% Open IntelligentFan.slx, change 'Set Point' (indoor) to a step from
% 25->30 at t=0, run it, and read the settling time of Wr_mech on the
% scope (time to reach ~98% of final value). Put that number below as
% tau_motor. This calibrates the reduced-order model to your real drive.
tau_motor = 0.156;     % <-- REPLACE with your measured settling time (s->hr, see below)
tau_motor_hr = tau_motor/3600;   % convert seconds to hours (sim runs in hours)

%% ---------------- Build new model ----------------
modelName = 'IntelligentFan_ClosedLoop';
if bdIsLoaded(modelName), close_system(modelName,0); end
new_system(modelName);
open_system(modelName);

%% ---- Outdoor temperature source (diurnal profile) ----
add_block('simulink/Sources/Sine Wave', [modelName '/Outdoor Temp Base'], ...
    'Position',[30 30 90 60], 'Amplitude','8', 'Bias','22', ...
    'Frequency', num2str(2*pi/24), 'Phase', num2str(-pi/2 - 2*pi*9/24));

%% ---- Internal heat gain source (simplified daytime bump) ----
add_block('simulink/Sources/Sine Wave', [modelName '/Heat Gain'], ...
    'Position',[30 120 90 150], 'Amplitude','250','Bias','300', ...
    'Frequency', num2str(2*pi/24), 'Phase', num2str(-2*pi*8/24));
add_block('simulink/Discontinuities/Saturation', [modelName '/Gain Floor'], ...
    'Position',[110 120 150 150], 'UpperLimit','700','LowerLimit','300');

%% ---- Fuzzy Logic Controller (reuses myflcc.fis) ----
open_system('fuzblock');   % opens the Fuzzy Logic Toolbox block library
fisBlockPath = find_system('fuzblock','SearchDepth',1,'Name','Fuzzy Logic Controller');
fisBlockPath = fisBlockPath{1};   % e.g. 'fuzblock/Fuzzy Logic Controller'
close_system('fuzblock');

add_block(fisBlockPath, [modelName '/Fuzzy Logic Controller'], 'Position',[200 30 320 90]);
set_param([modelName '/Fuzzy Logic Controller'], 'fis_matrix', '''myflcc''');

%% ---- Reduced-order motor lag: speed_ref -> actual speed ----
add_block('simulink/Continuous/Transfer Fcn', [modelName '/Motor Lag (reduced order)'], ...
    'Position',[360 30 460 60], ...
    'Numerator','[1]', 'Denominator', ['[' num2str(tau_motor_hr) ' 1]']);

%% ---- Building Thermal Plant subsystem ----
plantPath = [modelName '/Building Thermal Plant'];
add_block('simulink/Ports & Subsystems/Subsystem', plantPath, 'Position',[500 30 650 160]);
delete_line(plantPath, 'In1/1','Out1/1'); %#ok<*TRYNC>  % clear default wiring if present, ignore errors
open_system(plantPath);

% inputs: FanSpeed (0-30), Tout, Qgain  | output: Tin
add_block('simulink/Sources/In1',[plantPath '/FanSpeed'],'Position',[30 30 60 50]);
add_block('simulink/Sources/In1',[plantPath '/Tout'],   'Position',[30 100 60 120]);
add_block('simulink/Sources/In1',[plantPath '/Qgain'],  'Position',[30 170 60 190]);
add_block('simulink/Sinks/Out1',[plantPath '/Tin'],     'Position',[560 90 590 110]);

% (Tout - Tin) shared term
add_block('simulink/Math Operations/Sum',[plantPath '/Tout-Tin'], ...
    'Position',[120 90 145 120],'Inputs','+-');
% Qenv = (Tout-Tin)/R_env
add_block('simulink/Math Operations/Gain',[plantPath '/1_R_env'], ...
    'Position',[180 60 220 90],'Gain', num2str(1/0.02));
% Qvent = k_fan * (FanSpeed/30) * (Tout-Tin)
add_block('simulink/Math Operations/Gain',[plantPath '/Speed_norm'], ...
    'Position',[70 30 100 50],'Gain','1/30');
add_block('simulink/Math Operations/Product',[plantPath '/Qvent_mult'], ...
    'Position',[220 30 250 60]);
add_block('simulink/Math Operations/Gain',[plantPath '/k_fan'], ...
    'Position',[280 30 310 60],'Gain','60');
% sum all heat terms, divide by C, integrate
add_block('simulink/Math Operations/Add',[plantPath '/Qtotal'], ...
    'Position',[340 90 370 130],'Inputs','+++');
add_block('simulink/Math Operations/Gain',[plantPath '/1_C'], ...
    'Position',[400 90 430 120],'Gain', num2str(1/2500));
add_block('simulink/Continuous/Integrator',[plantPath '/Integrator'], ...
    'Position',[460 90 500 120],'InitialCondition','24');

% wiring inside the plant subsystem
add_line(plantPath,'Tout/1','Tout-Tin/1');
add_line(plantPath,'Integrator/1','Tout-Tin/2');   % Tin feeds back into (Tout-Tin)
add_line(plantPath,'Tout-Tin/1','1_R_env/1');
add_line(plantPath,'FanSpeed/1','Speed_norm/1');
add_line(plantPath,'Speed_norm/1','Qvent_mult/1');
add_line(plantPath,'Tout-Tin/1','Qvent_mult/2');
add_line(plantPath,'Qvent_mult/1','k_fan/1');
add_line(plantPath,'1_R_env/1','Qtotal/1');
add_line(plantPath,'k_fan/1','Qtotal/2');
add_line(plantPath,'Qgain/1','Qtotal/3');
add_line(plantPath,'Qtotal/1','1_C/1');
add_line(plantPath,'1_C/1','Integrator/1');
add_line(plantPath,'Integrator/1','Tin/1');
close_system(plantPath);

%% ---- Scope ----
add_block('simulink/Sinks/Scope',[modelName '/Tin vs Tout'], ...
    'Position',[700 30 740 60], 'NumInputPorts','2');

%% ---- Top-level wiring ----
add_line(modelName,'Outdoor Temp Base/1','Fuzzy Logic Controller/2');
add_line(modelName,'Heat Gain/1','Gain Floor/1');

% FLC needs Indoor Temp too -> feed back from plant output (create signal
% from the plant's Tin outport once it exists as a top-level line)
add_line(modelName,'Fuzzy Logic Controller/1','Motor Lag (reduced order)/1');
add_line(modelName,'Motor Lag (reduced order)/1','Building Thermal Plant/1');
add_line(modelName,'Outdoor Temp Base/1','Building Thermal Plant/2');
add_line(modelName,'Gain Floor/1','Building Thermal Plant/3');
add_line(modelName,'Building Thermal Plant/1','Tin vs Tout/1');
add_line(modelName,'Outdoor Temp Base/1','Tin vs Tout/2');

% close the loop: plant's Tin feeds back into FLC input 1
add_line(modelName,'Building Thermal Plant/1','Fuzzy Logic Controller/1');

%% ---- Solver settings: this is a stiff, multi-day system ----
set_param(modelName,'StopTime','240');        % 240 hours = 10 days
set_param(modelName,'Solver','ode15s');       % stiff solver: fast motor lag + slow thermal
set_param(modelName,'MaxStep','0.05');        % hours; tighten if you see warnings

save_system(modelName, fullfile(pwd,[modelName '.slx']));
fprintf('Built %s.slx -- open it, inspect the wiring, then press Run.\n', modelName);
