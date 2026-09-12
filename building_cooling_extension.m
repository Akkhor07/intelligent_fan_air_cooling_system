%% building_cooling_extension.m
% Extension of "Intelligent Fan Air Cooling System" project
% ------------------------------------------------------------------
% The original IntelligentFan.slx models a BLDC-fan DRIVE (fuzzy speed
% reference -> PI speed loop -> VSI -> commutation logic -> motor
% electrical/mechanical dynamics). It does NOT model the thing the
% project is actually meant to demonstrate: that running the fan on
% cool night-time air moderates the BUILDING's temperature over a
% day/night cycle.
%
% This script closes that gap with a lightweight, fast-to-run
% alternative: an ODE-based single-zone thermal (RC) model of a room,
% driven by a realistic diurnal outdoor temperature profile, coupled
% to THREE different fan controllers so you can compare them:
%   1) Fuzzy Logic Controller  -> uses myflcc.fis directly (readfis/evalfis)
%   2) PID controller          -> classic alternative, tracks a comfort setpoint
%   3) Bang-bang thermostat    -> simplest baseline (fan full ON/OFF)
%
% Outputs: indoor temperature trajectories, fan energy used, and how
% many degree-hours each strategy keeps the room within comfort band.
%
% Requires: base MATLAB + Fuzzy Logic Toolbox (for readfis/evalfis).
% If the Fuzzy Logic Toolbox is not available, set USE_FIS_FILE = false
% below and the script falls back to a hand-coded equivalent fuzzy rule
% table so it still runs end to end.
% ------------------------------------------------------------------

clear; clc; close all;

%% ---------------- USER SETTINGS ----------------
USE_FIS_FILE   = true;              % true -> read myflcc.fis, false -> built-in fallback
fisPath        = 'myflcc.fis';      % must be on the MATLAB path / same folder
Tsim           = 48;                % hours to simulate (2 days)
dt             = 1/60;              % hours per step (1-minute resolution)
comfortLow     = 22;                % deg C, lower edge of comfort band
comfortHigh    = 26;                % deg C, upper edge of comfort band

%% ---------------- BUILDING THERMAL MODEL ----------------
% Single-zone lumped RC model:
%   C * dTin/dt = (Tout - Tin)/R_env + Qvent(speed) + Qgain(t) - Qsolar_reject
% C      : thermal capacitance of room air + structure  [Wh/°C]
% R_env  : envelope thermal resistance (walls/roof, no fan)  [°C/W]
% Qvent  : extra heat exchange from the fan pumping outdoor air in/out
% Qgain  : internal heat gains (occupants, appliances), daytime-biased
C      = 2500;         % Wh/°C  (a fairly light, well-insulated room)
R_env  = 0.02;         % °C/W   passive envelope leakage
k_fan  = 60;           % W per (°C * fan-speed-unit) -> forced ventilation gain
speedMax = 30;         % matches FIS output range [16 30]

% Diurnal outdoor temperature: low ~04:00, high ~15:00
t = (0:dt:Tsim)';                      % hours
Tout = 22 + 8*sin(2*pi*(t-9)/24 - pi/2);  % swings roughly 14-30 C

% Internal heat gains, higher in daytime (people/appliances/solar through windows)
Qgain = 300 + 250*max(0, sin(2*pi*(mod(t,24)-8)/24));  % Watts

%% ---------------- FUZZY CONTROLLER SETUP ----------------
if USE_FIS_FILE
    fis = readfis(fisPath);
    fanSpeedFuzzy = @(Tin,Tout_) evalfis(fis, [Tin, Tout_]);
else
    % Fallback: hand-coded equivalent of myflcc.fis rule table using
    % simple triangular membership + weighted average (Sugeno-style
    % approximation), so the script still runs without the toolbox.
    fanSpeedFuzzy = @(Tin,Tout_) fallbackFuzzy(Tin,Tout_);
end

%% ---------------- SIMULATE THE THREE STRATEGIES ----------------
[Tin_fuzzy, speed_fuzzy] = simulateBuilding(t,dt,Tout,Qgain,C,R_env,k_fan, ...
    @(Tin,idx) min(speedMax, max(0, fanSpeedFuzzy(clampT(Tin,15,35), clampT(Tout(idx),12,40)))));

[Tin_pid, speed_pid] = simulateBuilding(t,dt,Tout,Qgain,C,R_env,k_fan, ...
    @(Tin,idx) pidController(Tin, (comfortLow+comfortHigh)/2, dt, speedMax));

[Tin_bb, speed_bb] = simulateBuilding(t,dt,Tout,Qgain,C,R_env,k_fan, ...
    @(Tin,idx) bangBangController(Tin, comfortHigh, comfortLow, speedMax));

%% ---------------- METRICS ----------------
energy_fuzzy = trapz(t, (speed_fuzzy/speedMax).^3) ;  % fan-affinity-law proxy, kWh-ish units
energy_pid   = trapz(t, (speed_pid/speedMax).^3);
energy_bb    = trapz(t, (speed_bb/speedMax).^3);

comfortPct_fuzzy = 100*mean(Tin_fuzzy>=comfortLow & Tin_fuzzy<=comfortHigh);
comfortPct_pid   = 100*mean(Tin_pid  >=comfortLow & Tin_pid  <=comfortHigh);
comfortPct_bb    = 100*mean(Tin_bb   >=comfortLow & Tin_bb   <=comfortHigh);

fprintf('%-12s %10s %14s\n','Controller','Energy*','Comfort %');
fprintf('%-12s %10.2f %14.1f\n','Fuzzy',energy_fuzzy,comfortPct_fuzzy);
fprintf('%-12s %10.2f %14.1f\n','PID',  energy_pid,  comfortPct_pid);
fprintf('%-12s %10.2f %14.1f\n','BangBang',energy_bb,comfortPct_bb);
fprintf('*Energy in normalized (speed/speedMax)^3-hours, a fan-affinity-law proxy for power draw.\n');
%% ---------------- PARETO SWEEP: PID Kp vs comfort/energy ----------------
% linearize the plant to get tau (same as before, used for the ti=tau rule)
Tin0 = 24; Tout0 = 22; spd0 = 15;
A_lin = -1/(R_env*C) - (k_fan*spd0)/(speedMax*C);
tau = -1/A_lin;                    % ~31.25 h

KpList = [2 4 6 7.6 10 15 20];
KiList = KpList/tau;               % keep ti = tau rule from before

E = zeros(size(KpList)); Cmf = zeros(size(KpList));
for k = 1:numel(KpList)
    clear pidControllerGains     % reset persistent integrator state
    [Tin_k, spd_k] = simulateBuilding(t,dt,Tout,Qgain,C,R_env,k_fan, ...
        @(Tin,idx) pidControllerGains(Tin,(comfortLow+comfortHigh)/2,dt,speedMax,KpList(k),KiList(k)));
    E(k)   = trapz(t,(spd_k/speedMax).^3);
    Cmf(k) = 100*mean(Tin_k>=comfortLow & Tin_k<=comfortHigh);
end

%% ---------------- PLOTS ----------------
figure('Name','Indoor temperature comparison');
plot(t,Tout,'k:','LineWidth',1.2); hold on;
plot(t,Tin_fuzzy,'LineWidth',1.6);
plot(t,Tin_pid,'LineWidth',1.6);
plot(t,Tin_bb,'LineWidth',1.6);
yline(comfortLow,'--',Color=[0.5 0.5 0.5]);
yline(comfortHigh,'--',Color=[0.5 0.5 0.5]);
xlabel('Time (hours)'); ylabel('Temperature (°C)');
legend('Outdoor','Fuzzy (indoor)','PID (indoor)','Bang-bang (indoor)','Location','best');
title('Building temperature under three fan-control strategies'); grid on;

figure('Name','Fan speed comparison');
plot(t,speed_fuzzy,'LineWidth',1.2); hold on;
plot(t,speed_pid,'LineWidth',1.2);
plot(t,speed_bb,'LineWidth',1.2);
xlabel('Time (hours)'); ylabel('Fan speed command');
legend('Fuzzy','PID','Bang-bang','Location','best');
title('Commanded fan speed'); grid on;
figure('Name','Energy vs comfort trade-off');
plot(E, Cmf, 'o-','LineWidth',1.5); hold on
plot(energy_fuzzy, comfortPct_fuzzy, 'p','MarkerSize',12,'MarkerFaceColor','b');
plot(energy_bb, comfortPct_bb, 's','MarkerSize',10,'MarkerFaceColor','m');
for k=1:numel(KpList), text(E(k),Cmf(k),sprintf('  Kp=%.0f',KpList(k))); 
end
xlabel('Energy'); ylabel('Comfort %');
legend('PID sweep','Fuzzy','Bang-bang','Location','best');
title('Energy vs comfort trade-off'); grid on

%% =================== LOCAL FUNCTIONS ===================
function [Tin, speedLog] = simulateBuilding(t,dt,Tout,Qgain,C,R_env,k_fan,controllerFcn)
    N = numel(t);
    Tin = zeros(N,1);
    speedLog = zeros(N,1);
    Tin(1) = 24;  % initial indoor temp
    for i = 1:N-1
        spd = controllerFcn(Tin(i), i);
        speedLog(i) = spd;
        Qenv  = (Tout(i) - Tin(i))/R_env;
        Qvent = k_fan * spd/30 * (Tout(i) - Tin(i));  % only helps when Tout<Tin (night) or hurts if forced when Tout>Tin
        dTdt = (Qenv + Qvent + Qgain(i)) / C;          % °C/hour  (C in Wh/°C, Q in W -> matches since Wh/hr = W)
        Tin(i+1) = Tin(i) + dTdt*dt;
    end
    speedLog(N) = speedLog(N-1);
end

function T = clampT(T, lo, hi)
    T = min(max(T,lo),hi);
end

function spd = pidController(Tin, setpoint, dt, speedMax)
    persistent integ prevErr
    if isempty(integ), integ = 0; prevErr = 0; end
    Kp = 7.6; Ki = 0.24; Kd = 0.000;
    err = Tin - setpoint;            % positive error -> too hot -> speed up
    integ = integ + err*dt;
    deriv = (err - prevErr)/dt;
    prevErr = err;
    spd = Kp*err + Ki*integ + Kd*deriv;
    spd = min(speedMax, max(0, spd));
end
function spd = pidControllerGains(Tin, setpoint, dt, speedMax, Kp, Ki)
    persistent integ prevErr
    if isempty(integ), integ = 0; prevErr = 0; end
    err = Tin - setpoint;
    spd_unsat = Kp*err + Ki*integ;

    if ~( (spd_unsat > speedMax && err > 0) || (spd_unsat < 0 && err < 0) )
        integ = integ + err*dt;
    end
    prevErr = err; 
    spd = Kp*err + Ki*integ;
    spd = min(speedMax, max(0, spd));
end

function spd = bangBangController(Tin, hi, lo, speedMax)
    persistent state
    if isempty(state), state = 0; end
    if Tin > hi, state = 1; end
    if Tin < lo, state = 0; end
    spd = state*speedMax;
end

function spd = fallbackFuzzy(Tin,Tout)
    % Coarse stand-in for myflcc.fis if Fuzzy Logic Toolbox is unavailable.
    % NOTE: this deliberately FIXES the "hot-inside/cold-outside" case
    % (see write-up) to command max speed, unlike the original FIS rule.
    if Tin>28 && Tout<18
        spd = 30;               % free night-cooling opportunity -> fast
    elseif abs(Tin-Tout) < 3
        spd = 16;                % little benefit either way -> low
    elseif Tin > Tout
        spd = 16 + 14*min(1,(Tin-Tout)/12);  % scale with benefit
    else
        spd = 16;                % outdoor hotter than indoor -> keep low
    end
end
