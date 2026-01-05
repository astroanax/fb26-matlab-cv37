% Run Acceleration Simulation for Team Unwired C37
% Formula Bharat 2026 - MathWorks Modeling Award
%
% This script configures and runs the acceleration event simulation
% using the Simscape Formula Student Vehicle model with variable-step solver.
%
% Outputs:
% - Wheel speeds (all four wheels)
% - Tire forces (longitudinal, lateral, normal)
% - Vehicle acceleration (longitudinal, lateral)
% - Power delivery
% - 0-75m time
%
% Author: Team Unwired, NIT Calicut
% Date: January 2026

clear; clc; close all;

%% Load Vehicle Data
fprintf('==============================================\n');
fprintf('Team Unwired C37 - Acceleration Simulation\n');
fprintf('==============================================\n');

% Load vehicle parameters
run('Vehicle_data_C37_Unwired.m');

%% Load Track Data
fprintf('\nLoading acceleration track...\n');
load('results/acceleration_track_data.mat', 'Track');
fprintf('Track loaded: %.2f m length\n', Track.s(end));

%% Configure Simulation Model
mdl = 'sm_car';  % Simscape Formula Student Vehicle model name

% Check if model exists
if ~bdIsLoaded(mdl)
    fprintf('Loading Simscape model: %s\n', mdl);
    load_system(mdl);
end

% Configure solver for desktop simulation (variable-step)
% Variable-step solver provides better accuracy for transient maneuvers
fprintf('\nConfiguring variable-step solver...\n');
sm_car_config_solver(mdl, 'variable step');

% Solver selection rationale:
% - Variable-step (ode23t): Better accuracy for stiff systems (tire-road contact)
% - Adaptive time stepping: Handles rapid changes during acceleration
% - Suitable for offline simulation and detailed analysis
% Alternative: Fixed-step for real-time applications (not required here)

fprintf('Solver configured: ode23t (variable-step)\n');
fprintf('  Rationale: Adaptive stepping for accurate tire dynamics\n');
fprintf('  Stiff solver handles suspension compliance\n');

%% Set Simulation Parameters
% Maneuver: Straight-line acceleration (0-75m)
fprintf('\nConfiguring acceleration maneuver...\n');

% Driver inputs (for straight-line acceleration)
% Throttle: 100% from start
% Steering: 0 degrees (straight)
% Braking: 0%

% Set simulation stop time based on expected 0-75m time (~5 seconds)
set_param(mdl, 'StopTime', '8');  % Allow extra time for data collection

fprintf('Maneuver: 0-75m acceleration\n');
fprintf('Throttle: 100%% WOT\n');
fprintf('Expected time: ~4.5-5.0 seconds\n');

%% Run Simulation
fprintf('\n--- Running Simulation ---\n');
tic;
simOut = sim(mdl);
sim_time = toc;
fprintf('Simulation completed in %.2f seconds\n', sim_time);

%% Extract Results
fprintf('\nExtracting simulation results...\n');

% Get logged signals
logsout = simOut.logsout;

% Vehicle velocity
vel_signal = logsout.get('Velocity');
if ~isempty(vel_signal)
    time = vel_signal.Values.Time;
    vx = vel_signal.Values.Data(:,1);  % Longitudinal velocity
    vy = vel_signal.Values.Data(:,2);  % Lateral velocity
    fprintf('  - Vehicle velocity extracted\n');
end

% Vehicle position
pos_signal = logsout.get('Position');
if ~isempty(pos_signal)
    pos_x = pos_signal.Values.Data(:,1);  % X position
    pos_y = pos_signal.Values.Data(:,2);  % Y position
    fprintf('  - Vehicle position extracted\n');
end

% Wheel speeds
whl_spd_signal = logsout.get('Wheel Speeds');
if ~isempty(whl_spd_signal)
    whl_spd_FL = whl_spd_signal.Values.Data(:,1);
    whl_spd_FR = whl_spd_signal.Values.Data(:,2);
    whl_spd_RL = whl_spd_signal.Values.Data(:,3);
    whl_spd_RR = whl_spd_signal.Values.Data(:,4);
    fprintf('  - Wheel speeds extracted (all 4 corners)\n');
end

% Tire forces
tire_fx_signal = logsout.get('Tire Fx');
if ~isempty(tire_fx_signal)
    tire_fx_FL = tire_fx_signal.Values.Data(:,1);
    tire_fx_FR = tire_fx_signal.Values.Data(:,2);
    tire_fx_RL = tire_fx_signal.Values.Data(:,3);
    tire_fx_RR = tire_fx_signal.Values.Data(:,4);
    fprintf('  - Tire longitudinal forces extracted\n');
end

tire_fy_signal = logsout.get('Tire Fy');
if ~isempty(tire_fy_signal)
    tire_fy_FL = tire_fy_signal.Values.Data(:,1);
    tire_fy_FR = tire_fy_signal.Values.Data(:,2);
    tire_fy_RL = tire_fy_signal.Values.Data(:,3);
    tire_fy_RR = tire_fy_signal.Values.Data(:,4);
    fprintf('  - Tire lateral forces extracted\n');
end

tire_fz_signal = logsout.get('Tire Fz');
if ~isempty(tire_fz_signal)
    tire_fz_FL = tire_fz_signal.Values.Data(:,1);
    tire_fz_FR = tire_fz_signal.Values.Data(:,2);
    tire_fz_RL = tire_fz_signal.Values.Data(:,3);
    tire_fz_RR = tire_fz_signal.Values.Data(:,4);
    fprintf('  - Tire normal forces extracted\n');
end

% Acceleration
accel_signal = logsout.get('Acceleration');
if ~isempty(accel_signal)
    ax = accel_signal.Values.Data(:,1);  % Longitudinal acceleration
    ay = accel_signal.Values.Data(:,2);  % Lateral acceleration
    fprintf('  - Vehicle acceleration extracted\n');
end

% Engine/Power
power_signal = logsout.get('Engine Power');
if ~isempty(power_signal)
    engine_power = power_signal.Values.Data;
    fprintf('  - Engine power extracted\n');
end

%% Calculate Key Metrics
fprintf('\n--- Performance Metrics ---\n');

% Find 75m point
dist = sqrt(pos_x.^2 + pos_y.^2);
idx_75m = find(dist >= 75, 1);
if ~isempty(idx_75m)
    time_75m = time(idx_75m);
    vel_75m = vx(idx_75m) * 3.6;  % Convert to km/h
    fprintf('0-75m Time:        %.3f seconds\n', time_75m);
    fprintf('Velocity at 75m:   %.2f km/h\n', vel_75m);
else
    fprintf('WARNING: 75m distance not reached in simulation\n');
    time_75m = NaN;
    vel_75m = NaN;
end

% Peak acceleration
[max_ax, idx_max_ax] = max(ax);
fprintf('Peak Accel (ax):   %.2f m/s² (%.2f g)\n', max_ax, max_ax/9.81);
fprintf('  Occurred at:     %.2f seconds\n', time(idx_max_ax));

% Peak power
if exist('engine_power', 'var')
    max_power = max(engine_power);
    fprintf('Peak Power:        %.2f kW\n', max_power/1000);
end

% Average wheel speeds at 75m
if exist('whl_spd_FL', 'var') && ~isempty(idx_75m)
    avg_whl_spd = (whl_spd_FL(idx_75m) + whl_spd_FR(idx_75m) + ...
                   whl_spd_RL(idx_75m) + whl_spd_RR(idx_75m)) / 4;
    fprintf('Avg Wheel Speed:   %.2f rad/s (at 75m)\n', avg_whl_spd);
end

% Total tire forces at peak acceleration
if exist('tire_fx_FL', 'var')
    total_fx_peak = tire_fx_FL(idx_max_ax) + tire_fx_FR(idx_max_ax) + ...
                    tire_fx_RL(idx_max_ax) + tire_fx_RR(idx_max_ax);
    fprintf('Total Fx (peak):   %.1f N\n', total_fx_peak);
end

%% Save Results
fprintf('\n--- Saving Results ---\n');

% Create results structure
Results = struct();
Results.time = time;
Results.position = [pos_x, pos_y];
Results.velocity = [vx, vy];
Results.acceleration = [ax, ay];
Results.distance = dist;
Results.wheel_speeds = [whl_spd_FL, whl_spd_FR, whl_spd_RL, whl_spd_RR];
Results.tire_fx = [tire_fx_FL, tire_fx_FR, tire_fx_RL, tire_fx_RR];
Results.tire_fy = [tire_fy_FL, tire_fy_FR, tire_fy_RL, tire_fy_RR];
Results.tire_fz = [tire_fz_FL, tire_fz_FR, tire_fz_RL, tire_fz_RR];
if exist('engine_power', 'var')
    Results.engine_power = engine_power;
end
Results.metrics.time_75m = time_75m;
Results.metrics.vel_75m = vel_75m;
Results.metrics.max_ax = max_ax;
Results.metrics.max_power = max_power;

save('results/acceleration_simulation_results.mat', 'Results');
fprintf('Results saved to: results/acceleration_simulation_results.mat\n');

%% Generate Plots
fprintf('\n--- Generating Plots ---\n');

% Plot 1: Velocity vs Time
figure('Position', [100 100 1200 800]);

subplot(3,2,1);
plot(time, vx*3.6, 'b-', 'LineWidth', 2);
hold on;
if ~isnan(time_75m)
    plot(time_75m, vel_75m, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
grid on;
xlabel('Time (s)');
ylabel('Velocity (km/h)');
title('Longitudinal Velocity');
legend('Velocity', '75m Point', 'Location', 'southeast');

% Plot 2: Acceleration vs Time
subplot(3,2,2);
plot(time, ax, 'r-', 'LineWidth', 2);
hold on;
plot(time, ay, 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
title('Vehicle Acceleration');
legend('Longitudinal (ax)', 'Lateral (ay)', 'Location', 'northeast');

% Plot 3: Wheel Speeds
subplot(3,2,3);
plot(time, whl_spd_FL, 'r-', time, whl_spd_FR, 'b-', ...
     time, whl_spd_RL, 'r--', time, whl_spd_RR, 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Wheel Speed (rad/s)');
title('Wheel Speeds');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'southeast');

% Plot 4: Tire Longitudinal Forces
subplot(3,2,4);
plot(time, tire_fx_FL, 'r-', time, tire_fx_FR, 'b-', ...
     time, tire_fx_RL, 'r--', time, tire_fx_RR, 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Tire Force Fx (N)');
title('Tire Longitudinal Forces');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'southeast');

% Plot 5: Tire Normal Forces
subplot(3,2,5);
plot(time, tire_fz_FL, 'r-', time, tire_fz_FR, 'b-', ...
     time, tire_fz_RL, 'r--', time, tire_fz_RR, 'b--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Tire Force Fz (N)');
title('Tire Normal Forces');
legend('FL', 'FR', 'RL', 'RR', 'Location', 'northeast');

% Plot 6: Power vs Time
subplot(3,2,6);
if exist('engine_power', 'var')
    plot(time, engine_power/1000, 'g-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel('Power (kW)');
    title('Engine Power Delivery');
end

sgtitle('Team Unwired C37 - Acceleration Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
saveas(gcf, 'results/acceleration_results.png');
fprintf('Plots saved to: results/acceleration_results.png\n');

% Plot 2: Trajectory
figure('Position', [150 150 800 600]);
plot(pos_x, pos_y, 'b-', 'LineWidth', 2);
hold on;
if ~isempty(idx_75m)
    plot(pos_x(idx_75m), pos_y(idx_75m), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
end
plot(0, 0, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Vehicle Trajectory');
legend('Path', '75m Point', 'Start', 'Location', 'best');

saveas(gcf, 'results/acceleration_trajectory.png');
fprintf('Trajectory saved to: results/acceleration_trajectory.png\n');

fprintf('\n==============================================\n');
fprintf('Simulation Complete!\n');
fprintf('==============================================\n');
