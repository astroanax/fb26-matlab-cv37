function compare_simulation_vs_measured(sim_logsout, imu_data, test_name)
% COMPARE_SIMULATION_VS_MEASURED Compare Simscape simulation to IMU test data
%
% Syntax:
%   compare_simulation_vs_measured(sim_logsout, imu_data, test_name)
%
% Description:
%   Creates side-by-side comparison plots of simulation predictions and
%   measured IMU data for model validation.
%
% Inputs:
%   sim_logsout - Simulink.SimulationData.Dataset from Simscape simulation
%   imu_data    - Structure from import_imu_data()
%   test_name   - String describing the test (e.g., 'Acceleration', 'Skidpad')
%
% Output:
%   Generates comparison figure with multiple subplots
%
% Validation Metrics:
%   Computes RMSE and correlation for each comparable signal
%
% Author: Team Unwired
% Date: January 2026

fprintf('=== Simulation vs Measured Comparison: %s ===\n\n', test_name);

% Extract simulation data
VehBus = sim_logsout.get('VehBus');

% Get simulation time and signals
t_sim = VehBus.Values.Chassis.Body.CG.gx.Time;
gx_sim = VehBus.Values.Chassis.Body.CG.gx.Data;
gy_sim = VehBus.Values.Chassis.Body.CG.gy.Data;
nYaw_sim = VehBus.Values.World.nYaw.Data;
nRoll_sim = VehBus.Values.World.nRoll.Data;
nPitch_sim = VehBus.Values.World.nPitch.Data;

% Get measured time and signals
t_meas = imu_data.time;
gx_meas = imu_data.ax;
gy_meas = imu_data.ay;
nYaw_meas = imu_data.gz;
nRoll_meas = imu_data.gx;
nPitch_meas = imu_data.gy;

% Time alignment: find overlap
t_start = max(min(t_sim), min(t_meas));
t_end = min(max(t_sim), max(t_meas));

if t_end <= t_start
    error('No time overlap between simulation and measured data.');
end

fprintf('Time overlap: %.2f to %.2f seconds\n', t_start, t_end);

% Resample to common time base
dt = 0.01;  % 100 Hz
t_common = (t_start:dt:t_end)';

% Interpolate signals to common time base
gx_sim_i = interp1(t_sim, gx_sim, t_common, 'linear', 'extrap');
gy_sim_i = interp1(t_sim, gy_sim, t_common, 'linear', 'extrap');
nYaw_sim_i = interp1(t_sim, nYaw_sim, t_common, 'linear', 'extrap');

gx_meas_i = interp1(t_meas, gx_meas, t_common, 'linear', 'extrap');
gy_meas_i = interp1(t_meas, gy_meas, t_common, 'linear', 'extrap');
nYaw_meas_i = interp1(t_meas, nYaw_meas, t_common, 'linear', 'extrap');

% Calculate validation metrics
rmse_gx = sqrt(mean((gx_sim_i - gx_meas_i).^2));
rmse_gy = sqrt(mean((gy_sim_i - gy_meas_i).^2));
rmse_nYaw = sqrt(mean((nYaw_sim_i - nYaw_meas_i).^2));

corr_gx = corrcoef(gx_sim_i, gx_meas_i); corr_gx = corr_gx(1,2);
corr_gy = corrcoef(gy_sim_i, gy_meas_i); corr_gy = corr_gy(1,2);
corr_nYaw = corrcoef(nYaw_sim_i, nYaw_meas_i); corr_nYaw = corr_nYaw(1,2);

fprintf('\nValidation Metrics:\n');
fprintf('  Longitudinal Acc: RMSE = %.3f m/s^2, Correlation = %.3f\n', rmse_gx, corr_gx);
fprintf('  Lateral Acc:      RMSE = %.3f m/s^2, Correlation = %.3f\n', rmse_gy, corr_gy);
fprintf('  Yaw Rate:         RMSE = %.3f rad/s, Correlation = %.3f\n', rmse_nYaw, corr_nYaw);

% Generate comparison plots
figure('Name', sprintf('Validation: %s', test_name), 'Position', [50 50 1400 900]);

% Plot 1: Longitudinal Acceleration
subplot(2, 3, 1);
plot(t_common, gx_sim_i, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulation');
hold on;
plot(t_common, gx_meas_i, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
xlabel('Time (s)');
ylabel('Longitudinal Acceleration (m/s^2)');
title(sprintf('Longitudinal Acceleration\nRMSE=%.3f, r=%.3f', rmse_gx, corr_gx));
legend('Location', 'best');
grid on;

% Plot 2: Lateral Acceleration
subplot(2, 3, 2);
plot(t_common, gy_sim_i, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulation');
hold on;
plot(t_common, gy_meas_i, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
xlabel('Time (s)');
ylabel('Lateral Acceleration (m/s^2)');
title(sprintf('Lateral Acceleration\nRMSE=%.3f, r=%.3f', rmse_gy, corr_gy));
legend('Location', 'best');
grid on;

% Plot 3: Yaw Rate
subplot(2, 3, 3);
plot(t_common, rad2deg(nYaw_sim_i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Simulation');
hold on;
plot(t_common, rad2deg(nYaw_meas_i), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
xlabel('Time (s)');
ylabel('Yaw Rate (deg/s)');
title(sprintf('Yaw Rate\nRMSE=%.3f rad/s, r=%.3f', rmse_nYaw, corr_nYaw));
legend('Location', 'best');
grid on;

% Plot 4: GG Diagram Overlay
subplot(2, 3, 4);
scatter(gy_sim_i/9.81, gx_sim_i/9.81, 10, 'b', 'filled', 'DisplayName', 'Simulation');
hold on;
scatter(gy_meas_i/9.81, gx_meas_i/9.81, 10, 'r', 'filled', 'DisplayName', 'Measured');
xlabel('Lateral Acceleration (g)');
ylabel('Longitudinal Acceleration (g)');
title('GG Diagram Comparison');
legend('Location', 'best');
axis equal;
grid on;

% Draw friction circles
max_g_sim = max(sqrt(gx_sim_i.^2 + gy_sim_i.^2))/9.81;
max_g_meas = max(sqrt(gx_meas_i.^2 + gy_meas_i.^2))/9.81;
theta = linspace(0, 2*pi, 100);
plot(max_g_sim*cos(theta), max_g_sim*sin(theta), 'b--', 'LineWidth', 1);
plot(max_g_meas*cos(theta), max_g_meas*sin(theta), 'r--', 'LineWidth', 1);

% Plot 5: Error histogram
subplot(2, 3, 5);
errors = [gx_sim_i - gx_meas_i; gy_sim_i - gy_meas_i];
histogram(errors, 50, 'Normalization', 'probability', 'FaceColor', [0.3 0.3 0.7]);
xlabel('Acceleration Error (m/s^2)');
ylabel('Probability');
title('Acceleration Prediction Error Distribution');
xline(0, 'r--', 'LineWidth', 2);
grid on;

% Plot 6: Correlation scatter
subplot(2, 3, 6);
scatter(gx_meas_i, gx_sim_i, 10, 'b', 'filled');
hold on;
scatter(gy_meas_i, gy_sim_i, 10, 'r', 'filled');
% Identity line
lims = [min([gx_meas_i; gy_meas_i; gx_sim_i; gy_sim_i]), ...
        max([gx_meas_i; gy_meas_i; gx_sim_i; gy_sim_i])];
plot(lims, lims, 'k--', 'LineWidth', 2);
xlabel('Measured Acceleration (m/s^2)');
ylabel('Simulated Acceleration (m/s^2)');
title('Correlation: Simulated vs Measured');
legend('Longitudinal', 'Lateral', 'Perfect Match', 'Location', 'best');
axis equal;
grid on;

sgtitle(sprintf('Team Unwired C37 - %s Test Validation', test_name));

% Print summary for report
fprintf('\n=== Validation Summary for Report ===\n');
fprintf('Test: %s\n', test_name);
fprintf('Duration: %.2f s\n', t_end - t_start);
fprintf('Sample Rate: %.0f Hz\n', 1/dt);
fprintf('\nAccuracy Metrics:\n');
fprintf('| Signal | RMSE | Correlation |\n');
fprintf('|--------|------|-------------|\n');
fprintf('| Long. Acc. | %.3f m/s^2 | %.3f |\n', rmse_gx, corr_gx);
fprintf('| Lat. Acc.  | %.3f m/s^2 | %.3f |\n', rmse_gy, corr_gy);
fprintf('| Yaw Rate   | %.3f rad/s | %.3f |\n', rmse_nYaw, corr_nYaw);
fprintf('\nPeak Values:\n');
fprintf('| Signal | Simulation | Measured | Error |\n');
fprintf('|--------|------------|----------|-------|\n');
fprintf('| Max Long. g | %.2f g | %.2f g | %.1f%% |\n', ...
    max(abs(gx_sim_i))/9.81, max(abs(gx_meas_i))/9.81, ...
    100*(max(abs(gx_sim_i)) - max(abs(gx_meas_i)))/max(abs(gx_meas_i)));
fprintf('| Max Lat. g  | %.2f g | %.2f g | %.1f%% |\n', ...
    max(abs(gy_sim_i))/9.81, max(abs(gy_meas_i))/9.81, ...
    100*(max(abs(gy_sim_i)) - max(abs(gy_meas_i)))/max(abs(gy_meas_i)));

fprintf('\n=== Comparison Complete ===\n');

end
