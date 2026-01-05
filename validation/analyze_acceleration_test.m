function results = analyze_acceleration_test(imu_data)
% ANALYZE_ACCELERATION_TEST Process IMU data from straight-line acceleration test
%
% Syntax:
%   results = analyze_acceleration_test(imu_data)
%
% Description:
%   Analyzes IMU data from a standing start acceleration test to extract
%   performance metrics for powertrain validation.
%
% Input:
%   imu_data - Structure from import_imu_data()
%
% Output:
%   results - Structure containing:
%       .time_0_to_75m  - Time to cover 75 m (s)
%       .peak_ax        - Maximum longitudinal acceleration (m/s^2)
%       .avg_ax         - Average acceleration during run (m/s^2)
%       .velocity       - Velocity profile (m/s)
%       .distance       - Distance profile (m)
%       .time           - Time vector (s)
%       .friction_coeff - Estimated traction-limited friction coefficient
%
% Competition Context:
%   Formula Bharat acceleration event is 75 m from standing start.
%   This analysis validates powertrain model output.
%
% Author: Team Unwired
% Date: January 2026

fprintf('=== Acceleration Test Analysis ===\n\n');

t = imu_data.time;
ax = imu_data.ax;  % Longitudinal acceleration (m/s^2)
ay = imu_data.ay;  % Lateral acceleration (for traction check)

% Detect start of acceleration run
% Look for sustained positive acceleration above threshold
threshold = 0.5;  % m/s^2
window = 10;      % samples for moving average

ax_smooth = movmean(ax, window);
start_idx = find(ax_smooth > threshold, 1, 'first');

if isempty(start_idx)
    warning('No significant acceleration detected. Check data quality.');
    start_idx = 1;
end

% Trim data from start of run
t = t(start_idx:end) - t(start_idx);
ax = ax(start_idx:end);
ay = ay(start_idx:end);

% Integrate acceleration to get velocity
velocity = cumtrapz(t, ax);

% Integrate velocity to get distance
distance = cumtrapz(t, velocity);

% Find time to 75 m (competition distance)
target_distance = 75;  % meters
idx_75m = find(distance >= target_distance, 1, 'first');

if isempty(idx_75m)
    fprintf('Warning: Vehicle did not reach 75 m in recorded data.\n');
    fprintf('Maximum distance: %.1f m\n', max(distance));
    time_0_to_75m = NaN;
else
    time_0_to_75m = t(idx_75m);
    fprintf('Time to 75 m: %.2f s\n', time_0_to_75m);
end

% Calculate peak and average acceleration
peak_ax = max(ax);
avg_ax = mean(ax(velocity > 0.5));  % Average when moving

fprintf('Peak acceleration: %.2f m/s^2 (%.2f g)\n', peak_ax, peak_ax/9.81);
fprintf('Average acceleration: %.2f m/s^2 (%.2f g)\n', avg_ax, avg_ax/9.81);

% Estimate friction coefficient from peak acceleration
% At launch, traction limited: F = m*a = mu*N = mu*m*g*(rear_weight_fraction + weight_transfer)
% Simplified: mu_eff = a/g (assuming all weight on drive wheels)
friction_coeff = peak_ax / 9.81;
fprintf('Estimated traction coefficient: %.2f\n', friction_coeff);

% Check for lateral deviation (driver steering during run)
max_lateral = max(abs(ay));
fprintf('Maximum lateral acceleration: %.2f m/s^2 (should be < 0.5)\n', max_lateral);

if max_lateral > 1.0
    fprintf('Warning: Significant lateral acceleration detected.\n');
    fprintf('         This may indicate steering input or uneven traction.\n');
end

% Maximum speed achieved
max_speed = max(velocity);
fprintf('Maximum speed: %.1f m/s (%.1f km/h)\n', max_speed, max_speed*3.6);

% Create result structure
results.time_0_to_75m = time_0_to_75m;
results.peak_ax = peak_ax;
results.avg_ax = avg_ax;
results.friction_coeff = friction_coeff;
results.velocity = velocity;
results.distance = distance;
results.time = t;
results.max_speed = max_speed;

% Generate plots
figure('Name', 'Acceleration Test Analysis', 'Position', [100 100 1200 800]);

% Plot 1: Velocity vs Time
subplot(2, 2, 1);
plot(t, velocity, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Profile');
grid on;

if ~isnan(time_0_to_75m)
    hold on;
    xline(time_0_to_75m, 'r--', sprintf('75m @ %.2fs', time_0_to_75m));
    hold off;
end

% Plot 2: Distance vs Time
subplot(2, 2, 2);
plot(t, distance, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Distance (m)');
title('Distance Profile');
grid on;
yline(75, 'r--', '75 m finish');

% Plot 3: Acceleration vs Time
subplot(2, 2, 3);
plot(t, ax, 'b-', 'LineWidth', 1);
hold on;
plot(t, movmean(ax, window), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Longitudinal Acceleration (m/s^2)');
title('Acceleration Profile');
legend('Raw', 'Smoothed', 'Location', 'best');
grid on;

% Plot 4: Acceleration vs Velocity
subplot(2, 2, 4);
plot(velocity, ax, 'b.', 'MarkerSize', 2);
hold on;
% Bin and average for trend line
v_bins = linspace(0, max(velocity), 20);
ax_binned = zeros(size(v_bins)-1);
for i = 1:length(v_bins)-1
    idx = velocity >= v_bins(i) & velocity < v_bins(i+1);
    if sum(idx) > 0
        ax_binned(i) = mean(ax(idx));
    end
end
v_centers = (v_bins(1:end-1) + v_bins(2:end))/2;
plot(v_centers, ax_binned, 'r-', 'LineWidth', 2);
xlabel('Velocity (m/s)');
ylabel('Acceleration (m/s^2)');
title('Acceleration vs Velocity (Shows Power Limiting)');
legend('Data', 'Trend', 'Location', 'best');
grid on;

sgtitle('Team Unwired C37 Acceleration Test Analysis');

fprintf('\n=== Analysis Complete ===\n');

end
