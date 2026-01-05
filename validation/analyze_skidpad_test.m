function results = analyze_skidpad_test(imu_data)
% ANALYZE_SKIDPAD_TEST Process IMU data from skidpad (circular) test
%
% Syntax:
%   results = analyze_skidpad_test(imu_data)
%
% Description:
%   Analyzes IMU data from steady-state cornering on a skidpad circle
%   to extract tire grip and handling characteristics.
%
% Input:
%   imu_data - Structure from import_imu_data()
%
% Output:
%   results - Structure containing:
%       .max_lateral_g  - Maximum sustained lateral acceleration (g)
%       .friction_coeff - Tire-road friction coefficient (mu)
%       .yaw_rate       - Yaw rate during steady state (rad/s)
%       .lap_time       - Estimated lap time for 18.25m radius (s)
%       .corner_speed   - Steady state cornering speed (m/s)
%
% Competition Context:
%   Formula Bharat skidpad is 18.25 m radius (9.125 m inner, 18.25 m centerline).
%   This analysis validates vehicle dynamics model tire friction.
%
% Author: Team Unwired
% Date: January 2026

fprintf('=== Skidpad Test Analysis ===\n\n');

t = imu_data.time;
ax = imu_data.ax;  % Longitudinal (should be near zero in steady state)
ay = imu_data.ay;  % Lateral acceleration
gz = imu_data.gz;  % Yaw rate

% Competition skidpad radius
skidpad_radius = 18.25;  % meters (centerline)

% Detect steady-state cornering segments
% Look for periods where lateral acceleration is high and relatively constant
ay_smooth = movmean(ay, 20);  % Smooth with 0.2s window at 100Hz
threshold = 0.3 * 9.81;  % At least 0.3g lateral

% Find regions of high lateral acceleration
high_lateral = abs(ay_smooth) > threshold;

% Find contiguous regions
labeled = bwlabel(high_lateral);
num_regions = max(labeled);

fprintf('Found %d cornering segments.\n\n', num_regions);

% Analyze each segment
all_ay_peaks = [];
all_yaw_rates = [];
all_speeds = [];

for region = 1:num_regions
    region_mask = (labeled == region);
    region_duration = sum(region_mask) / 100;  % seconds (assuming 100 Hz)
    
    if region_duration < 1.0
        continue;  % Skip very short segments
    end
    
    ay_region = ay(region_mask);
    gz_region = gz(region_mask);
    
    % Peak lateral acceleration in this region
    peak_ay = max(abs(ay_region));
    mean_ay = mean(abs(ay_region));
    mean_yaw = mean(abs(gz_region));
    
    % Estimate speed from lateral acceleration and yaw rate
    % a_y = v^2/R = v * omega  =>  v = a_y / omega
    if mean_yaw > 0.1  % Avoid division by zero
        speed = mean_ay / mean_yaw;
        all_speeds = [all_speeds; speed];
    end
    
    all_ay_peaks = [all_ay_peaks; peak_ay];
    all_yaw_rates = [all_yaw_rates; mean_yaw];
    
    fprintf('Segment %d: Duration = %.1f s, Peak Lateral = %.2f g, Mean Yaw Rate = %.1f deg/s\n', ...
        region, region_duration, peak_ay/9.81, rad2deg(mean_yaw));
end

% Overall results
if isempty(all_ay_peaks)
    warning('No valid cornering segments found. Check data quality.');
    max_lateral_g = NaN;
    friction_coeff = NaN;
    mean_yaw_rate = NaN;
    corner_speed = NaN;
    lap_time = NaN;
else
    max_lateral_g = max(all_ay_peaks) / 9.81;
    mean_yaw_rate = mean(all_yaw_rates);
    corner_speed = mean(all_speeds);
    
    % Friction coefficient: mu = a_y / g
    friction_coeff = max_lateral_g;
    
    % Calculate lap time for skidpad
    % v = sqrt(mu * g * R)
    predicted_speed = sqrt(friction_coeff * 9.81 * skidpad_radius);
    circumference = 2 * pi * skidpad_radius;
    lap_time = circumference / predicted_speed;
    
    fprintf('\n--- Summary ---\n');
    fprintf('Maximum lateral acceleration: %.2f g\n', max_lateral_g);
    fprintf('Tire friction coefficient (mu): %.2f\n', friction_coeff);
    fprintf('Mean yaw rate: %.1f deg/s (%.2f rad/s)\n', rad2deg(mean_yaw_rate), mean_yaw_rate);
    fprintf('Estimated cornering speed: %.1f m/s (%.1f km/h)\n', corner_speed, corner_speed*3.6);
    fprintf('Predicted skidpad lap time (R=%.2fm): %.2f s\n', skidpad_radius, lap_time);
end

% Create result structure
results.max_lateral_g = max_lateral_g;
results.friction_coeff = friction_coeff;
results.yaw_rate = mean_yaw_rate;
results.corner_speed = corner_speed;
results.lap_time = lap_time;
results.skidpad_radius = skidpad_radius;
results.time = t;
results.ay = ay;
results.gz = gz;

% Generate plots
figure('Name', 'Skidpad Test Analysis', 'Position', [100 100 1200 800]);

% Plot 1: Lateral acceleration vs time
subplot(2, 2, 1);
plot(t, ay/9.81, 'b-', 'LineWidth', 1);
hold on;
plot(t, ay_smooth/9.81, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Lateral Acceleration (g)');
title('Lateral Acceleration Profile');
legend('Raw', 'Smoothed', 'Location', 'best');
grid on;

% Plot 2: Yaw rate vs time
subplot(2, 2, 2);
plot(t, rad2deg(gz), 'b-', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Yaw Rate (deg/s)');
title('Yaw Rate Profile');
grid on;

% Plot 3: GG Diagram (lateral vs longitudinal acceleration)
subplot(2, 2, 3);
scatter(ay/9.81, ax/9.81, 3, t, 'filled');
colorbar('Title', 'Time (s)');
xlabel('Lateral Acceleration (g)');
ylabel('Longitudinal Acceleration (g)');
title('GG Diagram');
axis equal;
grid on;

% Draw friction circle
if ~isnan(friction_coeff)
    hold on;
    theta = linspace(0, 2*pi, 100);
    plot(friction_coeff*cos(theta), friction_coeff*sin(theta), 'r--', 'LineWidth', 2);
    legend('Data', sprintf('Friction Circle (mu=%.2f)', friction_coeff), 'Location', 'best');
end

% Plot 4: Lateral acceleration histogram
subplot(2, 2, 4);
histogram(abs(ay)/9.81, 30, 'Normalization', 'probability');
xlabel('Lateral Acceleration Magnitude (g)');
ylabel('Probability');
title('Lateral Acceleration Distribution');
if ~isnan(max_lateral_g)
    xline(max_lateral_g, 'r--', sprintf('Max = %.2fg', max_lateral_g), 'LineWidth', 2);
end
grid on;

sgtitle('Team Unwired C37 Skidpad Test Analysis');

fprintf('\n=== Analysis Complete ===\n');

end
