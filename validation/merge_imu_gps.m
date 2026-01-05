function merged = merge_imu_gps(imu_data, gps_data, time_offset)
% MERGE_IMU_GPS Combine IMU and GPS data with time synchronization
%
% Syntax:
%   merged = merge_imu_gps(imu_data, gps_data)
%   merged = merge_imu_gps(imu_data, gps_data, time_offset)
%
% Inputs:
%   imu_data    - Structure from import_imu_data()
%   gps_data    - Structure from import_phyphox_gps()
%   time_offset - Optional: GPS_time = IMU_time + offset (default: 0)
%
% Output:
%   merged - Structure with both IMU and GPS on common time base
%
% Time Synchronization:
%   If time_offset not provided, function attempts automatic sync by
%   correlating GPS speed with integrated IMU acceleration.
%
% Author: Team Unwired
% Date: January 2026

if nargin < 3
    time_offset = 0;
    auto_sync = true;
else
    auto_sync = false;
end

fprintf('Merging IMU and GPS data...\n');

% Apply time offset to GPS
gps_time_aligned = gps_data.time - time_offset;

% Find overlapping time range
t_start = max(min(imu_data.time), min(gps_time_aligned));
t_end = min(max(imu_data.time), max(gps_time_aligned));

if t_end <= t_start
    error('No time overlap between IMU and GPS data. Check time_offset parameter.');
end

fprintf('  Time overlap: %.2f to %.2f seconds\n', t_start, t_end);

% Use IMU time as master (higher rate)
merged.time = imu_data.time(imu_data.time >= t_start & imu_data.time <= t_end);

% Copy IMU data
imu_mask = imu_data.time >= t_start & imu_data.time <= t_end;
merged.ax = imu_data.ax(imu_mask);
merged.ay = imu_data.ay(imu_mask);
merged.az = imu_data.az(imu_mask);
merged.gx = imu_data.gx(imu_mask);
merged.gy = imu_data.gy(imu_mask);
merged.gz = imu_data.gz(imu_mask);

% Interpolate GPS data to IMU time base
merged.lat = interp1(gps_time_aligned, gps_data.lat, merged.time, 'linear', 'extrap');
merged.lon = interp1(gps_time_aligned, gps_data.lon, merged.time, 'linear', 'extrap');
merged.x = interp1(gps_time_aligned, gps_data.x, merged.time, 'linear', 'extrap');
merged.y = interp1(gps_time_aligned, gps_data.y, merged.time, 'linear', 'extrap');
merged.speed_gps = interp1(gps_time_aligned, gps_data.speed, merged.time, 'linear', 'extrap');
merged.bearing = interp1(gps_time_aligned, gps_data.bearing, merged.time, 'linear', 'extrap');

% Calculate velocity from IMU (integration of acceleration)
% Use GPS speed for bias correction
merged.speed_imu = cumtrapz(merged.time, merged.ax);

% Attempt automatic time synchronization if not provided
if auto_sync && time_offset == 0
    fprintf('  Attempting automatic time synchronization...\n');
    
    % Correlate GPS speed derivative with longitudinal acceleration
    % dv/dt from GPS should match ax from IMU
    dt_gps = mean(diff(gps_data.time));
    gps_accel = gradient(gps_data.speed) / dt_gps;
    
    % Resample IMU to GPS rate for correlation
    imu_ax_resampled = interp1(imu_data.time, imu_data.ax, gps_data.time, 'linear', 'extrap');
    
    % Find offset that maximizes correlation
    max_offset = 5;  % seconds
    offsets = -max_offset:0.1:max_offset;
    correlations = zeros(size(offsets));
    
    for i = 1:length(offsets)
        shifted_time = gps_data.time - offsets(i);
        imu_shifted = interp1(imu_data.time, imu_data.ax, shifted_time, 'linear', 0);
        valid = ~isnan(imu_shifted) & ~isnan(gps_accel);
        if sum(valid) > 10
            r = corrcoef(imu_shifted(valid), gps_accel(valid));
            correlations(i) = r(1,2);
        end
    end
    
    [max_corr, max_idx] = max(correlations);
    best_offset = offsets(max_idx);
    
    if max_corr > 0.5
        fprintf('  Auto-sync found offset: %.2f s (correlation: %.2f)\n', best_offset, max_corr);
        % Recursively call with found offset
        merged = merge_imu_gps(imu_data, gps_data, best_offset);
        return;
    else
        fprintf('  Auto-sync failed (max correlation: %.2f). Using zero offset.\n', max_corr);
    end
end

% Metadata
merged.metadata.samples = length(merged.time);
merged.metadata.duration = merged.time(end) - merged.time(1);
merged.metadata.imu_rate = 1 / mean(diff(imu_data.time));
merged.metadata.gps_rate = 1 / mean(diff(gps_data.time));
merged.metadata.time_offset = time_offset;

fprintf('Merge complete.\n');
fprintf('  Samples: %d\n', merged.metadata.samples);
fprintf('  Duration: %.1f s\n', merged.metadata.duration);

end
