function data = receive_imu_tcp(esp32_ip, port, duration_seconds)
% RECEIVE_IMU_TCP Receive IMU data directly in MATLAB via TCP
%
% Syntax:
%   data = receive_imu_tcp(esp32_ip, port, duration_seconds)
%
% Example:
%   data = receive_imu_tcp('192.168.1.100', 5000, 30);
%
% Author: Team Unwired
% Date: January 2026

if nargin < 3
    duration_seconds = 60;
end

fprintf('Connecting to ESP32 at %s:%d...\n', esp32_ip, port);

% Create TCP client
t = tcpclient(esp32_ip, port, 'Timeout', 30);

fprintf('Connected! Recording for %d seconds...\n', duration_seconds);
fprintf('Press Ctrl+C to stop early.\n\n');

% Preallocate arrays
max_samples = duration_seconds * 120;  % Slightly more than 100 Hz
time_ms = zeros(max_samples, 1);
ax = zeros(max_samples, 1);
ay = zeros(max_samples, 1);
az = zeros(max_samples, 1);
gx = zeros(max_samples, 1);
gy = zeros(max_samples, 1);
gz = zeros(max_samples, 1);

sample_count = 0;
start_time = tic;
buffer = '';

try
    while toc(start_time) < duration_seconds
        % Read available data
        if t.NumBytesAvailable > 0
            new_data = read(t, t.NumBytesAvailable, 'char');
            buffer = [buffer, char(new_data)];
            
            % Process complete lines
            lines = strsplit(buffer, newline);
            buffer = lines{end};  % Keep incomplete line
            
            for i = 1:length(lines)-1
                line = strtrim(lines{i});
                
                % Skip comments and empty lines
                if isempty(line) || line(1) == '#'
                    continue;
                end
                
                % Parse CSV
                values = str2double(strsplit(line, ','));
                if length(values) == 7 && ~any(isnan(values))
                    sample_count = sample_count + 1;
                    time_ms(sample_count) = values(1);
                    ax(sample_count) = values(2);
                    ay(sample_count) = values(3);
                    az(sample_count) = values(4);
                    gx(sample_count) = values(5);
                    gy(sample_count) = values(6);
                    gz(sample_count) = values(7);
                    
                    % Status update
                    if mod(sample_count, 100) == 0
                        fprintf('Samples: %d, ax=%.2fg, ay=%.2fg\n', ...
                            sample_count, ax(sample_count), ay(sample_count));
                    end
                end
            end
        else
            pause(0.01);  % Small delay to avoid busy waiting
        end
    end
catch ME
    fprintf('Recording interrupted: %s\n', ME.message);
end

% Cleanup
clear t;

% Trim arrays
time_ms = time_ms(1:sample_count);
ax = ax(1:sample_count);
ay = ay(1:sample_count);
az = az(1:sample_count);
gx = gx(1:sample_count);
gy = gy(1:sample_count);
gz = gz(1:sample_count);

% Convert to standard format
data.time = time_ms / 1000;  % Convert to seconds
data.ax = ax * 9.81;         % Convert g to m/s^2
data.ay = ay * 9.81;
data.az = az * 9.81;
data.gx = deg2rad(gx);       % Convert deg/s to rad/s
data.gy = deg2rad(gy);
data.gz = deg2rad(gz);

% Metadata
data.metadata.samples = sample_count;
data.metadata.duration = data.time(end) - data.time(1);
data.metadata.sample_rate = sample_count / data.metadata.duration;
data.metadata.source = sprintf('%s:%d', esp32_ip, port);

fprintf('\nRecording complete!\n');
fprintf('Samples: %d\n', sample_count);
fprintf('Duration: %.1f s\n', data.metadata.duration);
fprintf('Sample rate: %.1f Hz\n', data.metadata.sample_rate);

end
