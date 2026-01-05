function gps_data = import_phyphox_gps(filename)
% IMPORT_PHYPHOX_GPS Import GPS data exported from phyphox app
%
% Syntax:
%   gps_data = import_phyphox_gps('GPS.csv')
%
% Input:
%   filename - Path to CSV file exported from phyphox GPS experiment
%
% Output:
%   gps_data - Structure containing:
%       .time      - Time vector (s)
%       .lat       - Latitude (degrees)
%       .lon       - Longitude (degrees)
%       .alt       - Altitude (m)
%       .speed     - Speed from GPS (m/s)
%       .bearing   - Direction/heading (degrees from north)
%       .accuracy  - Horizontal accuracy (m)
%       .x         - East position relative to start (m)
%       .y         - North position relative to start (m)
%
% Note:
%   Position (x, y) is calculated from lat/lon using local tangent plane
%   approximation, valid for distances under 10 km.
%
% Author: Team Unwired
% Date: January 2026

fprintf('Importing phyphox GPS data from: %s\n', filename);

% Read CSV file
opts = detectImportOptions(filename);
data = readtable(filename, opts);

% phyphox GPS export columns (may vary by version)
% Try to find columns by name patterns
col_names = lower(data.Properties.VariableNames);

% Find time column
time_idx = find(contains(col_names, 't') & contains(col_names, 's'), 1);
if isempty(time_idx)
    time_idx = 1;  % Assume first column is time
end

% Find GPS columns
lat_idx = find(contains(col_names, 'lat'), 1);
lon_idx = find(contains(col_names, 'lon'), 1);
alt_idx = find(contains(col_names, 'alt'), 1);
speed_idx = find(contains(col_names, 'speed') | contains(col_names, 'v'), 1);
bearing_idx = find(contains(col_names, 'dir') | contains(col_names, 'bear'), 1);
acc_idx = find(contains(col_names, 'acc'), 1);

% Extract data
gps_data.time = data{:, time_idx};

if ~isempty(lat_idx) && ~isempty(lon_idx)
    gps_data.lat = data{:, lat_idx};
    gps_data.lon = data{:, lon_idx};
else
    error('Could not find latitude/longitude columns in GPS data.');
end

if ~isempty(alt_idx)
    gps_data.alt = data{:, alt_idx};
else
    gps_data.alt = zeros(size(gps_data.time));
end

if ~isempty(speed_idx)
    gps_data.speed = data{:, speed_idx};
else
    gps_data.speed = zeros(size(gps_data.time));
end

if ~isempty(bearing_idx)
    gps_data.bearing = data{:, bearing_idx};
else
    gps_data.bearing = zeros(size(gps_data.time));
end

if ~isempty(acc_idx)
    gps_data.accuracy = data{:, acc_idx};
else
    gps_data.accuracy = zeros(size(gps_data.time));
end

% Remove NaN entries
valid = ~isnan(gps_data.lat) & ~isnan(gps_data.lon);
fields = fieldnames(gps_data);
for i = 1:length(fields)
    gps_data.(fields{i}) = gps_data.(fields{i})(valid);
end

% Convert lat/lon to local XY coordinates (meters)
% Using equirectangular approximation (valid for small distances)
lat_ref = gps_data.lat(1);
lon_ref = gps_data.lon(1);

% Earth radius
R = 6371000;  % meters

% Calculate position relative to start
gps_data.x = R * deg2rad(gps_data.lon - lon_ref) * cosd(lat_ref);  % East
gps_data.y = R * deg2rad(gps_data.lat - lat_ref);                   % North

% Metadata
gps_data.metadata.samples = length(gps_data.time);
gps_data.metadata.duration = gps_data.time(end) - gps_data.time(1);
gps_data.metadata.sample_rate = gps_data.metadata.samples / gps_data.metadata.duration;
gps_data.metadata.start_lat = lat_ref;
gps_data.metadata.start_lon = lon_ref;
gps_data.metadata.total_distance = sum(sqrt(diff(gps_data.x).^2 + diff(gps_data.y).^2));

fprintf('GPS data imported successfully.\n');
fprintf('  Samples: %d\n', gps_data.metadata.samples);
fprintf('  Duration: %.1f s\n', gps_data.metadata.duration);
fprintf('  Sample rate: %.1f Hz\n', gps_data.metadata.sample_rate);
fprintf('  Total distance: %.1f m\n', gps_data.metadata.total_distance);
fprintf('  Max speed: %.1f m/s (%.1f km/h)\n', max(gps_data.speed), max(gps_data.speed)*3.6);

end
