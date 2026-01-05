function logsout_sm_car = imu_to_simscape_format(imu_data)
% IMU_TO_SIMSCAPE_FORMAT Convert IMU data to Simscape Vehicle Template format
%
% Syntax:
%   logsout_sm_car = imu_to_simscape_format(imu_data)
%
% Description:
%   Converts IMU data structure from import_imu_data() into the logsout
%   format used by Formula-Student-Vehicle-Simscape plotting functions.
%   This allows using sm_car_plot* functions with real test data.
%
% Input:
%   imu_data - Structure from import_imu_data()
%
% Output:
%   logsout_sm_car - Simulink.SimulationData.Dataset compatible with
%                    sm_car plotting functions
%
% Example:
%   imu = import_imu_data('run001.csv');
%   logsout_sm_car = imu_to_simscape_format(imu);
%   sm_car_plot5bodymeas;  % Now works with test data
%
% Note:
%   Not all fields are populated since IMU does not measure everything.
%   Fields that require wheel speeds, tire forces, etc. will be NaN.
%
% Author: Team Unwired
% Date: January 2026

fprintf('Converting IMU data to Simscape format...\n');

% Get time vector
t = imu_data.time;
n = length(t);

% Create timeseries objects for each signal
% World frame signals
ts_vx = timeseries(zeros(n, 1), t, 'Name', 'vx');
ts_vy = timeseries(zeros(n, 1), t, 'Name', 'vy');
ts_vz = timeseries(zeros(n, 1), t, 'Name', 'vz');
ts_x = timeseries(zeros(n, 1), t, 'Name', 'x');
ts_y = timeseries(zeros(n, 1), t, 'Name', 'y');
ts_z = timeseries(zeros(n, 1), t, 'Name', 'z');

% Calculate velocity by integrating acceleration
% Note: This drifts over time, use only for short tests
vx = cumtrapz(t, imu_data.ax);
vy = cumtrapz(t, imu_data.ay);
ts_vx = timeseries(vx, t, 'Name', 'vx');
ts_vy = timeseries(vy, t, 'Name', 'vy');

% Calculate position by integrating velocity
x = cumtrapz(t, vx);
y = cumtrapz(t, vy);
ts_x = timeseries(x, t, 'Name', 'x');
ts_y = timeseries(y, t, 'Name', 'y');

% Angles (integrate angular rates)
roll = cumtrapz(t, imu_data.gx);
pitch = cumtrapz(t, imu_data.gy);
yaw = cumtrapz(t, imu_data.gz);

ts_aRoll = timeseries(roll, t, 'Name', 'aRoll');
ts_aPitch = timeseries(pitch, t, 'Name', 'aPitch');
ts_aYaw = timeseries(yaw, t, 'Name', 'aYaw');

% Angular rates
ts_nRoll = timeseries(imu_data.gx, t, 'Name', 'nRoll');
ts_nPitch = timeseries(imu_data.gy, t, 'Name', 'nPitch');
ts_nYaw = timeseries(imu_data.gz, t, 'Name', 'nYaw');

% Body CG accelerations
ts_gx = timeseries(imu_data.ax, t, 'Name', 'gx');
ts_gy = timeseries(imu_data.ay, t, 'Name', 'gy');
ts_gz = timeseries(imu_data.az, t, 'Name', 'gz');

% Build VehBus structure
VehBus.Values.World.x = ts_x;
VehBus.Values.World.y = ts_y;
VehBus.Values.World.z = ts_z;
VehBus.Values.World.vx = ts_vx;
VehBus.Values.World.vy = ts_vy;
VehBus.Values.World.vz = ts_vz;
VehBus.Values.World.aRoll = ts_aRoll;
VehBus.Values.World.aPitch = ts_aPitch;
VehBus.Values.World.aYaw = ts_aYaw;
VehBus.Values.World.nRoll = ts_nRoll;
VehBus.Values.World.nPitch = ts_nPitch;
VehBus.Values.World.nYaw = ts_nYaw;

% Body CG measurements
VehBus.Values.Chassis.Body.CG.gx = ts_gx;
VehBus.Values.Chassis.Body.CG.gy = ts_gy;
VehBus.Values.Chassis.Body.CG.vx = ts_vx;
VehBus.Values.Chassis.Body.CG.vy = ts_vy;

% Placeholder for steering (not measured by IMU)
ts_xRack = timeseries(zeros(n, 1), t, 'Name', 'xRack');
ts_aWheel = timeseries(zeros(n, 1), t, 'Name', 'aWheel');
VehBus.Values.Chassis.SuspA1.Steer.xRack = ts_xRack;
VehBus.Values.Chassis.SuspA1.Steer.aWheel = ts_aWheel;
VehBus.Values.Chassis.SuspA2.Steer.xRack = ts_xRack;

% Placeholder for wheel angles (not measured by IMU)
ts_aToe = timeseries(zeros(n, 1), t, 'Name', 'aToe');
ts_aCamber = timeseries(zeros(n, 1), t, 'Name', 'aCamber');
VehBus.Values.Chassis.SuspA1.WhlL.aToe = ts_aToe;
VehBus.Values.Chassis.SuspA1.WhlR.aToe = ts_aToe;
VehBus.Values.Chassis.SuspA1.WhlL.aCamber = ts_aCamber;
VehBus.Values.Chassis.SuspA1.WhlR.aCamber = ts_aCamber;
VehBus.Values.Chassis.SuspA2.WhlL.aToe = ts_aToe;
VehBus.Values.Chassis.SuspA2.WhlR.aToe = ts_aToe;
VehBus.Values.Chassis.SuspA2.WhlL.aCamber = ts_aCamber;
VehBus.Values.Chassis.SuspA2.WhlR.aCamber = ts_aCamber;

% Placeholder for wheel data (not measured by IMU)
ts_n = timeseries(zeros(n, 1), t, 'Name', 'n');
ts_xyz = timeseries(zeros(n, 3), t, 'Name', 'xyz');
ts_Fx = timeseries(zeros(n, 1), t, 'Name', 'Fx');
ts_Fy = timeseries(zeros(n, 1), t, 'Name', 'Fy');
ts_Fz = timeseries(zeros(n, 1), t, 'Name', 'Fz');
ts_Mx = timeseries(zeros(n, 1), t, 'Name', 'Mx');
ts_My = timeseries(zeros(n, 1), t, 'Name', 'My');
ts_Mz = timeseries(zeros(n, 1), t, 'Name', 'Mz');
ts_aSlip = timeseries(zeros(n, 1), t, 'Name', 'aSlip');

wheel_names = {'WhlL1', 'WhlR1', 'WhlL2', 'WhlR2'};
for i = 1:length(wheel_names)
    whl = wheel_names{i};
    VehBus.Values.Chassis.(whl).n = ts_n;
    VehBus.Values.Chassis.(whl).xyz = ts_xyz;
    VehBus.Values.Chassis.(whl).Fx = ts_Fx;
    VehBus.Values.Chassis.(whl).Fy = ts_Fy;
    VehBus.Values.Chassis.(whl).Fz = ts_Fz;
    VehBus.Values.Chassis.(whl).Mx = ts_Mx;
    VehBus.Values.Chassis.(whl).My = ts_My;
    VehBus.Values.Chassis.(whl).Mz = ts_Mz;
    VehBus.Values.Chassis.(whl).aSlip = ts_aSlip;
end

% Create Simulink.SimulationData.Dataset
logsout_sm_car = Simulink.SimulationData.Dataset;
logsout_sm_car = logsout_sm_car.addElement(VehBus, 'VehBus');

% Create placeholder RdBus (road data, not available from IMU)
RdBus.Values.L1.gz = timeseries(zeros(n, 1), t, 'Name', 'gz');
RdBus.Values.R1.gz = timeseries(zeros(n, 1), t, 'Name', 'gz');
RdBus.Values.L2.gz = timeseries(zeros(n, 1), t, 'Name', 'gz');
RdBus.Values.R2.gz = timeseries(zeros(n, 1), t, 'Name', 'gz');
logsout_sm_car = logsout_sm_car.addElement(RdBus, 'RdBus');

fprintf('Conversion complete.\n');
fprintf('Available for plotting:\n');
fprintf('  - Position (integrated from acceleration)\n');
fprintf('  - Velocity (integrated from acceleration)\n');
fprintf('  - Roll/Pitch/Yaw angles (integrated from gyro)\n');
fprintf('  - Roll/Pitch/Yaw rates (direct measurement)\n');
fprintf('  - Longitudinal/Lateral/Vertical acceleration (direct measurement)\n');
fprintf('\n');
fprintf('Not available (set to zero):\n');
fprintf('  - Wheel speeds\n');
fprintf('  - Tire forces\n');
fprintf('  - Steering angle\n');
fprintf('  - Suspension travel\n');

end
