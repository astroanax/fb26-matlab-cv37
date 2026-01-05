# Vehicle Dynamics Modeling Guide

## Team Unwired - C37 (ICE)
## Formula Bharat 2026 MathWorks Modeling Award

---

## Table of Contents

1. [Overview](#1-overview)
2. [Required Vehicle Data](#2-required-vehicle-data)
3. [Modeling Approach Options](#3-modeling-approach-options)
4. [Using Formula-Student-Vehicle-Simscape](#4-using-formula-student-vehicle-simscape)
5. [Using Vehicle Dynamics Blockset](#5-using-vehicle-dynamics-blockset)
6. [Model Development Workflow](#6-model-development-workflow)
7. [Validation Setup](#7-validation-setup)
8. [Sensor Requirements](#8-sensor-requirements)
9. [Validation Procedure](#9-validation-procedure)
10. [Deliverables](#10-deliverables)
11. [References](#11-references)

---

## 1. Overview
                                                                                                                                                                                                                                                                                                                                                                                                                                                     
Vehicle dynamics modeling encompasses the study of vehicle motion in response to driver inputs, road conditions, and external forces. For Formula Bharat, the submission must demonstrate:

- Model Development (50 points): MATLAB/Simulink models of suspension, steering, tires, and vehicle dynamics
- Validation (30 points): Correlation between model predictions and real-world measurements 
- Application and Insights (20 points): Design decisions supported by model analysis

This guide provides a complete workflow from initial data collection through validated model delivery.

---

## 2. Required Vehicle Data

### 2.1 Mass Properties

| Parameter | Symbol | Unit | How to Obtain |
|-----------|--------|------|---------------|
| Total vehicle mass (with driver) | m | kg | Scale measurement |
| Front axle mass | m_f | kg | Corner weight measurement |
| Rear axle mass | m_r | kg | Corner weight measurement |
| Center of gravity height | h_cg | m | Tilt test or CAD mass properties |
| Roll moment of inertia | I_xx | kg-m^2 | CAD mass properties |
| Pitch moment of inertia | I_yy | kg-m^2 | CAD mass properties |
| Yaw moment of inertia | I_zz | kg-m^2 | CAD mass properties |

### 2.2 Geometry

| Parameter | Symbol | Unit | How to Obtain |
|-----------|--------|------|---------------|
| Wheelbase | L | m | Direct measurement |
| Front track width | t_f | m | Direct measurement |
| Rear track width | t_r | m | Direct measurement |
| Front roll center height | h_rcf | m | Kinematic analysis (LOTUS Shark or CAD) |
| Rear roll center height | h_rcr | m | Kinematic analysis (LOTUS Shark or CAD) |

### 2.3 Suspension Parameters

| Parameter | Symbol | Unit | How to Obtain |
|-----------|--------|------|---------------|
| Front spring rate (wheel rate) | k_sf | N/m | Spring specification or test |
| Rear spring rate (wheel rate) | k_sr | N/m | Spring specification or test |
| Front damping coefficient | c_f | N-s/m | Damper specification or dyno |
| Rear damping coefficient | c_r | N-s/m | Damper specification or dyno |
| Front motion ratio | MR_f | - | Kinematic analysis |
| Rear motion ratio | MR_r | - | Kinematic analysis |
| Front anti-roll bar stiffness | k_arbf | N-m/rad | Specification or calculation |
| Rear anti-roll bar stiffness | k_arbr | N-m/rad | Specification or calculation |

### 2.4 Tire Parameters

| Parameter | Symbol | Unit | How to Obtain |
|-----------|--------|------|---------------|
| Unloaded radius | R_0 | m | Measurement |
| Loaded radius | R_l | m | Measurement under load |
| Tire width | w | m | Specification |
| Cornering stiffness | C_alpha | N/rad | Tire test data or estimation |
| Peak friction coefficient | mu_peak | - | Skidpad test or tire data |
| Sliding friction coefficient | mu_slide | - | Tire test data or estimation |

### 2.5 Steering System

| Parameter | Symbol | Unit | How to Obtain |
|-----------|--------|------|---------------|
| Steering ratio | i_s | - | Direct measurement (steering wheel angle / road wheel angle) |
| Ackermann percentage | - | % | Kinematic analysis |
| Kingpin inclination | KPI | deg | CAD or measurement |
| Caster angle | caster | deg | CAD or measurement |
| Mechanical trail | t_m | m | CAD or measurement |

---

## 3. Modeling Approach Options

There are three viable approaches for vehicle dynamics modeling. The choice depends on available time, complexity requirements, and existing resources.

### 3.1 Option A: Formula-Student-Vehicle-Simscape (Recommended)

**Advantages:**
- Purpose-built for FSAE vehicles
- Complete Simscape Multibody model with double-wishbone suspension
- Includes tire models, driver models, and track definitions
- Skidpad and Hockenheim events pre-configured
- MathWorks official project (high credibility with judges)

**Disadvantages:**
- Designed for electric vehicles (requires powertrain modification)
- Complex model structure (steep learning curve)
- Requires Simscape Multibody license

**Recommendation:** Use this approach. Modify the existing model with C37 parameters.

### 3.2 Option B: Vehicle Dynamics Blockset

**Advantages:**
- Official MathWorks toolbox for vehicle dynamics
- Pre-built vehicle models (sedan, truck, motorcycle)
- Maneuver and scenario testing built-in
- Reference applications available

**Disadvantages:**
- Requires separate license (Vehicle Dynamics Blockset)
- Generic vehicle templates may not match FSAE geometry
- Less community support for FSAE-specific applications

**Recommendation:** Use if you already have the license and prefer a more abstracted approach.

### 3.3 Option C: Custom MATLAB Model

**Advantages:**
- Complete control over model structure
- No additional license requirements
- Simpler to understand and modify

**Disadvantages:**
- More development time required
- Less visually impressive to judges
- Must validate basic physics from scratch

**Recommendation:** Use only if other options are unavailable.

---

## 4. Using Formula-Student-Vehicle-Simscape

### 4.1 Project Structure

The Formula-Student-Vehicle-Simscape project contains:

```
Formula-Student-Vehicle-Simscape/
├── sm_car.slx                 # Main Simulink model
├── SSVT_FSAE.prj              # MATLAB project file
├── Libraries/
│   ├── Event/                 # Track and maneuver definitions
│   │   ├── Init_data_skidpad.m
│   │   ├── Init_data_hockenheim.m
│   │   ├── Maneuver_data_skidpad.m
│   │   └── ...
│   └── Vehicle/               # Vehicle component libraries
│       ├── Dampers/
│       ├── Springs/
│       ├── Tire/
│       └── ...
├── Scripts_Data/
│   └── Data_Vehicle/          # Vehicle parameter files
└── Testrigs/
    └── Quarter_Car/           # Isolated suspension testrig
```

### 4.2 Getting Started

Step 1: Open the project in MATLAB:

```matlab
cd('/home/astroanax/dev/tuw/fb2026/matlab/Formula-Student-Vehicle-Simscape')
open('SSVT_FSAE.prj')
```

Step 2: Run the startup script:

```matlab
startup_sm_car
```

Step 3: Open the main model:

```matlab
open_system('sm_car')
```

### 4.3 Configuring for C37

The project includes a double-wishbone pushrod suspension configuration that matches the C37 architecture. To configure:

Step 1: Locate the vehicle data file:

```
Scripts_Data/Data_Vehicle/Vehicle_data_dwpushrod.m
```

Step 2: Create a copy for C37:

```matlab
% Copy and rename
copyfile('Vehicle_data_dwpushrod.m', 'Vehicle_data_C37.m')
```

Step 3: Modify parameters in Vehicle_data_C37.m:

```matlab
% Mass properties
Vehicle.Mass.Total = 300;           % kg (with driver)
Vehicle.Mass.Front = 105;           % kg (35% front)
Vehicle.Mass.Rear = 195;            % kg (65% rear)

% Geometry
Vehicle.Chassis.Wheelbase = 1.560;  % m
Vehicle.Chassis.TrackFront = 1.200; % m
Vehicle.Chassis.TrackRear = 1.180;  % m

% Suspension
Vehicle.Suspension.Front.SpringRate = 35000;  % N/m
Vehicle.Suspension.Rear.SpringRate = 45000;   % N/m
```

### 4.4 Running Simulations

Skidpad simulation:

```matlab
% Configure for skidpad event
sm_car_config_event('Skidpad')

% Run simulation
sim('sm_car')

% Plot results
sm_car_plot_results
```

Acceleration simulation:

```matlab
% Configure for straight-line acceleration
sm_car_config_event('WOT_Braking')

% Run simulation
sim('sm_car')
```

---

## 5. Using Vehicle Dynamics Blockset

If using Vehicle Dynamics Blockset instead of Simscape, follow this approach.

### 5.1 Check License Availability

```matlab
license('test', 'Vehicle_Dynamics_Blockset')
```

### 5.2 Create Vehicle Model

```matlab
% Open reference application
openExample('vdynblks/VehicleBodyAnd3DOFSingleTrackExample')
```

### 5.3 Configure Vehicle Parameters

The Vehicle Dynamics Blockset uses a vehicle object:

```matlab
% Create vehicle object
v = vehicleDimensions;
v.Wheelbase = 1.560;
v.FrontOverhang = 0.5;
v.RearOverhang = 0.5;
v.Width = 1.4;

% Create chassis object
c = chassis;
c.Mass = 300;
c.CGHeight = 0.280;
```

### 5.4 Documentation

- Vehicle Dynamics Blockset User Guide: https://www.mathworks.com/help/vdynblks/
- Reference Applications: https://www.mathworks.com/help/vdynblks/examples.html

---

## 6. Model Development Workflow

### 6.1 Phase 1: Data Collection (Days 1-2)

Required measurements from physical vehicle:

1. Corner weights (all four wheels)
2. Wheelbase and track width
3. Steering ratio (steering wheel rotation vs road wheel angle)
4. Static ride height
5. Suspension travel (bump and droop)

Required data from CAD:

1. Center of gravity location (x, y, z from CAD mass properties)
2. Moments of inertia (I_xx, I_yy, I_zz)
3. Suspension hardpoints (for kinematic analysis)
4. Roll center heights (from LOTUS Shark or kinematic solver)
5. Motion ratios (from suspension geometry)

Required data from specifications:

1. Spring rates (from spring manufacturer)
2. Damper curves (from damper manufacturer)
3. Tire dimensions (from tire specification)

### 6.2 Phase 2: Model Construction (Days 3-5)

Step 1: Configure vehicle parameter file

Create or modify Vehicle_data_C37.m with all collected parameters.

Step 2: Configure tire model

Use the Pacejka Magic Formula tire model. If tire test data is unavailable, use estimated parameters based on tire type (bias-ply, 13-inch, sporty compound):

```matlab
% Tire parameters (estimated for G-max 170/50 R13)
Tire.Radius = 0.250;                    % m (loaded)
Tire.CorneringStiffness = 800;          % N/deg
Tire.PeakFriction = 1.3;                % mu
Tire.SlidingFriction = 1.0;             % mu
```

Step 3: Configure suspension

Define spring rates, damping coefficients, and geometry:

```matlab
% Front suspension
Susp.Front.SpringRate = 35000;          % N/m (wheel rate)
Susp.Front.DampingComp = 1500;          % N-s/m
Susp.Front.DampingReb = 2500;           % N-s/m
Susp.Front.MotionRatio = 0.85;

% Rear suspension
Susp.Rear.SpringRate = 45000;           % N/m (wheel rate)
Susp.Rear.DampingComp = 1800;           % N-s/m
Susp.Rear.DampingReb = 3000;            % N-s/m
Susp.Rear.MotionRatio = 0.70;
```

Step 4: Configure steering

```matlab
Steer.Ratio = 5.0;                      % steering wheel : road wheel
Steer.MaxAngle = 25;                    % deg (road wheel)
```

### 6.3 Phase 3: Baseline Simulation (Days 6-7)

Run baseline simulations to verify model behavior:

1. Steady-state cornering (skidpad): Verify lateral acceleration capability
2. Step steer: Verify transient response and understeer gradient
3. Straight-line acceleration: Verify longitudinal dynamics

Expected outputs:

| Test | Expected Result |
|------|-----------------|
| Skidpad (20m radius) | Max lateral acceleration 0.9-1.1 g |
| Step steer | Understeer gradient 1-3 deg/g |
| 0-60 km/h | Time 4-6 seconds |

### 6.4 Phase 4: Sensitivity Analysis (Day 8)

Perform parameter sweeps to understand design sensitivities:

```matlab
% Example: Spring rate sensitivity
spring_rates = [30000, 35000, 40000, 45000, 50000];  % N/m

for i = 1:length(spring_rates)
    Vehicle.Suspension.Front.SpringRate = spring_rates(i);
    sim('sm_car');
    results(i) = analyze_results(logsout);
end

plot(spring_rates, [results.MaxLateralG])
xlabel('Front Spring Rate (N/m)')
ylabel('Maximum Lateral Acceleration (g)')
```

---

## 7. Validation Setup

### 7.1 Test Equipment

Minimum required:

| Equipment | Purpose | Estimated Cost |
|-----------|---------|----------------|
| MPU9250 IMU | Acceleration and rotation rate measurement | INR 600 |
| Arduino Uno R3 | Data acquisition | INR 400 |
| Arduino MicroSD Shield | Data logging | INR 300 |

Optional:

| Equipment | Purpose | Estimated Cost |
|-----------|---------|----------------|
| NEO-6M GPS | Position and velocity | INR 500 |
| Laptop with MATLAB | Data processing | Existing |

### 7.2 IMU Specifications

The MPU9250 provides:

- 3-axis accelerometer: +/- 16g range, 16-bit resolution
- 3-axis gyroscope: +/- 2000 deg/s range, 16-bit resolution
- Sample rate: Up to 1000 Hz (use 100-200 Hz for vehicle dynamics)

### 7.3 Mounting Requirements

The IMU must be mounted:

1. At or near the vehicle center of gravity
2. Rigidly attached (no vibration isolation)
3. Aligned with vehicle axes (X forward, Y left, Z up)

Alignment procedure:

1. Level the vehicle on a flat surface
2. Mount IMU with X-axis pointing forward along vehicle centerline
3. Record any mounting offset angles for post-processing correction

### 7.4 Data Acquisition Code

Arduino sketch for data logging:

```cpp
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MPU9250.h>

MPU9250 mpu;
File dataFile;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.setup(0x68)) {
        Serial.println("MPU9250 connection failed");
        while (1);
    }
    
    if (!SD.begin(10)) {
        Serial.println("SD card failed");
        while (1);
    }
    
    dataFile = SD.open("data.csv", FILE_WRITE);
    dataFile.println("time_ms,ax,ay,az,gx,gy,gz");
}

void loop() {
    if (mpu.update()) {
        unsigned long t = millis();
        
        dataFile.print(t);
        dataFile.print(",");
        dataFile.print(mpu.getAccX(), 4);
        dataFile.print(",");
        dataFile.print(mpu.getAccY(), 4);
        dataFile.print(",");
        dataFile.print(mpu.getAccZ(), 4);
        dataFile.print(",");
        dataFile.print(mpu.getGyroX(), 4);
        dataFile.print(",");
        dataFile.print(mpu.getGyroY(), 4);
        dataFile.print(",");
        dataFile.println(mpu.getGyroZ(), 4);
    }
    
    delay(10);  // 100 Hz sample rate
}
```

---

## 8. Sensor Requirements

### 8.1 What the IMU Measures

| Measurement | Axis | Unit | Vehicle Dynamics Use |
|-------------|------|------|----------------------|
| Longitudinal acceleration | X | g | 0-60 time validation, braking performance |
| Lateral acceleration | Y | g | Tire grip, cornering capability |
| Vertical acceleration | Z | g | Suspension response, ride quality |
| Roll rate | X | deg/s | Roll dynamics, anti-roll bar effectiveness |
| Pitch rate | Y | deg/s | Pitch dynamics, weight transfer |
| Yaw rate | Z | deg/s | Understeer gradient, stability |

### 8.2 Derived Quantities

From raw IMU data, the following quantities can be calculated:

Tire friction coefficient (during skidpad):
```
mu = sqrt(ax^2 + ay^2) / g
```

Understeer gradient (during steady-state cornering):
```
K_us = (delta - L/R) / ay
where:
  delta = steering angle (rad)
  L = wheelbase (m)
  R = turn radius (m)
  ay = lateral acceleration (m/s^2)
```

Velocity (by integration):
```
v(t) = v(0) + integral(ax * dt)
```

Note: Velocity from accelerometer integration drifts over time. Use only for short-duration tests (less than 30 seconds).

### 8.3 Required Test Maneuvers

| Test | Duration | Data Collected | Model Validation |
|------|----------|----------------|------------------|
| Skidpad | 2-3 laps | Lateral acceleration, yaw rate | Tire grip coefficient |
| 0-60 km/h acceleration | 10-15 seconds | Longitudinal acceleration | Powertrain model |
| Step steer | 5-10 seconds | Lateral acceleration, yaw rate | Transient response |
| Braking | 5-10 seconds | Longitudinal acceleration | Brake model |

---

## 9. Validation Procedure

### 9.1 Pre-Test Preparation

1. Charge power bank fully
2. Format SD card (FAT32)
3. Verify IMU calibration (accelerometer should read 1g in Z when stationary)
4. Record ambient temperature
5. Check tire pressures and record values
6. Weigh vehicle with driver and record total mass

### 9.2 Test Procedure: Skidpad

Purpose: Validate tire grip model

Setup:
1. Mark a circle of known radius (15-20 meters recommended)
2. Mount IMU at vehicle CG
3. Start data logging

Procedure:
1. Enter circle at low speed
2. Gradually increase speed until tire slip occurs
3. Hold maximum speed for 2-3 complete laps
4. Repeat in opposite direction
5. Stop logging and retrieve SD card

Expected data:
- Maximum lateral acceleration: 0.9-1.1 g (for mu = 1.3 tires)
- Steady yaw rate at maximum speed

### 9.3 Test Procedure: Acceleration

Purpose: Validate powertrain model

Setup:
1. Find straight, flat surface (minimum 200 meters)
2. Mount IMU at vehicle CG
3. Start data logging

Procedure:
1. Start from standstill
2. Apply full throttle
3. Accelerate to maximum speed or end of course
4. Stop logging and retrieve SD card

Expected data:
- Peak longitudinal acceleration: 0.4-0.6 g (depending on power-to-weight ratio)
- Velocity profile over time (by integration)
- 0-60 km/h time

### 9.4 Data Processing

MATLAB script for processing logged data:

```matlab
function results = process_imu_data(filename)
    % Read CSV file
    data = readtable(filename);
    
    % Extract columns
    t = data.time_ms / 1000;  % Convert to seconds
    ax = data.ax;             % Longitudinal acceleration (g)
    ay = data.ay;             % Lateral acceleration (g)
    az = data.az;             % Vertical acceleration (g)
    gz = data.gz;             % Yaw rate (deg/s)
    
    % Calculate total lateral acceleration
    ay_total = sqrt(ax.^2 + ay.^2);
    
    % Find maximum values
    results.max_lateral_g = max(abs(ay));
    results.max_longitudinal_g = max(abs(ax));
    results.max_yaw_rate = max(abs(gz));
    
    % Calculate velocity (for acceleration test)
    v = cumtrapz(t, ax * 9.81);  % m/s
    results.velocity = v;
    results.time = t;
    
    % Find 0-60 km/h time
    v_kmh = v * 3.6;
    idx_60 = find(v_kmh >= 60, 1, 'first');
    if ~isempty(idx_60)
        results.time_0_60 = t(idx_60);
    else
        results.time_0_60 = NaN;
    end
    
    % Plot results
    figure;
    subplot(2,2,1);
    plot(t, ay);
    xlabel('Time (s)');
    ylabel('Lateral Acceleration (g)');
    title('Lateral Acceleration');
    grid on;
    
    subplot(2,2,2);
    plot(t, ax);
    xlabel('Time (s)');
    ylabel('Longitudinal Acceleration (g)');
    title('Longitudinal Acceleration');
    grid on;
    
    subplot(2,2,3);
    plot(t, gz);
    xlabel('Time (s)');
    ylabel('Yaw Rate (deg/s)');
    title('Yaw Rate');
    grid on;
    
    subplot(2,2,4);
    plot(ax, ay, '.');
    xlabel('Longitudinal Acceleration (g)');
    ylabel('Lateral Acceleration (g)');
    title('GG Diagram');
    axis equal;
    grid on;
end
```

### 9.5 Comparison with Model

After collecting test data, compare with simulation results:

```matlab
% Load test data
test_results = process_imu_data('data.csv');

% Run simulation with same conditions
sim('sm_car');

% Extract simulation results
sim_ay = logsout.get('LateralAccel').Values.Data;
sim_time = logsout.get('LateralAccel').Values.Time;

% Compare
figure;
plot(test_results.time, test_results.ay, 'b', 'DisplayName', 'Test');
hold on;
plot(sim_time, sim_ay, 'r--', 'DisplayName', 'Simulation');
xlabel('Time (s)');
ylabel('Lateral Acceleration (g)');
legend;
title('Model Validation: Lateral Acceleration');
grid on;

% Calculate correlation
correlation = corrcoef(interp1(sim_time, sim_ay, test_results.time), test_results.ay);
fprintf('Correlation coefficient: %.3f\n', correlation(1,2));

% Calculate RMS error
error_rms = rms(interp1(sim_time, sim_ay, test_results.time) - test_results.ay);
fprintf('RMS Error: %.3f g\n', error_rms);
```

---

## 10. Deliverables

### 10.1 Model Files

| File | Description |
|------|-------------|
| Vehicle_data_C37.m | Vehicle parameter file |
| sm_car_C37.slx | Configured Simulink model |
| tire_gmax_170_50_R13.tir | Pacejka tire file |
| validate_vehicle_dynamics.m | Validation script |

### 10.2 Report Sections

The Vehicle Dynamics section of the report should include:

1. Introduction
   - Vehicle overview
   - Modeling objectives

2. Model Development (50 points)
   - Modeling approach selection
   - Vehicle parameter definition
   - Suspension model description
   - Tire model description
   - Steering model description

3. Validation (30 points)
   - Test equipment description
   - Test procedure
   - Test results (with plots)
   - Model-to-test comparison
   - Correlation metrics (RMS error, correlation coefficient)

4. Application and Insights (20 points)
   - Sensitivity analysis results
   - Design recommendations based on model
   - Trade-off studies performed

5. Conclusion
   - Summary of findings
   - Limitations and future work

### 10.3 Required Figures

| Figure | Description |
|--------|-------------|
| GG diagram | Measured vs simulated |
| Lateral acceleration vs time | Skidpad test comparison |
| Longitudinal acceleration vs time | Acceleration test comparison |
| Velocity vs time | 0-60 test comparison |
| Sensitivity plots | Spring rate, damping, CG height effects |

### 10.4 Required Tables

| Table | Description |
|-------|-------------|
| Vehicle parameters | All input parameters with sources |
| Test conditions | Date, temperature, tire pressure, vehicle mass |
| Validation metrics | Max lateral g, 0-60 time, correlation coefficient |
| Model vs test comparison | Key metrics side-by-side |

---

## 11. References

### 11.1 MathWorks Documentation

- Simscape Multibody: https://www.mathworks.com/help/sm/
- Vehicle Dynamics Blockset: https://www.mathworks.com/help/vdynblks/
- Pacejka Tire Model: https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html

### 11.2 Formula Student Resources

- Formula-Student-Vehicle-Simscape GitHub: https://github.com/mathworks/Formula-Student-Vehicle-Simscape
- FSAE Tire Test Consortium: https://www.millikenresearch.com/fsaettc.html

### 11.3 Technical References

- Milliken, W.F. and Milliken, D.L. "Race Car Vehicle Dynamics" SAE International, 1995.
- Pacejka, H.B. "Tire and Vehicle Dynamics" Butterworth-Heinemann, 2012.
- Gillespie, T.D. "Fundamentals of Vehicle Dynamics" SAE International, 1992.

### 11.4 Sensor Documentation

- MPU9250 Datasheet: https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/
- Arduino SD Library: https://www.arduino.cc/en/Reference/SD

---

## Revision History

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2025-12-23 | 1.0 | Team Unwired | Initial document |

