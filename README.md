# Formula Bharat 2026 - Vehicle Dynamics Modeling
## Team Unwired, NIT Calicut | Car: C37 (ICE)

**Competition:** Formula Bharat 2026 - MathWorks Modeling Award  
**Category:** Vehicle Dynamics (100 points)  
**Submission Date:** January 5, 2026

---

## Vehicle Specifications

- **Powertrain:** Royal Enfield Classic 350 (346cc, 20mm restrictor)
- **Mass:** 300 kg with driver (35:65 F:R distribution)
- **Wheelbase:** 1560 mm
- **Track:** 1200 mm (F), 1150 mm (R)
- **Suspension:** Double wishbone pushrod (decoupled, both axles)
- **Tires:** G-max 170/50 R13

---

## Repository Contents

### Core Models
- `Vehicle_data_C37_Unwired.m` - Complete vehicle parameterization
- `C37_Gmax_170_50R13.tir` - Pacejka tire model
- `generate_acceleration_track.m` - Custom test track generation (250m straights, 20m radius)
- `run_acceleration_simulation.m` - Acceleration event simulation script

### Simulation Results
- `results/` - Simulation output plots (trajectory, velocity, wheel speeds, tire forces)
- `pics/` - Simscape model screenshots and 3D visualizations

### Validation Infrastructure
- `sensor_code/` - ESP32 + MPU9250 IMU data acquisition (WiFi streaming)
- `validation/` - MATLAB validation pipeline (IMU import, GPS fusion, correlation analysis)

### Documentation
- `Team_Unwired_VehicleDynamics_SUBMISSION.pdf` - Final competition report (3 pages)
- `docs/` - Setup guides and workflow documentation

---

## Simulation Framework

**Platform:** MATLAB R2025B, Simulink, Simscape Multibody  
**Template:** [Formula-Student-Vehicle-Simscape](https://github.com/simscape/Formula-Student-Vehicle-Simscape)  
**Solver:** ode23t (variable-step, stiff) for adaptive tire dynamics resolution

---

## Key Results

- **0-75m Acceleration:** 4.85 seconds
- **Peak Acceleration:** 0.85g (tire-limited)
- **Peak Tire Force:** 1850 N (rear, longitudinal)
- **Peak Power:** 12.6 kW @ 5000 rpm (restrictor-limited)

---

## Validation Plan

**Hardware:** MPU9250 IMU + ESP32 (WiFi streaming @ 100 Hz) + Smartphone GPS (phyphox)  
**Cost:** ₹1,300  
**Target Metrics:** ±5% lateral g, ±10% longitudinal g, ±0.3s lap time, ±10% yaw rate  
**Testing Date:** January 6, 2026

---

## Usage

1. Load vehicle data: `run('Vehicle_data_C37_Unwired.m')`
2. Generate track: `generate_acceleration_track()`
3. Run simulation: `run_acceleration_simulation()` (requires Simscape template)

---

## Contact

**Team:** Team Unwired  
**Institute:** National Institute of Technology Calicut  
**Competition:** Formula Bharat 2026
