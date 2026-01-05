# ESP32 MPU9250 IMU Data Logger

## Hardware Requirements

| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller | ESP32 DevKit V1 or equivalent | 1 |
| IMU Sensor | MPU9250 9-DOF | 1 |
| Storage | MicroSD Card Module | 1 |
| Storage Media | MicroSD Card (4GB minimum, FAT32 formatted) | 1 |
| Control | Momentary Push Button | 1 |
| Status | LED (any color) | 1 |
| Resistor | 220 ohm (for LED) | 1 |
| Resistor | 10k ohm (pull-down for button) | 1 |
| Cables | Jumper wires | As needed |

## Wiring Diagram

```
                    ESP32 DevKit V1
                   +---------------+
                   |               |
    MPU9250 VCC ---| 3.3V     VIN  |--- 5V Power
    MPU9250 GND ---| GND      GND  |--- Common Ground
    MPU9250 SDA ---| GPIO21        |
    MPU9250 SCL ---| GPIO22        |
                   |               |
    SD VCC --------| 3.3V          |
    SD GND --------| GND           |
    SD CS ---------| GPIO5         |
    SD MOSI -------| GPIO23        |
    SD MISO -------| GPIO19        |
    SD SCK --------| GPIO18        |
                   |               |
    Button --------| GPIO4    GND  |--- Button (other leg)
    LED (+) -------| GPIO2         |
    LED (-) -------| GND (via 220R)|
                   |               |
                   +---------------+
```

## MPU9250 Connection Detail

| MPU9250 Pin | ESP32 Pin | Wire Color (suggested) |
|-------------|-----------|------------------------|
| VCC | 3.3V | Red |
| GND | GND | Black |
| SDA | GPIO21 | Blue |
| SCL | GPIO22 | Yellow |
| AD0 | GND (or leave floating) | - |
| INT | Not connected | - |

The AD0 pin determines the I2C address. When connected to GND or floating, the address is 0x68. When connected to VCC, the address is 0x69.

## MicroSD Module Connection Detail

| SD Module Pin | ESP32 Pin | Wire Color (suggested) |
|---------------|-----------|------------------------|
| VCC | 3.3V | Red |
| GND | GND | Black |
| CS | GPIO5 | Orange |
| MOSI | GPIO23 | Green |
| MISO | GPIO19 | Blue |
| SCK | GPIO18 | Yellow |

## Button and LED Connection

### Push Button
- One leg to GPIO4
- Other leg to GND
- Internal pull-up is enabled in software

### Status LED
- Anode (long leg) to GPIO2 through 220 ohm resistor
- Cathode (short leg) to GND

## Software Setup

### Arduino IDE Configuration

1. Install the ESP32 board support:
   - Open Arduino IDE
   - Go to File > Preferences
   - Add to Additional Board Manager URLs:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to Tools > Board > Boards Manager
   - Search for "esp32" and install "esp32 by Espressif Systems"

2. Select the correct board:
   - Tools > Board > ESP32 Arduino > ESP32 Dev Module

3. Install required libraries:
   - Sketch > Include Library > Manage Libraries
   - Search and install: "MPU9250" by hideakitai
   - The SD library is included with ESP32 core

### PlatformIO Configuration (Alternative)

Create a `platformio.ini` file:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
    hideakitai/MPU9250@^0.4.8
monitor_speed = 115200
```

## Operation

### Starting the Logger

1. Power on the ESP32
2. Wait for calibration prompt on serial monitor (if connected)
3. Place the sensor on a flat, stable surface
4. Press the button to start calibration (5 seconds)
5. Wait for LED to stop blinking
6. Attach sensor to vehicle
7. Press button to start logging (LED turns on solid)
8. Press button again to stop logging (LED turns off)

### LED Status Indicators

| LED State | Meaning |
|-----------|---------|
| Off | Idle, not logging |
| Blinking (1 Hz) | Calibrating |
| Solid On | Logging active |
| Fast Blink | Error (SD card or sensor issue) |

### File Output

Files are saved to the SD card with naming convention:
- `RUN001.CSV`
- `RUN002.CSV`
- etc.

Each file contains:
```
time_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps
0.000,0.01,-0.02,1.00,0.1,-0.2,0.0
0.010,0.02,-0.01,0.99,0.1,-0.1,0.1
...
```

## Sensor Orientation

Mount the sensor with the correct orientation for SAE vehicle dynamics convention:

```
        +X (Forward)
           ^
           |
           |
    +Y <---+--- -Y
   (Left)  |   (Right)
           |
           v
        -X (Rearward)

    +Z points UP (out of the board)
```

### Mounting Guidelines

1. Align the MPU9250 X-axis with the vehicle longitudinal axis
2. Positive X should point toward the front of the vehicle
3. Positive Y should point toward the driver's left
4. Positive Z should point upward
5. Secure the sensor rigidly to prevent vibration-induced noise
6. Mount as close to the vehicle CG as practical

## Data Import in MATLAB

After retrieving the SD card, use the provided MATLAB function:

```matlab
% Import IMU data from CSV file
imu_data = import_imu_data('path/to/RUN001.CSV');

% View data summary
disp(imu_data.metadata);

% Analyze acceleration test
accel_results = analyze_acceleration_test(imu_data);

% Analyze skidpad test
skidpad_results = analyze_skidpad_test(imu_data);

% Convert to Simscape format for plotting
logsout_sm_car = imu_to_simscape_format(imu_data);
```

## Troubleshooting

### Sensor Not Detected

1. Check I2C connections (SDA to GPIO21, SCL to GPIO22)
2. Verify power connections (3.3V, not 5V)
3. Check I2C address (default 0x68)
4. Run I2C scanner sketch to verify address

### SD Card Not Working

1. Format card as FAT32
2. Check SPI connections
3. Verify CS pin is GPIO5
4. Try a different SD card
5. Check card capacity (some large cards have compatibility issues)

### Noisy Data

1. Verify rigid mounting
2. Add vibration isolation if needed
3. Increase averaging in calibration
4. Check for electrical interference from motor

### Drift in Integrated Values

Integration drift is expected with consumer-grade IMU sensors. For position and velocity:
- Keep test runs short (under 60 seconds)
- Use only for transient analysis, not position tracking
- Combine with GPS for longer tests (not covered in this setup)

## References

- MPU9250 Datasheet: https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/
- ESP32 Pinout Reference: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- hideakitai MPU9250 Library: https://github.com/hideakitai/MPU9250
- SAE J670 Vehicle Dynamics Terminology: https://www.sae.org/standards/content/j670_200801/
