# ESP32 IMU WiFi Streaming + Phone GPS Setup

## Overview

This setup uses:
- **ESP32 + MPU9250**: High-rate IMU data (100 Hz) streamed over WiFi
- **Phone (phyphox app)**: GPS position data (1-10 Hz depending on phone)

Data is synced by starting both recordings simultaneously.

## Hardware Requirements

| Component | Purpose |
|-----------|---------|
| ESP32 DevKit | WiFi streaming, IMU interface |
| MPU9250 | 9-DOF IMU (accelerometer + gyroscope) |
| Smartphone | GPS receiver |
| Laptop | Data receiver, MATLAB analysis |
| Power bank | Power for ESP32 during testing |

## Wiring

```
    MPU9250          ESP32
    -------          -----
    VCC     -------> 3.3V
    GND     -------> GND
    SDA     -------> GPIO21
    SCL     -------> GPIO22

    Button  -------> GPIO4 to GND
    LED     -------> GPIO2 (built-in on most boards)
```

No SD card module required.

## Software Setup

### ESP32 Configuration

1. Open `sensor_code/esp32_mpu9250_wifi/esp32_mpu9250_wifi.ino`

2. Edit WiFi credentials (lines 28-29):
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";
   const char* WIFI_PASSWORD = "YourWiFiPassword";
   ```

3. Upload to ESP32

4. Open Serial Monitor (115200 baud) to see the IP address

### Phone GPS App: phyphox

phyphox is a free physics app developed by RWTH Aachen University. It can log GPS and export to CSV.

**Installation:**
- Android: https://play.google.com/store/apps/details?id=de.rwth_aachen.phyphox
- iOS: https://apps.apple.com/app/phyphox/id1127319693

**Configuration:**

1. Open phyphox
2. Go to "GPS" experiment (under "Location" category)
3. Tap the three dots menu > "Export Data" settings
4. Set format to CSV

**During Test:**

1. Start GPS logging in phyphox
2. Note the exact time (use stopwatch or countdown)
3. Press ESP32 button to start IMU streaming at same moment
4. Run test
5. Stop both recordings
6. Export phyphox data to CSV

## Data Collection Workflow

### Network Setup

**Option A: Mobile Hotspot (Recommended)**
1. Enable mobile hotspot on phone
2. Connect ESP32 to hotspot (edit WiFi credentials)
3. Connect laptop to same hotspot
4. All devices on same network

**Option B: Existing WiFi**
1. Connect all devices to same WiFi network
2. Ensure devices can communicate (some networks block this)

### Recording Procedure

1. **Prepare ESP32:**
   - Power on ESP32
   - Open Serial Monitor, note IP address (e.g., `192.168.1.100`)
   - Wait for "TCP server started" message

2. **Start receiver on laptop:**
   ```bash
   # Python option
   python sensor_code/receive_imu_data.py 192.168.1.100 5000 run001.csv
   
   # Or MATLAB option
   matlab -r "data = receive_imu_tcp('192.168.1.100', 5000, 60);"
   ```

3. **Prepare phone:**
   - Open phyphox
   - Select "GPS" experiment
   - Position phone securely in vehicle

4. **Calibrate ESP32:**
   - Place sensor on flat surface
   - Press button once
   - Wait 5 seconds for calibration

5. **Synchronized start:**
   - Press play in phyphox
   - Immediately press ESP32 button
   - LED turns on = streaming

6. **Run test:**
   - Perform acceleration run or skidpad laps
   - Keep test under 60 seconds for best accuracy

7. **Stop recording:**
   - Press ESP32 button (LED off)
   - Stop phyphox recording
   - Export phyphox data (three dots > Export)

## Data Files

After recording you will have:

| File | Source | Contents |
|------|--------|----------|
| `run001.csv` | ESP32 via laptop | time_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps |
| `GPS.csv` | phyphox | time, latitude, longitude, altitude, speed, bearing |

## Data Import in MATLAB

```matlab
% Import IMU data
imu_data = import_imu_data('run001.csv');

% Import GPS data from phyphox
gps_raw = readtable('GPS.csv');
gps.time = gps_raw.t_s_;
gps.lat = gps_raw.Latitude;
gps.lon = gps_raw.Longitude;
gps.speed = gps_raw.Speed_m_s_;  % Speed from GPS
gps.bearing = gps_raw.Direction;

% Time synchronization (if needed)
% Align by finding peak acceleration event in both datasets
```

## Alternative: Phone-Only Solution

If the ESP32 setup is problematic, phyphox can also log accelerometer and gyroscope data from the phone sensors.

**Pros:**
- No external hardware needed
- GPS and IMU automatically synced
- Single device to manage

**Cons:**
- Phone accelerometer may be lower quality than MPU9250
- Harder to mount phone securely
- Phone orientation must be carefully controlled

**To use phone-only:**

1. Open phyphox
2. Create custom experiment combining:
   - Accelerometer (with g)
   - Gyroscope
   - GPS
3. Export all data
4. Import to MATLAB

## MATLAB Functions

| Function | Purpose |
|----------|---------|
| `receive_imu_tcp()` | Live receive IMU data in MATLAB |
| `import_imu_data()` | Import CSV file from ESP32/Python |
| `analyze_acceleration_test()` | Extract 0-75m time, peak g |
| `analyze_skidpad_test()` | Extract lateral g, friction coefficient |
| `compare_simulation_vs_measured()` | Validate model against test data |

## Troubleshooting

### ESP32 Not Connecting to WiFi

1. Check SSID and password (case sensitive)
2. Ensure 2.4 GHz network (ESP32 does not support 5 GHz)
3. Move closer to router/hotspot

### Cannot Connect from Laptop

1. Verify same network
2. Check firewall settings (allow port 5000)
3. Try pinging ESP32 IP address
4. Some public WiFi blocks device-to-device communication

### Data Rate Lower Than Expected

1. WiFi congestion can cause packet delays
2. Reduce sample rate if needed (edit SAMPLE_RATE_HZ)
3. Move closer to access point

### GPS Not Updating

1. Ensure location permission granted to phyphox
2. Be outdoors with clear sky view
3. Wait 30-60 seconds for GPS lock
4. Check GPS accuracy indicator in phyphox

## References

- phyphox Official Site: https://phyphox.org/
- phyphox Wiki: https://phyphox.org/wiki/
- ESP32 WiFi Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
- MPU9250 Library: https://github.com/hideakitai/MPU9250
