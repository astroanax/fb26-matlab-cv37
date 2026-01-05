/*
 * ESP32 MPU9250 Data Logger for Vehicle Dynamics
 * Team Unwired - C37 Formula Bharat 2026
 * 
 * This code reads acceleration and gyroscope data from MPU9250
 * and logs to SD card in CSV format compatible with MATLAB analysis.
 * 
 * Hardware Connections:
 * ---------------------
 * MPU9250 -> ESP32
 *   VCC   -> 3.3V
 *   GND   -> GND
 *   SDA   -> GPIO 21
 *   SCL   -> GPIO 22
 * 
 * MicroSD Module -> ESP32
 *   VCC   -> 3.3V
 *   GND   -> GND
 *   MISO  -> GPIO 19
 *   MOSI  -> GPIO 23
 *   SCK   -> GPIO 18
 *   CS    -> GPIO 5
 * 
 * Button (Start/Stop) -> GPIO 4 (pull-up to 3.3V with 10k resistor)
 * LED (Status)        -> GPIO 2 (built-in LED on most ESP32 boards)
 * 
 * Libraries Required:
 * -------------------
 * 1. MPU9250 by hideakitai: https://github.com/hideakitai/MPU9250
 * 2. SD (built-in with ESP32 Arduino core)
 * 
 * Install via Arduino Library Manager or PlatformIO.
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "MPU9250.h"

// Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_CS 5
#define BUTTON_PIN 4
#define LED_PIN 2

// MPU9250 I2C Address (0x68 if AD0 is LOW, 0x69 if AD0 is HIGH)
#define MPU9250_ADDRESS 0x68

// Sampling Configuration
#define SAMPLE_RATE_HZ 100          // 100 Hz sampling rate
#define SAMPLE_INTERVAL_US 10000    // 10000 us = 10 ms = 100 Hz

// Calibration offsets (update these after calibration)
float accel_offset_x = 0.0;
float accel_offset_y = 0.0;
float accel_offset_z = 0.0;
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

// Global objects
MPU9250 mpu;
File dataFile;

// State variables
bool isLogging = false;
bool lastButtonState = HIGH;
unsigned long lastSampleTime = 0;
unsigned long startTime = 0;
unsigned long sampleCount = 0;
char filename[20];

// Data buffer for batch writing (improves SD card performance)
#define BUFFER_SIZE 10
String dataBuffer = "";
int bufferCount = 0;

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait up to 3 seconds for serial
    
    Serial.println("========================================");
    Serial.println("ESP32 MPU9250 Vehicle Dynamics Logger");
    Serial.println("Team Unwired - C37");
    Serial.println("========================================");
    
    // Initialize pins
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // 400 kHz I2C clock
    
    // Initialize MPU9250
    Serial.print("Initializing MPU9250... ");
    if (!mpu.setup(MPU9250_ADDRESS)) {
        Serial.println("FAILED!");
        Serial.println("Check wiring: SDA->GPIO21, SCL->GPIO22");
        Serial.println("Check I2C address: 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)");
        errorBlink();
    }
    Serial.println("OK");
    
    // Configure MPU9250 settings
    // Accelerometer: +/- 4g (good for vehicle dynamics, ~1.5g max expected)
    // Gyroscope: +/- 500 deg/s (good for yaw rate measurement)
    mpu.setAccBias(0, 0, 0);
    mpu.setGyroBias(0, 0, 0);
    
    // Initialize SD card
    Serial.print("Initializing SD card... ");
    if (!SD.begin(SD_CS)) {
        Serial.println("FAILED!");
        Serial.println("Check wiring: MISO->GPIO19, MOSI->GPIO23, SCK->GPIO18, CS->GPIO5");
        errorBlink();
    }
    Serial.println("OK");
    
    // Perform calibration
    Serial.println("");
    Serial.println("CALIBRATION");
    Serial.println("-----------");
    Serial.println("Place sensor on FLAT, LEVEL surface.");
    Serial.println("Keep sensor STATIONARY for 5 seconds.");
    Serial.println("Starting in 3 seconds...");
    delay(3000);
    
    calibrateSensor();
    
    Serial.println("");
    Serial.println("READY");
    Serial.println("------");
    Serial.println("Press button to START logging.");
    Serial.println("Press button again to STOP logging.");
    Serial.println("");
    
    // Ready indication: 3 quick blinks
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void loop() {
    // Check button state
    bool buttonState = digitalRead(BUTTON_PIN);
    
    // Detect button press (falling edge with debounce)
    if (buttonState == LOW && lastButtonState == HIGH) {
        delay(50);  // Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (!isLogging) {
                startLogging();
            } else {
                stopLogging();
            }
        }
    }
    lastButtonState = buttonState;
    
    // If logging, sample at fixed rate
    if (isLogging) {
        unsigned long currentTime = micros();
        if (currentTime - lastSampleTime >= SAMPLE_INTERVAL_US) {
            lastSampleTime = currentTime;
            sampleData();
        }
    }
}

void calibrateSensor() {
    Serial.println("Calibrating... (keep sensor still)");
    
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int samples = 500;
    
    // Blink LED during calibration
    for (int i = 0; i < samples; i++) {
        if (mpu.update()) {
            sum_ax += mpu.getAccX();
            sum_ay += mpu.getAccY();
            sum_az += mpu.getAccZ();
            sum_gx += mpu.getGyroX();
            sum_gy += mpu.getGyroY();
            sum_gz += mpu.getGyroZ();
        }
        if (i % 50 == 0) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        delay(10);
    }
    digitalWrite(LED_PIN, LOW);
    
    // Calculate offsets
    // Note: Z accelerometer should read 1g when level, so offset is (measured - 1.0)
    accel_offset_x = sum_ax / samples;
    accel_offset_y = sum_ay / samples;
    accel_offset_z = (sum_az / samples) - 1.0;  // Subtract 1g for gravity
    gyro_offset_x = sum_gx / samples;
    gyro_offset_y = sum_gy / samples;
    gyro_offset_z = sum_gz / samples;
    
    Serial.println("Calibration complete!");
    Serial.println("Offsets:");
    Serial.print("  Accel: X="); Serial.print(accel_offset_x, 4);
    Serial.print(", Y="); Serial.print(accel_offset_y, 4);
    Serial.print(", Z="); Serial.println(accel_offset_z, 4);
    Serial.print("  Gyro:  X="); Serial.print(gyro_offset_x, 4);
    Serial.print(", Y="); Serial.print(gyro_offset_y, 4);
    Serial.print(", Z="); Serial.println(gyro_offset_z, 4);
}

void startLogging() {
    // Generate unique filename
    int fileNum = 0;
    do {
        sprintf(filename, "/run%03d.csv", fileNum);
        fileNum++;
    } while (SD.exists(filename) && fileNum < 1000);
    
    // Open file
    dataFile = SD.open(filename, FILE_WRITE);
    if (!dataFile) {
        Serial.println("ERROR: Could not create file!");
        errorBlink();
        return;
    }
    
    // Write CSV header
    // Format matches MATLAB import requirements
    dataFile.println("time_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
    
    // Initialize timing
    startTime = millis();
    lastSampleTime = micros();
    sampleCount = 0;
    dataBuffer = "";
    bufferCount = 0;
    
    isLogging = true;
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("");
    Serial.print("LOGGING STARTED: ");
    Serial.println(filename);
    Serial.println("Press button to stop.");
}

void stopLogging() {
    // Flush remaining buffer
    if (bufferCount > 0) {
        dataFile.print(dataBuffer);
    }
    
    dataFile.close();
    isLogging = false;
    digitalWrite(LED_PIN, LOW);
    
    float duration = (millis() - startTime) / 1000.0;
    float actualRate = sampleCount / duration;
    
    Serial.println("");
    Serial.println("LOGGING STOPPED");
    Serial.print("  File: ");
    Serial.println(filename);
    Serial.print("  Duration: ");
    Serial.print(duration, 2);
    Serial.println(" seconds");
    Serial.print("  Samples: ");
    Serial.println(sampleCount);
    Serial.print("  Actual rate: ");
    Serial.print(actualRate, 1);
    Serial.println(" Hz");
    Serial.println("");
    Serial.println("Press button to start new recording.");
}

void sampleData() {
    if (!mpu.update()) {
        return;  // Skip if no new data
    }
    
    // Get raw data and apply calibration offsets
    float ax = mpu.getAccX() - accel_offset_x;
    float ay = mpu.getAccY() - accel_offset_y;
    float az = mpu.getAccZ() - accel_offset_z;
    float gx = mpu.getGyroX() - gyro_offset_x;
    float gy = mpu.getGyroY() - gyro_offset_y;
    float gz = mpu.getGyroZ() - gyro_offset_z;
    
    // Calculate timestamp
    float timestamp = (millis() - startTime) / 1000.0;
    
    // Format data line
    // time_s: seconds since start
    // ax_g, ay_g, az_g: acceleration in g (1g = 9.81 m/s^2)
    // gx_dps, gy_dps, gz_dps: angular rate in degrees per second
    char line[100];
    sprintf(line, "%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
            timestamp, ax, ay, az, gx, gy, gz);
    
    // Add to buffer
    dataBuffer += line;
    bufferCount++;
    sampleCount++;
    
    // Write buffer to SD card when full
    if (bufferCount >= BUFFER_SIZE) {
        dataFile.print(dataBuffer);
        dataBuffer = "";
        bufferCount = 0;
        
        // Toggle LED to show activity
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    // Print to serial for debugging (every 100 samples)
    if (sampleCount % 100 == 0) {
        Serial.print("Sample ");
        Serial.print(sampleCount);
        Serial.print(": ax=");
        Serial.print(ax, 2);
        Serial.print("g, ay=");
        Serial.print(ay, 2);
        Serial.print("g, gz=");
        Serial.print(gz, 1);
        Serial.println(" dps");
    }
}

void errorBlink() {
    // Continuous fast blink to indicate error
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

/*
 * AXIS ORIENTATION
 * ----------------
 * Mount the sensor with the following orientation:
 * 
 *   +X axis: Forward (direction of travel)
 *   +Y axis: Left (driver's left)
 *   +Z axis: Up (opposite to gravity)
 * 
 * This matches SAE/ISO vehicle dynamics convention.
 * 
 * When mounted correctly:
 *   - Acceleration during forward motion: +ax
 *   - Acceleration during left turn: +ay
 *   - Acceleration when stationary: az = +1g
 *   - Yaw rate during left turn: +gz
 * 
 * 
 * DATA FORMAT
 * -----------
 * CSV columns:
 *   time_s   - Time in seconds since logging started
 *   ax_g     - Longitudinal acceleration in g (+ forward)
 *   ay_g     - Lateral acceleration in g (+ left)
 *   az_g     - Vertical acceleration in g (+ up, gravity removed)
 *   gx_dps   - Roll rate in deg/s (+ right roll)
 *   gy_dps   - Pitch rate in deg/s (+ nose up)
 *   gz_dps   - Yaw rate in deg/s (+ left turn)
 * 
 * 
 * MOUNTING NOTES
 * --------------
 * 1. Mount sensor rigidly (no rubber/foam isolation)
 * 2. Mount at or near vehicle CG if possible
 * 3. Align axes carefully with vehicle centerline
 * 4. Secure all wires to prevent movement
 * 5. Protect from water/dust during outdoor testing
 * 
 * 
 * TEST PROCEDURE
 * --------------
 * 1. Power on ESP32, wait for "READY" message
 * 2. Press button to start logging
 * 3. Perform test maneuver (skidpad, acceleration, etc.)
 * 4. Press button to stop logging
 * 5. Remove SD card and copy CSV file to computer
 * 6. Import to MATLAB using provided script
 */
