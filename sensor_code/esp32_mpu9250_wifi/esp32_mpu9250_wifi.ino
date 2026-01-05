/*
 * ESP32 MPU9250 WiFi Data Streamer
 * Team Unwired - Formula Bharat 2026
 * 
 * Streams IMU data over WiFi TCP socket to a laptop running MATLAB/Python.
 * Use with phone GPS app (phyphox) for position data.
 * 
 * Hardware Connections:
 *   MPU9250 VCC  -> ESP32 3.3V
 *   MPU9250 GND  -> ESP32 GND
 *   MPU9250 SDA  -> ESP32 GPIO21
 *   MPU9250 SCL  -> ESP32 GPIO22
 *   Button       -> ESP32 GPIO4 (to GND)
 *   LED          -> ESP32 GPIO2 (built-in LED on most boards)
 * 
 * Operation:
 *   1. Power on ESP32, connects to WiFi
 *   2. Note the IP address from Serial Monitor
 *   3. Run receiver script on laptop
 *   4. Press button to start/stop streaming
 *   
 * Data Format (CSV over TCP):
 *   timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n
 */

#include <WiFi.h>
#include <MPU9250.h>
#include <Wire.h>

// ============ CONFIGURATION ============
// WiFi credentials - CHANGE THESE
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// TCP Server port
const int TCP_PORT = 5000;

// Sampling rate
const int SAMPLE_RATE_HZ = 100;
const int SAMPLE_INTERVAL_US = 1000000 / SAMPLE_RATE_HZ;

// Pin definitions
const int BUTTON_PIN = 4;
const int LED_PIN = 2;

// ============ GLOBAL VARIABLES ============
MPU9250 mpu;
WiFiServer server(TCP_PORT);
WiFiClient client;

bool isStreaming = false;
bool lastButtonState = HIGH;
unsigned long lastSampleTime = 0;
unsigned long streamStartTime = 0;

// Calibration offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("ESP32 MPU9250 WiFi Streamer");
    Serial.println("Team Unwired - Formula Bharat 2026");
    Serial.println("========================================\n");
    
    // Initialize pins
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize I2C and MPU9250
    Wire.begin();
    if (!mpu.setup(0x68)) {
        Serial.println("ERROR: MPU9250 not found!");
        Serial.println("Check wiring: SDA->GPIO21, SCL->GPIO22");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    Serial.println("MPU9250 initialized successfully.");
    
    // Connect to WiFi
    Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nERROR: WiFi connection failed!");
        Serial.println("Check SSID and password, then reset.");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }
    
    digitalWrite(LED_PIN, LOW);
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("TCP Port: %d\n", TCP_PORT);
    
    // Start TCP server
    server.begin();
    Serial.println("\nTCP server started. Waiting for connection...");
    Serial.println("Press button to calibrate, then press again to start streaming.");
    
    // Perform initial calibration
    Serial.println("\n--- CALIBRATION ---");
    Serial.println("Place sensor on flat surface and press button...");
}

void calibrate() {
    Serial.println("Calibrating... keep sensor still for 5 seconds.");
    
    // Blink LED during calibration
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;
    
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        if (mpu.update()) {
            ax_sum += mpu.getAccX();
            ay_sum += mpu.getAccY();
            az_sum += mpu.getAccZ();
            gx_sum += mpu.getGyroX();
            gy_sum += mpu.getGyroY();
            gz_sum += mpu.getGyroZ();
            samples++;
        }
        
        if ((millis() / 200) % 2 == 0) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
        delay(10);
    }
    
    // Calculate offsets (assuming sensor is level, Z should read 1g)
    ax_offset = ax_sum / samples;
    ay_offset = ay_sum / samples;
    az_offset = (az_sum / samples) - 1.0;  // Subtract 1g for gravity
    gx_offset = gx_sum / samples;
    gy_offset = gy_sum / samples;
    gz_offset = gz_sum / samples;
    
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("Calibration complete!");
    Serial.printf("Offsets: ax=%.3f, ay=%.3f, az=%.3f\n", ax_offset, ay_offset, az_offset);
    Serial.printf("         gx=%.3f, gy=%.3f, gz=%.3f\n", gx_offset, gy_offset, gz_offset);
    Serial.println("\nPress button to start streaming.");
}

void loop() {
    // Check for new client connections
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("Client connected!");
            client.println("# ESP32 MPU9250 Data Stream");
            client.println("# timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
        }
    }
    
    // Read button with debouncing
    bool buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && lastButtonState == HIGH) {
        delay(50);  // Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            if (!isStreaming) {
                // First press = calibrate, second press = start streaming
                static bool calibrated = false;
                if (!calibrated) {
                    calibrate();
                    calibrated = true;
                } else {
                    isStreaming = true;
                    streamStartTime = millis();
                    digitalWrite(LED_PIN, HIGH);
                    Serial.println("Streaming STARTED");
                    if (client && client.connected()) {
                        client.println("# STREAM_START");
                    }
                }
            } else {
                isStreaming = false;
                digitalWrite(LED_PIN, LOW);
                Serial.println("Streaming STOPPED");
                if (client && client.connected()) {
                    client.println("# STREAM_STOP");
                }
            }
        }
    }
    lastButtonState = buttonState;
    
    // Stream data at specified rate
    if (isStreaming && (micros() - lastSampleTime >= SAMPLE_INTERVAL_US)) {
        lastSampleTime = micros();
        
        if (mpu.update()) {
            // Get corrected readings
            float ax = mpu.getAccX() - ax_offset;
            float ay = mpu.getAccY() - ay_offset;
            float az = mpu.getAccZ() - az_offset;
            float gx = mpu.getGyroX() - gx_offset;
            float gy = mpu.getGyroY() - gy_offset;
            float gz = mpu.getGyroZ() - gz_offset;
            
            // Timestamp in milliseconds
            unsigned long timestamp = millis() - streamStartTime;
            
            // Format data string
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
                     timestamp, ax, ay, az, gx, gy, gz);
            
            // Send to connected client
            if (client && client.connected()) {
                client.println(buffer);
            }
            
            // Also print to serial for debugging
            Serial.println(buffer);
        }
    }
}
