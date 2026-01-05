"""
TCP Data Receiver for ESP32 MPU9250 WiFi Streamer
Team Unwired - Formula Bharat 2026

Receives IMU data from ESP32 over TCP and saves to CSV file.
Run this on laptop connected to same WiFi as ESP32.

Usage:
    python receive_imu_data.py 192.168.1.100 5000 output.csv

Requirements:
    pip install pandas
"""

import socket
import sys
import csv
import time
from datetime import datetime

def receive_data(esp32_ip, port, output_file):
    """Connect to ESP32 and receive IMU data stream."""
    
    print(f"Connecting to ESP32 at {esp32_ip}:{port}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30)  # 30 second timeout for connection
    
    try:
        sock.connect((esp32_ip, port))
        print("Connected!")
        print(f"Saving data to: {output_file}")
        print("Press Ctrl+C to stop recording.\n")
        
        # Open CSV file for writing
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_ms', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'])
            
            sample_count = 0
            start_time = time.time()
            
            buffer = ""
            while True:
                try:
                    data = sock.recv(1024).decode('utf-8')
                    if not data:
                        print("Connection closed by ESP32")
                        break
                    
                    buffer += data
                    lines = buffer.split('\n')
                    buffer = lines[-1]  # Keep incomplete line in buffer
                    
                    for line in lines[:-1]:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            if 'STREAM_START' in line:
                                print("--- Stream started ---")
                            elif 'STREAM_STOP' in line:
                                print("--- Stream stopped ---")
                            continue
                        
                        # Parse CSV line
                        try:
                            values = line.split(',')
                            if len(values) == 7:
                                writer.writerow(values)
                                sample_count += 1
                                
                                # Print status every 100 samples
                                if sample_count % 100 == 0:
                                    elapsed = time.time() - start_time
                                    rate = sample_count / elapsed if elapsed > 0 else 0
                                    print(f"Samples: {sample_count}, Rate: {rate:.1f} Hz, "
                                          f"ax={float(values[1]):.2f}g, ay={float(values[2]):.2f}g")
                        except ValueError:
                            pass  # Skip malformed lines
                            
                except socket.timeout:
                    print("Waiting for data...")
                    continue
                    
    except KeyboardInterrupt:
        print(f"\n\nRecording stopped.")
        print(f"Total samples: {sample_count}")
        print(f"Duration: {time.time() - start_time:.1f} seconds")
        print(f"Average rate: {sample_count / (time.time() - start_time):.1f} Hz")
        
    except socket.error as e:
        print(f"Connection error: {e}")
        
    finally:
        sock.close()
        print(f"\nData saved to: {output_file}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python receive_imu_data.py <ESP32_IP> <PORT> [output_file]")
        print("Example: python receive_imu_data.py 192.168.1.100 5000 run001.csv")
        sys.exit(1)
    
    esp32_ip = sys.argv[1]
    port = int(sys.argv[2])
    
    if len(sys.argv) >= 4:
        output_file = sys.argv[3]
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f"imu_data_{timestamp}.csv"
    
    receive_data(esp32_ip, port, output_file)


if __name__ == "__main__":
    main()
