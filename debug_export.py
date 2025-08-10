#!/usr/bin/env python3
"""
Debug script to test radar export commands
"""

import serial
import time

def test_export_commands(port="/dev/ttyUSB0", baudrate=115200):
    """Test radar export commands"""
    print(f"Testing export commands on {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("✓ Serial port opened")
        
        # First take a measurement
        print("\n1. Taking measurement...")
        ser.write("radar measure\n".encode())
        time.sleep(3)  # Give time for measurement
        
        # Read measurement response
        response = []
        start_time = time.time()
        while (time.time() - start_time) < 5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response.append(line)
                    print(f"MEASURE: {line}")
                    if "uart:~$" in line:
                        break
            else:
                time.sleep(0.1)
        
        # Test raw export
        print("\n2. Testing raw export...")
        ser.write("radar export raw\n".encode())
        time.sleep(2)
        
        raw_lines = []
        start_time = time.time()
        while (time.time() - start_time) < 10:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    raw_lines.append(line)
                    print(f"RAW: {line}")
                    if "RAW_END" in line or "uart:~$" in line:
                        break
            else:
                time.sleep(0.1)
        
        print(f"\nRaw export returned {len(raw_lines)} lines")
        
        # Test FFT export (send extra newline to clear any issues)
        print("\n3. Testing FFT export...")
        ser.write("\n".encode())  # Clear any partial commands
        time.sleep(0.5)
        ser.write("radar export fft\n".encode())
        time.sleep(3)
        
        fft_lines = []
        start_time = time.time()
        while (time.time() - start_time) < 10:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    fft_lines.append(line)
                    print(f"FFT: {line}")
                    if "FFT_END" in line or "uart:~$" in line:
                        break
            else:
                time.sleep(0.1)
        
        print(f"\nFFT export returned {len(fft_lines)} lines")
        
        ser.close()
        print("\n✓ Test completed")
        
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    test_export_commands()