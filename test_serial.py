#!/usr/bin/env python3
"""
Simple serial test for BGT60 radar board
Tests basic serial communication without GUI
"""

import serial
import time
import sys

def test_serial_connection(port="/dev/ttyUSB0", baudrate=921600):
    """Test basic serial communication"""
    print(f"Testing serial connection to {port} at {baudrate} baud...")
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Serial port opened successfully")
        
        # Send a simple command
        command = "help\n"
        print(f"Sending command: {command.strip()}")
        ser.write(command.encode())
        
        # Read response
        print("Reading response...")
        time.sleep(0.5)  # Give time for response
        
        response_lines = []
        start_time = time.time()
        
        while (time.time() - start_time) < 3:  # 3 second timeout
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response_lines.append(line)
                    print(f"  < {line}")
                    if "uart:~$" in line or line.endswith("$"):
                        break
            else:
                time.sleep(0.1)
        
        if response_lines:
            print(f"✓ Received {len(response_lines)} lines of response")
        else:
            print("⚠ No response received")
            
        # Test radar commands
        print("\nTesting radar commands...")
        commands = ["radar status", "radar init"]
        
        for cmd in commands:
            print(f"\nSending: {cmd}")
            ser.write(f"{cmd}\n".encode())
            time.sleep(0.5)
            
            response_lines = []
            start_time = time.time()
            
            while (time.time() - start_time) < 3:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response_lines.append(line)
                        print(f"  < {line}")
                        if "uart:~$" in line or line.endswith("$"):
                            break
                else:
                    time.sleep(0.1)
            
            if not response_lines:
                print("  ⚠ No response")
        
        ser.close()
        print("\n✓ Serial test completed")
        
    except serial.SerialException as e:
        print(f"✗ Serial error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    # Test with different common ports
    ports_to_try = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]
    
    if len(sys.argv) > 1:
        ports_to_try = [sys.argv[1]]
    
    for port in ports_to_try:
        print(f"\n{'='*50}")
        print(f"Testing port: {port}")
        print('='*50)
        
        try:
            if test_serial_connection(port):
                print(f"✓ Success with port {port}")
                break
        except Exception as e:
            print(f"✗ Failed with port {port}: {e}")
    else:
        print("\n✗ All ports failed. Check:")
        print("  1. Is the radar board connected?")
        print("  2. Is the correct driver installed?")
        print("  3. Do you have permission to access serial ports?")
        print("  4. Try: sudo usermod -a -G dialout $USER")