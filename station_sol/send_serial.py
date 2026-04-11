#!/usr/bin/env python3
"""
Serial Command Sender for ESP32
Usage: python send_serial.py <port> <command>
Example: python send_serial.py /dev/ttyUSB0 "start"
"""

import sys
import serial
import time

if len(sys.argv) < 3:
    print("Usage: python send_serial.py <port> <command>")
    sys.exit(1)

port = sys.argv[1]
command = sys.argv[2]

try:
    ser = serial.Serial(port, 115200, timeout=1)
    ser.write((command + '\n').encode())
    print(f"Sent: {command}")
    time.sleep(0.1)  # Wait a bit
    response = ser.read(ser.in_waiting).decode()
    if response:
        print(f"Response: {response}")
    ser.close()
except Exception as e:
    print(f"Error: {e}")