#!/usr/bin/env python3
import serial
import time
from datetime import datetime
import sys

PORT = "COM5" if len(sys.argv) < 2 else sys.argv[1]
BAUD_RATE = 115200
WAIT_TIME = 8

def sync_time():
    now = datetime.now()
    time_str = now.strftime("%H:%M:%S")
    date_str = now.strftime("%y:%m:%d")
    
    print(f"\n=== ArsLed V606 Manual Time Sync ===")
    print(f"Port: {PORT}")
    print(f"Time: {time_str}")
    print(f"Date: {date_str}")
    print(f"\nWaiting {WAIT_TIME} seconds for ESP32 stability...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=2)
        time.sleep(WAIT_TIME)
        
        print("\n[1/2] Sending SETTIME command...")
        ser.write(f"SETTIME={time_str}\n".encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            print(f"Response: {ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()}")
        
        print("\n[2/2] Sending SETDATE command...")
        ser.write(f"SETDATE={date_str}\n".encode('utf-8'))
        ser.flush()
        time.sleep(1)
        
        if ser.in_waiting:
            print(f"Response: {ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()}")
        
        ser.close()
        print("\n✓ Time synchronization completed!")
        
    except serial.SerialException as e:
        print(f"\n✗ ERROR: {e}")
        sys.exit(1)

if __name__ == "__main__":
    sync_time()
