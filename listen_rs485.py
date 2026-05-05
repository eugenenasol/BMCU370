import serial
import time

PORT = 'COM4'
BAUD = 115200

print(f"Listening on {PORT} for 10 seconds... Press RESET on the board!")
try:
    ser = serial.Serial(PORT, BAUD, timeout=10)
    data = ser.read(1024)
    if data:
        print(f"Received {len(data)} bytes:")
        print(f"Hex: {data.hex(' ')}")
        print(f"Text: {data.decode('utf-8', errors='replace')}")
    else:
        print("Nothing received.")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
