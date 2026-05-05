import serial
import time

PORT = 'COM4'
BAUD = 115200

print(f"=== BMCU RS485 Deep Debug on {PORT} ===")

try:
    # Use a longer timeout for the startup phase
    ser = serial.Serial(PORT, BAUD, timeout=10)
    
    print("Waiting for STARTUP... (Press Reset now!)")
    data = ser.read_until(b'\n')
    if data:
        print(f"MCU ALIVE: {data.decode('utf-8', errors='replace').strip()}")
        
        # Give MCU some time to settle
        time.sleep(1.0)
        ser.reset_input_buffer()
        
        test_data = b"HELLO_MCU_CAN_YOU_HEAR_ME\r\n"
        print(f"Sending: {test_data.decode().strip()}")
        ser.write(test_data)
        ser.flush()
        
        # Wait and see if ANYTHING comes back in the next 5 seconds
        ser.timeout = 5
        print("Listening for echo...")
        resp = ser.read(1024)
        if resp:
            print(f"RECEIVED SOMETHING! ({len(resp)} bytes)")
            print(f"Hex: {resp.hex(' ')}")
            print(f"Text: {resp.decode('utf-8', errors='replace')}")
        else:
            print("STILL NOTHING. The board is deaf to PC commands.")
            
    else:
        print("Timed out waiting for STARTUP.")
        
    ser.close()
except Exception as e:
    print(f"Error: {e}")
