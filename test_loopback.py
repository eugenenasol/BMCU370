import serial
import time

PORT = 'COM4'
BAUD = 115200

print("=== USB-RS485 Loopback Test ===")
print("Connect A to B on the RS485 adapter (short them together).")
print("Disconnect everything else from the adapter.\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    
    test_data = b'HELLO_RS485_LOOPBACK\r\n'
    print(f"Sending: {test_data.strip()}")
    ser.write(test_data)
    ser.flush()
    time.sleep(0.2)
    
    resp = ser.read(256)
    if resp:
        print(f"Got back ({len(resp)} bytes): {resp}")
        print(f"Hex: {resp.hex(' ')}")
        if b'HELLO_RS485_LOOPBACK' in resp:
            print("\n>>> LOOPBACK OK! RS485 adapter sends and receives correctly. <<<")
        else:
            print("\n>>> Got data but corrupted. Adapter might have issues. <<<")
    else:
        print("\n>>> NOTHING received! RS485 adapter is NOT transmitting. <<<")
        print("    Check: is this really a USB-RS485 adapter (not USB-TTL)?")
        print("    Check: are A and B properly shorted?")
    
    ser.close()
except Exception as e:
    print(f"Error: {e}")
