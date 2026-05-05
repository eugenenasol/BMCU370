import serial
import time

PORT = 'COM4'
BAUD = 115200

def send_cmd(ser, json_cmd):
    # Try sending raw JSON first, then try with a single space
    data = json_cmd.encode() + b'\r\n'
    ser.write(data)
    ser.flush()
    return ser.readline()

try:
    print(f"=== BMCU RS485 RAW Test on {PORT} ===")
    print(">>> Press RESET within 60 seconds! <<<\n")
    
    ser = serial.Serial(PORT, BAUD, timeout=60)
    
    startup = ser.readline()
    if startup:
        print(f"STARTUP: {startup.decode('utf-8', errors='replace').strip()}")
    else:
        print("No STARTUP."); ser.close(); exit()
    
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.timeout = 1
    
    tests = [
        ('PING',        '{"id":1,"cmd":"PING"}'),
        ('STATUS',      '{"id":2,"cmd":"STATUS"}'),
    ]
    
    for name, cmd in tests:
        print(f"\n--- {name} ---")
        print(f"Sending: {cmd}")
        resp = send_cmd(ser, cmd)
        if resp:
            text = resp.decode('utf-8', errors='replace').strip()
            print(f"  Got: {text}")
        else:
            print("  No response!")
            # Last ditch effort: send with extra newlines
            print("  Trying with extra newlines...")
            ser.write(b'\n\n' + cmd.encode() + b'\n\n')
            ser.flush()
            resp = ser.readline()
            if resp:
                 print(f"  Got after extra: {resp.decode('utf-8', errors='replace').strip()}")
            else:
                 print("  Still nothing.")
    
    ser.close()
except Exception as e:
    print(f"Error: {e}")
