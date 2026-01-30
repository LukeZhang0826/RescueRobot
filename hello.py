import serial
import time

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
time.sleep(2)  # wait for Arduino reset

print("Type a number (ms) to change blink rate. Ctrl+C to exit.")

try:
    while True:
        val = input("> ").strip()
        if not val:
            continue

        # basic validation
        if not val.isdigit():
            print("Enter a number in milliseconds")
            continue

        ser.write((val + "\n").encode())
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
