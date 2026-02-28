from machine import Pin, PWM
import time

# Servo A is on GPIO 1, 14 for B
servo = PWM(Pin(14))
servo.freq(50) 

# Power Enable Pin for Servo A is GPIO 0
# We must set this to High to "turn on" the transistor/power
servo_power = Pin(15, Pin.OUT)  # 15 FOR B
servo_power.value(1) 

def set_angle(angle):
    """
    Translates an angle (0-180) to the duty cycle (u16)
    0 degrees   -> ~1638 (0.5ms)
    180 degrees -> ~8192 (2.5ms)
    """
    # Clamp angle between 0 and 180 to prevent out-of-range errors
    angle = max(0, min(180, angle))
    
    # Duty calculation for 50Hz
    duty = int((angle / 180) * (8192 - 1638) + 1638)
    servo.duty_u16(duty)

# --- Test Logic ---
try:
    print("Servo Power Enabled (GPIO 0 High)")
    
    print("Moving to 0 degrees...")
    set_angle(0)
    time.sleep(1)

    print("Moving to 90 degrees...")
    set_angle(90)
    time.sleep(1)

    print("Moving to 180 degrees...")
    set_angle(180)
    time.sleep(1)

    print("Test complete. Returning to center.")
    set_angle(90)
    time.sleep(1)

except KeyboardInterrupt:
    # Cleanup
    servo.duty_u16(0) # Stop sending PWM pulses
    servo_power.value(0) # Cut power to the servo
    print("Servo stopped and power disabled.")