from machine import Pin, PWM
import time

# Setup PWM on GPIO 4 and 5
# Frequency of 1000Hz is standard for most small DC motors
motor_a1 = PWM(Pin(4))
motor_a2 = PWM(Pin(5))
motor_a1.freq(1000)
motor_a2.freq(1000)

def set_speed(speed):
    """
    Speed should be between -65535 and 65535.
    Positive = Forward, Negative = Reverse, 0 = Stop.
    """
    if speed > 0:
        motor_a1.duty_u16(speed)
        motor_a2.duty_u16(0)
    elif speed < 0:
        motor_a1.duty_u16(0)
        motor_a2.duty_u16(abs(speed))
    else:
        motor_a1.duty_u16(0)
        motor_a2.duty_u16(0)

# --- Main Test Loop ---
try:
    while True:
        print("Spinning Forward...")
        set_speed(20000) # Roughly 60% speed
        time.sleep(2)

        print("Stopping...")
        set_speed(0)
        time.sleep(1)

        print("Spinning Reverse...")
        set_speed(-20000) # Roughly 60% speed in reverse
        time.sleep(2)

        print("Stopping...")
        set_speed(0)
        time.sleep(1)

except KeyboardInterrupt:
    # Safely stop the motor if we stop the script in Thonny
    set_speed(0)
    print("Motor Stopped.")
