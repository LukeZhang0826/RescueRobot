from machine import Pin, PWM

# DC Motor Config 
dc_enable = Pin(5, Pin.OUT)
dc_motors = [
    [PWM(Pin(6)), PWM(Pin(7))],  # Motor 0
    [PWM(Pin(8)), PWM(Pin(9))],  # Motor 1 
]

for m in dc_motors:
    for pin in m:
        pin.freq(1000)

# Servo Config [PWM_PIN, POWER_PIN] 
servos = [
    {"pwm": PWM(Pin(1)), "pwr": Pin(0, Pin.OUT)},  # Servo 0
    {"pwm": PWM(Pin(14)), "pwr": Pin(15, Pin.OUT)}, # Servo 1
]

for s in servos:
    s["pwm"].freq(50)
    s["pwr"].value(0) # Start with power OFF

def set_dc_speed(index, speed):
    # Sets speed for a DC motor. speed: -65535 to 65535.
    dc_enable.value(1) # Ensure bridge is active
    motor = dc_motors[index]
    
    if speed > 0:
        motor[0].duty_u16(speed)
        motor[1].duty_u16(0)
    elif speed < 0:
        motor[0].duty_u16(0)
        motor[1].duty_u16(abs(speed))
    else:
        motor[0].duty_u16(0)
        motor[1].duty_u16(0)

def set_servo_angle(index, angle):
    """Powers on specific servo, moves it, and kills power to others."""
    # Kill power to all servos first (One-at-a-time rule)
    for s in servos:
        s["pwr"].value(0)
    
    # Enable power for the target servo
    target = servos[index]
    target["pwr"].value(1)
    
    # Calculate duty (0.5ms to 2.5ms) and move
    angle = max(0, min(180, angle))
    duty = int((angle / 180) * (8192 - 1638) + 1638)
    target["pwm"].duty_u16(duty)

def stop_all():
    dc_enable.value(0)
    for m in dc_motors:
        m[0].duty_u16(0)
        m[1].duty_u16(0)
    for s in servos:
        s["pwr"].value(0)
        s["pwm"].duty_u16(0)