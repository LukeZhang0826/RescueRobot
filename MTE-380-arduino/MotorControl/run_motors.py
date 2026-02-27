from machine import Pin, PWM
import time

# --- DC Motors (dual-pin H-bridge) [pin_a, pin_b] ---
dc_motors = [
    [PWM(Pin(4)), PWM(Pin(5))],  # Motor A
    [PWM(Pin(6)), PWM(Pin(7))],  # Motor B
]

for motor in dc_motors:
    motor[0].freq(1000)
    motor[1].freq(1000)

# --- Servo Motors [pwm_pin, enable_pin] ---
servo_motors = [
    [PWM(Pin(10)), Pin(11, Pin.OUT)],  # Servo 1
    [PWM(Pin(12)), Pin(13, Pin.OUT)],  # Servo 2
]

for servo in servo_motors:
    servo[0].freq(50)


def set_dc_speed(motor_index, speed):
    motor = dc_motors[motor_index]
    if speed > 0:
        motor[0].duty_u16(speed)
        motor[1].duty_u16(0)
    elif speed < 0:
        motor[0].duty_u16(0)
        motor[1].duty_u16(abs(speed))
    else:
        motor[0].duty_u16(0)
        motor[1].duty_u16(0)


def set_servo_angle(servo, angle, power_on=True):
    """Angle: 0-180 degrees. power_on: True to enable servo power, False to cut it."""
    servo[1].value(1 if power_on else 0)
    if power_on:
        duty = int(1638 + (angle / 180) * (8192 - 1638))
        servo[0].duty_u16(duty)


def stop_all():
    for motor in dc_motors:
        set_dc_speed(motor, 0)
    for servo in servo_motors:
        set_servo_angle(servo, 90, power_on=False)


# --- Main Test Loop ---
try:
    while True:
        print("DC Motors Forward...")
        for motor in dc_motors:
            set_dc_speed(motor, 20000)
        time.sleep(2)

        print("DC Motors Reverse...")
        for motor in dc_motors:
            set_dc_speed(motor, -20000)
        time.sleep(2)

        print("DC Motors Stop...")
        for motor in dc_motors:
            set_dc_speed(motor, 0)
        time.sleep(1)

        print("Servos On - Sweep 0 to 180...")
        for angle in range(0, 181, 10):
            set_servo_angle(servo_motors[0], angle, power_on=True)
            set_servo_angle(servo_motors[1], 180 - angle, power_on=True)
            time.sleep(0.05)

        print("Servos Off...")
        for servo in servo_motors:
            set_servo_angle(servo, 90, power_on=False)
        time.sleep(1)

except KeyboardInterrupt:
    stop_all()
    print("All motors stopped.")