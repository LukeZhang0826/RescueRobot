from machine import Pin, PWM

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
    """Speed: -65535 to 65535. Positive=forward, Negative=reverse, 0=stop."""
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


def set_servo_angle(servo_index, angle, power_on=True):
    """Angle: 0-180 degrees. power_on: True to enable servo power, False to cut it."""
    servo = servo_motors[servo_index]
    servo[1].value(1 if power_on else 0)
    if power_on:
        duty = int(1638 + (angle / 180) * (8192 - 1638))
        servo[0].duty_u16(duty)


def stop_all():
    for i in range(len(dc_motors)):
        set_dc_speed(i, 0)
    for i in range(len(servo_motors)):
        set_servo_angle(i, 90, power_on=False)