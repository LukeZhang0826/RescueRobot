# motors.py
from machine import Pin, PWM

class Motors:
    """
    DRV8833 dual-motor driver.
    Current implementation is "forward only" per wheel (same as your original test).
    Later you can extend with signed percent for forward/backward.
    """
    def __init__(self, en_pin, left_pins, right_pins, pwm_freq=1000, pwm_max_raw=65535):
        self.pwm_max_raw = pwm_max_raw

        self.en = Pin(en_pin, Pin.OUT)
        self.l1 = PWM(Pin(left_pins[0]))
        self.l2 = PWM(Pin(left_pins[1]))
        self.r1 = PWM(Pin(right_pins[0]))
        self.r2 = PWM(Pin(right_pins[1]))

        for p in (self.l1, self.l2, self.r1, self.r2):
            p.freq(pwm_freq)
            p.duty_u16(0)

        self.en.value(0)

    def drive_percent(self, left_percent, right_percent):
        # clamp in caller; assume 0..100
        left_raw = int((left_percent / 100.0) * self.pwm_max_raw)
        right_raw = int((right_percent / 100.0) * self.pwm_max_raw)

        self.en.value(1)

        # Left forward
        self.l1.duty_u16(min(left_raw, self.pwm_max_raw))
        self.l2.duty_u16(0)

        # Right forward
        self.r1.duty_u16(min(right_raw, self.pwm_max_raw))
        self.r2.duty_u16(0)

    def stop(self):
        self.en.value(0)
        self.l1.duty_u16(0)
        self.l2.duty_u16(0)
        self.r1.duty_u16(0)
        self.r2.duty_u16(0)