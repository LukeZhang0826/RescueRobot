# servos.py
import utime
from machine import Pin, PWM


def us_to_duty_u16(us, hz=50):
    period_us = int(1_000_000 / hz)  # 20000 us at 50 Hz
    return int((us / period_us) * 65535)


def angle_to_us(deg, us_min=900, us_max=2100):
    # no clamp: allows your calibrated negative / >180 pseudo-angles
    return us_min + (float(deg) / 180.0) * (us_max - us_min)


class Servo:
    def __init__(self, pwm_pin, pwr_pin=None, hz=50, us_min=900, us_max=2100):
        self.hz = hz
        self.us_min = us_min
        self.us_max = us_max

        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(self.hz)

        self.pwr = Pin(pwr_pin, Pin.OUT) if pwr_pin is not None else None
        if self.pwr is not None:
            self.pwr.value(0)

        self.detach()

    def power_on(self):
        if self.pwr is not None:
            self.pwr.value(1)

    def power_off(self):
        if self.pwr is not None:
            self.pwr.value(0)

    def detach(self):
        self.pwm.duty_u16(0)

    def set_deg(self, deg):
        us = angle_to_us(deg, self.us_min, self.us_max)
        self.pwm.duty_u16(us_to_duty_u16(us, self.hz))

    def sleep_ms(self, ms):
        utime.sleep_ms(ms)