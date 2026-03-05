# servoControl.py
from machine import Pin, PWM

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class Servo:
    """
    Standard hobby servo on 50 Hz.
    duty_u16 mapping based on pulse widths.

    Defaults:
      0 deg   ~0.5 ms
      180 deg ~2.5 ms
    """
    def __init__(self, pwm_pin, power_pin=None, freq=50,
                 us_min=500, us_max=2500):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(freq)

        self.power = Pin(power_pin, Pin.OUT) if power_pin is not None else None
        if self.power is not None:
            self.power.value(0)

        self.us_min = us_min
        self.us_max = us_max
        self.freq = freq

    def power_on(self):
        if self.power is not None:
            self.power.value(1)

    def power_off(self):
        if self.power is not None:
            self.power.value(0)

    def detach(self):
        # stop sending pulses
        self.pwm.duty_u16(0)

    def angle(self, deg):
        deg = clamp(deg, 0, 180)

        # pulse width in microseconds
        us = self.us_min + (deg / 180.0) * (self.us_max - self.us_min)

        # Convert pulse width to duty_u16:
        # period_us = 1/freq seconds = 20ms = 20000 us at 50Hz
        period_us = int(1_000_000 / self.freq)
        duty = int((us / period_us) * 65535)

        self.pwm.duty_u16(duty)