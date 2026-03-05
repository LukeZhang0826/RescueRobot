# steeringControl.py
from machine import Pin, PWM

def _u16(x):
    if x < 0:
        return 0
    if x > 65535:
        return 65535
    return int(x)

class SteeringControl:
    """
    Low-level motor driver for DRV8833 (Pololu carrier).
    Uses sign-magnitude PWM:
      +pwm => IN1 = PWM(pwm), IN2 = 0
      -pwm => IN1 = 0,        IN2 = PWM(|pwm|)
       0   => coast (both 0)
    """
    def __init__(
        self,
        enable_pin=5,
        left_pins=(6, 7),     # (IN1, IN2)
        right_pins=(8, 9),    # (IN1, IN2)
        pwm_freq=1000,
        invert_left=False,
        invert_right=False
    ):
        self.enable = Pin(enable_pin, Pin.OUT)

        self.left_in1  = PWM(Pin(left_pins[0]))
        self.left_in2  = PWM(Pin(left_pins[1]))
        self.right_in1 = PWM(Pin(right_pins[0]))
        self.right_in2 = PWM(Pin(right_pins[1]))

        for p in (self.left_in1, self.left_in2, self.right_in1, self.right_in2):
            p.freq(pwm_freq)
            p.duty_u16(0)

        self.invert_left = invert_left
        self.invert_right = invert_right

        self.enable.value(0)

    def _set_motor(self, in1: PWM, in2: PWM, pwm_signed: int):
        if pwm_signed > 0:
            in1.duty_u16(_u16(pwm_signed))
            in2.duty_u16(0)
        elif pwm_signed < 0:
            in1.duty_u16(0)
            in2.duty_u16(_u16(-pwm_signed))
        else:
            in1.duty_u16(0)
            in2.duty_u16(0)

    def drive(self, left_pwm: int, right_pwm: int):
        # allow signed inputs
        if self.invert_left:
            left_pwm = -left_pwm
        if self.invert_right:
            right_pwm = -right_pwm

        if left_pwm == 0 and right_pwm == 0:
            self.stop()
            return

        self.enable.value(1)
        self._set_motor(self.left_in1, self.left_in2, left_pwm)
        self._set_motor(self.right_in1, self.right_in2, right_pwm)

    def stop(self):
        # Coast + disable
        self.enable.value(0)
        self._set_motor(self.left_in1, self.left_in2, 0)
        self._set_motor(self.right_in1, self.right_in2, 0)