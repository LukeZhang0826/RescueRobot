from machine import Pin, PWM


class Motors:
    """
    DRV8833 dual-motor driver.

    Supports signed speed:
        -100 .. +100

    positive = forward
    negative = reverse
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

    def _drive_motor(self, pwmA, pwmB, percent):
        percent = max(-100.0, min(100.0, percent))
        raw = int(abs(percent) / 100.0 * self.pwm_max_raw)

        if percent > 0:
            # forward
            pwmA.duty_u16(raw)
            pwmB.duty_u16(0)

        elif percent < 0:
            # reverse
            pwmA.duty_u16(0)
            pwmB.duty_u16(raw)

        else:
            # coast for zero command
            pwmA.duty_u16(0)
            pwmB.duty_u16(0)

    def drive_percent(self, left_percent, right_percent):
        self.en.value(1)
        self._drive_motor(self.l1, self.l2, left_percent)
        self._drive_motor(self.r1, self.r2, right_percent)

    def coast(self):
        self.en.value(1)
        self.l1.duty_u16(0)
        self.l2.duty_u16(0)
        self.r1.duty_u16(0)
        self.r2.duty_u16(0)

    def brake(self):
        self.en.value(1)
        self.l1.duty_u16(self.pwm_max_raw)
        self.l2.duty_u16(self.pwm_max_raw)
        self.r1.duty_u16(self.pwm_max_raw)
        self.r2.duty_u16(self.pwm_max_raw)

    def brake_ms(self, ms=100):
        self.brake()
        import utime
        utime.sleep_ms(ms)
        self.coast()

    def stop(self):
        self.brake()