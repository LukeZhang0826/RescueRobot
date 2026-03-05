# servo_test.py
# Choreographed 2-servo sequence using your calibrated "pseudo-angles".
#
# Pins (swapped wiring):
#   CLAW: PWM GPIO 1,  Power GPIO 0
#   LIFT: PWM GPIO 14, Power GPIO 15
#
# Your calibrated positions:
#   CLAW_OPEN   =  80
#   CLAW_CLOSED = -50
#   LIFT_DOWN   = -80
#   LIFT_UP     =  70
#
# Sequence (as requested):
#   1) Open claw
#   2) Lower lift
#   3) Close claw
#   4) Raise lift

import utime
from machine import Pin, PWM

# -----------------------------
# PINS
# -----------------------------
CLAW_PWM_PIN = 1
CLAW_PWR_PIN = 0

LIFT_PWM_PIN = 14
LIFT_PWR_PIN = 15

# -----------------------------
# SERVO SETTINGS
# -----------------------------
SERVO_HZ = 50
US_MIN = 900
US_MAX = 2100

# -----------------------------
# CALIBRATED "ANGLES"
# -----------------------------
CLAW_OPEN = 80
CLAW_CLOSED = -50

LIFT_DOWN = -80
LIFT_UP = 70

# -----------------------------
# TIMING (ms)
# -----------------------------
T_CLAW = 650
T_LIFT = 800
T_SETTLE = 250
T_HOLD_END = 2000

# -----------------------------
# Helpers
# -----------------------------
def us_to_duty_u16(us, hz=SERVO_HZ):
    period_us = int(1_000_000 / hz)  # 20000 us at 50Hz
    return int((us / period_us) * 65535)

def angle_to_us(deg, us_min=US_MIN, us_max=US_MAX):
    # no clamp: allow negative / >180 (your calibration uses negatives)
    return us_min + (float(deg) / 180.0) * (us_max - us_min)

class Servo:
    def __init__(self, pwm_pin, pwr_pin=None):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(SERVO_HZ)
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
        us = angle_to_us(deg)
        self.pwm.duty_u16(us_to_duty_u16(us))

def sleep_ms(ms):
    utime.sleep_ms(ms)

# -----------------------------
# Sequence
# -----------------------------
def run_sequence(claw: Servo, lift: Servo):
    print("1) CLAW OPEN =", CLAW_OPEN)
    claw.set_deg(CLAW_OPEN)
    sleep_ms(T_CLAW)
    sleep_ms(T_SETTLE)

    print("2) LIFT DOWN =", LIFT_DOWN)
    lift.set_deg(LIFT_DOWN)
    sleep_ms(T_LIFT)
    sleep_ms(T_SETTLE)

    print("3) CLAW CLOSE =", CLAW_CLOSED)
    claw.set_deg(CLAW_CLOSED)
    sleep_ms(T_CLAW)
    sleep_ms(T_SETTLE)

    print("4) LIFT UP =", LIFT_UP)
    lift.set_deg(LIFT_UP)
    sleep_ms(T_LIFT)
    sleep_ms(T_SETTLE)

def main():
    claw = Servo(CLAW_PWM_PIN, CLAW_PWR_PIN)
    lift = Servo(LIFT_PWM_PIN, LIFT_PWR_PIN)

    print("Enabling servo power...")
    claw.power_on()
    lift.power_on()
    sleep_ms(250)

    try:
        print("Starting sequence: open -> down -> close -> up")
        run_sequence(claw, lift)
        print("Done. Holding for", T_HOLD_END, "ms")
        sleep_ms(T_HOLD_END)

    finally:
        # If you want holding torque after the test, comment these 4 lines out.
        print("Powering off servos.")
        claw.detach()
        lift.detach()
        claw.power_off()
        lift.power_off()

if __name__ == "__main__":
    main()