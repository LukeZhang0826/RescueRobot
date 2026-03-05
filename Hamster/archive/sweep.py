# sweep.py
# Sweep PWM duty_u16 values to find the minimum that makes the robot *actually move*,
# using encoder counts as the detector.
#
# Wiring (edit if needed):
# - DRV8833: EN=5, Left IN1/IN2 = 6/7, Right IN1/IN2 = 8/9
# - Encoders:
#     Left:  A=2,  B=3
#     Right: A=12, B=13
#
# How it works:
# - For each PWM in a range, drive both wheels forward for a short burst.
# - Measure encoder delta counts during that burst.
# - If counts exceed a threshold for N consecutive tries, we say "moving".
#
# Tips:
# - Put the robot on the ground (not lifted) if you want real stiction threshold.
# - If encoders are not wired/working, this script will never detect movement.

import utime
from machine import Pin, PWM

# -----------------------------
# Config
# -----------------------------
EN_PIN = 5
LEFT_PINS = (6, 7)   # IN1, IN2
RIGHT_PINS = (8, 9)  # IN1, IN2

PWM_FREQ = 1000

# Sweep range (duty_u16 is 0..65535)
PWM_START = 0
PWM_STOP = 30000
PWM_STEP = 500

# Test burst timing
BURST_MS = 250       # time driving at a candidate PWM
SETTLE_MS = 150      # time stopped between tests

# Movement detection thresholds
# Total encoder counts (abs) across both wheels during the burst.
MOVE_COUNTS_TOTAL = 8
# Require this many consecutive hits to confirm (reduces false positives).
CONFIRM_CONSECUTIVE = 2

# Encoder inversion (set to -1 if direction is backwards; doesn't matter since we use abs)
ENC_INV_A = 1   # left
ENC_INV_B = 1   # right

# -----------------------------
# Motor driver (sign-magnitude PWM)
# -----------------------------
def _u16(x):
    if x < 0:
        return 0
    if x > 65535:
        return 65535
    return int(x)

class Motors:
    def __init__(self, en=EN_PIN, left=LEFT_PINS, right=RIGHT_PINS, pwm_freq=PWM_FREQ):
        self.en = Pin(en, Pin.OUT)
        self.l1 = PWM(Pin(left[0])); self.l2 = PWM(Pin(left[1]))
        self.r1 = PWM(Pin(right[0])); self.r2 = PWM(Pin(right[1]))
        for p in (self.l1, self.l2, self.r1, self.r2):
            p.freq(pwm_freq)
            p.duty_u16(0)
        self.en.value(0)

    def _set(self, in1, in2, pwm_signed):
        if pwm_signed > 0:
            in1.duty_u16(_u16(pwm_signed)); in2.duty_u16(0)
        elif pwm_signed < 0:
            in1.duty_u16(0); in2.duty_u16(_u16(-pwm_signed))
        else:
            in1.duty_u16(0); in2.duty_u16(0)

    def drive(self, left_pwm, right_pwm):
        if left_pwm == 0 and right_pwm == 0:
            self.stop()
            return
        self.en.value(1)
        self._set(self.l1, self.l2, left_pwm)
        self._set(self.r1, self.r2, right_pwm)

    def stop(self):
        self.en.value(0)
        self._set(self.l1, self.l2, 0)
        self._set(self.r1, self.r2, 0)

# -----------------------------
# Encoders (x4 quadrature ISR)
# -----------------------------
QDEC = (
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
)

EA_A = Pin(2, Pin.IN, Pin.PULL_UP)
EA_B = Pin(3, Pin.IN, Pin.PULL_UP)
EB_A = Pin(12, Pin.IN, Pin.PULL_UP)
EB_B = Pin(13, Pin.IN, Pin.PULL_UP)

encA = 0
encB = 0
oldA = (EA_A.value() << 1) | EA_B.value()
oldB = (EB_A.value() << 1) | EB_B.value()

def irqA(pin):
    global encA, oldA
    new = (EA_A.value() << 1) | EA_B.value()
    step = QDEC[(oldA << 2) | new]
    encA += ENC_INV_A * step
    oldA = new

def irqB(pin):
    global encB, oldB
    new = (EB_A.value() << 1) | EB_B.value()
    step = QDEC[(oldB << 2) | new]
    encB += ENC_INV_B * step
    oldB = new

EA_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqA)
EA_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqA)
EB_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqB)
EB_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqB)

# -----------------------------
# Sweep
# -----------------------------
motors = Motors()

print("\nPWM sweep: finding minimum PWM where robot starts to move (encoder-based).")
print("Range:", PWM_START, "to", PWM_STOP, "step", PWM_STEP)
print("Burst:", BURST_MS, "ms | Detect threshold total counts >=", MOVE_COUNTS_TOTAL,
      "| Confirm consecutive =", CONFIRM_CONSECUTIVE)
print("If encA/encB never change, your encoder wiring/power/pins are wrong.\n")

consec = 0
best_pwm = None

# Make sure stopped
motors.stop()
utime.sleep_ms(300)

baseA = encA
baseB = encB
utime.sleep_ms(50)

for pwm in range(PWM_START, PWM_STOP + 1, PWM_STEP):
    # Snapshot before burst
    a0 = encA
    b0 = encB

    # Drive forward
    motors.drive(pwm, pwm)
    utime.sleep_ms(BURST_MS)
    motors.stop()

    # Read deltas
    a1 = encA
    b1 = encB
    dA = a1 - a0
    dB = b1 - b0
    total = abs(dA) + abs(dB)

    moving = total >= MOVE_COUNTS_TOTAL

    if moving:
        consec += 1
    else:
        consec = 0

    print("pwm=%5d | dA=%4d dB=%4d | total=%4d | %s (%d/%d)"
          % (pwm, dA, dB, total, "MOVING" if moving else "----", consec, CONFIRM_CONSECUTIVE))

    utime.sleep_ms(SETTLE_MS)

    if consec >= CONFIRM_CONSECUTIVE:
        best_pwm = pwm - (CONFIRM_CONSECUTIVE - 1) * PWM_STEP
        break

print("\nDone.")
if best_pwm is None:
    print("No movement detected in range. Either:")
    print("- PWM_STOP too low, or")
    print("- robot is stuck / wheels slipping, or")
    print("- encoders not working (encA/encB never change).")
else:
    print("Estimated minimum move PWM duty_u16 =", best_pwm)

# Ensure stopped at end
motors.stop()