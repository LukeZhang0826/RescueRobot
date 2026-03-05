# main_pixy_line_closedloop_pid.py
# Pixy2 (SPI) line-follow + CLOSED-LOOP wheel speed PID (encoders) in ONE FILE.
#
# Key fixes:
# - Feedforward PWM (FF_PWM) to overcome stiction
# - NO "PWM_MIN_MOVE clamp" during steering (it was killing turns)
# - Instead: a short startup "kick" only when transitioning from stop -> move
# - Faster stop on "NO BLOCKS" (so it doesn't drift off tape)

import utime
from machine import Pin, PWM, SPI

# -----------------------------
# Pixy SPI (CCC blocks)
# -----------------------------
RESP_SYNC = b"\xAF\xC1"

def u16le(b0, b1):
    return b0 | (b1 << 8)

class PixySPI:
    def __init__(self, spi_id=0, cs_pin=21, sck=18, mosi=19, miso=20,
                 baudrate=500_000, read_len=256):
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.spi = SPI(
            spi_id, baudrate=baudrate,
            polarity=1, phase=1,
            bits=8, firstbit=SPI.MSB,
            sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso),
        )
        self.read_len = read_len

    def _req_with_checksum(self, ptype, payload):
        csum = sum(payload) & 0xFFFF
        return bytes([0xAE, 0xC1, ptype & 0xFF, len(payload) & 0xFF,
                      csum & 0xFF, (csum >> 8) & 0xFF]) + payload

    def _send_and_read(self, req):
        self.cs.value(0)
        utime.sleep_us(50)
        self.spi.write(req)
        buf = bytearray(self.read_len)
        self.spi.readinto(buf, 0x00)
        self.cs.value(1)
        return bytes(buf)

    def _parse_response(self, data):
        i = data.rfind(RESP_SYNC)
        if i < 0:
            return None, b""
        ptype = data[i + 2]
        length = data[i + 3]
        csum = u16le(data[i + 4], data[i + 5])
        payload = data[i + 6 : i + 6 + length]
        if (sum(payload) & 0xFFFF) != csum:
            return None, b""
        return ptype, payload

    def get_blocks(self, sigmap=0xFF, max_blocks=10):
        req = self._req_with_checksum(32, bytes([sigmap & 0xFF, max_blocks & 0xFF]))
        ptype, pl = self._parse_response(self._send_and_read(req))
        if ptype != 33:
            return []
        blocks = []
        stride = 14
        n = len(pl) - (len(pl) % stride)
        for off in range(0, n, stride):
            sig = u16le(pl[off + 0],  pl[off + 1])
            x   = u16le(pl[off + 2],  pl[off + 3])
            y   = u16le(pl[off + 4],  pl[off + 5])
            w   = u16le(pl[off + 6],  pl[off + 7])
            h   = u16le(pl[off + 8],  pl[off + 9])
            blocks.append({"sig": sig, "x": x, "y": y, "w": w, "h": h, "area": w*h})
        return blocks

    def best_block(self, area_min=2000, sigmap=0xFF):
        blocks = self.get_blocks(sigmap=sigmap, max_blocks=10)
        blocks = [b for b in blocks if b["area"] >= area_min]
        if not blocks:
            return None
        return max(blocks, key=lambda bb: bb["area"])


# -----------------------------
# Motor driver (DRV8833 sign-magnitude PWM)
# -----------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def _u16(x):
    if x < 0:
        return 0
    if x > 65535:
        return 65535
    return int(x)

class Motors:
    def __init__(self, en=5, left=(6, 7), right=(8, 9), pwm_freq=1000,
                 invert_left=False, invert_right=False):
        self.en = Pin(en, Pin.OUT)
        self.l1 = PWM(Pin(left[0])); self.l2 = PWM(Pin(left[1]))
        self.r1 = PWM(Pin(right[0])); self.r2 = PWM(Pin(right[1]))
        for p in (self.l1, self.l2, self.r1, self.r2):
            p.freq(pwm_freq)
            p.duty_u16(0)
        self.invL = invert_left
        self.invR = invert_right
        self.en.value(0)

    def _set(self, in1, in2, pwm_signed):
        if pwm_signed > 0:
            in1.duty_u16(_u16(pwm_signed)); in2.duty_u16(0)
        elif pwm_signed < 0:
            in1.duty_u16(0); in2.duty_u16(_u16(-pwm_signed))
        else:
            in1.duty_u16(0); in2.duty_u16(0)

    def drive(self, left_pwm, right_pwm):
        if self.invL:
            left_pwm = -left_pwm
        if self.invR:
            right_pwm = -right_pwm

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

# Keep these at -1 if that makes forward motion read as POSITIVE speed in your control loop.
ENC_INV_A = -1  # left
ENC_INV_B = -1  # right

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
# PID (speed error -> PWM delta)
# -----------------------------
class PID:
    def __init__(self, kp, ki, kd=0.0, i_limit=30000.0, out_limit=65535.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.out_limit = abs(out_limit)
        self.i = 0.0
        self.prev_e = 0.0

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0

    def update(self, e, dt):
        if dt <= 0:
            return 0.0
        self.i += e * dt
        self.i = clamp(self.i, -self.i_limit, self.i_limit)
        de = (e - self.prev_e) / dt
        self.prev_e = e
        u = (self.kp * e) + (self.ki * self.i) + (self.kd * de)
        u = clamp(u, -self.out_limit, self.out_limit)
        return u


# -----------------------------
# Tuning knobs
# -----------------------------
FRAME_W = 316
CENTER_X = FRAME_W // 2

AREA_MIN = 2000
DEADBAND_PX = 5

TICK_MS = 20
PRINT_MS = 200

PWM_MAX = 65535

# From your sweep
FF_PWM = 12500

# Startup kick (ONLY used when starting from stop; does not clamp steering)
START_KICK_PWM = 16000
START_KICK_MS = 120

# Slew limiting
PWM_SLEW_PER_SEC = 60000

# Speed targets
BASE_SPEED_CPS = 250.0
TURN_GAIN_CPS_PER_PX = 3.0
TURN_MAX_CPS = 350.0

# PID gains for PWM DELTA
KP = 30.0
KI = 5.0
KD = 0.0

SPEED_ALPHA = 0.35
STEER_SIGN = +1

# No-blocks: stop quickly so it doesn't drift off
NO_BLOCK_TIMEOUT_MS = 120


# -----------------------------
# Main
# -----------------------------
pixy = PixySPI()
motors = Motors()

pidL = PID(KP, KI, KD, i_limit=10000.0, out_limit=PWM_MAX)
pidR = PID(KP, KI, KD, i_limit=10000.0, out_limit=PWM_MAX)

last_ms = utime.ticks_ms()
last_print = last_ms

last_encA = encA
last_encB = encB

vL_f = 0.0
vR_f = 0.0

pwmL = 0
pwmR = 0

last_seen_ms = utime.ticks_ms()

kick_until_ms = 0
was_stopped = True

print("Closed-loop Pixy line follow (PID wheel speed).")
print("Using feedforward FF_PWM =", FF_PWM)
print("Startup kick:", START_KICK_PWM, "for", START_KICK_MS, "ms")
print("If encoders stay 0 forever, it's wiring/pins/encoder power/voltage.")
print("If forward counts are negative, flip ENC_INV_A/B sign.\n")

while True:
    now = utime.ticks_ms()
    dt_ms = utime.ticks_diff(now, last_ms)
    if dt_ms < TICK_MS:
        continue
    last_ms = now
    dt = dt_ms / 1000.0

    # --- measure speeds (counts/sec) ---
    cA = encA
    cB = encB
    dA = cA - last_encA
    dB = cB - last_encB
    last_encA = cA
    last_encB = cB

    vL = dA / dt
    vR = dB / dt

    vL_f = vL_f + SPEED_ALPHA * (vL - vL_f)
    vR_f = vR_f + SPEED_ALPHA * (vR - vR_f)

    # --- get pixy error ---
    b = pixy.best_block(area_min=AREA_MIN)
    if b is None:
        if utime.ticks_diff(now, last_seen_ms) > NO_BLOCK_TIMEOUT_MS:
            pidL.reset()
            pidR.reset()
            pwmL = 0
            pwmR = 0
            motors.stop()
            was_stopped = True
        if utime.ticks_diff(now, last_print) >= PRINT_MS:
            last_print = now
            print("NO BLOCKS | vL=%.0f vR=%.0f | pwmL=%d pwmR=%d | encA=%d encB=%d"
                  % (vL_f, vR_f, pwmL, pwmR, encA, encB))
        continue
    else:
        last_seen_ms = now

    err_px = b["x"] - CENTER_X
    if -DEADBAND_PX <= err_px <= DEADBAND_PX:
        err_px = 0

    # --- make speed targets (counts/sec) ---
    turn_cps = STEER_SIGN * TURN_GAIN_CPS_PER_PX * err_px
    turn_cps = clamp(turn_cps, -TURN_MAX_CPS, TURN_MAX_CPS)

    tgtL = BASE_SPEED_CPS + turn_cps
    tgtR = BASE_SPEED_CPS - turn_cps

    turn_frac = min(1.0, abs(turn_cps) / max(1.0, TURN_MAX_CPS))
    slowdown = 1.0 - 0.35 * turn_frac
    tgtL *= slowdown
    tgtR *= slowdown

    # --- PID update (speed error -> PWM delta) ---
    eL = tgtL - vL_f
    eR = tgtR - vR_f

    uL = pidL.update(eL, dt)
    uR = pidR.update(eR, dt)

    # Feedforward + PID delta
    tgt_pwmL = int(clamp(FF_PWM + uL, 0, PWM_MAX))
    tgt_pwmR = int(clamp(FF_PWM + uR, 0, PWM_MAX))

    # Forward-only for now
    if tgtL <= 0:
        tgt_pwmL = 0
        pidL.reset()
    if tgtR <= 0:
        tgt_pwmR = 0
        pidR.reset()

    # --- startup "kick" (only when starting from stop) ---
    moving_cmd = (tgt_pwmL > 0 or tgt_pwmR > 0)
    if was_stopped and moving_cmd:
        kick_until_ms = utime.ticks_add(now, START_KICK_MS)

    if moving_cmd and utime.ticks_diff(kick_until_ms, now) > 0:
        tgt_pwmL = max(tgt_pwmL, START_KICK_PWM)
        tgt_pwmR = max(tgt_pwmR, START_KICK_PWM)

    was_stopped = not moving_cmd

    # --- slew limit ---
    max_step = int(PWM_SLEW_PER_SEC * dt)
    pwmL = int(clamp(tgt_pwmL, pwmL - max_step, pwmL + max_step))
    pwmR = int(clamp(tgt_pwmR, pwmR - max_step, pwmR + max_step))

    motors.drive(pwmL, pwmR)

    # --- debug print ---
    if utime.ticks_diff(now, last_print) >= PRINT_MS:
        last_print = now
        print(
            "x=%d err=%d | tgtL=%.0f tgtR=%.0f | vL=%.0f vR=%.0f | eL=%.0f eR=%.0f | uL=%.0f uR=%.0f | pwmL=%d pwmR=%d | encA=%d encB=%d"
            % (b["x"], err_px, tgtL, tgtR, vL_f, vR_f, eL, eR, uL, uR, pwmL, pwmR, encA, encB)
        )