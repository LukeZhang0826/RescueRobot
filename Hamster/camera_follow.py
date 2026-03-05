# camera_follow.py
# =================
# Camera-based line following with cascaded control:
#   - Outer loop: PD control on camera error (pixel offset)
#   - Inner loop: PI speed control per wheel (encoder feedback)
#   - CSV logging: camera + motor data
#
# Usage:
#   1. Upload this file to Pico
#   2. Press button (GPIO 4) to start
#   3. Robot follows red object/line using camera
#   4. Data saved to camera_tests/run_XXX.csv

import utime
import os
import machine
from machine import Pin, PWM, SPI


# =============================================================================
# CONFIGURATION
# =============================================================================

# Test settings
TEST_DURATION_S = 30       # How long to run (seconds), 0 = run forever until button
STARTUP_DELAY_S = 1        # Wait before starting
LOG_INTERVAL_MS = 100      # Log to CSV every 100ms

# Camera frame properties
FRAME_W = 316              # Pixy2 frame width
CENTER_X = FRAME_W // 2    # Center pixel (target position)

# Detection thresholds
AREA_MIN = 2000            # Minimum block area to consider valid
DEADBAND_PX = 5            # Ignore errors smaller than this (pixels)
NO_BLOCK_TIMEOUT_MS = 150  # Stop if no blocks seen for this long

# -----------------------------------------------------------------------------
# OUTER LOOP: PD Control (camera error -> speed targets)
# -----------------------------------------------------------------------------
# Converts pixel error to differential wheel speed targets (counts/sec)

BASE_SPEED_CPS = 300.0     # Base forward speed (counts/sec) when centered
Kp_STEER = 3.5             # Proportional: CPS per pixel error
Kd_STEER = 0.05            # Derivative: dampens oscillation
TURN_MAX_CPS = 400.0       # Maximum turn differential
STEER_SIGN = +1            # +1 or -1 to flip turn direction

# Slowdown during turns (reduce base speed proportionally to turn amount)
TURN_SLOWDOWN_FACTOR = 0.3 # 0 = no slowdown, 1 = full slowdown at max turn

# -----------------------------------------------------------------------------
# INNER LOOP: PI Speed Control (speed error -> PWM)
# -----------------------------------------------------------------------------
# Per-wheel gains (tune these based on your motors)

Kp_L = 0.23                # Left motor proportional gain
Ki_L = 0.05                # Left motor integral gain
Kp_R = 0.20                # Right motor proportional gain
Ki_R = 0.05                # Right motor integral gain

# Integral anti-windup limits
INTEGRAL_LIMIT = 100.0

# Minimum PWM to overcome motor deadzone
PWM_MIN_MOVE_L = 8.0       # Left motor minimum effective PWM %
PWM_MIN_MOVE_R = 8.0       # Right motor minimum effective PWM %
USE_MIN_PWM_BOOST = True

# Control loop timing
CTRL_INTERVAL_MS = 20      # Inner loop runs every 20ms
SPEED_FILTER_ALPHA = 0.35  # Low-pass filter for speed readings


# =============================================================================
# CONSTANTS (don't change)
# =============================================================================

# Encoder constants
ENCODER_CPR = 12
QUADRATURE_MULT = 4
GEAR_RATIO = 10
COUNTS_PER_REV = ENCODER_CPR * QUADRATURE_MULT * GEAR_RATIO  # = 480
CPS_TO_RPM = 60.0 / COUNTS_PER_REV

# PWM
PWM_MAX_RAW = 65535

# Button pin
BUTTON_PIN = 4

# CSV folder
CSV_FOLDER = "camera_tests"


# =============================================================================
# PIXY2 SPI DRIVER
# =============================================================================

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
            sig = u16le(pl[off + 0], pl[off + 1])
            x   = u16le(pl[off + 2], pl[off + 3])
            y   = u16le(pl[off + 4], pl[off + 5])
            w   = u16le(pl[off + 6], pl[off + 7])
            h   = u16le(pl[off + 8], pl[off + 9])
            blocks.append({"sig": sig, "x": x, "y": y, "w": w, "h": h, "area": w*h})
        return blocks

    def best_block(self, area_min=2000, sigmap=0xFF):
        """Get the largest block above area_min threshold."""
        blocks = self.get_blocks(sigmap=sigmap, max_blocks=10)
        blocks = [b for b in blocks if b["area"] >= area_min]
        if not blocks:
            return None
        return max(blocks, key=lambda bb: bb["area"])


# =============================================================================
# BUTTON SETUP
# =============================================================================

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

def wait_for_button_press():
    """Wait for button press with debounce."""
    while button.value() == 1:
        utime.sleep_ms(10)
    while button.value() == 0:
        utime.sleep_ms(10)
    utime.sleep_ms(50)

def button_pressed():
    """Check if button is currently pressed (non-blocking)."""
    return button.value() == 0


# =============================================================================
# CSV FILE HANDLING
# =============================================================================

def get_next_run_number():
    """Find the next available run number."""
    try:
        os.mkdir(CSV_FOLDER)
    except OSError:
        pass
    
    try:
        files = os.listdir(CSV_FOLDER)
    except OSError:
        files = []
    
    max_run = 0
    for f in files:
        if f.startswith("run_") and f.endswith(".csv"):
            try:
                num = int(f[4:7])
                if num > max_run:
                    max_run = num
            except ValueError:
                pass
    
    return max_run + 1


def save_csv(run_number, data):
    """Save test data to CSV file.
    
    data is a list of tuples:
    (time_s, cam_x, cam_error, target_L, target_R, rpm_L, rpm_R, pwm_L, pwm_R)
    """
    filename = "{}/run_{:03d}.csv".format(CSV_FOLDER, run_number)
    
    with open(filename, "w") as f:
        f.write("time_s,cam_x,cam_error,target_cps_L,target_cps_R,")
        f.write("rpm_L,rpm_R,pwm_L,pwm_R\n")
        for row in data:
            f.write("{:.3f},{:.0f},{:.0f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}\n".format(*row))
    
    return filename


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def clamp(value, min_val, max_val):
    """Clamp value to range."""
    if value < min_val:
        return min_val
    if value > max_val:
        return max_val
    return value


# =============================================================================
# ENCODER SETUP (Quadrature x4 decoding via ISR)
# =============================================================================

QDEC_TABLE = (
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
)

# Encoder pins
EA_A = Pin(2, Pin.IN, Pin.PULL_UP)
EA_B = Pin(3, Pin.IN, Pin.PULL_UP)
EB_A = Pin(12, Pin.IN, Pin.PULL_UP)
EB_B = Pin(13, Pin.IN, Pin.PULL_UP)

# Global encoder counts
encA = 0
encB = 0
oldA = (EA_A.value() << 1) | EA_B.value()
oldB = (EB_A.value() << 1) | EB_B.value()

# Encoder inversion (adjust so forward = positive)
ENC_INV_A = -1
ENC_INV_B = -1


def irqA(pin):
    global encA, oldA
    new = (EA_A.value() << 1) | EA_B.value()
    step = QDEC_TABLE[(oldA << 2) | new]
    encA += ENC_INV_A * step
    oldA = new


def irqB(pin):
    global encB, oldB
    new = (EB_A.value() << 1) | EB_B.value()
    step = QDEC_TABLE[(oldB << 2) | new]
    encB += ENC_INV_B * step
    oldB = new


# Register interrupts
EA_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqA)
EA_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqA)
EB_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqB)
EB_B.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irqB)


# =============================================================================
# MOTOR DRIVER (DRV8833)
# =============================================================================

class Motors:
    def __init__(self, en=5, left=(8, 9), right=(6, 7), pwm_freq=1000):
        self.en = Pin(en, Pin.OUT)
        self.l1 = PWM(Pin(left[0]))
        self.l2 = PWM(Pin(left[1]))
        self.r1 = PWM(Pin(right[0]))
        self.r2 = PWM(Pin(right[1]))
        
        for p in (self.l1, self.l2, self.r1, self.r2):
            p.freq(pwm_freq)
            p.duty_u16(0)
        
        self.en.value(0)

    def drive_percent(self, left_percent, right_percent):
        """Drive both motors at given PWM percentages (0-100%)."""
        left_raw = int(left_percent / 100.0 * PWM_MAX_RAW)
        right_raw = int(right_percent / 100.0 * PWM_MAX_RAW)
        
        self.en.value(1)
        
        # Left motor forward
        self.l1.duty_u16(min(left_raw, PWM_MAX_RAW))
        self.l2.duty_u16(0)
        
        # Right motor forward
        self.r1.duty_u16(min(right_raw, PWM_MAX_RAW))
        self.r2.duty_u16(0)

    def stop(self):
        """Stop both motors."""
        self.en.value(0)
        self.l1.duty_u16(0)
        self.l2.duty_u16(0)
        self.r1.duty_u16(0)
        self.r2.duty_u16(0)


# =============================================================================
# CASCADED CONTROL LOOP
# =============================================================================

def run_follow(motors, pixy):
    """
    Main control loop with cascaded PD (outer) + PI (inner) control.
    
    Outer loop: PD on camera error -> speed targets
    Inner loop: PI on each wheel -> PWM commands
    """
    global encA, encB
    
    data = []  # CSV data
    
    # Reset encoder counts
    irq_state = machine.disable_irq()
    encA = 0
    encB = 0
    machine.enable_irq(irq_state)
    
    # Inner loop PI state
    integral_left = 0.0
    integral_right = 0.0
    
    # Outer loop PD state
    last_cam_error = 0.0
    
    # Speed measurements
    cps_left_filtered = 0.0
    cps_right_filtered = 0.0
    last_encA = 0
    last_encB = 0
    
    # Current targets and outputs
    target_cps_L = 0.0
    target_cps_R = 0.0
    pwm_left = 0.0
    pwm_right = 0.0
    
    # Camera values for logging
    cam_x = CENTER_X
    cam_error = 0
    
    # Timing
    start_time_ms = utime.ticks_ms()
    last_ctrl_ms = start_time_ms
    last_log_ms = start_time_ms
    last_seen_ms = start_time_ms
    last_print_ms = start_time_ms
    
    utime.sleep(STARTUP_DELAY_S)
    
    print("Starting camera follow...")
    
    try:
        while True:
            now = utime.ticks_ms()
            elapsed_s = utime.ticks_diff(now, start_time_ms) / 1000.0
            
            # Check stop conditions
            if TEST_DURATION_S > 0 and elapsed_s >= TEST_DURATION_S:
                print("Test duration reached")
                break
            
            if button_pressed():
                print("Button pressed - stopping")
                break
            
            # =========================================================
            # OUTER LOOP: Camera -> Speed Targets (runs every frame)
            # =========================================================
            block = pixy.best_block(area_min=AREA_MIN)
            
            if block is None:
                # No block detected
                if utime.ticks_diff(now, last_seen_ms) > NO_BLOCK_TIMEOUT_MS:
                    # Timeout - stop motors
                    target_cps_L = 0.0
                    target_cps_R = 0.0
                    integral_left = 0.0
                    integral_right = 0.0
                    cam_error = 0
                    cam_x = CENTER_X
            else:
                # Block detected - update targets
                last_seen_ms = now
                cam_x = block["x"]
                
                # Calculate pixel error from center
                raw_error = cam_x - CENTER_X
                
                # Apply deadband
                if -DEADBAND_PX <= raw_error <= DEADBAND_PX:
                    cam_error = 0
                else:
                    cam_error = raw_error
                
                # PD control: compute turn differential
                # dt for derivative (use ctrl interval as approximation)
                dt_outer = CTRL_INTERVAL_MS / 1000.0
                error_derivative = (cam_error - last_cam_error) / dt_outer if dt_outer > 0 else 0
                last_cam_error = cam_error
                
                # Turn amount from PD
                turn_cps = STEER_SIGN * (Kp_STEER * cam_error + Kd_STEER * error_derivative)
                turn_cps = clamp(turn_cps, -TURN_MAX_CPS, TURN_MAX_CPS)
                
                # Calculate slowdown during turns
                turn_frac = abs(turn_cps) / TURN_MAX_CPS if TURN_MAX_CPS > 0 else 0
                slowdown = 1.0 - TURN_SLOWDOWN_FACTOR * turn_frac
                
                # Differential drive: base speed +/- turn
                target_cps_L = (BASE_SPEED_CPS + turn_cps) * slowdown
                target_cps_R = (BASE_SPEED_CPS - turn_cps) * slowdown
                
                # Don't allow negative targets (forward only)
                target_cps_L = max(0.0, target_cps_L)
                target_cps_R = max(0.0, target_cps_R)
            
            # =========================================================
            # INNER LOOP: Speed Control (runs every CTRL_INTERVAL_MS)
            # =========================================================
            ctrl_dt_ms = utime.ticks_diff(now, last_ctrl_ms)
            if ctrl_dt_ms >= CTRL_INTERVAL_MS:
                ctrl_dt = ctrl_dt_ms / 1000.0
                last_ctrl_ms = now
                
                # Atomic encoder read
                irq_state = machine.disable_irq()
                current_encA = encA
                current_encB = encB
                machine.enable_irq(irq_state)
                
                delta_encA = current_encA - last_encA
                delta_encB = current_encB - last_encB
                last_encA = current_encA
                last_encB = current_encB
                
                # Calculate speed (counts/sec)
                # Note: Adjust which encoder is L/R based on your wiring
                cps_left_raw = delta_encB / ctrl_dt
                cps_right_raw = delta_encA / ctrl_dt
                
                # Low-pass filter
                cps_left_filtered += SPEED_FILTER_ALPHA * (cps_left_raw - cps_left_filtered)
                cps_right_filtered += SPEED_FILTER_ALPHA * (cps_right_raw - cps_right_filtered)
                
                # Speed error
                error_left = target_cps_L - cps_left_filtered
                error_right = target_cps_R - cps_right_filtered
                
                # Update integral (with anti-windup)
                integral_left += error_left * ctrl_dt
                integral_right += error_right * ctrl_dt
                integral_left = clamp(integral_left, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
                integral_right = clamp(integral_right, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
                
                # Reset integral if target is zero (stopped)
                if target_cps_L <= 0:
                    integral_left = 0.0
                if target_cps_R <= 0:
                    integral_right = 0.0
                
                # PI control: error -> PWM %
                pwm_left = (Kp_L * error_left) + (Ki_L * integral_left)
                pwm_right = (Kp_R * error_right) + (Ki_R * integral_right)
                
                # Clamp to valid range
                pwm_left = clamp(pwm_left, 0.0, 100.0)
                pwm_right = clamp(pwm_right, 0.0, 100.0)
                
                # Minimum PWM boost to overcome deadzone
                if USE_MIN_PWM_BOOST:
                    if target_cps_L > 0 and 0 < pwm_left < PWM_MIN_MOVE_L:
                        pwm_left = PWM_MIN_MOVE_L
                    if target_cps_R > 0 and 0 < pwm_right < PWM_MIN_MOVE_R:
                        pwm_right = PWM_MIN_MOVE_R
                
                # If target is zero, force PWM to zero
                if target_cps_L <= 0:
                    pwm_left = 0.0
                if target_cps_R <= 0:
                    pwm_right = 0.0
                
                # Apply PWM
                motors.drive_percent(pwm_left, pwm_right)
            
            # =========================================================
            # LOGGING
            # =========================================================
            log_dt_ms = utime.ticks_diff(now, last_log_ms)
            if log_dt_ms >= LOG_INTERVAL_MS:
                last_log_ms = now
                
                # Convert CPS to RPM for logging
                rpm_left = cps_left_filtered * CPS_TO_RPM
                rpm_right = cps_right_filtered * CPS_TO_RPM
                
                data.append((
                    elapsed_s,
                    cam_x,
                    cam_error,
                    target_cps_L,
                    target_cps_R,
                    rpm_left,
                    rpm_right,
                    pwm_left,
                    pwm_right
                ))
            
            # Debug print
            print_dt_ms = utime.ticks_diff(now, last_print_ms)
            if print_dt_ms >= 200:
                last_print_ms = now
                rpm_L = cps_left_filtered * CPS_TO_RPM
                rpm_R = cps_right_filtered * CPS_TO_RPM
                print("x={:3d} err={:4d} | tgt={:5.0f}/{:5.0f} | rpm={:5.1f}/{:5.1f} | pwm={:4.1f}/{:4.1f}".format(
                    int(cam_x), int(cam_error),
                    target_cps_L, target_cps_R,
                    rpm_L, rpm_R,
                    pwm_left, pwm_right
                ))
            
            utime.sleep_ms(5)
    
    finally:
        motors.stop()
    
    return data


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("=" * 50)
    print("CAMERA FOLLOW - Cascaded PD+PI Control")
    print("=" * 50)
    print(f"Outer PD: Kp_STEER={Kp_STEER}  Kd_STEER={Kd_STEER}")
    print(f"Inner PI Left:  Kp={Kp_L}  Ki={Ki_L}")
    print(f"Inner PI Right: Kp={Kp_R}  Ki={Ki_R}")
    print(f"Base speed: {BASE_SPEED_CPS} CPS ({BASE_SPEED_CPS * CPS_TO_RPM:.1f} RPM)")
    print(f"Duration: {TEST_DURATION_S}s (0=forever)")
    print("=" * 50)
    
    motors = Motors()
    pixy = PixySPI()
    
    while True:
        run_number = get_next_run_number()
        print(f"\nRun #{run_number} ready - press button to start")
        print("(Press button again during run to stop)")
        
        wait_for_button_press()
        
        print("Starting in {}s...".format(STARTUP_DELAY_S))
        data = run_follow(motors, pixy)
        
        if len(data) > 0:
            filename = save_csv(run_number, data)
            print(f"Saved: {filename} ({len(data)} samples)")
            
            # Quick summary
            if len(data) > 10:
                # Skip first few samples (startup)
                d = data[5:]
                avg_err = sum(r[2] for r in d) / len(d)
                avg_rpm_L = sum(r[5] for r in d) / len(d)
                avg_rpm_R = sum(r[6] for r in d) / len(d)
                print(f"Avg error: {avg_err:.1f}px  RPM: L={avg_rpm_L:.1f} R={avg_rpm_R:.1f}")
        else:
            print("No data collected")
        
        print("Done. Press button for next run.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
        motors = Motors()
        motors.stop()
