#main.py
# =================
# Closed-loop RPM test - PI control for each motor.
# 
# Purpose:
#   - Set a TARGET RPM for both motors
#   - Use PI control to adjust PWM based on RPM error
#   - Record actual RPM and PWM to CSV
#
# Usage:
#   1. Upload this file
#   2. Press button (GPIO 4) to start a test run
#   3. Data saved to motor_tests/run_XXX_YYrpm.csv
#   4. Tune Kp/Ki and press button for next run

import utime
import os
import machine
from machine import Pin, PWM


# =============================================================================
# CONFIGURATION - CHANGE THESE FOR EACH TEST RUN
# =============================================================================

TARGET_RPM = 250.0        # Target RPM for both motors
TEST_DURATION_S = 7       # How long to run the test (seconds)
PRINT_INTERVAL_MS = 100   # Log every 100ms for better data
STARTUP_DELAY_S = 1       # Wait before starting motors

# PI Controller Gains (per-wheel, tune these!)
# Start with small Kp, Ki=0, then increase
# Motors are mismatched - separate gains handle differences directly
Kp_L = 0.23               # Left proportional gain: PWM% change per RPM error
Ki_L = 0.00               # Left integral gain: PWM% / (RPM*s)
Kp_R = 0.20               # Right proportional gain: PWM% change per RPM error
Ki_R = 0.00               # Right integral gain: PWM% / (RPM*s)

# Minimum PWM to overcome motor deadzone/friction (optional)
# Motors won't move below this PWM - helps pure PI startup
PWM_MIN_MOVE_L = 5.0     # Left motor minimum effective PWM
PWM_MIN_MOVE_R = 5.0     # Right motor minimum effective PWM
USE_MIN_PWM_BOOST = True  # Enable deadzone compensation


# =============================================================================
# CONSTANTS (don't change these)
# =============================================================================

# Motor/Encoder Physical Constants
ENCODER_CPR = 12          # Counts per revolution (motor shaft)
QUADRATURE_MULT = 4       # x4 decoding
GEAR_RATIO = 10           # 10:1 gear reduction
COUNTS_PER_REV = ENCODER_CPR * QUADRATURE_MULT * GEAR_RATIO  # = 480
CPS_TO_RPM = 60.0 / COUNTS_PER_REV  # = 0.125

# PWM
PWM_MAX_RAW = 65535

# Speed filter (for smoother readings)
SPEED_FILTER_ALPHA = 0.3

# Button pin (GPIO 4, active HIGH)
BUTTON_PIN = 4

# CSV output folder
CSV_FOLDER = "motor_tests"


# =============================================================================
# BUTTON SETUP (GPIO 4, active HIGH with pull-up)
# =============================================================================

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)


def wait_for_button_press():
    """Wait for button press (rising edge) with debounce."""
    while button.value() == 1:
        utime.sleep_ms(10)
    while button.value() == 0:
        utime.sleep_ms(10)
    utime.sleep_ms(50)

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


def save_csv(run_number, target_rpm, data):
    """Save test data to CSV file.
    
    data is a list of tuples:
    (time_s, target_rpm, left_rpm, right_rpm)
    """
    filename = "{}/run_{:03d}_{:.0f}rpm.csv".format(
        CSV_FOLDER, run_number, target_rpm
    )
    
    with open(filename, "w") as f:
        f.write("time_s,target_rpm,left_rpm,right_rpm\n")
        for row in data:
            f.write("{:.2f},{:.0f},{:.1f},{:.1f}\n".format(*row))
    
    return filename


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
# MAIN TEST
# =============================================================================

def run_test(motors):
    """Execute closed-loop PI test and return data."""
    global encA, encB
    
    data = []  # List of (time_s, target, l_rpm, r_rpm, l_pwm, r_pwm, err_l, err_r)
    
    # Initial PWM: start at 0 (pure PI, no feedforward)
    pwm_left = 0.0
    pwm_right = 0.0
    
    # PI controller state (integral term)
    integral_left = 0.0
    integral_right = 0.0
    
    utime.sleep(STARTUP_DELAY_S)
    
    # Reset encoder counts
    encA = 0
    encB = 0
    
    # Timing
    start_time_ms = utime.ticks_ms()
    last_print_ms = start_time_ms
    last_ctrl_time_ms = start_time_ms
    last_encA = 0
    last_encB = 0
    
    # Filtered RPM values
    rpm_left_filtered = 0.0
    rpm_right_filtered = 0.0
    
    # Current errors for logging
    error_left = 0.0
    error_right = 0.0
    
    # Control loop interval (ms) - 50ms reduces encoder noise
    CTRL_INTERVAL_MS = 50
    
    # Start motors with initial PWM
    motors.drive_percent(pwm_left, pwm_right)
    
    try:
        while True:
            now = utime.ticks_ms()
            elapsed_s = utime.ticks_diff(now, start_time_ms) / 1000.0
            
            if elapsed_s >= TEST_DURATION_S:
                break
            
            # Control loop - run every CTRL_INTERVAL_MS
            ctrl_dt_ms = utime.ticks_diff(now, last_ctrl_time_ms)
            if ctrl_dt_ms >= CTRL_INTERVAL_MS:
                ctrl_dt = ctrl_dt_ms / 1000.0
                
                # Atomic encoder read (disable IRQ to prevent race condition)
                irq_state = machine.disable_irq()
                current_encA = encA
                current_encB = encB
                machine.enable_irq(irq_state)
                
                delta_encA = current_encA - last_encA
                delta_encB = current_encB - last_encB
                
                last_encA = current_encA
                last_encB = current_encB
                last_ctrl_time_ms = now
                
                # Calculate RPM
                cps_left = delta_encB / ctrl_dt
                cps_right = delta_encA / ctrl_dt
                rpm_left_raw = cps_left * CPS_TO_RPM
                rpm_right_raw = cps_right * CPS_TO_RPM
                
                # Low-pass filter
                rpm_left_filtered += SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_filtered)
                rpm_right_filtered += SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_filtered)
                
                # Compute error
                error_left = TARGET_RPM - rpm_left_filtered
                error_right = TARGET_RPM - rpm_right_filtered
                
                # Update integral (with anti-windup clamp)
                integral_left += error_left * ctrl_dt
                integral_right += error_right * ctrl_dt
                integral_left = clamp(integral_left, -100.0, 100.0)
                integral_right = clamp(integral_right, -100.0, 100.0)
                
                # Pure PI control: per-wheel gains, no feedforward
                pwm_left = (Kp_L * error_left) + (Ki_L * integral_left)
                pwm_right = (Kp_R * error_right) + (Ki_R * integral_right)
                
                # Clamp PWM to valid range
                pwm_left = clamp(pwm_left, 0.0, 100.0)
                pwm_right = clamp(pwm_right, 0.0, 100.0)
                
                # Apply minimum PWM boost to overcome motor deadzone
                if USE_MIN_PWM_BOOST and TARGET_RPM > 0:
                    if 0 < pwm_left < PWM_MIN_MOVE_L:
                        pwm_left = PWM_MIN_MOVE_L
                    if 0 < pwm_right < PWM_MIN_MOVE_R:
                        pwm_right = PWM_MIN_MOVE_R
                
                # Apply new PWM
                motors.drive_percent(pwm_left, pwm_right)
            
            # Log data at PRINT_INTERVAL_MS
            print_dt_ms = utime.ticks_diff(now, last_print_ms)
            if print_dt_ms >= PRINT_INTERVAL_MS:
                last_print_ms = now
                data.append((
                    elapsed_s,
                    TARGET_RPM,
                    rpm_left_filtered,
                    rpm_right_filtered,
                    pwm_left,
                    pwm_right,
                    error_left,
                    error_right
                ))
            
            utime.sleep_ms(5)
    
    finally:
        motors.stop()
    
    return data


def main():
    print("MOTOR TEST (Pure PI Control) - Press button to start")
    print(f"Target: {TARGET_RPM} RPM  Duration: {TEST_DURATION_S}s")
    print(f"Left:  Kp={Kp_L}  Ki={Ki_L}")
    print(f"Right: Kp={Kp_R}  Ki={Ki_R}")
    print(f"Min PWM boost: L={PWM_MIN_MOVE_L}% R={PWM_MIN_MOVE_R}% (enabled={USE_MIN_PWM_BOOST})")
    
    motors = Motors()
    
    while True:
        run_number = get_next_run_number()
        print(f"Run #{run_number} ready - press button")
        
        wait_for_button_press()
        
        print("Running...")
        data = run_test(motors)
        
        if len(data) > 0:
            filename = save_csv(run_number, TARGET_RPM, data)
            print(f"Saved: {filename}")
            
            # Quick summary (average RPM and PWM)
            avg_l_rpm = sum(r[2] for r in data) / len(data)
            avg_r_rpm = sum(r[3] for r in data) / len(data)
#             avg_l_pwm = sum(r[4] for r in data) / len(data)
#             avg_r_pwm = sum(r[5] for r in data) / len(data)
#             print(f"Avg: L={avg_l_rpm:.1f}rpm/{avg_l_pwm:.1f}%  R={avg_r_rpm:.1f}rpm/{avg_r_pwm:.1f}%")
        
        print("Done. Press button for next run.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped")
        motors = Motors()
        motors.stop()
