# quick_testrun.py
# =================
# Open-loop RPM test - set target RPM, record actual RPM.
# 
# Purpose:
#   - Set a TARGET RPM, convert to estimated PWM
#   - Run motors at fixed PWM (no feedback control)
#   - Record actual RPM from encoders to CSV
#
# Usage:
#   1. Upload this file
#   2. Press button (GPIO 4) to start a test run
#   3. Data saved to motor_tests/run_XXX_YYrpm.csv
#   4. Change TARGET_RPM and press button for next run

import utime
import os
from machine import Pin, PWM


# =============================================================================
# CONFIGURATION - CHANGE THESE FOR EACH TEST RUN
# =============================================================================

TARGET_RPM = 100.0        # Target RPM (converts to estimated PWM)
TEST_DURATION_S = 10      # How long to run the test (seconds)
PRINT_INTERVAL_MS = 500   # Log RPM every 0.5 seconds
STARTUP_DELAY_S = 2       # Wait before starting motors

# RPM to PWM conversion (adjust based on your motor characteristics)
# Rough estimate: at no-load, 310 RPM max output shaft at 100% PWM
# With load, efficiency drops, so we use a conservative factor
RPM_TO_PWM_FACTOR = 0.35  # PWM% = TARGET_RPM * factor (tune this!)


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
# BUTTON SETUP (GPIO 4, active HIGH with pull-down)
# =============================================================================

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)


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


def save_csv(run_number, target_rpm, pwm_used, data_left, data_right):
    """Save test data to CSV file."""
    filename = "{}/run_{:03d}_{:.0f}rpm.csv".format(
        CSV_FOLDER, run_number, target_rpm
    )
    
    with open(filename, "w") as f:
        f.write("time_s,target_rpm,pwm_used,left_rpm,right_rpm,diff_rpm\n")
        for i in range(len(data_left)):
            time_s = (i + 1) * (PRINT_INTERVAL_MS / 1000.0)
            left = data_left[i]
            right = data_right[i]
            diff = right - left
            f.write("{:.1f},{:.0f},{:.1f},{:.1f},{:.1f},{:.1f}\n".format(
                time_s, target_rpm, pwm_used, left, right, diff))
    
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
    def __init__(self, en=5, left=(6, 7), right=(8, 9), pwm_freq=1000):
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
    """Execute open-loop test and return RPM data."""
    global encA, encB
    
    data_left = []
    data_right = []
    
    # Convert target RPM to estimated PWM
    pwm_estimate = clamp(TARGET_RPM * RPM_TO_PWM_FACTOR, 0.0, 100.0)
    
    utime.sleep(STARTUP_DELAY_S)
    
    # Reset encoder counts
    encA = 0
    encB = 0
    
    # Timing
    start_time_ms = utime.ticks_ms()
    last_print_ms = start_time_ms
    last_enc_time_ms = start_time_ms
    last_encA = 0
    last_encB = 0
    
    # Filtered RPM values
    rpm_left_filtered = 0.0
    rpm_right_filtered = 0.0
    
    # Start motors at fixed PWM
    motors.drive_percent(pwm_estimate, pwm_estimate)
    
    try:
        while True:
            now = utime.ticks_ms()
            elapsed_s = utime.ticks_diff(now, start_time_ms) / 1000.0
            
            if elapsed_s >= TEST_DURATION_S:
                break
            
            # Read encoders every 20ms
            enc_dt_ms = utime.ticks_diff(now, last_enc_time_ms)
            if enc_dt_ms >= 20:
                enc_dt = enc_dt_ms / 1000.0
                
                current_encA = encA
                current_encB = encB
                
                delta_encA = current_encA - last_encA
                delta_encB = current_encB - last_encB
                
                last_encA = current_encA
                last_encB = current_encB
                last_enc_time_ms = now
                
                cps_left = delta_encA / enc_dt
                cps_right = delta_encB / enc_dt
                rpm_left_raw = cps_left * CPS_TO_RPM
                rpm_right_raw = cps_right * CPS_TO_RPM
                
                rpm_left_filtered += SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_filtered)
                rpm_right_filtered += SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_filtered)
            
            # Log data every 0.5s
            print_dt_ms = utime.ticks_diff(now, last_print_ms)
            if print_dt_ms >= PRINT_INTERVAL_MS:
                last_print_ms = now
                data_left.append(rpm_left_filtered)
                data_right.append(rpm_right_filtered)
            
            utime.sleep_ms(5)
    
    finally:
        motors.stop()
    
    return pwm_estimate, data_left, data_right


def main():
    print("MOTOR TEST (Open Loop) - Press button to start")
    print(f"Target: {TARGET_RPM} RPM  Duration: {TEST_DURATION_S}s")
    print(f"PWM estimate: {TARGET_RPM * RPM_TO_PWM_FACTOR:.1f}%")
    
    motors = Motors()
    
    while True:
        run_number = get_next_run_number()
        print(f"Run #{run_number} ready - press button")
        
        wait_for_button_press()
        
        print("Running...")
        pwm_used, data_left, data_right = run_test(motors)
        
        if len(data_left) > 0:
            filename = save_csv(run_number, TARGET_RPM, pwm_used, data_left, data_right)
            print(f"Saved: {filename}")
            
            # Quick summary
            avg_l = sum(data_left) / len(data_left)
            avg_r = sum(data_right) / len(data_right)
            print(f"PWM={pwm_used:.1f}% -> L={avg_l:.1f} R={avg_r:.1f} RPM")
        
        print("Done. Press button for next run.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped")
        motors = Motors()
        motors.stop()
