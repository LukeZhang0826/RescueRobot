# config.py
# Central config for the robot speed test.

# =========================
# Test parameters
# =========================
TARGET_RPM = 200.0
TEST_DURATION_S = 7
CTRL_INTERVAL_MS = 50
STARTUP_DELAY_S = 1

# Debug / logging
ENABLE_LOGGING = False          # master switch (CSV + periodic samples)
PRINT_INTERVAL_MS = 100         # sample/log rate when ENABLE_LOGGING = True
CSV_FOLDER = "motor_tests"

# =========================
# PI gains (per wheel)
# =========================
Kp_L = 0.20
Ki_L = 0.19
Kp_R = 0.20
Ki_R = 0.19

# Motor deadzone compensation
USE_MIN_PWM_BOOST = True
PWM_MIN_MOVE_L = 5.0
PWM_MIN_MOVE_R = 5.0

# Turning speed (percent PWM, signed internally)
TURN_PWM = 25.0        # increase = faster turn (try 25..45)
TURN_PWM_MIN = 10.0    # minimum to overcome deadzone
TURN_RAMP_STEP = 2.0   # ramp-up step (bigger = faster ramp)
TURN_RAMP_INTERVAL_MS = 30

# =========================
# Encoder constants
# =========================
ENCODER_CPR = 12
QUADRATURE_MULT = 4
GEAR_RATIO = 10

COUNTS_PER_REV = ENCODER_CPR * QUADRATURE_MULT * GEAR_RATIO  # 480
CPS_TO_RPM = 60.0 / COUNTS_PER_REV

# Speed filter
SPEED_FILTER_ALPHA = 0.3

# =========================
# Pins
# =========================
BUTTON_PIN = 4

# Encoder pins
EA_A_PIN = 2
EA_B_PIN = 3
EB_A_PIN = 12
EB_B_PIN = 13

# Encoder inversion (forward should be positive)
ENC_INV_A = -1
ENC_INV_B = -1

# Motor driver pins (DRV8833)
MOTOR_EN_PIN = 5
# NOTE: this keeps your original mapping:
# left = (8,9), right=(6,7)
LEFT_MOTOR_PINS = (8, 9)
RIGHT_MOTOR_PINS = (6, 7)

PWM_FREQ = 1000
PWM_MAX_RAW = 65535


# =========================
# Robot physical constants
# =========================
WHEEL_RADIUS = 0.01      # meters
TRACK_WIDTH = 0.092       # distance between wheels (meters)