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
LEFT_MOTOR_PINS = (11, 9)
RIGHT_MOTOR_PINS = (6, 7)

PWM_FREQ = 1000
PWM_MAX_RAW = 65535


# =========================
# Robot physical constants
# =========================
WHEEL_RADIUS = 0.01      # meters
TRACK_WIDTH = 0.092       # distance between wheels (meters)


# =========================
# Pixy2 Camera (SPI)
# =========================
PIXY_SPI_ID = 0
PIXY_CS_PIN = 21
PIXY_SCK_PIN = 18
PIXY_MOSI_PIN = 19
PIXY_MISO_PIN = 20
PIXY_BAUDRATE = 500_000

# Camera frame
PIXY_FRAME_WIDTH = 316
PIXY_FRAME_HEIGHT = 208
PIXY_FRAME_AREA = PIXY_FRAME_WIDTH * PIXY_FRAME_HEIGHT
PIXY_CENTER_X = PIXY_FRAME_WIDTH // 2  # 158

# Detection
PIXY_AREA_MIN = 2000     # Minimum block area to consider valid
PIXY_SIGNATURE_1 = 0x01  # Red Line
PIXY_SIGNATURE_2 = 0x02
PIXY_SIGNATURE_3 = 0x04  # Blue Line
PIXY_SIGNATURE_4 = 0x08
PIXY_SIGNATURE_5 = 0x10
PIXY_SIGNATURE_6 = 0x20
PIXY_SIGNATURE_7 = 0x40   # Safe Zones (Green Lines)
PIXY_SIGNATURE_ALL = 0xFF # All signatures (for get_blocks sigmap)

# =========================
# Line Follower (Outer PD Loop)
# =========================
LINE_FOLLOW_DURATION_S = 10    # How long to run (seconds)

LINE_FOLLOW_BASE_RPM = 150.0     # Forward speed when centered

LINE_FOLLOW_MAX_RPM = 400.0      # Maximum wheel RPM
LINE_FOLLOW_MIN_RPM = 40.0       # Absolute floor speed to prevent stalling

LINE_FOLLOW_MAX_DIFFERENTIAL_RPM = 200.0  # Maximum turn differential in RPM (added to one wheel, subtracted from the other)

LINE_FOLLOW_TALL_BLOCK_H = 80   # If block height exceeds this, it's "too much" of the line
LINE_FOLLOW_TALL_BLOCK_ERROR_SCALE = 0.65  # Scale down error when seeing "too much" of the line to prevent overreacting

# PD gains for steering
Kp_STEER = 0.30           # RPM per pixel error
Kd_STEER = 0.3            # Derivative gain (dampen oscillations)

LINE_FOLLOW_STEER_SIGN = +1      # +1 or -1 to flip turn direction
LINE_FOLLOW_DEADBAND_PX = 5      # Ignore small errors (pixels)


# =========================
# Green Target Approach
# =========================
TARGET_AREA_MIN = 200   # Minimum area for valid target block

TARGET_X = 158            # desired blob x in image
TARGET_Y = 180            # desired blob y in image

TARGET_DEADBAND_X = 10           # pixels
TARGET_DEADBAND_Y = 10           # pixels

TARGET_SEARCH_TURN_COUNTS = 12   # tiny search nudge
TARGET_SEARCH_TURN_PERCENT = 16.0

TARGET_TURN_COUNTS_SMALL = 4
TARGET_TURN_COUNTS_MED = 7
TARGET_TURN_COUNTS_LARGE = 12
TARGET_TURN_PERCENT = 16.0

TARGET_MOVE_COUNTS_SMALL = 8
TARGET_MOVE_COUNTS_MED = 12
TARGET_MOVE_COUNTS_LARGE = 14
TARGET_MOVE_PERCENT = 16.0

TARGET_APPROACH_MAX_STEPS = 40   # safety cap
TARGET_SETTLE_MS = 120           # pause after each primitive
TARGET_DEBUG = True

# =========================
# Basic movement tuning
# =========================
BASIC_MAX_PERCENT = 35.0

LEFT_FWD_MIN = 16.0
LEFT_REV_MIN = 22.0
RIGHT_FWD_MIN = 16.0
RIGHT_REV_MIN = 22.0

# =========================
# Blue tape stop logic
# =========================
STOP_TARGET_AREA = 10000  
STOP_TARGET_AREA_MIN = 1000
STOP_BRAKE_MS = 150
STOP_CONFIRM_FRAMES = 5