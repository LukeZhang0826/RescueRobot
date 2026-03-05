# basic_movements.py
import utime
import math
from utils import clamp


class BasicMovements:
    def __init__(self, motors, encoders, cfg):
        self.motors = motors
        self.encoders = encoders
        self.cfg = cfg

    # ---------- small utility ----------
    def pause(self, seconds=0.25):
        """Stop motors and wait. Use between movements."""
        self.motors.stop()
        utime.sleep(seconds)

    # ---------- move forward ----------
    def move_forward_distance(self, distance_m):
        """
        Move forward a specific distance in meters using encoder feedback.
        """

        cfg = self.cfg

        # convert distance → encoder counts
        wheel_rotations = distance_m / (2 * math.pi * cfg.WHEEL_RADIUS)
        target_counts = int(wheel_rotations * cfg.COUNTS_PER_REV)

        pwm_left = 0.0
        pwm_right = 0.0

        integral_left = 0.0
        integral_right = 0.0

        rpm_left_filtered = 0.0
        rpm_right_filtered = 0.0

        self.encoders.reset()

        last_ctrl_ms = utime.ticks_ms()
        last_encA, last_encB = self.encoders.read_counts()

        try:

            while True:

                encA, encB = self.encoders.read_counts()

                # same mapping as before
                left_counts = abs(encB)
                right_counts = abs(encA)

                # stop when both wheels reach distance
                if left_counts >= target_counts and right_counts >= target_counts:
                    break

                now = utime.ticks_ms()
                dt_ms = utime.ticks_diff(now, last_ctrl_ms)

                if dt_ms >= cfg.CTRL_INTERVAL_MS:

                    dt = dt_ms / 1000.0
                    last_ctrl_ms = now

                    dA = encA - last_encA
                    dB = encB - last_encB

                    last_encA = encA
                    last_encB = encB

                    cps_left = dB / dt
                    cps_right = dA / dt

                    rpm_left_raw = cps_left * cfg.CPS_TO_RPM
                    rpm_right_raw = cps_right * cfg.CPS_TO_RPM

                    rpm_left_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_filtered)
                    rpm_right_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_filtered)

                    err_l = cfg.TARGET_RPM - rpm_left_filtered
                    err_r = cfg.TARGET_RPM - rpm_right_filtered

                    integral_left = clamp(integral_left + err_l * dt, -100.0, 100.0)
                    integral_right = clamp(integral_right + err_r * dt, -100.0, 100.0)

                    pwm_left = (cfg.Kp_L * err_l) + (cfg.Ki_L * integral_left)
                    pwm_right = (cfg.Kp_R * err_r) + (cfg.Ki_R * integral_right)

                    pwm_left = clamp(pwm_left, 0.0, 100.0)
                    pwm_right = clamp(pwm_right, 0.0, 100.0)

                    if cfg.USE_MIN_PWM_BOOST and cfg.TARGET_RPM > 0:

                        if 0 < pwm_left < cfg.PWM_MIN_MOVE_L:
                            pwm_left = cfg.PWM_MIN_MOVE_L

                        if 0 < pwm_right < cfg.PWM_MIN_MOVE_R:
                            pwm_right = cfg.PWM_MIN_MOVE_R

                    self.motors.drive_percent(pwm_left, pwm_right)

                utime.sleep_ms(5)

        finally:
            self.motors.stop()

    # ---------- turn in place ----------
    def turn_degrees(self, degrees, pause_after_s=0.2):
        cfg = self.cfg

        theta = math.radians(abs(degrees))
        wheel_travel = (cfg.TRACK_WIDTH / 2.0) * theta
        wheel_rotations = wheel_travel / (2.0 * math.pi * cfg.WHEEL_RADIUS)
        target_counts = int(wheel_rotations * cfg.COUNTS_PER_REV)

        # use config values
        TURN_PWM = cfg.TURN_PWM
        TURN_PWM_MIN = cfg.TURN_PWM_MIN
        RAMP_STEP = cfg.TURN_RAMP_STEP
        RAMP_INTERVAL_MS = cfg.TURN_RAMP_INTERVAL_MS

        self.encoders.reset()

        if degrees >= 0:
            sign_l, sign_r = -1, +1
        else:
            sign_l, sign_r = +1, -1

        pwm = TURN_PWM_MIN
        last_ramp_ms = utime.ticks_ms()

        try:
            while True:
                encA, encB = self.encoders.read_counts()
                left_counts = abs(encB)   # same mapping
                right_counts = abs(encA)

                if left_counts >= target_counts and right_counts >= target_counts:
                    break

                now = utime.ticks_ms()
                if utime.ticks_diff(now, last_ramp_ms) >= RAMP_INTERVAL_MS:
                    last_ramp_ms = now
                    pwm = min(TURN_PWM, pwm + RAMP_STEP)

                self.motors.drive_percent(sign_l * pwm, sign_r * pwm)
                utime.sleep_ms(10)

        finally:
            self.motors.stop()
            if pause_after_s and pause_after_s > 0:
                utime.sleep(pause_after_s)