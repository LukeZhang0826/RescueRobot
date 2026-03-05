# pi_speed_test.py
import utime

from utils import clamp

class PISpeedTest:
    """
    Closed-loop PI speed test (same logic as your original), but isolated.
    """
    def __init__(self, motors, encoders, cfg):
        self.motors = motors
        self.encoders = encoders
        self.cfg = cfg

    def run(self):
        cfg = self.cfg

        # Data rows for optional logging
        data = []

        pwm_left = 0.0
        pwm_right = 0.0
        integral_left = 0.0
        integral_right = 0.0

        utime.sleep(cfg.STARTUP_DELAY_S)
        self.encoders.reset()

        start_ms = utime.ticks_ms()
        last_ctrl_ms = start_ms
        last_log_ms = start_ms

        last_encA, last_encB = self.encoders.read_counts()

        rpm_left_f = 0.0
        rpm_right_f = 0.0

        err_l = 0.0
        err_r = 0.0

        self.motors.drive_percent(pwm_left, pwm_right)

        try:
            while True:
                now = utime.ticks_ms()
                elapsed_s = utime.ticks_diff(now, start_ms) / 1000.0
                if elapsed_s >= cfg.TEST_DURATION_S:
                    break

                # Control loop
                dt_ms = utime.ticks_diff(now, last_ctrl_ms)
                if dt_ms >= cfg.CTRL_INTERVAL_MS:
                    dt = dt_ms / 1000.0

                    encA, encB = self.encoders.read_counts()
                    dA = encA - last_encA
                    dB = encB - last_encB
                    last_encA, last_encB = encA, encB
                    last_ctrl_ms = now

                    # Keep your original mapping:
                    # left uses encB, right uses encA
                    cps_left = dB / dt
                    cps_right = dA / dt

                    rpm_left_raw = cps_left * cfg.CPS_TO_RPM
                    rpm_right_raw = cps_right * cfg.CPS_TO_RPM

                    rpm_left_f += cfg.SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_f)
                    rpm_right_f += cfg.SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_f)

                    err_l = cfg.TARGET_RPM - rpm_left_f
                    err_r = cfg.TARGET_RPM - rpm_right_f

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

                # Optional logging samples
                if cfg.ENABLE_LOGGING:
                    log_dt_ms = utime.ticks_diff(now, last_log_ms)
                    if log_dt_ms >= cfg.PRINT_INTERVAL_MS:
                        last_log_ms = now
                        data.append((
                            elapsed_s,
                            cfg.TARGET_RPM,
                            rpm_left_f,
                            rpm_right_f,
                            pwm_left,
                            pwm_right,
                            err_l,
                            err_r
                        ))

                utime.sleep_ms(5)

        finally:
            self.motors.stop()

        return data