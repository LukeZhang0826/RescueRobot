import utime
from utils import clamp


class ReturnLineFollower:
    STATE_TRACKING = "tracking"
    STATE_RECOVERING = "recovering"

    def __init__(self, motors, encoders, pixy, mover, cfg):
        self.motors = motors
        self.encoders = encoders
        self.pixy = pixy
        self.mover = mover
        self.cfg = cfg

    def _start_recovery(self, now_ms, last_seen_side):
        self.state = self.STATE_RECOVERING
        self.recovery_start_ms = now_ms
        self.last_seen_side = last_seen_side

    def _do_recovery_step(self):
        if self.last_seen_side < 0:
            self.mover.turn_left_counts(
                self.cfg.LINE_RECOVERY_STEP_COUNTS,
                self.cfg.LINE_RECOVERY_TURN_PERCENT,
                self.cfg.LINE_RECOVERY_STEP_TIMEOUT_S,
                False,
            )
        else:
            self.mover.turn_right_counts(
                self.cfg.LINE_RECOVERY_STEP_COUNTS,
                self.cfg.LINE_RECOVERY_TURN_PERCENT,
                self.cfg.LINE_RECOVERY_STEP_TIMEOUT_S,
                False,
            )

    def run(self):
        cfg = self.cfg

        integral_left = 0.0
        integral_right = 0.0
        last_error_px = 0.0

        rpm_left_filtered = 0.0
        rpm_right_filtered = 0.0

        target_rpm_left = 0.0
        target_rpm_right = 0.0

        self.state = self.STATE_TRACKING
        self.recovery_start_ms = 0
        self.last_seen_side = cfg.LINE_RECOVERY_DEFAULT_SIDE

        self.encoders.reset()
        last_encA, last_encB = self.encoders.read_counts()

        start_ms = utime.ticks_ms()
        last_outer_ms = start_ms
        last_inner_ms = start_ms

        try:
            while True:
                now = utime.ticks_ms()

                if utime.ticks_diff(now, start_ms) / 1000.0 >= cfg.RETURN_LINE_FOLLOW_DURATION_S:
                    return "timeout"

                # OUTER
                if utime.ticks_diff(now, last_outer_ms) >= 20:
                    outer_dt = utime.ticks_diff(now, last_outer_ms) / 1000.0
                    last_outer_ms = now

                    blocks = self.pixy.get_blocks(cfg.PIXY_SIGNATURE_ALL, 10)
                    line_block = self.pixy.best_block_by_sig(blocks, 1, cfg.PIXY_AREA_MIN)

                    if line_block is None:
                        target_rpm_left = target_rpm_right = 0

                        if self.state != self.STATE_RECOVERING:
                            self._start_recovery(now, self.last_seen_side)
                        elif utime.ticks_diff(now, self.recovery_start_ms) >= cfg.LINE_RECOVERY_TIMEOUT_MS:
                            return "line_lost"

                    else:
                        if line_block["x"] < cfg.PIXY_CENTER_X:
                            self.last_seen_side = -1
                        else:
                            self.last_seen_side = 1

                        self.state = self.STATE_TRACKING

                        error = line_block["x"] - cfg.PIXY_CENTER_X
                        d = (error - last_error_px) / outer_dt
                        last_error_px = error

                        turn = cfg.LINE_FOLLOW_STEER_SIGN * (
                            cfg.Kp_STEER * error + cfg.Kd_STEER * d
                        )

                        turn = clamp(turn,
                                     -cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM,
                                      cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM)

                        base = cfg.RETURN_LINE_FOLLOW_BASE_RPM

                        target_rpm_left = clamp(base + turn, 0, cfg.LINE_FOLLOW_MAX_RPM)
                        target_rpm_right = clamp(base - turn, 0, cfg.LINE_FOLLOW_MAX_RPM)

                # INNER
                if utime.ticks_diff(now, last_inner_ms) >= 20:
                    dt = utime.ticks_diff(now, last_inner_ms) / 1000.0
                    last_inner_ms = now

                    if self.state == self.STATE_RECOVERING:
                        self.motors.coast()
                        self._do_recovery_step()

                        last_encA, last_encB = self.encoders.read_counts()
                        integral_left = integral_right = 0
                        continue

                    encA, encB = self.encoders.read_counts()
                    dA, dB = encA - last_encA, encB - last_encB
                    last_encA, last_encB = encA, encB

                    rpmL = (dB / dt) * cfg.CPS_TO_RPM
                    rpmR = (dA / dt) * cfg.CPS_TO_RPM

                    rpm_left_filtered += cfg.SPEED_FILTER_ALPHA * (rpmL - rpm_left_filtered)
                    rpm_right_filtered += cfg.SPEED_FILTER_ALPHA * (rpmR - rpm_right_filtered)

                    eL = target_rpm_left - rpm_left_filtered
                    eR = target_rpm_right - rpm_right_filtered

                    integral_left = clamp(integral_left + eL * dt, -100, 100)
                    integral_right = clamp(integral_right + eR * dt, -100, 100)

                    pwmL = clamp(cfg.Kp_L * eL + cfg.Ki_L * integral_left, -100, 100)
                    pwmR = clamp(cfg.Kp_R * eR + cfg.Ki_R * integral_right, -100, 100)

                    self.motors.drive_percent(pwmL, pwmR)

                utime.sleep_ms(5)

        finally:
            self.motors.coast()