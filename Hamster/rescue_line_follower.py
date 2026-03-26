import utime
from utils import clamp


class RescueLineFollower:
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
        print("Line lost -> RECOVERING", "right" if last_seen_side > 0 else "left")

    def _do_recovery_step(self):
        if self.last_seen_side < 0:
            self.mover.turn_left_counts(
                target_counts=self.cfg.LINE_RECOVERY_STEP_COUNTS,
                base_percent=self.cfg.LINE_RECOVERY_TURN_PERCENT,
                timeout_s=self.cfg.LINE_RECOVERY_STEP_TIMEOUT_S,
                debug=False,
            )
        else:
            self.mover.turn_right_counts(
                target_counts=self.cfg.LINE_RECOVERY_STEP_COUNTS,
                base_percent=self.cfg.LINE_RECOVERY_TURN_PERCENT,
                timeout_s=self.cfg.LINE_RECOVERY_STEP_TIMEOUT_S,
                debug=False,
            )

    def run(self):
        cfg = self.cfg

        OUTER_INTERVAL_MS = 20
        INNER_INTERVAL_MS = 20

        integral_left = 0.0
        integral_right = 0.0
        last_error_px = 0.0

        rpm_left_filtered = 0.0
        rpm_right_filtered = 0.0

        target_rpm_left = 0.0
        target_rpm_right = 0.0

        safe_confirm_count = 0
        result = "unknown"

        self.state = self.STATE_TRACKING
        self.recovery_start_ms = 0
        self.last_seen_side = cfg.LINE_RECOVERY_DEFAULT_SIDE

        self.encoders.reset()
        last_encA, last_encB = self.encoders.read_counts()

        start_ms = utime.ticks_ms()
        last_outer_ms = start_ms
        last_inner_ms = start_ms

        print("Rescue line follower started")

        try:
            while True:
                now = utime.ticks_ms()

                if utime.ticks_diff(now, start_ms) / 1000.0 >= cfg.RESCUE_LINE_FOLLOW_DURATION_S:
                    result = "timeout"
                    break

                # ===== OUTER LOOP =====
                if utime.ticks_diff(now, last_outer_ms) >= OUTER_INTERVAL_MS:
                    outer_dt = utime.ticks_diff(now, last_outer_ms) / 1000.0
                    last_outer_ms = now

                    blocks = self.pixy.get_blocks(cfg.PIXY_SIGNATURE_ALL, 10)

                    safe_block = self.pixy.best_block_by_sig(blocks, 7, cfg.SAFE_ZONE_AREA_MIN)
                    line_block = self.pixy.best_block_by_sig(blocks, 1, cfg.PIXY_AREA_MIN)

                    # SAFE ZONE
                    if safe_block and safe_block["area"] >= cfg.SAFE_ZONE_STOP_AREA:
                        safe_confirm_count += 1
                    else:
                        safe_confirm_count = 0

                    if safe_confirm_count >= cfg.SAFE_ZONE_CONFIRM_FRAMES:
                        self.motors.brake_ms(cfg.SAFE_ZONE_BRAKE_MS)
                        return "safe_zone"

                    # LINE LOST
                    if line_block is None:
                        target_rpm_left = target_rpm_right = 0.0
                        integral_left = integral_right = 0.0
                        last_error_px = 0.0

                        if self.state != self.STATE_RECOVERING:
                            self._start_recovery(now, self.last_seen_side)
                        elif utime.ticks_diff(now, self.recovery_start_ms) >= cfg.LINE_RECOVERY_TIMEOUT_MS:
                            return "line_lost"

                    else:
                        # update last seen side
                        if line_block["x"] < cfg.PIXY_CENTER_X:
                            self.last_seen_side = -1
                        elif line_block["x"] > cfg.PIXY_CENTER_X:
                            self.last_seen_side = 1

                        self.state = self.STATE_TRACKING

                        error_px = line_block["x"] - cfg.PIXY_CENTER_X

                        if abs(error_px) <= cfg.LINE_FOLLOW_DEADBAND_PX:
                            error_px = 0

                        if line_block["h"] >= cfg.LINE_FOLLOW_TALL_BLOCK_H:
                            error_px *= cfg.LINE_FOLLOW_TALL_BLOCK_ERROR_SCALE

                        error_derivative = (error_px - last_error_px) / outer_dt
                        last_error_px = error_px

                        turn = cfg.LINE_FOLLOW_STEER_SIGN * (
                            cfg.Kp_STEER * error_px +
                            cfg.Kd_STEER * error_derivative
                        )

                        turn = clamp(turn, -cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM,
                                           cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM)

                        speed_scale = 1.0 - clamp(
                            abs(turn) / cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM,
                            0.0,
                            cfg.MAX_TURNING_SLOWDOWN
                        )

                        base = cfg.LINE_FOLLOW_BASE_RPM * speed_scale

                        target_rpm_left = clamp(base + turn, 0, cfg.LINE_FOLLOW_MAX_RPM)
                        target_rpm_right = clamp(base - turn, 0, cfg.LINE_FOLLOW_MAX_RPM)

                # ===== INNER LOOP =====
                if utime.ticks_diff(now, last_inner_ms) >= INNER_INTERVAL_MS:
                    inner_dt = utime.ticks_diff(now, last_inner_ms) / 1000.0
                    last_inner_ms = now

                    if self.state == self.STATE_RECOVERING:
                        self.motors.coast()
                        self._do_recovery_step()

                        # reset everything after blocking move
                        integral_left = integral_right = 0.0
                        rpm_left_filtered = rpm_right_filtered = 0.0
                        last_error_px = 0.0
                        last_encA, last_encB = self.encoders.read_counts()
                        continue

                    encA, encB = self.encoders.read_counts()
                    dA, dB = encA - last_encA, encB - last_encB
                    last_encA, last_encB = encA, encB

                    rpm_left = (dB / inner_dt) * cfg.CPS_TO_RPM
                    rpm_right = (dA / inner_dt) * cfg.CPS_TO_RPM

                    rpm_left_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_left - rpm_left_filtered)
                    rpm_right_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_right - rpm_right_filtered)

                    eL = target_rpm_left - rpm_left_filtered
                    eR = target_rpm_right - rpm_right_filtered

                    integral_left = clamp(integral_left + eL * inner_dt, -100, 100)
                    integral_right = clamp(integral_right + eR * inner_dt, -100, 100)

                    pwmL = clamp(cfg.Kp_L * eL + cfg.Ki_L * integral_left, -100, 100)
                    pwmR = clamp(cfg.Kp_R * eR + cfg.Ki_R * integral_right, -100, 100)

                    self.motors.drive_percent(pwmL, pwmR)

                utime.sleep_ms(5)

        finally:
            self.motors.coast()

        return result

    def stop(self):
        self.motors.coast()