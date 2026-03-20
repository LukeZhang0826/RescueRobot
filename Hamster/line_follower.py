# line_follower.py
# Camera-based line following with cascaded control:
#   - Outer loop: PD control on camera error (20Hz)
#   - Inner loop: PI speed control per wheel (50Hz)

import utime
from utils import clamp


class LineFollower:
    """
    Cascaded controller for camera-based line following.
    
    Outer loop (PD): Camera pixel error -> target RPM for each wheel
    Inner loop (PI): RPM error -> PWM commands
    """

    def __init__(self, motors, encoders, pixy, cfg):
        self.motors = motors
        self.encoders = encoders
        self.pixy = pixy
        self.cfg = cfg

    def run(self):
        """
        Run line following until timeout or stop target detected.
        
        Line follow:
        - steering from signature 1
        
        Stop condition:
        - signature 3 area >= STOP_TARGET_AREA for STOP_CONFIRM_FRAMES frames
        - then active brake and exit
        """
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

        pwm_left = 0.0
        pwm_right = 0.0

        stop_confirm_count = 0

        self.encoders.reset()
        last_encA, last_encB = self.encoders.read_counts()

        start_ms = utime.ticks_ms()
        last_outer_ms = start_ms
        last_inner_ms = start_ms

        print("Line follower started")
        print(f"Duration: {cfg.LINE_FOLLOW_DURATION_S}s")
        print(f"Base RPM: {cfg.LINE_FOLLOW_BASE_RPM}")

        try:
            while True:
                now = utime.ticks_ms()
                elapsed_s = utime.ticks_diff(now, start_ms) / 1000.0

                if elapsed_s >= cfg.LINE_FOLLOW_DURATION_S:
                    print("Duration reached - stopping")
                    break

                # =============================================================
                # OUTER LOOP: Camera PD + stop-target detection
                # =============================================================
                outer_dt_ms = utime.ticks_diff(now, last_outer_ms)
                if outer_dt_ms >= OUTER_INTERVAL_MS:
                    outer_dt = outer_dt_ms / 1000.0
                    last_outer_ms = now

                    # ONE Pixy read only
                    blocks = self.pixy.get_blocks(
                        sigmap=cfg.PIXY_SIGNATURE_ALL,
                        max_blocks=10
                    )

                    # Best stop block: signature 3
                    stop_block = self.pixy.best_block_by_sig(
                        blocks,
                        sig=3,
                        area_min=cfg.STOP_TARGET_AREA_MIN
                    )

                    # Best line block: signature 1
                    line_block = self.pixy.best_block_by_sig(
                        blocks,
                        sig=1,
                        area_min=cfg.PIXY_AREA_MIN
                    )

                    # This is causing the robot to move forward jittery, why?
                    if stop_block is not None and stop_block["area"] >= cfg.STOP_TARGET_AREA:
                        stop_confirm_count += 1
                        print(
                            "STOP target seen | area={} count={}".format(
                                stop_block["area"], stop_confirm_count
                            )
                        )
                    else:
                        stop_confirm_count = 0

                    if stop_confirm_count >= cfg.STOP_CONFIRM_FRAMES:
                        print("STOP target confirmed - braking")
                        self.motors.brake_ms(cfg.STOP_BRAKE_MS)
                        break


                    if line_block is None:
                        target_rpm_left = 0.0
                        target_rpm_right = 0.0
                        integral_left = 0.0
                        integral_right = 0.0
                        last_error_px = 0.0
                    else:
                        error_px = line_block["x"] - cfg.PIXY_CENTER_X

                        if abs(error_px) <= cfg.LINE_FOLLOW_DEADBAND_PX:
                            error_px = 0

                        if line_block["h"] >= cfg.LINE_FOLLOW_TALL_BLOCK_H:
                            error_px *= cfg.LINE_FOLLOW_TALL_BLOCK_ERROR_SCALE

                        error_derivative = (error_px - last_error_px) / outer_dt
                        last_error_px = error_px

                        turn_rpm_differential = cfg.LINE_FOLLOW_STEER_SIGN * (
                            cfg.Kp_STEER * error_px +
                            cfg.Kd_STEER * error_derivative
                        )

                        turn_rpm_differential = clamp(
                            turn_rpm_differential,
                            -cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM,
                            cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM
                        )

                        target_rpm_left = cfg.LINE_FOLLOW_BASE_RPM + turn_rpm_differential
                        target_rpm_right = cfg.LINE_FOLLOW_BASE_RPM - turn_rpm_differential

                        # no reverse during line follow
                        target_rpm_left = clamp(target_rpm_left, -cfg.LINE_FOLLOW_MAX_RPM, cfg.LINE_FOLLOW_MAX_RPM)
                        target_rpm_right = clamp(target_rpm_right, -cfg.LINE_FOLLOW_MAX_RPM, cfg.LINE_FOLLOW_MAX_RPM)

                # =============================================================
                # INNER LOOP: Motor PI
                # =============================================================
                inner_dt_ms = utime.ticks_diff(now, last_inner_ms)
                if inner_dt_ms >= INNER_INTERVAL_MS:
                    inner_dt = inner_dt_ms / 1000.0
                    last_inner_ms = now

                    encA, encB = self.encoders.read_counts()
                    delta_encA = encA - last_encA
                    delta_encB = encB - last_encB
                    last_encA = encA
                    last_encB = encB

                    cps_left = delta_encB / inner_dt
                    cps_right = delta_encA / inner_dt
                    rpm_left_raw = cps_left * cfg.CPS_TO_RPM
                    rpm_right_raw = cps_right * cfg.CPS_TO_RPM

                    rpm_left_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_filtered)
                    rpm_right_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_filtered)

                    error_left = target_rpm_left - rpm_left_filtered
                    error_right = target_rpm_right - rpm_right_filtered

                    integral_left = clamp(integral_left + error_left * inner_dt, -100.0, 100.0)
                    integral_right = clamp(integral_right + error_right * inner_dt, -100.0, 100.0)

                    if target_rpm_left <= 0:
                        integral_left = 0.0
                    if target_rpm_right <= 0:
                        integral_right = 0.0

                    pwm_left = (cfg.Kp_L * error_left) + (cfg.Ki_L * integral_left)
                    pwm_right = (cfg.Kp_R * error_right) + (cfg.Ki_R * integral_right)

                    pwm_left = clamp(pwm_left, -100.0, 100.0)
                    pwm_right = clamp(pwm_right, -100.0, 100.0)

                    if cfg.USE_MIN_PWM_BOOST:
                        if target_rpm_left > 0 and 0 < pwm_left < cfg.PWM_MIN_MOVE_L:
                            pwm_left = cfg.PWM_MIN_MOVE_L
                        if target_rpm_right > 0 and 0 < pwm_right < cfg.PWM_MIN_MOVE_R:
                            pwm_right = cfg.PWM_MIN_MOVE_R

                    if target_rpm_left <= 0:
                        pwm_left = 0.0
                    if target_rpm_right <= 0:
                        pwm_right = 0.0

                    self.motors.drive_percent(pwm_left, pwm_right)

                utime.sleep_ms(5)

        finally:
            self.motors.coast()
            print("Line follower stopped")

    def stop(self):
        """Immediately stop motors."""
        self.motors.coast()
