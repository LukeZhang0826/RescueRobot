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
        Run line following for cfg.LINE_FOLLOW_DURATION_S seconds.
        
        Outer PD loop runs at 50Hz (20ms interval).
        Inner PI loop runs at 50Hz (20ms interval).
        """
        cfg = self.cfg
        
        # Timing intervals
        OUTER_INTERVAL_MS = 20   # 50Hz for camera/PD (20/1000 = 50Hz)
        INNER_INTERVAL_MS = 20   # 50Hz for motor PI  (20/1000 = 50Hz)
        
        # Inner loop PI state
        integral_left = 0.0
        integral_right = 0.0
        
        # Outer loop PD state
        last_error_px = 0.0
        
        # Speed measurements (filtered)
        rpm_left_filtered = 0.0
        rpm_right_filtered = 0.0
        
        # Current targets (RPM)
        target_rpm_left = 0.0
        target_rpm_right = 0.0
        
        # Current PWM outputs
        pwm_left = 0.0
        pwm_right = 0.0
        
        # Encoder tracking
        self.encoders.reset()
        last_encA, last_encB = self.encoders.read_counts()
        
        # Timing
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
                
                # Check stop condition
                if elapsed_s >= cfg.LINE_FOLLOW_DURATION_S:
                    print("Duration reached - stopping")
                    break
                
                # =============================================================
                # OUTER LOOP: Camera PD (50Hz)
                # =============================================================
                outer_dt_ms = utime.ticks_diff(now, last_outer_ms)
                if outer_dt_ms >= OUTER_INTERVAL_MS:
                    outer_dt = outer_dt_ms / 1000.0
                    last_outer_ms = now
                    
                    # Get camera reading
                    block = self.pixy.best_block(
                        sigmap=cfg.PIXY_SIGNATURE_1,
                        area_min=cfg.PIXY_AREA_MIN
                    )
                    
                    if block is None:
                        # No line detected - stop motors immediately
                        target_rpm_left = 0.0
                        target_rpm_right = 0.0
                        integral_left = 0.0
                        integral_right = 0.0
                        last_error_px = 0.0
                    else:
                        # Calculate pixel error (positive = line is to the right)
                        error_px = block["x"] - cfg.PIXY_CENTER_X
                        
                        # Apply deadband
                        if abs(error_px) <= cfg.LINE_FOLLOW_DEADBAND_PX:
                            error_px = 0

                        # if the block is very tall, it means it's seeing 'too much" of the line, which can cause erratic behavior. In this case, we can scale down the error to prevent overreacting.
                        if block["h"] >= cfg.LINE_FOLLOW_TALL_BLOCK_H:
                            error_px *= cfg.LINE_FOLLOW_TALL_BLOCK_ERROR_SCALE

                        # PD control: compute turn differential (in RPM)
                        error_derivative = (error_px - last_error_px) / outer_dt
                        last_error_px = error_px
                        
                        turn_rpm_differential = cfg.LINE_FOLLOW_STEER_SIGN * (
                            cfg.Kp_STEER * error_px +
                            cfg.Kd_STEER * error_derivative
                        )
                        
                        # Clamp turn differential
                        turn_rpm_differential = clamp(turn_rpm_differential, -cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM, cfg.LINE_FOLLOW_MAX_DIFFERENTIAL_RPM)
                        
                        # Differential drive: base speed +/- turn
                        # Positive error (line right) -> turn right -> left faster, right slower
                        target_rpm_left = cfg.LINE_FOLLOW_BASE_RPM + turn_rpm_differential
                        target_rpm_right = cfg.LINE_FOLLOW_BASE_RPM - turn_rpm_differential
                        
                        # Clamp targets to valid range (no reverse)
                        target_rpm_left = clamp(target_rpm_left, -cfg.LINE_FOLLOW_MAX_RPM, cfg.LINE_FOLLOW_MAX_RPM)
                        target_rpm_right = clamp(target_rpm_right, -cfg.LINE_FOLLOW_MAX_RPM, cfg.LINE_FOLLOW_MAX_RPM)
                
                # =============================================================
                # INNER LOOP: Motor PI (50Hz)
                # =============================================================
                inner_dt_ms = utime.ticks_diff(now, last_inner_ms)
                if inner_dt_ms >= INNER_INTERVAL_MS:
                    inner_dt = inner_dt_ms / 1000.0
                    last_inner_ms = now
                    
                    # Read encoders
                    encA, encB = self.encoders.read_counts()
                    delta_encA = encA - last_encA
                    delta_encB = encB - last_encB
                    last_encA = encA
                    last_encB = encB
                    
                    # Calculate RPM (same mapping as existing code)
                    cps_left = delta_encB / inner_dt
                    cps_right = delta_encA / inner_dt
                    rpm_left_raw = cps_left * cfg.CPS_TO_RPM
                    rpm_right_raw = cps_right * cfg.CPS_TO_RPM
                    
                    # Low-pass filter
                    rpm_left_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_left_raw - rpm_left_filtered)
                    rpm_right_filtered += cfg.SPEED_FILTER_ALPHA * (rpm_right_raw - rpm_right_filtered)
                    
                    # Compute speed errors
                    error_left = target_rpm_left - rpm_left_filtered
                    error_right = target_rpm_right - rpm_right_filtered
                    
                    # Update integrals (with anti-windup)
                    integral_left = clamp(integral_left + error_left * inner_dt, -100.0, 100.0)
                    integral_right = clamp(integral_right + error_right * inner_dt, -100.0, 100.0)
                    
                    # Reset integral if target is zero
                    if target_rpm_left <= 0:
                        integral_left = 0.0
                    if target_rpm_right <= 0:
                        integral_right = 0.0
                    
                    # PI control: compute PWM
                    pwm_left = (cfg.Kp_L * error_left) + (cfg.Ki_L * integral_left)
                    pwm_right = (cfg.Kp_R * error_right) + (cfg.Ki_R * integral_right)
                    
                    # Clamp PWM
                    pwm_left = clamp(pwm_left, -100.0, 100.0)
                    pwm_right = clamp(pwm_right, -100.0, 100.0)
                    
                    # Minimum PWM boost for deadzone
                    if cfg.USE_MIN_PWM_BOOST:
                        if target_rpm_left > 0 and 0 < pwm_left < cfg.PWM_MIN_MOVE_L:
                            pwm_left = cfg.PWM_MIN_MOVE_L
                        if target_rpm_right > 0 and 0 < pwm_right < cfg.PWM_MIN_MOVE_R:
                            pwm_right = cfg.PWM_MIN_MOVE_R
                    
                    # Force zero if target is zero
                    if target_rpm_left <= 0:
                        pwm_left = 0.0
                    if target_rpm_right <= 0:
                        pwm_right = 0.0
                    
                    # Apply PWM
                    self.motors.drive_percent(pwm_left, pwm_right)
                
                utime.sleep_ms(5)
        
        finally:
            self.motors.stop()
            print("Line follower stopped")

    def stop(self):
        """Immediately stop motors."""
        self.motors.stop()
