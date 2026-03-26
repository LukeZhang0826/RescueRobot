import utime
from utils import clamp


class BasicMovements:
    """
    Basic robot movement primitives using encoder counts.

    Assumptions:
      - left wheel progress comes from encB
      - right wheel progress comes from encA

    Motor commands use signed percent:
      + = forward
      - = reverse
    """

    def __init__(self, motors, encoders, cfg):
        self.motors = motors
        self.encoders = encoders
        self.cfg = cfg

        self.max_percent = cfg.BASIC_MAX_PERCENT

        self.left_fwd_min = cfg.LEFT_FWD_MIN
        self.left_rev_min = cfg.LEFT_REV_MIN
        self.right_fwd_min = cfg.RIGHT_FWD_MIN
        self.right_rev_min = cfg.RIGHT_REV_MIN

    def stop(self, use_brake=True, brake_ms=None):
        if use_brake:
            if brake_ms is None:
                brake_ms = self.cfg.BASIC_BRAKE_MS
            self.motors.stop(brake_ms=brake_ms)
        else:
            self.motors.coast()

    def _read_wheels(self):
        encA, encB = self.encoders.read_counts()
        left_counts = encB
        right_counts = encA
        return left_counts, right_counts

    def _apply_deadzone(self, left_cmd, right_cmd):
        """
        Enforce different minimum magnitudes depending on wheel + direction.
        """
        if left_cmd > 0:
            left_cmd = max(left_cmd, self.left_fwd_min)
        elif left_cmd < 0:
            left_cmd = -max(abs(left_cmd), self.left_rev_min)

        if right_cmd > 0:
            right_cmd = max(right_cmd, self.right_fwd_min)
        elif right_cmd < 0:
            right_cmd = -max(abs(right_cmd), self.right_rev_min)

        left_cmd = clamp(left_cmd, -self.max_percent, self.max_percent)
        right_cmd = clamp(right_cmd, -self.max_percent, self.max_percent)

        return left_cmd, right_cmd

    def move_forward(self, target_counts, base_percent=22.0, timeout_s=5.0, debug=False):
        target_counts = int(target_counts)
        if target_counts <= 0:
            self.stop()
            return

        base_percent = clamp(base_percent, self.left_fwd_min, self.max_percent)

        SYNC_KP = 0.35
        FINISH_SLOWDOWN = 20

        self.encoders.reset()
        utime.sleep_ms(20)

        left_done = False
        right_done = False
        start_ms = utime.ticks_ms()

        try:
            while True:
                now_ms = utime.ticks_ms()
                elapsed_ms = utime.ticks_diff(now_ms, start_ms)
                if elapsed_ms >= int(timeout_s * 1000):
                    if debug:
                        print("move_forward timeout")
                    break

                left_counts, right_counts = self._read_wheels()

                if (not left_done) and (left_counts >= target_counts):
                    left_done = True
                    if debug:
                        print("Left reached target")

                if (not right_done) and (right_counts >= target_counts):
                    right_done = True
                    if debug:
                        print("Right reached target")

                if left_done and right_done:
                    break

                left_remaining = max(0, target_counts - left_counts)
                right_remaining = max(0, target_counts - right_counts)

                left_base = base_percent
                right_base = base_percent

                if left_remaining < FINISH_SLOWDOWN:
                    frac = left_remaining / FINISH_SLOWDOWN
                    left_base = self.left_fwd_min + frac * (base_percent - self.left_fwd_min)

                if right_remaining < FINISH_SLOWDOWN:
                    frac = right_remaining / FINISH_SLOWDOWN
                    right_base = self.right_fwd_min + frac * (base_percent - self.right_fwd_min)

                error = left_counts - right_counts
                correction = SYNC_KP * error

                left_cmd = 0.0 if left_done else (left_base - correction)
                right_cmd = 0.0 if right_done else (right_base + correction)

                if not left_done or not right_done:
                    left_cmd, right_cmd = self._apply_deadzone(left_cmd, right_cmd)

                self.motors.drive_percent(left_cmd, right_cmd)

                if debug:
                    print(
                        "FWD | L={}, R={}, tgt={}, err={}, cmdL={:.1f}, cmdR={:.1f}".format(
                            left_counts, right_counts, target_counts, error, left_cmd, right_cmd
                        )
                    )

                utime.sleep_ms(10)

        finally:
            self.stop(use_brake=True)

    def turn_left_counts(self, target_counts, base_percent=20.0, timeout_s=5.0, debug=False):
        self._turn_in_place(
            target_counts=target_counts,
            left_sign=-1.0,
            right_sign=+1.0,
            base_percent=base_percent,
            timeout_s=timeout_s,
            debug=debug,
            label="LEFT"
        )

    def turn_right_counts(self, target_counts, base_percent=20.0, timeout_s=5.0, debug=False):
        self._turn_in_place(
            target_counts=target_counts,
            left_sign=+1.0,
            right_sign=-1.0,
            base_percent=base_percent,
            timeout_s=timeout_s,
            debug=debug,
            label="RIGHT"
        )

    def _turn_in_place(self, target_counts, left_sign, right_sign, base_percent, timeout_s, debug, label):
        target_counts = int(target_counts)
        if target_counts <= 0:
            self.stop()
            return

        min_turn_percent = max(
            self.left_fwd_min,
            self.left_rev_min,
            self.right_fwd_min,
            self.right_rev_min
        )

        base_percent = clamp(base_percent, min_turn_percent, self.max_percent)

        TURN_SYNC_KP = 0.30
        FINISH_SLOWDOWN = 20

        self.encoders.reset()
        utime.sleep_ms(20)

        start_ms = utime.ticks_ms()

        try:
            while True:
                now_ms = utime.ticks_ms()
                elapsed_ms = utime.ticks_diff(now_ms, start_ms)
                if elapsed_ms >= int(timeout_s * 1000):
                    if debug:
                        print("turn_{} timeout".format(label.lower()))
                    break

                left_counts, right_counts = self._read_wheels()

                left_mag = abs(left_counts)
                right_mag = abs(right_counts)

                progress = (left_mag + right_mag) / 2.0
                if progress >= target_counts:
                    break

                remaining = max(0.0, target_counts - progress)

                turn_base = base_percent
                if remaining < FINISH_SLOWDOWN:
                    frac = remaining / FINISH_SLOWDOWN
                    turn_base = min_turn_percent + frac * (base_percent - min_turn_percent)

                error = left_mag - right_mag
                correction = TURN_SYNC_KP * error

                left_mag_cmd = turn_base - correction
                right_mag_cmd = turn_base + correction

                left_cmd = left_sign * left_mag_cmd
                right_cmd = right_sign * right_mag_cmd

                left_cmd, right_cmd = self._apply_deadzone(left_cmd, right_cmd)

                self.motors.drive_percent(left_cmd, right_cmd)

                if debug:
                    print(
                        "TURN_{} | L={}, R={}, Lmag={}, Rmag={}, prog={:.1f}/{}, err={}, cmdL={:.1f}, cmdR={:.1f}".format(
                            label,
                            left_counts,
                            right_counts,
                            left_mag,
                            right_mag,
                            progress,
                            target_counts,
                            error,
                            left_cmd,
                            right_cmd
                        )
                    )

                utime.sleep_ms(10)

        finally:
            self.stop(use_brake=True, brake_ms=self.cfg.TURN_BRAKE_MS)

    def move_backward(self, target_counts, base_percent=22.0, timeout_s=5.0, debug=False):
        target_counts = int(target_counts)
        if target_counts <= 0:
            self.stop()
            return

        base_percent = clamp(base_percent, self.left_rev_min, self.max_percent)

        SYNC_KP = 0.35
        FINISH_SLOWDOWN = 20

        self.encoders.reset()
        utime.sleep_ms(20)

        left_done = False
        right_done = False
        start_ms = utime.ticks_ms()

        try:
            while True:
                now_ms = utime.ticks_ms()
                elapsed_ms = utime.ticks_diff(now_ms, start_ms)
                if elapsed_ms >= int(timeout_s * 1000):
                    if debug:
                        print("move_backward timeout")
                    break

                left_counts, right_counts = self._read_wheels()

                if (not left_done) and (abs(left_counts) >= target_counts):
                    left_done = True
                    if debug:
                        print("Left reached target")

                if (not right_done) and (abs(right_counts) >= target_counts):
                    right_done = True
                    if debug:
                        print("Right reached target")

                if left_done and right_done:
                    break

                left_remaining = max(0, target_counts - abs(left_counts))
                right_remaining = max(0, target_counts - abs(right_counts))

                left_base = base_percent
                right_base = base_percent

                if left_remaining < FINISH_SLOWDOWN:
                    frac = left_remaining / FINISH_SLOWDOWN
                    left_base = self.left_rev_min + frac * (base_percent - self.left_rev_min)

                if right_remaining < FINISH_SLOWDOWN:
                    frac = right_remaining / FINISH_SLOWDOWN
                    right_base = self.right_rev_min + frac * (base_percent - self.right_rev_min)

                error = left_counts - right_counts
                correction = SYNC_KP * error

                left_cmd = 0.0 if left_done else -(left_base - correction)
                right_cmd = 0.0 if right_done else -(right_base + correction)

                if not left_done or not right_done:
                    left_cmd, right_cmd = self._apply_deadzone(left_cmd, right_cmd)

                self.motors.drive_percent(left_cmd, right_cmd)

                if debug:
                    print(
                        "BACK | L={}, R={}, tgt={}, err={}, cmdL={:.1f}, cmdR={:.1f}".format(
                            left_counts, right_counts, target_counts, error, left_cmd, right_cmd
                        )
                    )

                utime.sleep_ms(10)

        finally:
            self.stop(use_brake=True)