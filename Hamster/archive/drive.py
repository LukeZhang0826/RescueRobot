# drive.py
import utime

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class DriveBase:
    """
    Interface the state machine uses.
    Later you can replace DriveOpenLoop with DriveClosedLoop (PI + encoders)
    without changing any state logic.
    """
    def set_forward_turn(self, v_fwd, turn):
        """
        v_fwd: forward command (arbitrary units for now)
        turn:  turn command (arbitrary units, positive = turn right)
        """
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

class DriveOpenLoop(DriveBase):
    """
    Maps (v_fwd, turn) to (left_pwm, right_pwm) and applies slew limiting.
    Uses your SteeringControl (PWM to DRV8833).
    """
    def __init__(
        self,
        steering_control,
        base_pwm=16000,
        turn_gain_pwm=30,     # pwm per unit of `turn`
        min_pwm=0,
        max_pwm=30000,
        slew_per_sec=40000   # max duty_u16 change per second
    ):
        self.steer = steering_control
        self.base_pwm = base_pwm
        self.turn_gain_pwm = turn_gain_pwm
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm

        self.slew_per_sec = slew_per_sec
        self._l_cmd = 0
        self._r_cmd = 0
        self._last_ms = utime.ticks_ms()

    def _slew(self, current, target, dt):
        max_step = int(self.slew_per_sec * dt)
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def set_forward_turn(self, v_fwd, turn):
        # v_fwd is [0..1] ideally; clamp just to be safe
        v_fwd = clamp(v_fwd, 0.0, 1.0)

        base = int(self.base_pwm * v_fwd)
        delta = int(self.turn_gain_pwm * turn)

        l_t = base + delta
        r_t = base - delta

        # clamp
        l_t = clamp(l_t, -self.max_pwm, self.max_pwm)
        r_t = clamp(r_t, -self.max_pwm, self.max_pwm)

        # per-wheel minimum to avoid buzz zone (only when moving forward)
        MIN_MOVE = 0
        if v_fwd > 0.05:
            # forward-only floor
            if l_t > 0: l_t = max(l_t, MIN_MOVE)
            if r_t > 0: r_t = max(r_t, MIN_MOVE)

        # apply slew limiting
        now = utime.ticks_ms()
        dt = utime.ticks_diff(now, self._last_ms) / 1000.0
        self._last_ms = now

        self._l_cmd = self._slew(self._l_cmd, l_t, dt)
        self._r_cmd = self._slew(self._r_cmd, r_t, dt)

        # If you only want forward right now, clamp negatives to 0:
        # self._l_cmd = max(0, self._l_cmd)
        # self._r_cmd = max(0, self._r_cmd)

        self.steer.drive(self._l_cmd, self._r_cmd)

    def stop(self):
        self.steer.stop()
        self._l_cmd = 0
        self._r_cmd = 0
        self._last_ms = utime.ticks_ms()