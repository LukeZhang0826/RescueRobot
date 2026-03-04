# states.py
import utime

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

class Context:
    def __init__(self, drive, pixy, claw_servo, lift_servo):
        self.drive = drive
        self.pixy = pixy

        self.claw = claw_servo
        self.lift = lift_servo

        self.now_ms = utime.ticks_ms()

        # shared “mission” variables
        self.last_seen_ms = 0
        self.center_x = 316 // 2

class State:
    name = "STATE"

    def enter(self, ctx: Context):
        pass

    def tick(self, ctx: Context):
        """
        Return next state object (or self to stay).
        """
        return self

    def exit(self, ctx: Context):
        pass

class Init(State):
    name = "INIT"

    def enter(self, ctx):
        ctx.drive.stop()
        self.t0 = ctx.now_ms

    def tick(self, ctx):
        if utime.ticks_diff(ctx.now_ms, self.t0) > 300:
            return FollowLine()   # not FollowLine()
        return self

class FollowLine(State):
    name = "FOLLOW_LINE"

    # tuning knobs (start conservative)
    AREA_MIN = 2000
    DEADBAND_PX = 20

    # open-loop “speed”
    V_FWD = 0.7             # 0..1
    TURN_PX_GAIN = 0.02    # turn units per pixel error
    TURN_MAX = 8.0          # clamp

    LOST_TIMEOUT_MS = 200   # if no blob for this long -> stop

    def enter(self, ctx):
        ctx.last_seen_ms = ctx.now_ms

    def tick(self, ctx):
        b = ctx.pixy.best_block(area_min=self.AREA_MIN)

        if b is None:
            # lost target
            if utime.ticks_diff(ctx.now_ms, ctx.last_seen_ms) > self.LOST_TIMEOUT_MS:
                ctx.drive.stop()
            return self

        ctx.last_seen_ms = ctx.now_ms

        err = b["x"] - ctx.center_x
        if -self.DEADBAND_PX <= err <= self.DEADBAND_PX:
            err = 0

        turn = clamp(self.TURN_PX_GAIN * err, -self.TURN_MAX, self.TURN_MAX)

        # optional: slow down on hard turns
        v = self.V_FWD * (1.0 - 0.35 * min(1.0, abs(turn) / self.TURN_MAX))
        v = clamp(v, 0.0, 1.0)

        ctx.drive.set_forward_turn(v, turn)

        # placeholder transition conditions (later)
        # if reached_goal(): return Pickup()
        return self

class Pickup(State):
    name = "PICKUP"
    def enter(self, ctx):
        ctx.drive.stop()
        self.t0 = ctx.now_ms

    def tick(self, ctx):
        # TODO: servo actions etc.
        if utime.ticks_diff(ctx.now_ms, self.t0) > 1000:
            return FollowLine()   # or next state
        return self
    
class ServoTest(State):
    name = "SERVO_TEST"

    # YOU MUST TUNE THESE ANGLES FOR YOUR MECHANISM
    CLAW_OPEN = 180
    CLAW_CLOSED = 0
    LIFT_UP = 180
    LIFT_DOWN = 0

    def enter(self, ctx):
        ctx.drive.stop()
        ctx.claw.power_on()
        ctx.lift.power_on()
        self.step = 0
        self.t_step = ctx.now_ms

    def tick(self, ctx):
        t = ctx.now_ms

        # step machine with timed transitions (no sleep())
        if self.step == 0:
            ctx.lift.angle(self.LIFT_UP)
            self.step, self.t_step = 1, t
            return self

        if self.step == 1 and utime.ticks_diff(t, self.t_step) > 600:
            ctx.claw.angle(self.CLAW_OPEN)
            self.step, self.t_step = 2, t
            return self

        if self.step == 2 and utime.ticks_diff(t, self.t_step) > 600:
            ctx.lift.angle(self.LIFT_DOWN)
            self.step, self.t_step = 3, t
            return self

        if self.step == 3 and utime.ticks_diff(t, self.t_step) > 800:
            ctx.claw.angle(self.CLAW_CLOSED)
            self.step, self.t_step = 4, t
            return self

        if self.step == 4 and utime.ticks_diff(t, self.t_step) > 800:
            ctx.lift.angle(self.LIFT_UP)
            self.step, self.t_step = 5, t
            return self

        if self.step == 5 and utime.ticks_diff(t, self.t_step) > 800:
            # Done: hold position or power off
            # ctx.claw.detach(); ctx.lift.detach()
            # ctx.claw.power_off(); ctx.lift.power_off()
            return FollowLine()   # or return self to keep holding

        return self

    def exit(self, ctx):
        # keep powered if you want holding torque
        pass