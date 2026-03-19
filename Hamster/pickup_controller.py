# pickup_controller.py
import utime
from servo import Servo


class PickupController:
    def __init__(self, cfg):
        self.cfg = cfg

        self.claw = Servo(
            pwm_pin=cfg.CLAW_PWM_PIN,
            pwr_pin=cfg.CLAW_PWR_PIN,
            hz=cfg.SERVO_HZ,
            us_min=cfg.SERVO_US_MIN,
            us_max=cfg.SERVO_US_MAX,
        )

        self.lift = Servo(
            pwm_pin=cfg.LIFT_PWM_PIN,
            pwr_pin=cfg.LIFT_PWR_PIN,
            hz=cfg.SERVO_HZ,
            us_min=cfg.SERVO_US_MIN,
            us_max=cfg.SERVO_US_MAX,
        )

        self.holding = False
        self.initialized = False

    def initialize(self):
        if self.initialized:
            return

        print("Enabling servo power...")
        self.claw.power_on()
        self.lift.power_on()
        utime.sleep_ms(self.cfg.SERVO_POWER_ON_DELAY_MS)
        self.initialized = True

    def open_claw(self):
        print("CLAW OPEN =", self.cfg.CLAW_OPEN)
        self.claw.set_deg(self.cfg.CLAW_OPEN)
        utime.sleep_ms(self.cfg.T_CLAW_MS)
        utime.sleep_ms(self.cfg.T_SETTLE_MS)

    def close_claw(self):
        print("CLAW CLOSE =", self.cfg.CLAW_CLOSED)
        self.claw.set_deg(self.cfg.CLAW_CLOSED)
        utime.sleep_ms(self.cfg.T_CLAW_MS)
        utime.sleep_ms(self.cfg.T_SETTLE_MS)

    def lower_lift(self):
        print("LIFT DOWN =", self.cfg.LIFT_DOWN)
        self.lift.set_deg(self.cfg.LIFT_DOWN)
        utime.sleep_ms(self.cfg.T_LIFT_MS)
        utime.sleep_ms(self.cfg.T_SETTLE_MS)

    def raise_lift(self):
        print("LIFT UP =", self.cfg.LIFT_UP)
        self.lift.set_deg(self.cfg.LIFT_UP)
        utime.sleep_ms(self.cfg.T_LIFT_MS)
        utime.sleep_ms(self.cfg.T_SETTLE_MS)

    def move_to_hold_pose(self):
        self.claw.set_deg(self.cfg.CLAW_CLOSED)
        self.lift.set_deg(self.cfg.LIFT_UP)

    def run_pickup_sequence(self):
        self.initialize()

        print("Pickup sequence start")
        self.open_claw()
        self.lower_lift()
        self.close_claw()
        self.raise_lift()

        self.holding = True
        print("Pickup sequence complete")
        return True

    def hold_object(self):
        if not self.holding:
            return

        # Re-command final pose so servos keep applying torque.
        self.move_to_hold_pose()

    def release_and_power_off(self):
        self.holding = False
        self.claw.detach()
        self.lift.detach()
        self.claw.power_off()
        self.lift.power_off()
        self.initialized = False