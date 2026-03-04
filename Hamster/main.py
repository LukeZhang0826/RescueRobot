from machine import Pin
import utime

from steeringControl import SteeringControl
from drive import DriveOpenLoop
from pixy import PixySPI
from servoControl import Servo
from states import Context, Init

# -----------------------------
# START/STOP BUTTON (GPIO4)
# -----------------------------
button = Pin(4, Pin.IN, Pin.PULL_UP)
running = False
last_btn = 1

# -----------------------------
# HARDWARE SETUP
# -----------------------------
steer = SteeringControl(enable_pin=5, left_pins=(6,7), right_pins=(8,9), pwm_freq=1000)
pixy = PixySPI()

drive = DriveOpenLoop(
    steering_control=steer,
    base_pwm=14000,
    turn_gain_pwm=55,
    max_pwm=18000,
    slew_per_sec=45000
)

claw_servo = Servo(pwm_pin=14, power_pin=15, us_min=1000, us_max=2000)
lift_servo = Servo(pwm_pin=1, power_pin=0, us_min=1000, us_max=2000)

ctx = Context(drive=drive, pixy=pixy, claw_servo=claw_servo, lift_servo=lift_servo)

state = Init()
state.enter(ctx)

# -----------------------------
# MAIN LOOP
# -----------------------------
TICK_MS = 20
next_tick = utime.ticks_ms()

while True:

    # -----------------------------
    # BUTTON TOGGLE
    # -----------------------------
    btn = button.value()

    if last_btn == 1 and btn == 0:   # falling edge = press
        running = not running

        if running:
            print("Robot STARTED")
        else:
            print("Robot STOPPED")
            ctx.drive.stop()

        utime.sleep_ms(200)  # debounce

    last_btn = btn

    # -----------------------------
    # STATE MACHINE ONLY IF RUNNING
    # -----------------------------
    if not running:
        utime.sleep_ms(10)
        continue

    now = utime.ticks_ms()

    if utime.ticks_diff(now, next_tick) < 0:
        continue

    next_tick = utime.ticks_add(next_tick, TICK_MS)

    ctx.now_ms = now
    nxt = state.tick(ctx)

    if nxt is not state:
        state.exit(ctx)
        nxt.enter(ctx)
        state = nxt