# main.py
import config as cfg

from button import Button
from logger import RunLogger
from encoders import DualEncoder
from motors import Motors
from pi_speed_test import PISpeedTest

def main():
    motors = Motors(
        en_pin=cfg.MOTOR_EN_PIN,
        left_pins=cfg.LEFT_MOTOR_PINS,
        right_pins=cfg.RIGHT_MOTOR_PINS,
        pwm_freq=cfg.PWM_FREQ,
        pwm_max_raw=cfg.PWM_MAX_RAW
    )

    encoders = DualEncoder(
        ea_a_pin=cfg.EA_A_PIN, ea_b_pin=cfg.EA_B_PIN,
        eb_a_pin=cfg.EB_A_PIN, eb_b_pin=cfg.EB_B_PIN,
        enc_inv_a=cfg.ENC_INV_A, enc_inv_b=cfg.ENC_INV_B
    )

    button = Button(cfg.BUTTON_PIN)
    logger = RunLogger(cfg.CSV_FOLDER)

    print("MOTOR TEST (Pure PI Control) - Press button to start")
    print("Target: {} RPM  Duration: {}s".format(cfg.TARGET_RPM, cfg.TEST_DURATION_S))
    print("Left:  Kp={}  Ki={}".format(cfg.Kp_L, cfg.Ki_L))
    print("Right: Kp={}  Ki={}".format(cfg.Kp_R, cfg.Ki_R))
    print("Min PWM boost: L={} R={} (enabled={})".format(
        cfg.PWM_MIN_MOVE_L, cfg.PWM_MIN_MOVE_R, cfg.USE_MIN_PWM_BOOST
    ))
    print("Logging enabled: {}".format(cfg.ENABLE_LOGGING))

    tester = PISpeedTest(motors, encoders, cfg)

    while True:
        run_number = logger.next_run_number()
        print("Run #{} ready - press button".format(run_number))
        button.wait_for_press()

        print("Running...")
        data = tester.run()

        if cfg.ENABLE_LOGGING and len(data) > 0:
            filename = logger.save_csv(run_number, cfg.TARGET_RPM, data)
            print("Saved: {}".format(filename))

            avg_l = sum(r[2] for r in data) / len(data)
            avg_r = sum(r[3] for r in data) / len(data)
            print("Avg RPM: L={:.1f}  R={:.1f}".format(avg_l, avg_r))

        print("Done. Press button for next run.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # safest stop
        try:
            from motors import Motors
            import config as cfg
            Motors(cfg.MOTOR_EN_PIN, cfg.LEFT_MOTOR_PINS, cfg.RIGHT_MOTOR_PINS, cfg.PWM_FREQ, cfg.PWM_MAX_RAW).stop()
        except Exception:
            pass
        print("Stopped")