# # main_basic_motor.py
# import utime
# import config as cfg

# from motors import Motors
# from encoders import DualEncoder
# from button import Button
# from basic_movements import BasicMovements


# def main():
#     motors = Motors(
#         en_pin=cfg.MOTOR_EN_PIN,
#         left_pins=cfg.LEFT_MOTOR_PINS,
#         right_pins=cfg.RIGHT_MOTOR_PINS,
#         pwm_freq=cfg.PWM_FREQ,
#         pwm_max_raw=cfg.PWM_MAX_RAW
#     )

#     encoders = DualEncoder(
#         ea_a_pin=cfg.EA_A_PIN,
#         ea_b_pin=cfg.EA_B_PIN,
#         eb_a_pin=cfg.EB_A_PIN,
#         eb_b_pin=cfg.EB_B_PIN,
#         enc_inv_a=cfg.ENC_INV_A,
#         enc_inv_b=cfg.ENC_INV_B
#     )

#     button = Button(cfg.BUTTON_PIN)

#     mover = BasicMovements(
#         motors=motors,
#         encoders=encoders,
#         max_percent=35.0,
#         left_fwd_min=16.0,
#         left_rev_min=22.0,
#         right_fwd_min=16.0,
#         right_rev_min=22.0,
#     )

#     print("Basic motor test ready.")
#     print("Press button to run forward + left turn + right turn.")

#     try:
#         while True:
#             button.wait_for_press()

#             print("Running move_forward(100)")
#             mover.move_forward(100, base_percent=18.0, debug=True)
#             utime.sleep_ms(500)

#             print("Running turn_left_counts(40)")
#             mover.turn_left_counts(80, base_percent=22.0, debug=True)
#             utime.sleep_ms(500)

#             print("Running turn_right_counts(40)")
#             mover.turn_right_counts(80, base_percent=22.0, debug=True)
#             utime.sleep_ms(500)

#             print("Running move_backward(100)")
#             mover.move_backward(100, base_percent=18.0, debug=True)
#             utime.sleep_ms(500)

#             print("Done\n")

#     finally:
#         motors.stop()


# if __name__ == "__main__":
#     main()