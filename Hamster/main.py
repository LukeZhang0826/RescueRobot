import config as cfg

from motors import Motors
from encoders import DualEncoder
from basic_movements import BasicMovements
from button import Button


def main():

    motors = Motors(
        cfg.MOTOR_EN_PIN,
        cfg.LEFT_MOTOR_PINS,
        cfg.RIGHT_MOTOR_PINS,
        cfg.PWM_FREQ,
        cfg.PWM_MAX_RAW
    )

    encoders = DualEncoder(
        cfg.EA_A_PIN,
        cfg.EA_B_PIN,
        cfg.EB_A_PIN,
        cfg.EB_B_PIN,
        cfg.ENC_INV_A,
        cfg.ENC_INV_B
    )

    button = Button(cfg.BUTTON_PIN)

    movement = BasicMovements(motors, encoders, cfg)

    print("Robot ready")
    print("Press button to run movement sequence")

    while True:

        button.wait_for_press()

        print("Running sequence")
        
        movement.pause(0.5)

        movement.move_forward_distance(0.1) 
        movement.pause(0.5)

        # movement.turn_degrees(90)
        # movement.pause(0.5)

        movement.move_forward_distance(0.1) 

        print("Sequence finished")
        print("Press button to run again")


if __name__ == "__main__":
    main()