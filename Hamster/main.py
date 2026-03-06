import config as cfg

from motors import Motors
from encoders import DualEncoder
from basic_movements import BasicMovements
from button import Button
from pixy import PixySPI
from line_follower import LineFollower


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

    pixy = PixySPI(
        cfg.PIXY_SPI_ID,
        cfg.PIXY_CS_PIN,
        cfg.PIXY_SCK_PIN,
        cfg.PIXY_MOSI_PIN,
        cfg.PIXY_MISO_PIN,
        cfg.PIXY_BAUDRATE
    )

    #movement = BasicMovements(motors, encoders, cfg)
    follower = LineFollower(motors, encoders, pixy, cfg)

    print("Robot ready")
    print("Press button to start line following")
    print(f"Duration: {cfg.LINE_FOLLOW_DURATION_S}s")

    while True:

        button.wait_for_press()

        print("Starting line follower...")
        follower.run()

        print("Done. Press button to run again")


if __name__ == "__main__":
    main()	