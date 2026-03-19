# main.py
import config as cfg

from motors import Motors
from encoders import DualEncoder
from basic_movements import BasicMovements
from button import Button
from pixy import PixySPI
from search_line_follower import SearchLineFollower
from target_approach import TargetApproach
from pickup_controller import PickupController
from robot_mission import RobotMission


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

    mover = BasicMovements(motors, encoders, cfg)
    follower = SearchLineFollower(motors, encoders, pixy, cfg)
    target_approach = TargetApproach(pixy, mover, cfg)
    pickup = PickupController(cfg)

    mission = RobotMission(
        button=button,
        follower=follower,
        target_approach=target_approach,
        pickup=pickup
    )

    print("Robot ready")

    while True:
        mission.run_once()


if __name__ == "__main__":
    main()