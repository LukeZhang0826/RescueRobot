# main.py
import config as cfg

from motors import Motors
from encoders import DualEncoder
from basic_movements import BasicMovements
from button import Button
from pixy import PixySPI
from search_line_follower import SearchLineFollower
from rescue_line_follower import RescueLineFollower
from return_line_follow import ReturnLineFollower
from target_approach import TargetApproach
from claw_controller import ClawController
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
    search_follower = SearchLineFollower(motors, encoders, pixy, mover, cfg)
    rescue_follower = RescueLineFollower(motors, encoders, pixy, mover, cfg)
    return_follower = ReturnLineFollower(motors, encoders, pixy, mover, cfg)
    target_approach = TargetApproach(pixy, mover, cfg)
    claw = ClawController(cfg)

    mission = RobotMission(
        button=button,
        search_follower=search_follower,
        rescue_follower=rescue_follower,
        return_follower=return_follower,
        target_approach=target_approach,
        claw=claw,
        mover=mover,
        cfg=cfg
    )
    
    print("Robot ready")

    while True:
        mission.run_once()


if __name__ == "__main__":
    main()