# robot_mission.py
import utime


class RobotMission:
    STATE_IDLE = "idle"
    STATE_SEARCH_LINE_FOLLOW = "search_line_follow"
    STATE_APPROACH_TARGET = "approach_target"
    STATE_PICKUP = "pickup"
    STATE_HOLDING_OBJECT = "holding_object"
    STATE_DONE = "done"
    STATE_ERROR = "error"

    def __init__(self, button, follower, target_approach, pickup):
        self.button = button
        self.follower = follower
        self.target_approach = target_approach
        self.pickup = pickup
        self.state = self.STATE_IDLE

    def reset(self):
        self.state = self.STATE_IDLE

    def run_once(self):
        self.state = self.STATE_IDLE
        print("Mission ready. Waiting for button.")

        while True:
            if self.state == self.STATE_IDLE:
                self.button.wait_for_press()
                print("Button pressed")
                self.state = self.STATE_SEARCH_LINE_FOLLOW

            elif self.state == self.STATE_SEARCH_LINE_FOLLOW:
                print("[STATE] SEARCH_LINE_FOLLOW")
                result = self.follower.run()

                if result == "stop_target":
                    self.state = self.STATE_APPROACH_TARGET
                else:
                    print("Line follow failed with result:", result)
                    self.state = self.STATE_ERROR

            elif self.state == self.STATE_APPROACH_TARGET:
                print("[STATE] APPROACH_TARGET")
                success = self.target_approach.run()

                if success:
                    self.state = self.STATE_PICKUP
                else:
                    print("Target approach failed")
                    self.state = self.STATE_ERROR

            elif self.state == self.STATE_PICKUP:
                print("[STATE] PICKUP")

                self.pickup.initialize()
                success = self.pickup.run_pickup_sequence()

                if success:
                    self.state = self.STATE_HOLDING_OBJECT
                else:
                    self.state = self.STATE_ERROR

            elif self.state == self.STATE_HOLDING_OBJECT:
                print("[STATE] HOLDING_OBJECT")
                self.pickup.hold_object()
                print("Mission segment complete")
                self.state = self.STATE_DONE

            elif self.state == self.STATE_DONE:
                print("Done. Holding object. Press button to run again.")
                return True

            elif self.state == self.STATE_ERROR:
                print("Mission error. Press button to retry.")
                return False

            utime.sleep_ms(20)