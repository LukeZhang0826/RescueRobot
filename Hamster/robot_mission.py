# robot_mission.py
import utime


class RobotMission:
    STATE_IDLE = "idle"
    STATE_SEARCH_LINE_FOLLOW = "search_line_follow"
    STATE_APPROACH_TARGET = "approach_target"
    STATE_PICKUP = "pickup"
    STATE_HOLDING_OBJECT = "holding_object"
    STATE_TURN_AROUND = "turn_around"
    STATE_RESCUE_LINE_FOLLOW = "rescue_line_follow"
    STATE_APPROACH_SAFE_ZONE = "approach_safe_zone"
    STATE_RELEASE = "release"
    STATE_FIND_LINE = "find_line"
    STATE_RETURN_LINE_FOLLOW = "return_line_follow"
    STATE_DONE = "done"
    STATE_ERROR = "error"

    def __init__(self, button, search_follower, rescue_follower, return_follower, target_approach, claw, mover, cfg):
        self.button = button
        self.search_follower = search_follower
        self.rescue_follower = rescue_follower
        self.return_follower = return_follower
        self.target_approach = target_approach
        self.claw = claw
        self.mover = mover
        self.cfg = cfg
        self.state = self.STATE_IDLE

    def reset(self):
        self.state = self.STATE_IDLE

    def run_once(self):
        self.state = self.STATE_IDLE
        print("Mission ready. Waiting for button.")

        while True:
            # Waits for button press to start, then runs through the mission states. After completion or error, returns to idle state for next run.
            if self.state == self.STATE_IDLE:
                self.button.wait_for_press()
                print("Button pressed")
                self.state = self.STATE_SEARCH_LINE_FOLLOW

            # In the line follow state, we run the line follower until it either signals to stop for target approach, or fails (e.g. loses line for too long).
            elif self.state == self.STATE_SEARCH_LINE_FOLLOW:
                print("[STATE] SEARCH_LINE_FOLLOW")
                result = self.search_follower.run()

                if result == "stop_target":
                    print("target reached - stopping line follow")
                    self.state = self.STATE_APPROACH_TARGET
                else:
                    print("Line follow failed with result:", result)
                    self.state = self.STATE_ERROR

            # In the target approach state, we run the target approach sequence until it either successfully aligns with the target, or fails (e.g. can't find target, or can't align after many attempts).
            elif self.state == self.STATE_APPROACH_TARGET:
                print("[STATE] APPROACH_TARGET")
                success = self.target_approach.run()

                if success:
                    self.state = self.STATE_PICKUP
                else:
                    print("Target approach failed")
                    self.state = self.STATE_ERROR

            # In the pickup state, we run the pickup sequence which should attempt to grab the object. If successful, we transition to holding object state. If it fails, we go to error state.
            elif self.state == self.STATE_PICKUP:
                print("[STATE] PICKUP")

                self.claw.initialize()
                success = self.claw.run_pickup_sequence()

                if success:
                    self.state = self.STATE_HOLDING_OBJECT
                else:
                    self.state = self.STATE_ERROR

            # In the holding object state, we can optionally run some code to ensure the claw is holding the object securely. Then we transition to the turn around state to prepare for returning.
            elif self.state == self.STATE_HOLDING_OBJECT:
                print("[STATE] HOLDING_OBJECT")
                self.claw.hold_object()
                self.state = self.STATE_TURN_AROUND

            # In the turn around state, we execute a 180 degree turn to face back towards the line. After the turn is complete, we transition to the rescue line follow state.
            elif self.state == self.STATE_TURN_AROUND:
                print("[STATE] TURN_AROUND")

                # self.claw.hold_object()
                self.mover.turn_right_counts(
                    target_counts=self.cfg.TURN_AROUND_180_COUNTS,
                    base_percent=self.cfg.TURN_AROUND_PERCENT,
                    timeout_s=self.cfg.TURN_AROUND_TIMEOUT_S,
                    debug=self.cfg.TURN_AROUND_DEBUG
                )

                utime.sleep_ms(self.cfg.TURN_AROUND_SETTLE_MS)
                print("Turn around complete")

                self.state = self.STATE_RESCUE_LINE_FOLLOW

            # In the rescue line follow state, we run a line follower that attempts to find the line again and follow it back towards the safe zone. If it signals that the safe zone is reached, we transition to approach safe zone state. If it fails (e.g. loses line for too long), we go to error state.
            elif self.state == self.STATE_RESCUE_LINE_FOLLOW:
                print("[STATE] RESCUE_LINE_FOLLOW")
                result = self.rescue_follower.run()

                if result == "safe_zone":
                    print("Safe zone reached - stopping")
                    self.state = self.STATE_APPROACH_SAFE_ZONE
                else:
                    print("Rescue line follow failed with result:", result)
                    self.state = self.STATE_ERROR

            # In the approach safe zone state, we execute the sequence to turn into the safe zone and drive forward to drop off the object. After this sequence, we transition to the release state.
            elif self.state == self.STATE_APPROACH_SAFE_ZONE:
                print("[STATE] APPROACH_SAFE_ZONE")

                self.mover.turn_right_counts(
                    target_counts=self.cfg.TURN_INTO_SAFE_ZONE_COUNTS,
                    base_percent=self.cfg.TURN_INTO_SAFE_ZONE_PERCENT,
                    timeout_s=self.cfg.TURN_INTO_SAFE_ZONE_TIMEOUT_S,
                    debug=self.cfg.TURN_INTO_SAFE_ZONE_DEBUG
                )
                utime.sleep_ms(self.cfg.TURN_INTO_SAFE_ZONE_SETTLE_MS)
                self.mover.move_forward(
                    target_counts=self.cfg.DRIVE_INTO_SAFE_ZONE_COUNTS,
                    base_percent=self.cfg.DRIVE_INTO_SAFE_ZONE_PERCENT,
                    timeout_s=self.cfg.DRIVE_INTO_SAFE_ZONE_TIMEOUT_S,
                    debug=False
                )
                utime.sleep_ms(self.cfg.DRIVE_INTO_SAFE_ZONE_SETTLE_MS)

                print("Approach safe zone complete")
                self.state = self.STATE_RELEASE

            # In the release state, we run the claw release sequence to drop the object. If successful, we transition to find line state to try to find the line again after dropping. If it fails, we go to error state.
            elif self.state == self.STATE_RELEASE:
                print("[STATE] RELEASE")

                success = self.claw.run_release_sequence()

                if success:
                    self.state = self.STATE_FIND_LINE
                else:
                    self.state = self.STATE_ERROR

            # In the find line state, we execute a turn to try to find the line again. If successful, we transition to the return line follow state to follow the line back towards the start. If it fails (e.g. can't find line), we go to error state.  
            elif self.state == self.STATE_FIND_LINE:
                print("[STATE] FIND_LINE")

                self.mover.move_backward(
                    target_counts=self.cfg.FIND_LINE_REVERSE_COUNTS,
                    base_percent=self.cfg.FIND_LINE_REVERSE_PERCENT,
                    timeout_s=self.cfg.FIND_LINE_REVERSE_TIMEOUT_S,
                    debug=self.cfg.FIND_LINE_REVERSE_DEBUG
                )
                utime.sleep_ms(self.cfg.FIND_LINE_REVERSE_SETTLE_MS)

                self.mover.turn_left_counts(
                    target_counts=self.cfg.FIND_LINE_TURN_COUNTS,
                    base_percent=self.cfg.FIND_LINE_TURN_PERCENT,
                    timeout_s=self.cfg.FIND_LINE_TIMEOUT_S,
                    debug=self.cfg.FIND_LINE_SETTLE_DEBUG
                )
                utime.sleep_ms(self.cfg.FIND_LINE_SETTLE_MS)

                print("Find line complete")
                self.state = self.STATE_RETURN_LINE_FOLLOW

            # In the return line follow state, we run a line follower that attempts to follow the line back towards the start. If it signals that the return is complete, we transition to the done state. If it fails (e.g. loses line for too long), we go to error state.
            elif self.state == self.STATE_RETURN_LINE_FOLLOW:
                print("[STATE] RETURN_LINE_FOLLOW")

                result = self.return_follower.run()

                if result == "timeout":
                    print("Return line follow complete")
                    self.state = self.STATE_DONE
                else:
                    print("Return line follow failed with result:", result)
                    self.state = self.STATE_ERROR

            elif self.state == self.STATE_DONE:
                print("Done. Holding object. Press button to run again.")
                return True

            elif self.state == self.STATE_ERROR:
                print("Mission error. Press button to retry.")
                return False

            utime.sleep_ms(20)