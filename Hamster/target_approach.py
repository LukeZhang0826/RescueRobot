# target_approach.py
import utime


class TargetApproach:
    """
    Primitive visual servo:
      - use Pixy largest target block (sig 7)
      - align x first with tiny turns
      - then align y with tiny forward/backward moves
      - search if no block is found
    """

    def __init__(self, pixy, basic_movements, cfg):
        self.pixy = pixy
        self.mover = basic_movements
        self.cfg = cfg

    def _get_target_block(self):
        return self.pixy.best_block(
            sigmap=self.cfg.PIXY_SIGNATURE_7,
            area_min=self.cfg.TARGET_AREA_MIN
        )

    def _turn_counts_from_error(self, ex):
        aex = abs(ex)
        if aex < 25:
            return self.cfg.TARGET_TURN_COUNTS_SMALL
        elif aex < 60:
            return self.cfg.TARGET_TURN_COUNTS_MED
        else:
            return self.cfg.TARGET_TURN_COUNTS_LARGE

    def _move_counts_from_error(self, ey):
        aey = abs(ey)
        if aey < 20:
            return self.cfg.TARGET_MOVE_COUNTS_SMALL
        elif aey < 50:
            return self.cfg.TARGET_MOVE_COUNTS_MED
        else:
            return self.cfg.TARGET_MOVE_COUNTS_LARGE

    def run(self):
        cfg = self.cfg

        print("Target approach started")
        print(f"Target image position: ({cfg.TARGET_X}, {cfg.TARGET_Y})")

        for step in range(cfg.TARGET_APPROACH_MAX_STEPS):
            block = self._get_target_block()

            if block is None:
                if cfg.TARGET_DEBUG:
                    print(f"[{step}] no target block -> move forward")
                self.mover.move_forward(
                    target_counts=12,
                    base_percent=cfg.TARGET_MOVE_PERCENT,
                    timeout_s=0.5,
                    debug=False
                )
                utime.sleep_ms(cfg.TARGET_SETTLE_MS)
                continue

            x = block["x"]
            y = block["y"]
            area = block["area"]

            ex = x - cfg.TARGET_X
            ey = y - cfg.TARGET_Y

            x_ok = abs(ex) <= cfg.TARGET_DEADBAND_X
            y_ok = abs(ey) <= cfg.TARGET_DEADBAND_Y

            if cfg.TARGET_DEBUG:
                print(
                    f"[{step}] sig={block['sig']} x={x} y={y} w={block['w']} h={block['h']} area={area} "
                    f"ex={ex} ey={ey}"
                )

            if x_ok and y_ok:
                print("Target aligned")
                self.mover.stop()
                return True

            # Priority 1: align horizontally
            if not x_ok:
                turn_counts = self._turn_counts_from_error(ex)

                if ex < 0:
                    if cfg.TARGET_DEBUG:
                        print(f"  turn left  counts={turn_counts}")
                    self.mover.turn_left_counts(
                        target_counts=turn_counts,
                        base_percent=cfg.TARGET_TURN_PERCENT,
                        timeout_s=1.0,
                        debug=False
                    )
                else:
                    if cfg.TARGET_DEBUG:
                        print(f"  turn right counts={turn_counts}")
                    self.mover.turn_right_counts(
                        target_counts=turn_counts,
                        base_percent=cfg.TARGET_TURN_PERCENT,
                        timeout_s=1.0,
                        debug=False
                    )

                utime.sleep_ms(cfg.TARGET_SETTLE_MS)
                continue

            # Priority 2: align vertically / distance
            if not y_ok:
                move_counts = self._move_counts_from_error(ey)

                # If blob is too high in image, it is usually farther away -> move forward
                if ey < 0:
                    if cfg.TARGET_DEBUG:
                        print(f"  move forward counts={move_counts}")
                    self.mover.move_forward(
                        target_counts=move_counts,
                        base_percent=cfg.TARGET_MOVE_PERCENT,
                        timeout_s=1.0,
                        debug=False
                    )
                else:
                    if cfg.TARGET_DEBUG:
                        print(f"  move backward counts={move_counts}")
                    self.mover.move_backward(
                        target_counts=move_counts,
                        base_percent=cfg.TARGET_MOVE_PERCENT,
                        timeout_s=1.0,
                        debug=False
                    )

                utime.sleep_ms(cfg.TARGET_SETTLE_MS)
                continue

        print("Target approach stopped: max steps reached")
        self.mover.stop()
        return False