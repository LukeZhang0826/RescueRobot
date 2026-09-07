# RescueRobot

An autonomous line-following rescue robot, built for a competition at the University of
Waterloo's IDEAS Clinic. It follows a track, finds a coloured target, picks it up with a
servo claw, carries it to a safe zone, drops it, then finds the line again and drives
back.

We called it Hamster, which is why the robot code lives in `Hamster/`.

[![Watch the competition run](https://img.youtube.com/vi/ffi4BYylVoE/hqdefault.jpg)](https://youtu.be/ffi4BYylVoE)

**[Watch the run](https://youtu.be/ffi4BYylVoE)** ·
**[Final report](https://docs.google.com/document/d/1Vht0kWhE6cIuxJ4qUaxeXvZQmihU7ujaP2H0h0Pqgro/edit)**

## The mission

`Hamster/robot_mission.py` is a flat state machine. A button press starts it, and every
stage either advances or drops into an error state.

```
idle -> search_line_follow -> approach_target -> pickup -> holding_object
     -> turn_around -> rescue_line_follow -> approach_safe_zone -> release
     -> find_line -> return_line_follow -> done
```

Three separate line followers exist because each leg of the run has different exit
conditions. `search_line_follower` stops when the target is large enough in frame,
`rescue_line_follower` stops on the safe zone signature, and `return_line_follow` runs
the trip home. All three share a PD steering loop over the Pixy2 line centroid, with a
recovery state that turns back toward the side the line was last seen on.

## Hardware

| Part | Notes |
| --- | --- |
| Raspberry Pi Pico | Runs the MicroPython mission code in `Hamster/` |
| Arduino Mega 2560 | PlatformIO firmware in `firmware/`, used for board bring-up |
| Pixy2 camera | Line tracking and colour connected component detection, 7 signatures |
| DRV8833 | Dual motor driver, signed speed from -100 to +100 |
| Servo claw | Pickup and release |
| Custom PCBs | An Arduino Mega shield and a base board, KiCad sources in `Electrical/` |

Both PCBs were manufactured through JLCPCB. Gerbers, BOM, and pick-and-place files are
committed under `Electrical/KiCad/BasePCB/jlcpcb/`.

## Layout

| Path | What it is |
| --- | --- |
| `Hamster/` | MicroPython mission code for the Pico |
| `Hamster/archive/` | Superseded scripts kept from earlier milestones |
| `Hamster/pixycamParams/` | Saved Pixy2 tuning, including the values used on the day |
| `firmware/` | PlatformIO project for the Arduino Mega |
| `Electrical/KiCad/` | KiCad projects, gerbers, and production files |

## Running it

The Pico code is deployed with the MicroPico VS Code extension. Copy the contents of
`Hamster/` to the board and reset it. `Hamster/main.py` wires up the hardware objects and
hands control to `RobotMission.run_once()`, which waits for the start button.

Everything the robot is tuned by lives in `Hamster/config.py`, including the PI gains per
wheel, the PD steering constants, base speeds in RPM, and the Pixy2 area thresholds that
decide when a target is close enough to stop. The competition values are the ones
currently committed.

For the Arduino side, `cd firmware && pio run -t upload`.
