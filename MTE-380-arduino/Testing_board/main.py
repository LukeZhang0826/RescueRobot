import sys
from motors import set_dc_speed, set_servo_angle, stop_all, dc_motors, servo_motors

HELP = """
Commands:
  motor <index> <speed>            Set DC motor speed (-65535 to 65535)
  servo <index> <angle>            Set servo angle (0-180), powers on automatically
  servo <index> <angle> off        Set angle then cut servo power
  stop                             Stop all motors and servos
  help                             Show this message

Examples:
  motor 0 20000      Motor A forward at ~30%
  motor 1 -20000     Motor B reverse at ~30%
  servo 0 90         Servo 1 to 90 degrees
  servo 1 0 off      Servo 2 to 0 degrees then power off
"""

def parse_command(line):
    parts = line.strip().split()
    if not parts:
        return

    cmd = parts[0].lower()

    if cmd == "help":
        print(HELP)

    elif cmd == "stop":
        stop_all()
        print("All stopped.")

    elif cmd == "motor":
        if len(parts) != 3:
            print("Usage: motor <index> <speed>")
            return
        index = int(parts[1])
        speed = int(parts[2])
        if not (0 <= index < len(dc_motors)):
            print(f"Invalid motor index. Valid: 0-{len(dc_motors)-1}")
            return
        if not (-65535 <= speed <= 65535):
            print("Speed must be between -65535 and 65535.")
            return
        set_dc_speed(index, speed)
        print(f"Motor {index} set to {speed}.")

    elif cmd == "servo":
        if len(parts) < 3:
            print("Usage: servo <index> <angle> [off]")
            return
        index = int(parts[1])
        angle = int(parts[2])
        power_on = not (len(parts) == 4 and parts[3].lower() == "off")
        if not (0 <= index < len(servo_motors)):
            print(f"Invalid servo index. Valid: 0-{len(servo_motors)-1}")
            return
        if not (0 <= angle <= 180):
            print("Angle must be between 0 and 180.")
            return
        set_servo_angle(index, angle, power_on)
        print(f"Servo {index} set to {angle} degrees, power {'on' if power_on else 'off'}.")

    else:
        print(f"Unknown command: '{cmd}'. Type 'help' for commands.")


print("Line follower robot - test terminal. Type 'help' for commands.")

while True:
    try:
        sys.stdout.write("> ")
        line = sys.stdin.readline()
        if line:
            parse_command(line)
    except KeyboardInterrupt:
        stop_all()
        print("\nStopped. Exiting.")
        break