import sys
import motors

HELP_TEXT = """
Robot Controller Terminal 
Commands:
  m <idx> <speed>  : DC Motor (0-1) speed (-65535 to 65535)
  s <idx> <angle>  : Servo (0-1) angle (0-180) - Auto-powers target
  stop             : Kill all power and motion
  help             : Show this menu
  exit             : Quit script
"""

def run_terminal():
    print(HELP_TEXT)
    
    while True:
        try:
            sys.stdout.write("\nrobot_cmd> ")
            line = sys.stdin.readline().strip().lower()
            if not line: continue
            
            parts = line.split()
            cmd = parts[0]

            if cmd == "exit":
                break
            
            elif cmd == "help":
                print(HELP_TEXT)

            elif cmd == "stop":
                motors.stop_all()
                print("Emergency stop deployed.")

            elif cmd == "m" and len(parts) == 3:
                idx, speed = int(parts[1]), int(parts[2])
                motors.set_dc_speed(idx, speed)
                print(f"DC Motor {idx} -> {speed}")

            elif cmd == "s" and len(parts) == 3:
                idx, angle = int(parts[1]), int(parts[2])
                motors.set_servo_angle(idx, angle)
                print(f"Servo {idx} -> {angle} degrees (Power isolated)")

            else:
                print("Invalid command or arguments.")

        except Exception as e:
            print(f"Error: {e}")
        except KeyboardInterrupt:
            motors.stop_all()
            print("\nHardware safety shutdown. Exiting.")
            break

if __name__ == "__main__":
    run_terminal()