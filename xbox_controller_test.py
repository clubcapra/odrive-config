from xbox_controller import XboxController
from time import sleep

def main():
    xbox = XboxController()

    try:
        print("Reading Xbox controller inputs. Press Ctrl+C to exit.\n")
        while True:
            sleep(0.1)
            print(f"""
Connected:         {xbox.Connected}
A:                 {xbox.A}
B:                 {xbox.B}
X:                 {xbox.X}
Y:                 {xbox.Y}
Left Bumper:       {xbox.LeftBumper}
Right Bumper:      {xbox.RightBumper}
Left Trigger:      {xbox.LeftTrigger:.2f}
Right Trigger:     {xbox.RightTrigger:.2f}
Left Joystick X:   {xbox.LeftJoystickX:.2f}
Left Joystick Y:   {xbox.LeftJoystickY:.2f}
Right Joystick X:  {xbox.RightJoystickX:.2f}
Right Joystick Y:  {xbox.RightJoystickY:.2f}
DPad Up:           {xbox.UpDPad}
DPad Down:         {xbox.DownDPad}
DPad Left:         {xbox.LeftDPad}
DPad Right:        {xbox.RightDPad}
""")
    except KeyboardInterrupt:
        print("Exited input test.")

if __name__ == "__main__":
    main()
