from inputs import get_gamepad, UnpluggedError
import math
import threading


class XboxController(object):
    """
    XboxController class for interfacing with an Xbox controller.

    Attributes:
    - MAX_TRIG_VAL: Maximum trigger value.
    - MAX_JOY_VAL: Maximum joystick value.
    - LeftJoystickY: Y-axis value of the left joystick.
    - LeftJoystickX: X-axis value of the left joystick.
    - RightJoystickY: Y-axis value of the right joystick.
    - RightJoystickX: X-axis value of the right joystick.
    - LeftTrigger: Value of the left trigger.
    - RightTrigger: Value of the right trigger.
    - LeftBumper: State of the left bumper button.
    - RightBumper: State of the right bumper button.
    - A: State of the A button.
    - X: State of the X button.
    - Y: State of the Y button.
    - B: State of the B button.
    - LeftThumb: State of the left thumbstick button.
    - RightThumb: State of the right thumbstick button.
    - Back: State of the back button.
    - Start: State of the start button.
    - LeftDPad: State of the left direction pad button.
    - RightDPad: State of the right direction pad button.
    - UpDPad: State of the up direction pad button.
    - DownDPad: State of the down direction pad button.
    """

    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        """
        Initializes the XboxController object and starts the monitoring thread.
        """
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def _monitor_controller(self):
        """
        Monitors the Xbox controller for input events and updates the attributes accordingly.
        """
        try:
            while True:
                events = get_gamepad()
                for event in events:
                    if event.code == 'ABS_Y':
                        self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL
                    elif event.code == 'ABS_X':
                        self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL
                    elif event.code == 'ABS_RY':
                        self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL
                    elif event.code == 'ABS_RX':
                        self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL
                    elif event.code == 'ABS_Z':
                        self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL
                    elif event.code == 'ABS_RZ':
                        self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL
                    elif event.code == 'BTN_TL':
                        self.LeftBumper = event.state
                    elif event.code == 'BTN_TR':
                        self.RightBumper = event.state
                    elif event.code == 'BTN_SOUTH':
                        self.A = event.state
                    elif event.code == 'BTN_NORTH':
                        self.Y = event.state
                    elif event.code == 'BTN_WEST':
                        self.X = event.state
                    elif event.code == 'BTN_EAST':
                        self.B = event.state
                    elif event.code == 'BTN_THUMBL':
                        self.LeftThumb = event.state
                    elif event.code == 'BTN_THUMBR':
                        self.RightThumb = event.state
                    elif event.code == 'BTN_SELECT':
                        self.Back = event.state
                    elif event.code == 'BTN_START':
                        self.Start = event.state
                    elif event.code == 'ABS_HAT0X':
                        if event.state == 1:
                            self.RightDPad = 1
                        elif event.state == -1:
                            self.LeftDPad = 1
                        elif event.state == 0:
                            self.RightDPad = event.state
                            self.LeftDPad = event.state
                    elif event.code == 'ABS_HAT0Y':
                        if event.state == 1:
                            self.DownDPad = 1
                        elif event.state == -1:
                            self.UpDPad = 1
                        elif event.state == 0:
                            self.UpDPad = event.state
                            self.DownDPad = event.state
        except UnpluggedError:
            print("No gamepad found. Exiting the controller monitoring thread.")
            # You can add additional handling for this case, such as setting all values to 0 or terminating the program.
