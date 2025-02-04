import math
import threading
import evdev

class XboxController(object):
    """
    XboxController class for interfacing with an Xbox controller using evdev.

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

    MAX_TRIG_VAL = 1_023
    MAX_JOY_VAL = 32_768

    def __init__(self, deadzone=0.1):
        self._deadzone = deadzone
        """
        Initializes the XboxController object and starts the monitoring thread.
        """
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        self.device_path = ""
        for device in devices:
            if device.name == "Xbox Wireless Controller":
                self.device_path = device.path
                break

        if self.device_path == "":
            print("Controller disconnected. Exiting the controller monitor thread.")
            self.Connected = False
            return

        self.device = evdev.InputDevice(self.device_path)

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
        self.Connected = False

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def _monitor_controller(self):
        """
        Monitors the Xbox controller for input events and updates the attributes accordingly.
        """
        try:
            self.Connected = True
            for event in self.device.read_loop():
                if event.type == evdev.ecodes.EV_ABS:
                    if event.code == evdev.ecodes.ABS_Y:
                        self.LeftJoystickY = self._apply_deadzone((-event.value + XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                    elif event.code == evdev.ecodes.ABS_X:
                        self.LeftJoystickX = self._apply_deadzone((event.value - XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                    elif event.code == evdev.ecodes.ABS_RZ:
                        self.RightJoystickY = self._apply_deadzone((-event.value + XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                    elif event.code == evdev.ecodes.ABS_Z:
                        self.RightJoystickX = self._apply_deadzone((event.value - XboxController.MAX_JOY_VAL) / XboxController.MAX_JOY_VAL)
                    elif event.code == evdev.ecodes.ABS_BRAKE:
                        self.LeftTrigger = event.value / XboxController.MAX_TRIG_VAL
                    elif event.code == evdev.ecodes.ABS_GAS:
                        self.RightTrigger = event.value / XboxController.MAX_TRIG_VAL
                    elif event.code == evdev.ecodes.ABS_HAT0X:
                        if event.value == 1:
                            self.RightDPad = 1
                        elif event.value == -1:
                            self.LeftDPad = 1
                        elif event.value == 0:
                            self.RightDPad = self.LeftDPad = 0
                    elif event.code == evdev.ecodes.ABS_HAT0Y:
                        if event.value == 1:
                            self.DownDPad = 1
                        elif event.value == -1:
                            self.UpDPad = 1
                        elif event.value == 0:
                            self.UpDPad = self.DownDPad = 0
                elif event.type == evdev.ecodes.EV_KEY:
                    if event.code == evdev.ecodes.BTN_TL:
                        self.LeftBumper = event.value
                    elif event.code == evdev.ecodes.BTN_TR:
                        self.RightBumper = event.value
                    elif event.code == evdev.ecodes.BTN_SOUTH:
                        self.A = event.value
                    elif event.code == evdev.ecodes.BTN_NORTH:
                        self.Y = event.value
                    elif event.code == evdev.ecodes.BTN_WEST:
                        self.X = event.value
                    elif event.code == evdev.ecodes.BTN_EAST:
                        self.B = event.value
                    elif event.code == evdev.ecodes.BTN_THUMBL:
                        self.LeftThumb = event.value
                    elif event.code == evdev.ecodes.BTN_THUMBR:
                        self.RightThumb = event.value
                    elif event.code == evdev.ecodes.BTN_SELECT:
                        self.Back = event.value
                    elif event.code == evdev.ecodes.BTN_START:
                        self.Start = event.value
        except OSError:
            print("Controller disconnected. Exiting the controller monitor thread.")
            self.Connected = False

    def _apply_deadzone(self, value):
        if value > 0:
            return max(0, value - self._deadzone) / (1.0 - self._deadzone)
        else:
            return min(0, value + self._deadzone) / (1.0 - self._deadzone)
