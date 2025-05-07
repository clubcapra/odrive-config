from enum import IntEnum

# Non exhaustive list of odrive codes

# odrive commands
class ODriveCommand(IntEnum):
    GET_VERSION_CMD = 0x00
    HEARTBEAT_CMD = 0x01
    ESTOP_CMD = 0x02
    GET_ERROR_CMD = 0x03
    RX_SDO_CMD = 0x04
    TX_SDO_CMD = 0x05
    ADDRESS_CMD = 0x06
    SET_AXIS_STATE_CMD = 0x07
    GET_ENCODER_ESTIMATES_CMD = 0x09
    SET_CONTROLLER_MODE_CMD = 0x0B
    SET_INPUT_POS_CMD = 0x0C
    SET_INPUT_VEL_CMD = 0x0D
    SET_TRAJ_VEL_LIMIT_CMD = 0x11
    GET_TEMPERATURE_CMD = 0x15
    REBOOT_CMD = 0x16
    GET_BUS_VOLTAGE_CURRENT_CMD = 0x17
    CLEAR_ERRORS_CMD = 0x18
    GET_TORQUES_CMD = 0x1C
    GET_POWERS_CMD = 0x1D

class ODriveRebootAction(IntEnum):
    REBOOT_ACTION_REBOOT = 0
    REBOOT_ACTION_SAVE = 1
    REBOOT_ACTION_ERASE = 2

# odrive axis state
class ODriveAxisState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    CALIBRATION = 3
    CLOSED_LOOP_CONTROL = 8

# odrive control mode
class ODriveControlMode(IntEnum):
    MODE_VOLTAGE_CONTROL = 0
    MODE_TORQUE_CONTROL = 1
    MODE_VELOCITY_CONTROL = 2
    MODE_POSITION_CONTROL = 3

# odrive input mode
class ODriveInputMode(IntEnum):
    INPUT_INACTIVE = 0
    INPUT_PASSTHROUGH = 1
    INPUT_VEL_RAMP = 2
    INPUT_POS_FILTER = 3
    INPUT_MIX_CHANNELS = 4
    INPUT_TRAP_TRAJ = 5
    INPUT_TORQUE_RAMP = 6

class ODriveEndpoints(IntEnum):
    CONFIG_INERTIA = 413 # float
    CONFIG_INPUT_FILTER_BANDWIDTH = 414 # float
    TRAP_TRAJ_VEL_LIMIT = 429 # float
    TRAP_TRAJ_ACCEL_LIMIT = 430 # float
    TRAP_TRAJ_DECEL_LIMIT = 431 # float