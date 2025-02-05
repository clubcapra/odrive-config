# odrive_error_codes.py

ERROR_CODES = {
    1: "INITIALIZING - The system is initializing or reconfiguring.",
    2: "SYSTEM_LEVEL - Unexpected system error such as memory corruption, stack overflow, frozen thread, assert fail, etc.",
    4: "TIMING_ERROR - An internal hard timing requirement was violated. Likely due to system overload.",
    8: "MISSING_ESTIMATE - The position/velocity/phase estimate was invalid.",
    16: "BAD_CONFIG - The ODrive configuration is invalid or incomplete.",
    32: "DRV_FAULT - The gate driver chip reported an error.",
    64: "MISSING_INPUT - No value was provided for input_pos, input_vel, or input_torque.",
    256: "DC_BUS_OVER_VOLTAGE - The DC voltage exceeded the configured overvoltage trip level.",
    512: "DC_BUS_UNDER_VOLTAGE - The DC voltage fell below the configured undervoltage trip level.",
    1024: "DC_BUS_OVER_CURRENT - Too much DC current was pulled.",
    2048: "DC_BUS_OVER_REGEN_CURRENT - Too much DC current was regenerated.",
    4096: "CURRENT_LIMIT_VIOLATION - The motor current exceeded the specified hard max current.",
    8192: "MOTOR_OVER_TEMP - The motor temperature exceeded the specified upper limit.",
    16384: "INVERTER_OVER_TEMP - The inverter temperature exceeded the specified upper limit.",
    32768: "VELOCITY_LIMIT_VIOLATION - The velocity exceeds the velocity limit.",
    65536: "POSITION_LIMIT_VIOLATION - The position exceeded the position limit.",
    16777216: "WATCHDOG_TIMER_EXPIRED - The axis watchdog timer expired.",
    33554432: "ESTOP_REQUESTED - An emergency stop was requested.",
    67108864: "SPINOUT_DETECTED - A spinout situation was detected.",
    134217728: "BRAKE_RESISTOR_DISARMED - The brake resistor was disarmed.",
    268435456: "THERMISTOR_DISCONNECTED - The motor thermistor is disconnected.",
    1073741824: "CALIBRATION_ERROR - A calibration procedure failed."
}

def get_error_description(error_code):
    """
    Returns a human-readable description for a given error code.
    This handles both individual errors and combined error bitmasks.
    """
    description = []
    for code, desc in ERROR_CODES.items():
        if error_code & code:
            description.append(desc)
    return "\n".join(description) if description else "No error."
