#!/bin/python3

import time
from pymavlink import mavutil

mavutil.set_dialect("ardupilotmega")

autopilot = mavutil.mavlink_connection('udpin:localhost:14550')

while msg is None:
    msg = autopilot.recv_msg()


# The values of these heartbeat fields is not really important here
# I just used the same numbers that QGC uses
# It is standard practice for any system communicating via mavlink emit
# the HEARTBEAT message at 1Hz! Your autopilot may not behave the way you want otherwise!
autopilot.mav.heartbeat_send(
    6,  # type
    8,  # autopilot
    192,  # base_mode
    0,  # custom_mode
    4,  # system_status
    3  # mavlink_version
)

# Arm the vehicle
autopilot.mav.command_long_send(
    1,  # autopilot system id
    1,  # autopilot component id
    400,  # command id, ARM/DISARM
    0,  # confirmation
    1,  # arm!
    0, 0, 0, 0, 0, 0  # unused parameters for this command
)

# Wait two seconds
time.sleep(2)

# Disarm the vehicle.
autopilot.mav.command_long_send(
    1,  # autopilot system id
    1,  # autopilot component id
    400,  # command id, ARM/DISARM
    0,  # confirmation
    0,  # disarm!
    0, 0, 0, 0, 0, 0  # unused parameters for this command
)

# Documentation : 
CMD_NAV_WAYPOINT                    = 16  # Applies to Rover
CMD_NAV_LOITER_UNLIM                = 17
CMD_NAV_LOITER_TURNS                = 18
CMD_NAV_LOITER_TIME                 = 19
CMD_NAV_RETURN_TO_LAUNCH            = 20  # Applies to Rover.
CMD_NAV_LAND                        = 21
CMD_NAV_TAKEOFF                     = 22
CMD_NAV_ROI                         = 80
CMD_NAV_PATHPLANNING                = 81
CMD_NAV_LAST                        = 95
CMD_CONDITION_DELAY                 = 112  # Applies to Rover.
CMD_CONDITION_CHANGE_ALT            = 113
CMD_CONDITION_DISTANCE              = 114  # Applies to Rover
CMD_CONDITION_YAW                   = 115
CMD_CONDITION_LAST                  = 159
CMD_DO_SET_MODE                     = 176
CMD_DO_JUMP                         = 177
CMD_DO_CHANGE_SPEED                 = 178  # Applies to Rover.
CMD_DO_SET_HOME                     = 179  # Applies to Rover.
CMD_DO_SET_PARAMETER                = 180
CMD_DO_SET_RELAY                    = 181  # Applies to Rover
CMD_DO_REPEAT_RELAY                 = 182  # Applies to Rover
CMD_DO_SET_SERVO                    = 183  # Applies to servos on Rover.
CMD_DO_REPEAT_SERVO                 = 184  # Applies to Rover
CMD_DO_CONTROL_VIDEO                = 200
CMD_DO_DIGICAM_CONFIGURE            = 202
CMD_DO_DIGICAM_CONTROL              = 203
CMD_DO_MOUNT_CONFIGURE              = 204
CMD_DO_MOUNT_CONTROL                = 205
CMD_DO_LAST                         = 240
CMD_PREFLIGHT_CALIBRATION           = 241
CMD_PREFLIGHT_SET_SENSOR_OFFSETS    = 242
CMD_PREFLIGHT_STORAGE               = 245
CMD_PREFLIGHT_REBOOT_SHUTDOWN       = 246
CMD_OVERRIDE_GOTO                   = 252
CMD_MISSION_START                   = 300
D_COMPONENT_ARM_DISARM              = 400
