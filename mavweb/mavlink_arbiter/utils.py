#!/bin/python3
from mavlink_arbiter.singleton import singleton
import queue


class BCOLORS:
    """
    Static class for OS
    Constant use on headers.
    """

    HEADER      = '\033[95m'
    OKBLUE      = '\033[94m'
    OKGREEN     = '\033[92m'
    WARNING     = '\033[93m'
    FAIL        = '\033[91m'
    ENDC        = '\033[0m'
    BOLD        = '\033[1m'
    SUCCESS     = '\033[1;42m'
    UNDERLINE   = '\033[4m'
    ERROR       = '\033[1;41m'


@singleton
class Utils:
    def __init__(self):
        """
        Utilities class for loging messages in the console
        class takes on a singleton pattern.
        """

        self.logs = []
        self.currLog = 'Initialized'

    def log(self, string):
        """
        Logs a message in the entire program.

        :string: String that you want to log.
        """

        print(BCOLORS.BOLD + string + BCOLORS.ENDC)
        self.currLog = string
        self.logs.append(string)

    # def prompt(self, question, affirm_opt):
    #     '''
    #         TODO: Not functional yet. Will be used for CLI.
    #     '''
    #     opt = input(BCOLORS.BOLD + question + BCOLORS.ENDC)
    #     if opt in affirm_opt:
    #         return True
    #     else:
    #         return False

    def errLog(self, string):
        """
        Logs an error in the entire program.
        Will highlight red.

        :string: String that you want to log.
        """

        print(BCOLORS.ERROR + string + BCOLORS.ENDC)

        self.currLog = string
        self.logs.append(string)

    def succLog(self, string):
        """
        Logs a success message in the entire program.
        Will highlight green.

        :string: String that you want to log.
        """

        print(BCOLORS.SUCCESS + string + BCOLORS.ENDC)

        self.currLog = string
        self.logs.append(string)

    def getPreviousLogs(self):
        """
        Get the last log that was made.
        """

        return self.logs

    @staticmethod
    def meters_to_feet(meters):
        """
        Meters to Feet conversion function
        for use by Interoperbility publishing
        """

        feet = meters * 3.280839895
        return feet


if __name__ == "main":
    print("Testing Queue buffer utility:")
    queue = queue.Queue()

    obj1 = dict()
    obj1['1'] = 'String1'

    obj2 = dict()
    obj2['2'] = 'String2'

    print(queue.empty())
    queue.put(obj1)
    print(queue.empty())


# https://mavlink.io/en/messages/common.html#MAV_MODE
class ModeMavlink:
    MAV_MODE_PREFLIGHT              = 0
    MAV_MODE_STABILIZE_DISARMED     = 80
    MAV_MODE_STABILIZE_ARMED        = 208
    MAV_MODE_MANUAL_DISARMED        = 64
    MAV_MODE_MANUAL_ARMED           = 192
    MAV_MODE_GUIDED_ARMED           = 216
    MAV_MODE_GUIDED_DISARMED        = 88
    MAV_MODE_AUTO_DISARMED          = 92
    MAV_MODE_AUTO_ARMED             = 220
    MAV_MODE_TEST_DISARMED          = 66
    MAV_MODE_TEST_ARMED             = 194


# https://mavlink.io/en/messages/common.html
class CommandMavlink:
    CMD_NAV_WAYPOINT                    = 16
    CMD_NAV_LOITER_UNLIM                = 17
    CMD_NAV_LOITER_TURNS                = 18
    CMD_NAV_LOITER_TIME                 = 19
    CMD_NAV_RETURN_TO_LAUNCH            = 20
    CMD_NAV_LAND                        = 21
    CMD_NAV_TAKEOFF                     = 22
    CMD_NAV_ROI                         = 80
    CMD_NAV_PATHPLANNING                = 81
    CMD_NAV_LAST                        = 95
    CMD_CONDITION_DELAY                 = 112
    CMD_CONDITION_CHANGE_ALT            = 113
    CMD_CONDITION_DISTANCE              = 114
    CMD_CONDITION_YAW                   = 115
    CMD_CONDITION_LAST                  = 159
    CMD_DO_SET_MODE                     = 176
    CMD_DO_JUMP                         = 177
    CMD_DO_CHANGE_SPEED                 = 178
    CMD_DO_SET_HOME                     = 179
    CMD_DO_SET_PARAMETER                = 180
    CMD_DO_SET_RELAY                    = 181
    CMD_DO_REPEAT_RELAY                 = 182
    CMD_DO_SET_SERVO                    = 183
    CMD_DO_REPEAT_SERVO                 = 184
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
