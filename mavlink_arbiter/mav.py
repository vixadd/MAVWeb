#!/bin/python3

# ===========================================================#
#                                                           #
#                      Mavlink Library                      #
#                                                           #
#                    Author: davidkroell                    #
#                    Version: 2.1.0                         #
#                                                           #
#         This is the mavlink library that is used	        #
#    to send and recieve mavlink data from/to the plane.    #
#                                                           #
# ===========================================================#

from fdg.mavlink_arbiter.utils import Utils, CommandMavlink, ModeMavlink

from pymavlink import mavutil
from pymavlink import mavwp
from multiprocessing import Process, Queue

import sys
import traceback
import time


class Mavlink(object):
    def __init__(self, pin=14551, host="0.0.0.0", pout=14550):
        """
        Mavlink module class. Creates new instance of Mavlink Module

        Parameters
        ----------
            pin: int
                Is the port on which you recieve packets. (usually 14550)

            pout: int
                Is the port on which you send packages out. (usually 14551)

            host: string
                Is the hostname of the place recieving packets.
        """
        self.util = Utils()

        # Track down server data.
        self.target_ip = host
        self.target_port = pout
        self.input_port = pin

        self.util.log("Starting mavudp session on - "
                      + str(self.target_ip) + ":"
                      + str(self.target_port)
                      )

        # Calibration of mavlink utility to allow us to send/recieve waypoints.
        mavutil.set_dialect("ardupilotmega")

        # Mavlink arbiter that allows us to arbitrate the command logs.
        self.mavlink = mavutil.mavlink

        # Waypoint handeling utilities initiated.
        self.wp = mavwp.MAVWPLoader()
        self.wp_items = []

        self.util.log("Starting mavudp session for autopilot control on - {}:{}"
                      .format(self.target_ip, self.target_port))

        # Connect to the autopilot system.
        self.autopilot = mavutil.mavlink_connection('udpin:{}:{}'.format(self.target_ip, self.input_port))
        self.system_id = self.autopilot.target_system
        self.component_id = self.autopilot.target_component

        self.mode_mapping = self.autopilot.mode_mapping()

        self.wait_conn( master=self.autopilot )
        self.util.succLog("Received MavProxy GPS Fix...")

        self.MavBuffer = Queue()
        self.procMav = Process(target=self.start_udp_stream, args=())

        self.home_waypoint = (0, 0)

    def start_udp_stream(self):
        """
        Function called in the thread to constantly update packets.
        """
        while True:
            try:
                status = self.autopilot.recv_msg()
                if status is not None:
                    self.MavBuffer.put(status)

            except KeyboardInterrupt:
                self.util.errLog("Keyboard interrupt: UDP stream termination in progress...")
                sys.exit(0)

            except Exception as err:
                self.util.errLog("UDP Stream Error Occured.")
                self.util.errLog(f"{err}")
                traceback.print_exc()

    def get_mav_packet(self):
        """
        Accessor, to get the current packet
        """
        if not (self.MavBuffer.empty()):
            packet = self.MavBuffer.get()
            return packet
        else:
            return None

    def disarm_vehicle(self):
        """
        Disarm the UGV Vehicle.
        """
        cmd = CommandMavlink.D_COMPONENT_ARM_DISARM
        self.autopilot.mav.command_long_send(
            self.system_id,  # autopilot system id
            self.component_id,  # autopilot component id
            cmd,  # command id, ARM/DISARM
            0,  # confirmation
            0,  # disarm!
            0, 0, 0, 0, 0, 0  # unused parameters for this command
        )

    def arm_vehicle(self):
        """
        Arms the UGV vehicle.
        """
        cmd = CommandMavlink.D_COMPONENT_ARM_DISARM
        self.autopilot.mav.command_long_send(
            self.system_id,  # autopilot system id
            self.component_id,  # autopilot component id
            cmd,  # command id, ARM/DISARM
            0,  # confirmation
            1,  # arm!
            0, 0, 0, 0, 0, 0  # unused parameters for this command
        )

    def set_mode(self, mode=ModeMavlink.MAV_MODE_PREFLIGHT):
        """
        Set the mavmode to a custom specified mode.

        Parameters
        ----------
            mode: COMMAND_IDENTIFICATION
                Integer that represents a given mode defined
                in COMMAND_IDENTIFICATION
        """
        cmd = CommandMavlink.CMD_DO_SET_MODE
        self.autopilot.mav.command_long_send(
            self.system_id,  # autopilot system id
            self.component_id,  # autopilot component id
            cmd,  # command id
            0,  # confirmation
            mode,  # Mode setting
            0,  # custom mode.
            0,  # custom sub-mode
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0  # Empty
        )

    def set_mode_auto(self):
        """
        Sets the UGV mode to AUTO.
        """
        self.autopilot.set_mode_apm(10)

    def set_mode_manual(self):
        """
        Sets the UGV mode to MANUAL.
        """
        self.autopilot.set_mode_apm(0)

    def load_waypoints(self):
        """
        Load a sequence of waypoints to the mission.
        """
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        radius = 10
        seq = 1

        for wp_item in self.wp_items:
            lat, lon = wp_item
            alt = 0

            self.wp.add(
                mavutil.mavlink.MAVLink_mission_item_message(
                    self.system_id,
                    self.component_id,
                    seq,
                    frame,
                    CommandMavlink.CMD_NAV_WAYPOINT,
                    0, 0, 0, radius, 0, 0,
                    lat, lon, alt
                )
            )

        self.autopilot.waypoint_clear_all_send()
        self.autopilot.waypoint_count_send(self.wp.count())

        for i in range(self.wp.count()):
            msg = self.autopilot.recv_match(type=['MISSION_REQUEST'], blocking=True)
            self.autopilot.mav.send(self.wp.wp(msg.seq))
            print('sending waypoint {0}'.format(msg.seq))

    def add_waypoint_to_mission(self, waypoint_tuple):
        """
        Add a waypoint to the buffer list of waypoints.

        Parameters
        ----------
        waypoint_tuple: tuple
            Tuple of waypoints formatted as ( lattitude, longitude )
        """
        self.wp_items.append(waypoint_tuple)

    def clear_waypoints(self):
        """
        Clear all added waypoints from memory.
        """
        self.autopilot.waypoint_clear_all_send()
        self.wp.clear()

    def return_to_launch(self):
        """
        Return to the launch location
        """
        cmd = CommandMavlink.CMD_NAV_RETURN_TO_LAUNCH
        self.autopilot.mav.command_long_send(
            self.system_id,  # autopilot system id
            self.component_id,  # autopilot component id
            cmd,  # command id
            0,  # confirmation
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0,  # Empty
            0   # Empty
        )

    # Wait for server connection
    def wait_conn(self, master):
        msg = None
        while not msg:
            msg = master.recv_msg()

    time.sleep(0.5)
