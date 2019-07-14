#!/bin/python3
# ================================================== #
#   Main class for the Mavlink Arbiter process.      #
#                                                    #
#                Author: David Kroell                #
#               Version: 3.0.0                       #
#                                                    #
#        For use by the Future Defense Group         #
#                                                    #
# ================================================== #

import sys
import json

from multiprocessing import Process, Queue

from fdg.mavlink_arbiter.utils import Utils
from fdg.mavlink_arbiter.mav import Mavlink
from fdg.mavlink_arbiter.mission import Mission
from fdg.mavlink_arbiter.singleton import singleton

MAVL_INCOMING = "MAVL_INCOMING"
MAVL_OUTGOING = "MAVL_OUTGOING"
HOST          = "host"
PORT          = "port"


@singleton
class Main(object):
    util = Utils()

    def __init__(self):
        """
        Initialization for the Min Mavlink arbiter class.
        """
        self.util = Utils()

        try:
            with open('./config.json') as data_file:
                constants = json.load(data_file)

            self.util.succLog("Successfully extracted config data")

        except IOError:
            self.util.errLog("WARNING: Config file not found!")
            self.util.errLog("Aborting operation! Make sure config.json exists in the /src directory.")
            sys.exit(0)

        # Instantiate the mavlink retriever module.
        self.mavl = Mavlink(
            host=constants[MAVL_INCOMING][HOST],
            pin=constants[MAVL_INCOMING][PORT],
            pout=constants[MAVL_OUTGOING][PORT]
        )

        # Instantiate the mission environment
        self.mission = Mission( )

        self.util.succLog("Starting Arbitration Process")
        
        # Start a new thread that runs all other sub-processes.
        self.proc_arbitrate = Process(target=self._start_process, args=())

        self.display_queue = Queue()
        self.start()

        self._start_process()

    def start(self):
        """
        Starts all of the processes needed for
        """
        self.util.log("Starting the process for arbitration.")
        # self.proc_arbitrate.start()
        self.mavl.procMav.start()
        self.mission.process_gps_sequence.start()
        self.util.log("Process for arbitration started.")

    def mavproxy(self):
        """
        Getter method for the mavlink instance tied to Mavproxy.

        Returns
        -------
        response: fdg.mavlink_arbiter.mav.Mavlink
            Mavlink object associated with this main arbiter.
        """
        return self.mavl

    def _start_process(self):
        """
        Initiates the buffers required to couple with the arbitration process.
        Couples with the Mavlink process that runs to grab the mavlink packages from MavProxy.
        """
        while True:
            try:
                udp_packet = self.mavl.get_mav_packet()
                telemetry    = {}
                if udp_packet is not None:
                    if udp_packet.get_type() == "GLOBAL_POSITION_INT":
                        telem_packet = udp_packet

                        # populate the coordinate elements of the telemetry module
                        telemetry['longitude']  = float(telem_packet.lon)/10000000
                        telemetry['latitude']   = float(telem_packet.lat)/10000000
                        telemetry['heading']    = float(telem_packet.hdg)/1000

                        # Update GPS Position.
                        self.mission.update_gps(
                            lat=telemetry['latitude'],
                            lon=telemetry['longitude'],
                            hdg=int(telemetry['heading']),
                        )

            except KeyboardInterrupt:
                break
            except sys.stderr:
                self.util.errLog("ERR: Exit main on sys call. Terminating Sys call.")

    def __del__(self):
        # Join processes before termination.
        # self.proc_arbitrate.join()
        self.mavl.procMav.join()
        self.mission.process_gps_sequence.join()

        # Terminate processes.
        # self.proc_arbitrate.close()
        self.mavl.procMav.close()
        self.mission.process_gps_sequence.close()


if __name__ == "__main__":
    main = Main()
