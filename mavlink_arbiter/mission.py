#!/bin/python3

# ===================================================== #
#        Mission arbiter class to log mission           #
#   execution data in the course of the mission timer.  #
#                                                       #
#                   Author: David Kroell                #
#                  Version: 2.1.5                       #
# ===================================================== #

import sys
import _thread as thread

from multiprocessing import Queue, Process
from google.protobuf import json_format

from fdg.mavlink_arbiter.utils import Utils
from fdg.proto import dumbo_pb2
from fdg.mavlink_arbiter import http_handeler
from fdg.client.fdg_client import Client


class Mission( object ):
    """
    Mission Class sequence that involves
    """

    def __init__(self):
        """
        Constructor for the Mission class to deal with the server.
        """

        self.util = Utils()
        self.telemetry_buffer = Queue()

        # GPS sequence process that is run.
        self.process_gps_sequence = Process(target=self._post_gps_sequence, args=( ))

    def _post_gps_sequence( self ):
        """
        Post GPS sequence that is run continuously to update the GPS
        Position of the UGV.

        Parameters
        ----------
        http_const: fdg.mavlink_arbiter.http_handeler.HTTPConstants
            HTTP Constants instance that is passed along to the webserver.
        """

        HTTPConst = http_handeler.HTTPConstants()
        client = Client(
            url="http://localhost:8080",
            username="admin",
            password="password",
        )

        # HTTP Constants and REST Static API handelling.
        try:
            WebAppAPIPort = 5002
            thread.start_new_thread(http_handeler.WebappAPIInit, (WebAppAPIPort,))

            while True:
                try:
                    if not (self.telemetry_buffer.empty()):
                        gps = self.telemetry_buffer.get()
                        HTTPConst.set_telemetry(gps)
                        client.post_gps_update(
                            longitude=gps['telemetry']['longitude'],
                            latitude=gps['telemetry']['latitude'],
                            heading=gps['heading']
                        )

                except KeyboardInterrupt:
                    self.util.errLog("Executing keyboard interrupt program: TERMINATE")
                    sys.exit(0)
        except OSError as e:
            self.util.errLog(f'{e}')

    def update_gps( self, lat, lon, hdg ):
        """
        Updates the telemetry buffer with a new GPS object ready for posting.

        Parameters
        ----------
        lon: float
            Longitude reading of the telemetry GPS update.
        lat: float
            Latitude reading of the telemetry GPS update.
        hdg: int
            Heading of the GPS position that the compass is in.
        Returns
        -------
        response: GPSPosition
            Return the GPS position object that is pushed to the buffer.
        """
        gps_position = dumbo_pb2.GPSPosition( )

        gps_position.telemetry.longitude = lon
        gps_position.telemetry.latitude  = lat
        gps_position.heading             = float( hdg )

        self.telemetry_buffer.put( json_format.MessageToDict( gps_position ) )

    def __del__(self):
        """
        Destructor that kills the mission class object.
        """
        print("Destruct the Mission")


if __name__ == "__main__":
    Mission()
