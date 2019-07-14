#!/usr/bin/env python

#==========================================#
#        HTTP Handeler Class for           #
#    Websocket publication and retrieval   #
#                                          #
#         Author: David Kroell             #
#        Version: 2.0.3                    #
#                                          #
#==========================================#

import flask
import requests

from fdg.mavlink_arbiter.singleton import singleton
from flask import request, jsonify


#===================================
#   HTTP Constants Singleton
#
#  The purpose of this class is to hold
# the variables that are served by the
# http web api
#===================================
@singleton
class HTTPConstants:
    def __init__(self):
        print("Starting the Constants Singleton.")
        self.mission = dict()
        self.mission['initilized'] = True

        self.telemetry = dict()
        self.telemetry['initialized'] = True

        self.obstical_stationary = dict()
        self.obstical_stationary['initialized'] = True

        self.obstical_moving = dict()
        self.obstical_moving['initialized'] = True

    # Getters for the API components
    def get_mission(self):
        return self.mission

    def get_telemetry(self):
        return self.telemetry

    # Setters for the API Components
    def set_mission(self, p_mis):
        self.mission = p_mis

    def set_telemetry(self, p_telem):
        self.telemetry = p_telem


# Initialization of the Web Application.
# Utilizes the Singleton instance of the HTTPConstants that it serves.
def WebappAPIInit(pPort):
    app = flask.Flask(__name__)
    app.config["DEBUG"] = True
    const = HTTPConstants()

    @app.route('/', methods=['GET'])
    def home():
        return '''<h1>FDG Dumbo Web API</h1>
<p>Mission Configuration System API for class constants including such things like telemetry <br> and Mission constants from the competition server.</p>'''

    @app.route('/api/v1/telemetry', methods=['GET'])
    def api_telemetry():
        # const = HTTPConstants()
        return jsonify(const.get_telemetry())

    @app.route('/api/v1/mission', methods=['GET'])
    def api_mission():
        # const = HTTPConstants()
        return jsonify(const.get_mission())

    try:
        app.run(host="0.0.0.0", port=pPort, debug=False, use_reloader=False)
    except OSError as e:
        print(f'{e}')


# ================================
#  Function used to return JSON
# data with Waypoints required by comp.
# ================================
def grabWaypoints():
    obj = requests.get("http://0.0.0.0:5000/api/v1/mission")
    WYP = obj.json()['WYP']

    return WYP
