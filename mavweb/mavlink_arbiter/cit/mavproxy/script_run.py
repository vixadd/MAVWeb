#
# Running this script to test specific commands
# on the SITL program.
#
# Waypoints are transmitted to the APM through MAVLink commands. Transmission follows the protocol defined at Waypoint Protocol.

# This file is used for DAVID KROELLS personal notes on mavproxy handeling and module development. Please do not change this file.

# Using the mission_item_send function, a waypoint may be structured and transmitted to the APM through MAVLink. The mission_item_send function accepts multiple parameters and a specified action.

# def mission_item_send(self, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z):

#     Message encoding a mission item. This message is emitted to announce
#     the presence of a mission item and to set a mission
#     item on the system. The mission item can be either in
#     x, y, z meters (type: LOCAL) or x:lat, y:lon,
#     z:altitude. Local frame is Z-down, right handed (NED),
#     global frame is Z-up, right handed (ENU).
#     http://qgroundcontrol.org/mavlink/waypoint_protocol
  
#     target_system             : System ID (uint8_t)
#     target_component          : Component ID (uint8_t)
#     seq                       : Sequence (uint16_t)
#     frame                     : The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h (uint8_t)
#     command                   : The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs (uint16_t)
#     current                   : false:0, true:1 (uint8_t)
#     autocontinue              : autocontinue to next wp (uint8_t)
#     param1                    : PARAM1 / For NAV command MISSIONs: Radius in which the MISSION is accepted as reached, in meters (float)
#     param2                    : PARAM2 / For NAV command MISSIONs: Time that the MAV should stay inside the PARAM1 radius before advancing, in milliseconds (float)
#     param3                    : PARAM3 / For LOITER command MISSIONs: Orbit to circle around the MISSION, in meters. If positive the orbit direction should be clockwise, if negative the orbit direction should be counter-clockwise. (float)
#     param4                    : PARAM4 / For NAV and LOITER command MISSIONs: Yaw orientation in degrees, [0..360] 0 = NORTH (float)
#     x                         : PARAM5 / local: x position, global: latitude (float)
#     y                         : PARAM6 / y position: global: longitude (float)
#     z                         : PARAM7 / z position: global: altitude (float)


#     return self.send(self.mission_item_encode(target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z))

# Mission item actions may be seen in the Common Message Documentation under MAV_CMD.
# MAV_CMD_NAV_WAYPOINT

#         Mission Param #1 Hold time in decimal seconds.
#         Mission Param #2 Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
#         Mission Param #3 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
#         Mission Param #4 Desired yaw angle at MISSION
#         Mission Param #5 Latitude
#         Mission Param #6 Longitude
#         Mission Param #7 Altitude

# The MAV_CMD_NAV_WAYPOINT action description for Ardupilot is shown in the GCS_MAVLink library common.xml code.

# NOTE: There is a conflict between the "mission_item_send" description and MAV_CMD_NAV_WAYPOINT description. Param 1 and 2 are flipped.

# The "handle_message" function of GCS_MAVLink.pde processes the MAV_CMD_NAV_WAYPOINT parsing. Specifically, the case "MAVLINK_MSG_ID_MISSION_ITEM:" specifies how the mission_item is received from the APM code (GCS).

# Through this code, it is shown that Params 2,3,4 from the MAV command are not used. Param 1 designates the Hold Time, following the same structure as defined in the Message Documentations, but not the mission_item_send function descriptions.

# To define Yaw angle, a second waypoint should be generated at the same position to change orientation (TBD). To define Acceptance Radius, a parameter must be set on the APM, WPNAV_RADIUS (default is 2 meters).
# MAV_CMD_CONDITION_YAW

#         Mission Param #1 target angle: [0-360], 0 is north
#         Mission Param #2 speed during yaw change:[deg per second]
#         Mission Param #3 direction: negative: counter clockwise, positive: clockwise [-1,1]
#         Mission Param #4 relative offset or absolute angle: [ 1,0]
#         Mission Param #5 Empty
#         Mission Param #6 Empty
#         Mission Param #7 Empty

# The MAV_CMD_CONDITION_YAW action description for Ardupilot is shown in the GCS_MAVLink library common.xml code.

# The "do_yaw" function in the APM file commands_logic.pde is used to handle the condition yaw waypoint.
# Pages 6

#     Home
#     Exploring Param Control
#     Exploring Waypoint Control
#     Mavlink Usage
#     Noticed APM Behaviors
#     Simple Commands While Running

# Clone this wiki locally

#
#
from pymavlink import mavutil
from pymavlink import mavwp
import os
import time

# def printMissions():
# 	pass

# def writeToFile(argString, argFile):
# 	pass

# #generate output filename from scriptname and time
# fileOut = open("log_{0}_{1}_{2}.txt".format(str(os.path.splitext(__file__)[0]), time.strftime("%Y%m%d", time.localtime()), time.strftime("%H%M%S", time.localtime())), "w")

# writeToFile("Program start", fileOut)

# # create a mavlink serial instance
# master = mavutil.mavlink_connection("udpin:localhost:14550", baud=125571)

# # wait for the heartbeat msg to find the system ID
# master.wait_heartbeat()
# writeToFile(str("Heartbeat from APM (system {0} component {1})".format(master.target_system, master.target_component)), fileOut)

# writeToFile("Sending all stream request for rate {0}".format(20), fileOut)
# for i in range(0, 3) :
# 	master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

# writeToFile("wait for gps fix to acquire current location ...", fileOut)
# master.wait_gps_fix()
# currentLoc = master.location()
# writeToFile(currentLoc, fileOut)
# #currentLoc = mavutil.location(lat = 50.891215, lng = 3.499862)

# printMissions()

# writeToFile("clearing missions", fileOut)
# master.waypoint_clear_all_send()

# #waypoint write protocol : send count; as much mission items demanded as count; mission ack receive
# writeToFile("populating mission items", fileOut)

# master.mav.mission_count_send(master.target_system, master.target_component, 4)
# msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)

# writeToFile("mission request 0 from copter", fileOut)
# writeToFile(msgMissionRequest, fileOut)

# # 1
# master.mav.mission_item_send(
# 	master.target_system, 
# 	master.target_component, 
# 	0, 0, 16, 0, 
# 	1, 0, 0, 0, 0, 
# 	currentLoc.lat, 
# 	currentLoc.lng, 
# 	0
# )

# msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
# # writeToFile("mission request 1 from copter", fileOut)
# # writeToFile(msgMissionRequest, fileOut)


# #MAV_CMD_NAV_WAYPOINT
# # 2
# master.mav.mission_item_send(
# 	master.target_system,
# 	master.target_component,
# 	1, 3, 16, 0, 2,
# 	0, 0, 0, 0,
# 	-35.362447,
# 	149.164354,
# 	0
# )

# msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)

# writeToFile("mission request 3 from copter", fileOut)
# writeToFile(msgMissionRequest, fileOut)


# #MAV_CMD_NAV_LOITER_TIME for 10 sec at drop coordinates (args)
# # 3
# master.mav.mission_item_send(
# 	master.target_system,
# 	master.target_component,
# 	2, 3, 19, 0, 1, 10,
# 	0, 0, 0,
# 	-35.363963,
# 	149.164230,
# 	0
# )

# # msgMissionRequest = master.recv_match(type = "MISSION_REQUEST", blocking = True)
# msgMissionAck = master.recv_match(type = "MISSION_ACK", blocking = True)

# writeToFile("mission request 9 from copter", fileOut)
# writeToFile(msgMissionRequest, fileOut)


#MAV_CMD_NAV_RETURN_TO_LAUNCH
# 4
# master.mav.mission_item_send(
# 	master.target_system,
# 	master.target_component,
# 	4, 3, 20, 0, 1,
# 	0, 0, 0, 0, 0,
# 	0, 0
# )

# msgMissionAck = master.recv_match(type = "MISSION_ACK", blocking = True)

# writeToFile("mission ack copter", fileOut)
# writeToFile(msgMissionAck, fileOut)


#mavwp set-up
wp = mavwp.MAVWPLoader()

mavutil.set_dialect("ardupilotmega")

autopilot = mavutil.mavlink_connection('udpin:localhost:14551')

msg = None

# wait for autopilot connection
while msg is None:
        msg = autopilot.recv_msg()

print(msg)

# The values of these heartbeat fields is not really important here
# I just used the same numbers that QGC uses
# It is standard practice for any system communicating via mavlink emit the HEARTBEAT message at 1Hz! Your autopilot may not behave the way you want otherwise!
autopilot.mav.heartbeat_send(
6, # type
8, # autopilot
192, # base_mode
0, # custom_mode
4, # system_status
3  # mavlink_version
)

autopilot.mav.command_long_send(
autopilot.target_system, 
autopilot.target_component, 
400, # command id, ARM/DISARM
0, # confirmation
1, # arm!
0,0,0,0,0,0 # unused parameters for this command
)

time.sleep(2)

autopilot.mav.command_long_send(
autopilot.target_system, 
autopilot.target_component, 
400, # command id, ARM/DISARM
0, # confirmation
0, # disarm!
0,0,0,0,0,0 # unused parameters for this command
)

currentLoc = autopilot.location()

autopilot.mav.mission_item_send(
	autopilot.target_system, 
	autopilot.target_component, 
	0, 0, 16, 0, 
	1, 0, 0, 0, 0, 
	currentLoc.lat, 
	currentLoc.lng, 
	0
)

waypoints = [
	(-35.360964, 149.164643),
	(-35.361155, 149.164302),
	(-35.364311, 149.164178),
	(-35.363659, 149.164437),
	(-35.362155, 149.163214)
]

frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
radius = 10
seq = 1

for wp_item in waypoints:
	lat, lon = wp_item
	alt = 0

	wp.add(
		mavutil.mavlink.MAVLink_mission_item_message(
			autopilot.target_system,
			autopilot.target_component,
			seq,
			frame,
			mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
			0,0,0,radius,0,0,
			lat, lon, alt
		)
	)

autopilot.waypoint_clear_all_send()
autopilot.waypoint_count_send(wp.count())

for i in range(wp.count()):
	msg = autopilot.recv_match(type=['MISSION_REQUEST'], blocking=True)
	autopilot.mav.send(wp.wp(msg.seq))
	print('sending waypoint {0}'.format(msg.seq))

autopilot.set_mode_auto()

# MOVE THE DAMN THING!!! :D
autopilot.mav.command_long_send(
autopilot.target_system, 
autopilot.target_component, 
400, # command id, ARM/DISARM
0, # confirmation
1, # arm!
0,0,0,0,0,0 # unused parameters for this command
)
