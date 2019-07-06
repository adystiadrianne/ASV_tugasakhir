# Import mavutil
import time
from pymavlink import mavutil, mavwp

waypoint_count = 0
missionLat = 0.0
missionLon = 0.0
missionAlt = 0.0
# Create the connection
master = mavutil.mavlink_connection("COM5", baud=115200)

def setMode(mode):
    global master

    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    # Check ACK
    ack = False
    while not ack:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

def readWayPoints():
    global master, waypoint_count
    
    # Read Waypoint from airframe
    master.waypoint_request_list_send()
    waypoint_count = 0

    msg = master.recv_match(type=['MISSION_COUNT'],blocking=True)
    waypoint_count = msg.count
    print(msg.count)

    for i in range(waypoint_count):
        master.waypoint_request_send(i)
        msg = master.recv_match(type=['MISSION_ITEM'],blocking=True)
        print('Receving waypoint {0}'.format(msg.seq))
        print(msg)

    master.mav.mission_ack_send(master.target_system, master.target_component, 0) # OKAY


# Set Home location
def cmd_set_home(home_location, altitude):
    global master
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) # alt

def cmd_get_home():
    global master
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    print(msg)
    msg = master.recv_match(type=['HOME_POSITION'],blocking=False)
    print(msg)
    return (msg.latitude, msg.longitude, msg.altitude)

def writeWayPoints(waypoints):
    global master
    
    master.wait_heartbeat() 
    print("HEARTBEAT OK\n")

    # Make Waypoints
    wp = mavwp.MAVWPLoader()

    seq = 0
    for waypoint in enumerate(waypoints):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        seq = waypoint[0]
        lat, lon = waypoint[1]
        altitude = 15 # 15 meter
        autocontinue = 1
        current = 0
        param1 = 15.0 # minimum pitch
        if seq == 0: # first waypoint to takeoff
            current = 1
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, current, autocontinue, param1, 0, 0, 0, lat, lon, altitude)
        elif seq == len(waypoints) - 1: # last waypoint to land
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        else:
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        wp.add(p)

    # Send Waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

    msg = master.recv_match(type=['MISSION_ACK'],blocking=True) # OKAY
    print(msg.type)
       

master.wait_heartbeat() 
print("HEARTBEAT OK\n")

waypoints = [
        (37.5090904347, 127.045094298),
        (37.509070898, 127.048905867),
        (37.5063678607, 127.048960654),
        (37.5061713129, 127.044741936),
        (37.5078823794, 127.046914506)
        ]

"""
home_location = waypoints[0]
cmd_set_home(home_location, 0)
msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
print(msg)
print('Set home location: {0} {1}'.format(home_location[0], home_location[1]))
"""

#writeWayPoints(waypoints)
readWayPoints()
        
setMode("AUTO")   

# Monitor mission progress
nextwaypoint = 0

def handle_mission_current(msg, nextwaypoint):
    if msg.seq > nextwaypoint:
        print("Moving to waypoint %s" % msg.seq)
        nextwaypoint = msg.seq + 1
        print("Next Waypoint %s" % nextwaypoint)
    return nextwaypoint

def handle_global_position_int(msg):
    global missionLon, missionLat, missionAlt
    
    missionLat = msg.lat
    missionLon = msg.lon
    missionAlt = msg.relative_alt
    print("Latitude: ", missionLat / 100.0)
    print("Longitude: ", missionLon / 100.0)
    print("Altitude: ", missionAlt / 1000.0)
    #Break and return from function just below target altitude.        
    #if msg.relative_alt>=15*1000*0.95: 
    #    print "Reached target altitude"
    return

relative_alt = 0
time.sleep(0.2)

while True:
    msg = master.recv_match(type=['GLOBAL_POSITION_INT', 'MISSION_CURRENT', 'MISSION_COUNT', 'HEARTBEAT'],blocking=True,timeout=0.5)
    try:
        if msg is None:
            print("Message None")
            break
        #print(msg.to_dict())
        if msg.get_type() == 'HEARTBEAT':
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
        #if msg.get_type() == 'MISSION_COUNT':
            #waypoint_count = msg.count
        #print "waypoint_count %s" % waypoint_count
       
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            handle_global_position_int(msg)
            relative_alt = msg.relative_alt
        if msg.get_type() == 'MISSION_CURRENT':
            nextwaypoint = handle_mission_current(msg, nextwaypoint)
            #print 'Next Waypoint %s' % nextwaypoint
            if nextwaypoint >= waypoint_count - 1:
                print("Reached land altitude")
                break
        time.sleep(0.2)
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4) # send heart beat to airframe per 1 sec

    except KeyboardInterrupt:
        break

setMode("MANUAL")    
 