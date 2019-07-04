from pymavlink import mavutil, mavwp

# lat = (-6.9546329, -6.9549265)
# lon = (107.6339599, 107.6339461)
lat = (-6.9537420, -6.9537420, -6.9533000, -6.9729727, -6.9729527)
lon = (107.6347370, 107.6347370, 107.6356155, 107.6316525, 107.6310638)
N = 5
master = mavutil.mavlink_connection("COM5", baud=57600)
master.wait_heartbeat(blocking=True)                                       
wp = mavwp.MAVWPLoader()                                                    
seq = 1
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
radius = 10
for i in range(N):                  
            wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                         master.target_component,
                         seq,
                         frame,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, radius, 0, 0,
                         lat[i],lon[i],0))
            seq += 1                                                                       

master.waypoint_clear_all_send()                                     
master.waypoint_count_send(wp.count())                          

for i in range(wp.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)             
            master.mav.send(wp.wp(msg.seq))                                                                      
            print('Sending waypoint '+format(msg.seq))          
mission_ack_msg = master.recv_match(type=['MISSION_ACK'], blocking=True)
mdata = mission_ack_msg.to_dict()
print(mdata)


