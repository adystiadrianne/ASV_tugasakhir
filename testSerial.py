from pymavlink import mavutil
import sys

def handle_heartbeat(msg):
  mode = mavutil.mode_string_v10(msg)
  is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
  is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

def handle_rc_raw(msg):
  channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)

def handle_hud(msg):
  hud_data = (msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb)
  print("Aspd\tGspd\tHead\tThro\tAlt\tClimb")
  print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data)

def handle_attitude(msg):
  attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed)
  print("Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd")
  print("%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data)
  
def handle_gps(msg):
  gps_data = (msg.lat, msg.lon, msg.alt)
  print("Lat\t\tLon\t\tAlt")
  print("%s\t%s\t%s" % gps_data)  

def read_loop(m):

  while(True):

    # grab a mavlink message
    msg = m.recv_match(blocking=True)
    if not msg:
      return
    mdata = msg.to_dict()
    #print(mdata)
    # handle the message based on its type
    msg_type = msg.get_type()
    if msg_type == "BAD_DATA":
      if mavutil.all_printable(msg.data):
        print("Bad Data")
        #sys.stdout.write(msg.data)
        #sys.stdout.flush()
    """
    elif msg_type == "RC_CHANNELS_RAW": 
      handle_rc_raw(msg)
    elif msg_type == "HEARTBEAT":
      handle_heartbeat(msg)
    elif msg_type == "VFR_HUD":
      handle_hud(msg)
      print()
    elif msg_type == "ATTITUDE":
      handle_attitude(msg)
      print()
    """
    if msg_type == "GLOBAL_POSITION_INT":
      print("Latitude : ", float(msg.lat)/10000000)
      print("Longitude: ", float(msg.lon)/10000000)
      print("Altitude : ", float(msg.alt)/1000)
      print()
    
  
# create a mavlink serial instance
master = mavutil.mavlink_connection("COM12", baud=57600)

# wait for the heartbeat msg to find the system ID
master.wait_heartbeat()

# request data to be sent at the given rate
master.mav.request_data_stream_send(master.target_system, master.target_component, 
 mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

# enter the data loop
read_loop(master)
