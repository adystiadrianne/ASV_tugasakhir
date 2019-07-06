import serial
import threading
import time

port1 = serial.Serial("/dev/ttyUSB0", 57600)
port2 = serial.Serial("/dev/ttyUSB1", 57600)

def serialBroker():
  print("Serial Broker thread started ...")
  while True:
    mdata = port1.readline()
    
	port2.write(mdata)

th01 = threading.Thread(target = serialBroker)
th01.daemon = True
th01.start()

try:
  while True:
    time.sleep(1)
	
except KeyboardInterrupt:
  port1.close()
  port2.close()