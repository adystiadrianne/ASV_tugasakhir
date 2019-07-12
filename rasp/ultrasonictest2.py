import RPi.GPIO as GPIO

import time

 
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


TRIG = [23, 10]

ECHO = [24, 9]

distance = [2000.0,2000.0]

pulse_start = [0.0,0.0]

pulse_end = [0.0,0.0]

pulse_duration = [0.0,0.0]

 

for i in range(len(ECHO)):

   GPIO.setup(TRIG[i],GPIO.OUT)
   GPIO.setup(ECHO[i],GPIO.IN)

for i in range(len(ECHO)):

   GPIO.output(TRIG[i],False)
   
while True:
   for i in range(len(ECHO)):
      time.sleep(0.2)
     
      GPIO.output(TRIG[i],True)
      time.sleep(0.00001)

      GPIO.output(TRIG[i],False)

      while GPIO.input(ECHO[i])==0:
         pulse_start[i]=time.time()

      while GPIO.input(ECHO[i])==1:
         pulse_end[i]=time.time() 

      pulse_duration[i]=pulse_end[i]-pulse_start[i]
      distance[i]=pulse_duration[i]*17150
      distance[i]=round(distance[i],2)

      print ("Distance: ", str(distance) + " cm")

