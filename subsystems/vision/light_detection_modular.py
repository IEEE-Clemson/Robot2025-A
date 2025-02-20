import machine
from machine import Pin
import time

def detect_light():
    # constants
    PR_PIN = 1 		# ADC Photoresistor Pin 
    PCT_INC = 1.03	# % increase from avg for detection
    POLL_RATE = 0.5	# # of times/second values poll

    # set pins
    photoresistor = machine.ADC(PR_PIN)

    avg = 0
    n = 1
    lightsum = 0

    LOOP = True
    light = 0
    while LOOP:
        light = photoresistor.read_u16()        
        if ((light > (avg*PCT_INC)) and (n > 1)):
            LOOP = False
            continue
        
        lightsum = lightsum + light
        avg = lightsum / n
        n = n + 1

        time.sleep(POLL_RATE)

    return 1