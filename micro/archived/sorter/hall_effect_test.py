from machine import ADC
from utime import sleep
adc = ADC(28)

while True:
    sleep(1)
    print(adc.read_u16())