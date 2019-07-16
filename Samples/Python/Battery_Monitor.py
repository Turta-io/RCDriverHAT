#!/usr/bin/env python3

#This sample demonstrates measuring battery Voltage.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Read battery Voltage
        voltage = rc.read_voltage()

        #Print the reading
        print("Battery...: " + str(voltage) + "V")

        #Wait
        sleep(2.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
