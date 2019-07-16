#!/usr/bin/env python3

#This sample demonstrates reading analog inputs.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Option one: Read all analog inputs in one shot
        #Useful when you need all the readings within a minimal time
        an_all = rc.read_analogs()

        #Print the readings
        if an_all is not None:  #Check if data is received
            print("AN 1......: " + str(an_all[0]))
            print("AN 2......: " + str(an_all[1]))
            print("AN 3......: " + str(an_all[2]))
            print("AN 4......: " + str(an_all[3]))

        #Wait
        sleep(1.0)

        #Option two: Read analog inputs one by one
        #Useful when you need only one reading
        an_one = rc.read_analog(Turta_RCDriver.ANALOG_IN.CH_1)

        #Print the reading
        if an_one is not None:  #Check if data is received
            print("AN 1......: " + str(an_one))

        #Wait
        sleep(1.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
