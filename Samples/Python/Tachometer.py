#!/usr/bin/env python3

#This sample demonstrates reading vehicle speed using tachometer inputs.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Option one: Read both tach data in one shot
        #Useful when you need all the readings within a minimal time
        tach_all = rc.read_tachs()

        #Print the readings
        if tach_all is not None:  #Check if data is received
            print("Tach L....: " + str(tach_all[0]) + " rpm")
            print("Tach R....: " + str(tach_all[1]) + " rpm")

        #Wait
        sleep(1.0)

        #Option two: Read tachometer data one by one
        #Useful when you need only one reading
        tach_r = rc.read_tach(Turta_RCDriver.TACH_IN.RIGHT)

        #Print the reading
        if tach_r is not None:  #Check if data is received
            print("Tach R....: " + str(tach_r) + " rpm")

        #Wait
        sleep(1.0)

        #Option three: Calculate speed as KMH
        #Parameters: Ticks per revolution, Diamater in cm, MPH Output (Empty for KM/H)
        speed_all = rc.read_tachs(1, 10)

        #Print the readings
        if speed_all is not None:  #Check if data is received
            print("Speed L...: " + str(speed_all[0]) + " km/h")
            print("Speed R...: " + str(speed_all[1]) + " km/h")

        #Wait
        sleep(1.0)

        #Option four: Calculate speed as MPH
        #Parameters: Ticks per revolution, Diamater in cm, MPH Output (Empty for KM/H)
        speed_all = rc.read_tachs(1, 10, True)

        #Print the readings
        if speed_all is not None:  #Check if data is received
            print("Speed L...: " + str(speed_all[0]) + " mph")
            print("Speed R...: " + str(speed_all[1]) + " mph")

        #Wait
        sleep(1.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
