#!/usr/bin/env python3

#This sample demonstrates controlling LED lighting.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Option one: Set all the LED lights in one shot
        #0 is LED off, 4095 is LED full bright
        #Parameters: Front, Read, Stop, Left, Right
        rc.set_leds([1000, 1000, 1000, 1000, 1000])
        print("LED Brightness: 1000/4096")

        #Wait
        sleep(1.0)

        #Option two: Set the LED lights one by one
        #0 is LED off, 4095 is LED full bright
        rc.set_led(Turta_RCDriver.LED_OUT.FRONT, 2000)
        rc.set_led(Turta_RCDriver.LED_OUT.REAR, 2000)
        rc.set_led(Turta_RCDriver.LED_OUT.STOP, 2000)
        rc.set_led(Turta_RCDriver.LED_OUT.LEFT, 2000)
        rc.set_led(Turta_RCDriver.LED_OUT.RIGHT, 2000)
        print("LED Brightness: 2000/4096")

        #Wait
        sleep(1.0)

        #Showcase: Fade the front LED
        print("LED is fading in")
        for b in range(0, 1000, 5):
            rc.set_led(Turta_RCDriver.LED_OUT.FRONT, b)
            sleep(0.05)

        print("LED is fading out")
        for b in range(1000, 0, -5):
            rc.set_led(Turta_RCDriver.LED_OUT.FRONT, b)
            sleep(0.05)

        #Wait
        sleep(1.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
