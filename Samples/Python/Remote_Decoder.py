#!/usr/bin/env python3

#This sample demonstrates reading remote controller PWM channels from an RC receiver.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Option one: Read all PWM channels in one shot
        #Useful when you need all the reading wihtin a minimal time
        pwm_all = rc.read_pwms()

        #Print the readings
        if pwm_all is not None:  #Check if data is received
            print("PWM St....: " + str(pwm_all[0]))
            print("PWM Th....: " + str(pwm_all[1]))
            print("PWM A3....: " + str(pwm_all[2]))
            print("PWM A4....: " + str(pwm_all[3]))

        #Wait
        sleep(1.0)

        #Option two: Read PWM inputs one by one
        #Useful when you need only one reading
        pwm_s = rc.read_pwm(Turta_RCDriver.PWM_IN.STEERING)

        #Print the reading
        if pwm_s is not None:  #Check if data is received
            print("PWM St....: " + str(pwm_s))

        #Wait
        print("-----")
        sleep(1.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
    