#!/usr/bin/env python3

#This sample demonstrates using PWM outputs on controller port.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Option one: Set all the pulses in one shot
        #-128 is 1000us, 0 is 1500us, 128 is 2000us
        #Channels: Steering, Throttle, Aux 3, Aux 4
        print("PWMs are S: 32, T: 64, 3: 96, 4: 128")
        rc.set_pwms([32, 64, 96, 128])

        #Wait
        sleep(5.0)

        #Option two: Set the pulses one by one
        #-128 is 1000us, 0 is 1500us, 128 is 2000us
        print("PWMs are S: -32, T: -64, 3: -96, 4: -128")
        rc.set_pwm(Turta_RCDriver.PWM_OUT.STEERING, -32)
        rc.set_pwm(Turta_RCDriver.PWM_OUT.THROTTLE, -64)
        rc.set_pwm(Turta_RCDriver.PWM_OUT.AUX_3, -96)
        rc.set_pwm(Turta_RCDriver.PWM_OUT.AUX_4, -128)

        #Wait
        sleep(5.0)

        #Demo: Changing pulses
        print("Steering servo ch: 1100 to 1900us")
        for p in range(-100, 100, 2):
            rc.set_pwm(Turta_RCDriver.PWM_OUT.STEERING, p)
            sleep(0.05)

        print("Steering servo ch: 1900 to 1100us")
        for p in range(100, -100, -2):
            rc.set_pwm(Turta_RCDriver.PWM_OUT.STEERING, p)
            sleep(0.05)

        #Wait
        sleep(1.0)

        print("Turn off pulse generation")
        rc.set_pwms_off()

        #Wait
        sleep(5.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
