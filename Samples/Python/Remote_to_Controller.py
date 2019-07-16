#!/usr/bin/env python3

#This sample demonstrates transferring RC remote inputs to steering servo and ESC.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
import math
from turta_rcdriverhat import Turta_RCDriver

#Initialize
rc = Turta_RCDriver.RCDriver()

try:
    #Activate fan at 50% speed
    rc.set_fan(50)

    while True:
        #Read multiple PWMs
        pwm_all = rc.read_pwms()
        if pwm_all is not None:  #Check if data is received

            #Convert PWM timespans to PWM output that RC servo motors and ESCs understand
            for p in range(4):
                if pwm_all[p] == 0:
                    pwm_all[p] = 1500
                elif pwm_all[p] < 1000:
                    pwm_all[p] = 1000
                elif pwm_all[p] > 2000:
                    pwm_all[p] = 2000

                temp = math.floor((pwm_all[p] * 0.256) - 384)

                pwm_all[p] = temp

            #Set Multiple PWMs
            rc.set_pwms(pwm_all)

            #Print the output data
            print("PWM S.....: " + str(pwm_all[0]))
            print("PWM T.....: " + str(pwm_all[1]))
            print("PWM 3.....: " + str(pwm_all[2]))
            print("PWM 4.....: " + str(pwm_all[3]))

        #Wait
        print("-----")
        sleep(0.2)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
