#!/usr/bin/env python3

#This sample demonstrates setting fan speed according to CPU temperature.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_IMU
from turta_rcdriverhat import Turta_RCDriver

#Initialize
imu = Turta_IMU.IMU()
rc = Turta_RCDriver.RCDriver()

try:
    while True:
        #Read IMU temperature
        imu_temp_c = imu.read_temp()
        imu_temp_f = imu.read_temp(True)

        #Read CPU temperature
        cpu_temp_c = rc.read_cpu_temp()
        cpu_temp_f = rc.read_cpu_temp(True)

        #Set fan speed
        #0 is fan off, 100 is maximum speed
        fan_speed = int(8 * cpu_temp_c) - 300
        rc.set_fan(fan_speed)

        #Print the readings
        print("IMU Temp..: " + str(imu_temp_c) + "C" + \
              " / " + str(imu_temp_f) + "F")
        print("CPU Temp..: " + str(cpu_temp_c) + "C" + \
              " / " + str(cpu_temp_f) + "F")

        print("Fan speed.: " + str(fan_speed) + "%")

        #Wait
        print("-----")
        sleep(1.0)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
