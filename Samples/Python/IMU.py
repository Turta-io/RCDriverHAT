#!/usr/bin/env python3

#This sample demonstrates reading 3D gyro and acceleration data.
#Install RC Driver HAT library with "pip3 install turta-rcdriverhat"

from time import sleep
from turta_rcdriverhat import Turta_IMU

#Initialize
imu = Turta_IMU.IMU()

try:
    while True:
        #Read gyro & accel data
        imu_all = imu.read_gyro_accel_xyz()

        #Print the readings
        print("Pitch.....: " + str(imu_all[0]).zfill(5))
        print("Roll......: " + str(imu_all[1]).zfill(5))
        print("Yaw.......: " + str(imu_all[2]).zfill(5))
        print("X-Accel...: " + str(imu_all[3]).zfill(5))
        print("Y-Accel...: " + str(imu_all[4]).zfill(5))
        print("Z-Accel...: " + str(imu_all[5]).zfill(5))

        #Wait
        sleep(0.1)

#Exit on CTRL+C
except KeyboardInterrupt:
    print('Bye.')
