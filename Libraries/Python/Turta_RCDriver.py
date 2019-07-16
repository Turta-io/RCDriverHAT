# Turta RC Driver HAT Helper for Raspbian.
# Distributed under the terms of the MIT license.

# Python Library for PWM Driver, Power Monitor, ADC, Tachometer and PWM Decoder.
# Version 1.0.0
# Released: July 16th, 2019

# Visit https://docs.turta.io for documentation.

from enum import IntEnum
from smbus import SMBus
from time import sleep

#PWM_OUT: PWM Out Channels
class PWM_OUT(IntEnum):
    STEERING =              0x06
    THROTTLE =              0x0A
    AUX_3 =                 0x0E
    AUX_4 =                 0x12

#PWM_IN: In Channels
class PWM_IN(IntEnum):
    STEERING =              0x01
    THROTTLE =              0x02
    AUX_3 =                 0x03
    AUX_4 =                 0x04

#LED_OUT: Led Out Channels
class LED_OUT(IntEnum):
    FRONT =                 0x3A
    REAR =                  0x32
    STOP =                  0x36
    LEFT =                  0x42
    RIGHT =                 0x3E

#TACH_IN: Tachometer In Channels
class TACH_IN(IntEnum):
    LEFT =                  0x21
    RIGHT =                 0x22

#ANALOG_IN: Analog In Channels
class ANALOG_IN(IntEnum):
    CH_1 =                  0x11
    CH_2 =                  0x12
    CH_3 =                  0x13
    CH_4 =                  0x14

class RCDriver(object):
    """Turta RC Driver HAT combined class for PWM driver, power monitor and MCU."""

    #I2C Slave Addresses
    PCA9685_I2C_ADDRESS =   0x40
    INA219A_I2C_ADDRESS =   0x41
    MCU_I2C_ADDRESS =       0x42

    #PCA9685 PWM Driver Registers
    PCA9685_MODE1 =         0x00
    PCA9685_MODE2 =         0x01
    PCA9685_SUBADR1 =       0x02
    PCA9685_SUBADR2 =       0x03
    PCA9685_SUBADR3 =       0x04
    PCA9685_ALLCALLADR =    0x05
    PCA9685_LED0_ON_L =     0x06
    PCA9685_LED0_ON_H =     0x07
    PCA9685_LED0_OFF_L =    0x08
    PCA9685_LED0_OFF_H =    0x09
    PCA9685_LED1_ON_L =     0x0A
    PCA9685_LED1_ON_H =     0x0B
    PCA9685_LED1_OFF_L =    0x0C
    PCA9685_LED1_OFF_H =    0x0D
    PCA9685_LED2_ON_L =     0x0E
    PCA9685_LED2_ON_H =     0x0F
    PCA9685_LED2_OFF_L =    0x10
    PCA9685_LED2_OFF_H =    0x11
    PCA9685_LED3_ON_L =     0x12
    PCA9685_LED3_ON_H =     0x13
    PCA9685_LED3_OFF_L =    0x14
    PCA9685_LED3_OFF_H =    0x15
    PCA9685_LED4_ON_L =     0x16
    PCA9685_LED4_ON_H =     0x17
    PCA9685_LED4_OFF_L =    0x18
    PCA9685_LED4_OFF_H =    0x19
    PCA9685_LED5_ON_L =     0x1A
    PCA9685_LED5_ON_H =     0x1B
    PCA9685_LED5_OFF_L =    0x1C
    PCA9685_LED5_OFF_H =    0x1D
    PCA9685_LED6_ON_L =     0x1E
    PCA9685_LED6_ON_H =     0x1F
    PCA9685_LED6_OFF_L =    0x20
    PCA9685_LED6_OFF_H =    0x21
    PCA9685_LED7_ON_L =     0x22
    PCA9685_LED7_ON_H =     0x23
    PCA9685_LED7_OFF_L =    0x24
    PCA9685_LED7_OFF_H =    0x25
    PCA9685_LED8_ON_L =     0x26
    PCA9685_LED8_ON_H =     0x27
    PCA9685_LED8_OFF_L =    0x28
    PCA9685_LED8_OFF_H =    0x29
    PCA9685_LED9_ON_L =     0x2A
    PCA9685_LED9_ON_H =     0x2B
    PCA9685_LED9_OFF_L =    0x2C
    PCA9685_LED9_OFF_H =    0x2D
    PCA9685_LED10_ON_L =    0x2E
    PCA9685_LED10_ON_H =    0x2F
    PCA9685_LED10_OFF_L =   0x30
    PCA9685_LED10_OFF_H =   0x31
    PCA9685_LED11_ON_L =    0x32
    PCA9685_LED11_ON_H =    0x33
    PCA9685_LED11_OFF_L =   0x34
    PCA9685_LED11_OFF_H =   0x35
    PCA9685_LED12_ON_L =    0x36
    PCA9685_LED12_ON_H =    0x37
    PCA9685_LED12_OFF_L =   0x38
    PCA9685_LED12_OFF_H =   0x39
    PCA9685_LED13_ON_L =    0x3A
    PCA9685_LED13_ON_H =    0x3B
    PCA9685_LED13_OFF_L =   0x3C
    PCA9685_LED13_OFF_H =   0x3D
    PCA9685_LED14_ON_L =    0x3E
    PCA9685_LED14_ON_H =    0x3F
    PCA9685_LED14_OFF_L =   0x40
    PCA9685_LED14_OFF_H =   0x41
    PCA9685_LED15_ON_L =    0x42
    PCA9685_LED15_ON_H =    0x43
    PCA9685_LED15_OFF_L =   0x44
    PCA9685_LED15_OFF_H =   0x45
    PCA9685_ALL_LED_ON_L =  0xFA
    PCA9685_ALL_LED_ON_H =  0xFB
    PCA9685_ALL_LED_OFF_L = 0xFC
    PCA9685_ALL_LED_OFF_H = 0xFD
    PCA9685_PRE_SCALE =     0xFE
    PCA9685_TEST_MODE =     0xFF

    #INA219A Power Monitor Registers
    INA219A_CONFIGURATION = 0x00
    INA219A_SHUNT_VOLTAGE = 0x01
    INA219A_BUS_VOLTAGE =   0x02
    INA219A_POWER =         0x03
    INA219A_CURRENT =       0x04
    INA219A_CALIBRATION =   0x05

    #MCU Registers
    MCU_WHO_AM_I =          0x00
    MCU_PWM_S =             0x01
    MCU_PWM_T =             0x02
    MCU_PWM_3 =             0x03
    MCU_PWM_4 =             0x04
    MCU_PWM_ALL =           0x05
    MCU_AN_1 =              0x11
    MCU_AN_2 =              0x12
    MCU_AN_3 =              0x13
    MCU_AN_4 =              0x14
    MCU_AN_ALL =            0x15
    MCU_TACH_L =            0x21
    MCU_TACH_R =            0x22
    MCU_TACH_ALL =          0x23
    I_AM_MCU =              0xBD       

    #Constants 
    RPM_TO_SPD = 0.00188495559

    #Variables
    is_initialized = False

    #I2C Config
    bus = SMBus(1)

    #I2C Communication
    def _write_register(self, dev, reg, data):
        """Writes data to the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Write register address
        data (byte): Data to be written to the device"""

        self.bus.write_i2c_block_data(dev, reg, [ data & 0xFF ])

    def _write_register_list(self, dev, reg, data):
        """Writes multiple bytes to the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Write register address
        data (byte list): Data to be written to the device"""

        self.bus.write_i2c_block_data(dev, reg, data)

    def _read_register_1ubyte(self, dev, reg):
        """Reads data from the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Read register address

        Returns:
        byte: Response from the device"""

        buffer = self.bus.read_i2c_block_data(dev, reg, 1)
        return buffer[0] & 0xFF

    def _read_2bytes_msbfirst(self, dev, reg):
        """Reads data from the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Read register address

        Returns:
        int: Response from the device, MSB first"""

        buffer = self.bus.read_i2c_block_data(dev, reg, 2)
        return buffer[0] << 8 | (buffer[1] & 0xFF)

    def _read_list(self, dev, reg, cnt):
        """Reads data from the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Read register start address
        cnt (byte): Number of bytes to read

        Returns:
        byte list: Response from the device"""

        return self.bus.read_i2c_block_data(dev, reg, cnt)

    def _read_int_list_msbf(self, dev, reg, cnt):
        """Reads an int list from the I2C device.

        Parameters:
        dev (byte): Device address
        reg (byte): Read register start address
        cnt (byte): Number of int to read

        Returns:
        int list: Device response as int list"""

        val = [0x0000] * cnt
        buffer = self.bus.read_i2c_block_data(dev, reg, cnt * 2)
        for w in range(cnt):
            val[w] = (buffer[w * 2] << 8) | (buffer[(w * 2) + 1] & 0xFF)
        return val

    #Initialization

    def __init__(self):
        self._config_pca9685()
        self._config_ina219a()
        self.is_initialized = True
        return

    #PCA9685 PWM Driver Methods

    def _config_pca9685(self):
        """Configures the PWM driver for RC hardware."""

        #Sleep before prescaler modification
        mode1 = 0x30
        self._write_register(self.PCA9685_I2C_ADDRESS, self.PCA9685_MODE1, mode1)

        #Set prescaler to generate 60Hz frequency
        prescale = 0x64
        self._write_register(self.PCA9685_I2C_ADDRESS, self.PCA9685_PRE_SCALE, prescale)

        #Wait for inter nal oscillator to stabilise
        sleep(0.005)

        #Mode register 2:
        #No invert, outputs change on STOP command, outputs are configured with a totem pole structure.
        mode2 = 0x04
        self._write_register(self.PCA9685_I2C_ADDRESS, self.PCA9685_MODE2, mode2)

        #Mode register 1:
        #Restart enabled, use internal clock, register auto increment enabled,
        #Sleep: normal mode, subaddresses and allcall address are ignored.
        mode1 = 0xA0
        self._write_register(self.PCA9685_I2C_ADDRESS, self.PCA9685_MODE1, mode1)
        return

    def set_pwm(self, ch, pos):
        """Sets the PWM pulse length of selected channel at 60Hz.

        Caution: For RC servo motors, start with center value and try larger values step by step. Extreme values may harm the servo motor.

        Parameters:
        ch (PWM_OUT): PWM Output channel
        pos (int): Pulse length (-128 is 1000us, 0 is 1500us, 128 is 2000us)"""

        if ch not in PWM_OUT:
            raise ValueError('ch is not a member of PWM_OUT.')
        if not (-128 <= pos <= 128):
            raise ValueError('pos should be between -128 and 128.')
        p = pos + 369
        val = [0x00, 0x00, p & 0xFF, (p >> 8) & 0x0F]
        self._write_register_list(self.PCA9685_I2C_ADDRESS, ch, val)
        return

    def set_pwms(self, pos):
        """Sets all PWM pulse lengths at 60Hz.

        Caution: For RC servo motors, start with center value and try larger values step by step. Extreme values may harm the servo motor.

        Parameters:
        pos (int list): Pulse length (-128 is 1000us, 0 is 1500us, 128 is 2000us), channels S, T, 3 and 4 respectively"""

        p = [0x0000] * 4
        p[0] = pos[0] + 369
        p[1] = pos[1] + 369
        p[2] = pos[2] + 369
        p[3] = pos[3] + 369
        val = [0x00, 0x00, p[0] & 0xFF, (p[0] >> 8) & 0x0F, 0x00, 0x00, p[1] & 0xFF, (p[1] >> 8) & 0x0F, 0x00, 0x00, p[2] & 0xFF, (p[2] >> 8) & 0x0F, 0x00, 0x00, p[3] & 0xFF, (p[3] >> 8) & 0x0F]

        self._write_register_list(self.PCA9685_I2C_ADDRESS, self.PCA9685_LED0_ON_L, val)
        return

    def set_fan(self, speed):
        """Sets the fan speed.

        Parameters:
        speed (byte): Fan speed (0 to 100, 0 is off, 100 is full speed)"""

        if speed > 100:
            val = [0x00, 0x10, 0x00, 0x00]
        elif speed == 0:
            val = [0x00, 0x00, 0x00, 0x10]
        else:
            conf = (speed * 30) + 1095
            val = [0x00, 0x00, conf & 0xFF, (conf >> 8) & 0x0F]

        self._write_register_list(self.PCA9685_I2C_ADDRESS, self.PCA9685_LED4_ON_L, val)
        return

    def set_led(self, led, b):
        """Sets the birghtness of the selected LED.

        Parameters:
        led (LED_OUT): LED channel
        b (int): LED brightness (0 to 4095, 0 is off, 4095 is full bright)"""

        if led not in LED_OUT:
            raise ValueError('led is not a member of LED_OUT.')

        if b > 4095:
            b = 4095

        val = [0x00, 0x00, b & 0xFF, (b >> 8) & 0x0F]
        self._write_register_list(self.PCA9685_I2C_ADDRESS, led, val)
        return

    def set_leds(self, b):
        """Sets the birghtness of the LEDs.

        Parameters:
        b (int list): LED brightness (0 to 4095), front, rear, stop, left and right respectively"""

        val = [0x00, 0x00, b[0] & 0xFF, (b[0] >> 8) & 0x0F, 0x00, 0x00, b[1] & 0xFF, (b[1] >> 8) & 0x0F, 0x00, 0x00, b[2] & 0xFF, (b[2] >> 8) & 0x0F, 0x00, 0x00, b[3] & 0xFF, (b[3] >> 8) & 0x0F, 0x00, 0x00, b[4] & 0xFF, (b[4] >> 8) & 0x0F]

        self._write_register_list(self.PCA9685_I2C_ADDRESS, self.PCA9685_LED11_ON_L, val)
        return

    def set_pwms_off(self):
        """Sets controller PWM outputs off. (S, T, 3, 4)"""

        val = [0] * 16
        self._write_register_list(self.PCA9685_I2C_ADDRESS, self.PCA9685_LED0_ON_L, val)
        return

    def set_leds_off(self):
        """Sets all LEDs off."""

        val = [0] * 20
        self._write_register_list(self.PCA9685_I2C_ADDRESS, self.PCA9685_LED11_ON_L, val)
        return

    #INA219A Power Monitor Methods

    def _config_ina219a(self):
        """Configures the power monitor."""

        #Set configuration register:
        #Bus Voltage Range: 32V
        #Bus ADC Resolution/Averaging: 128
        #Shunt ADC Resolution/Averaging: 9 bit
        #Operating Mode: Bus Voltage, Continuous
        conf = [0x3F, 0x86]
        self._write_register_list(self.INA219A_I2C_ADDRESS, self.INA219A_CONFIGURATION, conf)
        return

    def read_voltage(self):
        """Reads the battery Voltage.

        Returns:
        byte: Battery Voltage"""

        tmp = self._read_2bytes_msbfirst(self.INA219A_I2C_ADDRESS, self.INA219A_BUS_VOLTAGE)
        #Check for math overflow flag
        if (tmp & 0b1 != 0b1): #If calculations are correct
            v = (tmp >> 3) / 250.0
            return round(v, 2) if v > 5.0 else 0
        else: #If math overflow flag is set
            return 0

    #Microcontroller Methods

    def read_pwm(self, ch):
        """Reads the pulse length of the selected PWM channel.

        Parameters:
        ch (PWM_IN): PWM Channel

        Returns:
        int: Pulse length in microseconds"""

        if ch not in PWM_IN:
            raise ValueError('ch is not a member of PWM_IN.')

        try:
            return self._read_2bytes_msbfirst(self.MCU_I2C_ADDRESS, ch)
        except IOError:
            return None

    def read_pwms(self):
        """Reads the pulse length of the all PWM channels.

        Returns:
        int list: Pulse length in microseconds, channel S, T, 3 and 4 respectively"""

        try:
            return self._read_int_list_msbf(self.MCU_I2C_ADDRESS, self.MCU_PWM_ALL, 4)
        except IOError:
            return None

    def read_analog(self, ch):
        """Reads the analog input of the selected channel.

        Parameters:
        ch (ANALOG_IN): Analog input channel

        Returns:
        int: 10-Bits analog value (0 to 1023)"""

        if ch not in ANALOG_IN:
            raise ValueError('ch is not a member of ANALOG_IN.')
        try:
            return self._read_2bytes_msbfirst(self.MCU_I2C_ADDRESS, ch)
        except IOError:
            return None

    def read_analogs(self):
        """Reads the analog inputs of all four channels.

        Returns:
        int list: 10-Bits analog value (0 to 1023), channel 1 to 4 respectively"""

        try:
            return self._read_int_list_msbf(self.MCU_I2C_ADDRESS, self.MCU_AN_ALL, 4)
        except IOError:
            return None

    def _kph_to_mph(self, kph):
        """Converts KPH to MPH.
        
        Parameters:
        kph (float): Kilometers per hour

        Returns:
        float: Miles per hour"""

        return kph * 0.6213711922

    def read_tach(self, ch, ticks_per_rev = 1, dia_cm = None, mph = False):
        """Reads the speed of vehicle using the selected tachometer input.

        Parameters:
        ch (TACH_IN): Tachometer channel.
        ticks_per_rev (int): Ticks per revolution (Default is 1)
        dia_cm (int): Diameter in cm (Default none returns RPM)
        mph (bool): True for MPH, False for KMH output, checked only if dia_cm is set (Default is False)

        Returns:
        float: Vehicle speed or RPM"""

        if ch not in TACH_IN:
            raise ValueError('ch is not a member of TACH_IN.')

        try:
            #Calculate RPM
            res = self._read_register_1ubyte(self.MCU_I2C_ADDRESS, ch)
            res = 60 * res / ticks_per_rev

            #Calculate speed if wheel diameter is given
            if dia_cm is not None:
                res = res * dia_cm * self.RPM_TO_SPD
                if mph:
                    res = self._kph_to_mph(res)

        except IOError:
            return None

        return round(res, 2)

    def read_tachs(self, ticks_per_rev = 1, dia_cm = None, mph = False):
        """Reads the speed of vehicle using the tachometer inputs.

        Parameters:
        ticks_per_rev (int): Ticks per revolution (Default is 1)
        dia_cm (int): Diameter in cm (Default none returns RPM)
        mph (bool): True for MPH, False for KMH output, checked only if dia_cm is set (Default is False)

        Returns:
        float list: Vehicle speed or RPM, left and right respectively"""

        try:
            #Calculate RPM
            res = self._read_list(self.MCU_I2C_ADDRESS, self.MCU_TACH_ALL, 2)
            res[0] = 60 * res[0] / ticks_per_rev
            res[1] = 60 * res[1] / ticks_per_rev

            #Calculate speed if wheel diameter is given
            if dia_cm is not None:
                res[0] = res[0] * dia_cm * self.RPM_TO_SPD
                res[1] = res[1] * dia_cm * self.RPM_TO_SPD
                if mph:
                    res[0] = self._kph_to_mph(res[0])
                    res[1] = self._kph_to_mph(res[1])
                res[0] = round(res[0], 2)
                res[1] = round(res[1], 2)

        except IOError:
            return None

        return res

    def test_mcu(self):
        """Sends Who_Am_I command to the MCU, checks if it responses correctly.

        Returns:
        bool: True if the device responses with 0xBD, False if not"""

        try:
            res = self._read_list(self.MCU_I2C_ADDRESS, self.MCU_WHO_AM_I, 2)
        except IOError:
            res = [0, 0]

        return True if res[0] == 0xBD else False

    #Raspberry Pi Methods

    def read_cpu_temp(self, fahrenheit = False):
        """Reads the temperature of the Raspberry Pi's CPU.

        Parameters:
        fahrenheit (bool): True for Fahrenheit output, False for Celcius output (False is default)

        Returns:
        float: CPU Temperature"""

        tFile = open('/sys/class/thermal/thermal_zone0/temp')
        temp = float(tFile.read()) / 1000

        if (fahrenheit):
            temp = (temp * 1.8) + 32

        return round(temp, 1)

    #Disposal

    def __del__(self):
        """Releases the resources."""

        try:
            if self.is_initialized:
                self.set_pwms_off()
                self.set_leds_off()
                self.set_fan(0)
                del self.is_initialized
        except:
            pass
