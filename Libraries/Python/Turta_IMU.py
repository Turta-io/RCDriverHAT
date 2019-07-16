# Turta RC Driver HAT Helper for Raspbian.
# Distributed under the terms of the MIT license.

# Python Library for ST LSM6DS33 IMU.
# Version 1.0.0
# Released: July 16th, 2019

# Visit https://docs.turta.io for documentation.

import RPi.GPIO as GPIO
import spidev

class IMU(object):
    "LSM6DS33 IMU"

    #Variables
    is_initialized = False

    #Interrupt Pins
    lsm6ds33_int_1, lsm6ds33_int_2 = 22, 27

    #SPI Config
    spi = None

    #Registers
    LSM6DS33_FUNC_CFG_ACCESS  = 0x01
    LSM6DS33_FIFO_CTRL1       = 0x06
    LSM6DS33_FIFO_CTRL2       = 0x07
    LSM6DS33_FIFO_CTRL3       = 0x08
    LSM6DS33_FIFO_CTRL4       = 0x09
    LSM6DS33_FIFO_CTRL5       = 0x0A
    LSM6DS33_ORIENT_CFG_G     = 0x0B
    LSM6DS33_INT1_CTRL        = 0x0D
    LSM6DS33_INT2_CTRL        = 0x0E
    LSM6DS33_WHO_AM_I         = 0x0F
    LSM6DS33_CTRL1_XL         = 0x10
    LSM6DS33_CTRL2_G          = 0x11
    LSM6DS33_CTRL3_C          = 0x12
    LSM6DS33_CTRL4_C          = 0x13
    LSM6DS33_CTRL5_C          = 0x14
    LSM6DS33_CTRL6_C          = 0x15
    LSM6DS33_CTRL7_G          = 0x16
    LSM6DS33_CTRL8_XL         = 0x17
    LSM6DS33_CTRL9_XL         = 0x18
    LSM6DS33_CTRL10_C         = 0x19
    LSM6DS33_WAKE_UP_SRC      = 0x1B
    LSM6DS33_TAP_SRC          = 0x1C
    LSM6DS33_D6D_SRC          = 0x1D
    LSM6DS33_STATUS_REG       = 0x1E
    LSM6DS33_OUT_TEMP_L       = 0x20
    LSM6DS33_OUT_TEMP_H       = 0x21
    LSM6DS33_OUTX_L_G         = 0x22
    LSM6DS33_OUTX_H_G         = 0x23
    LSM6DS33_OUTY_L_G         = 0x24
    LSM6DS33_OUTY_H_G         = 0x25
    LSM6DS33_OUTZ_L_G         = 0x26
    LSM6DS33_OUTZ_H_G         = 0x27
    LSM6DS33_OUTX_L_XL        = 0x28
    LSM6DS33_OUTX_H_XL        = 0x29
    LSM6DS33_OUTY_L_XL        = 0x2A
    LSM6DS33_OUTY_H_XL        = 0x2B
    LSM6DS33_OUTZ_L_XL        = 0x2C
    LSM6DS33_OUTZ_H_XL        = 0x2D
    LSM6DS33_FIFO_STATUS1     = 0x3A
    LSM6DS33_FIFO_STATUS2     = 0x3B
    LSM6DS33_FIFO_STATUS3     = 0x3C
    LSM6DS33_FIFO_STATUS4     = 0x3D
    LSM6DS33_FIFO_DATA_OUT_L  = 0x3E
    LSM6DS33_FIFO_DATA_OUT_H  = 0x3F
    LSM6DS33_TIMESTAMP0_REG   = 0x40
    LSM6DS33_TIMESTAMP1_REG   = 0x41
    LSM6DS33_TIMESTAMP2_REG   = 0x42
    LSM6DS33_STEP_TIMESTAMP_L = 0x49
    LSM6DS33_STEP_TIMESTAMP_H = 0x4A
    LSM6DS33_STEP_COUNTER_L   = 0x4B
    LSM6DS33_STEP_COUNTER_H   = 0x4C
    LSM6DS33_FUNC_SRC         = 0x53
    LSM6DS33_TAP_CFG          = 0x58
    LSM6DS33_TAP_THS_6D       = 0x59
    LSM6DS33_INT_DUR2         = 0x5A
    LSM6DS33_WAKE_UP_THS      = 0x5B
    LSM6DS33_WAKE_UP_DUR      = 0x5C
    LSM6DS33_FREE_FALL        = 0x5D
    LSM6DS33_MD1_CFG          = 0x5E
    LSM6DS33_MD2_CFG          = 0x5F

    #Initialization

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)                 #Bus 0, Device 0
        self.spi.max_speed_hz = 1000000     #1MHz
        self.spi.mode = 0b00                #CPOL 0, CHPA 0
        self.spi.bits_per_word = 8          #8 Bits per word
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.lsm6ds33_int_1, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.setup(self.lsm6ds33_int_2, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        self._set_initial_settings()
        self.is_initialized = True
        return

    #Helpers

    def _to_signed(self, val):
        """Converts raw data to signed data.

        Parameters:
        val (int): Raw data

        Returns:
        int: Signed data"""

        return val if val < 32768 else (val - 65536)

    #SPI Communication

    def _write_spi_uint8(self, reg, val):
        """Writes a single byte to the SPI device.

        Parameters:
        reg (byte): Write register address
        val (byte): Data to be written"""

        self.spi.xfer2([reg, val])
        return

    def _read_spi_uint8(self, reg):
        """Reads a single byte from the SPI device.

        Parameters:
        reg (byte): Read register address

        Returns:
        byte: Device response"""

        temp = self.spi.xfer2([reg | 0x80, 0x00])
        return temp[1] & 0xFF

    def _read_spi_uint16(self, reg):
        """Reads an int from the SPI device.

        Parameters:
        reg (byte): Read register address

        Returns:
        int: Device response converted to int"""

        temp = self.spi.xfer2([reg | 0x80, 0x00, 0x00])
        return temp[2] << 8 | temp[1] & 0xFF

    def _read_spi_int16_array(self, reg, cnt):
        """Reads an int array from the SPI device.

        Parameters:
        reg (byte): Read register start address
        cnt (byte): Number of int to read

        Returns:
        int array: Device response as int array"""

        tmp = [0x00] * (cnt * 2 + 1)
        val = [0x0000] * cnt
        tmp[0] = reg | 0x80
        buffer = self.spi.xfer2(tmp)
        for b in range(cnt):
            val[b] = self._to_signed((buffer[(b * 2) + 2] << 8) | (buffer[(b * 2) + 1] & 0xFF))
        return val

    #Configuration

    def _set_initial_settings(self):
        """Sets the default configuration."""

        #Disable accel and gyro
        self._disable_imu()

        #Bypass mode, disable FIFO buffer
        self._write_spi_uint8(self.LSM6DS33_FIFO_CTRL5, 0x00)

        #Configure accel
        #ODR: 416 Hz
        #Full-scale selection: +/- 16g
        #Anti-aliasing filter: 200 Hz
        self._write_spi_uint8(self.LSM6DS33_CTRL1_XL, 0x65)

        #Configure gyro
        #ODR: 416 Hz
        #Full-scale selection: 1000 dps
        self._write_spi_uint8(self.LSM6DS33_CTRL2_G, 0x68)

        #Control register 3
        #Block data update is enabled
        #Register address auto increment is enabled
        self._write_spi_uint8(self.LSM6DS33_CTRL3_C, 0x44)

        #Control register 4
        #I2C is disabled
        self._write_spi_uint8(self.LSM6DS33_CTRL4_C, 0x04)

        #Angular rate sensor control register 7
        #Gyroscope digital high-pass filter is enabled
        #Gyroscope high-pass filter cutoff frequency is 16.32 Hz
        self._write_spi_uint8(self.LSM6DS33_CTRL7_G, 0x70)

        #Linear acceleration sensor control register 8
        #Accelerometer low-pass filter is enabled
        #HP filter cutoff frequency is ODR_XL/400
        self._write_spi_uint8(self.LSM6DS33_CTRL8_XL, 0xE0)

        #Disable embedded functions configuration
        self._write_spi_uint8(self.LSM6DS33_FUNC_CFG_ACCESS, 0x00)
        return

    def _disable_imu(self):
        """Disables accel and gyro."""

        self._write_spi_uint8(self.LSM6DS33_CTRL1_XL, 0x00)
        self._write_spi_uint8(self.LSM6DS33_CTRL2_G, 0x00)
        self._write_spi_uint8(self.LSM6DS33_CTRL3_C, 0x00)
        return

    def test_imu(self):
        """Sends Who_Am_I command to the IMU, checks if it responses correctly.

        Returns:
        bool: True if the device responses with 0x69, False if not"""

        res = self._read_spi_uint8(self.LSM6DS33_WHO_AM_I)
        return True if res == 0x69 else False

    def read_accel_x(self):
        """Reads the X-axis acceleration.

        Returns:
        int: X-axis acceleration"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTX_L_XL)
        return self._to_signed(res)
        
    def read_accel_y(self):
        """Reads the Y-axis acceleration.

        Returns:
        int: Y-axis acceleration"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTY_L_XL)
        return self._to_signed(res)

    def read_accel_z(self):
        """Reads the Z-axis acceleration.

        Returns:
        int: Z-axis acceleration"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTZ_L_XL)
        return self._to_signed(res)

    def read_accel_xyz(self):
        """Reads the accelerations of X, Y and Z-axes.

        Returns:
        int array: Acceleration of X, Y and Z-axes respectively"""

        return self._read_spi_int16_array(self.LSM6DS33_OUTX_L_XL, 3)

    def read_gyro_x_pitch(self):
        """Reads the pitch on X-axis.

        Returns:
        int: Pitch"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTX_L_G)
        return self._to_signed(res)

    def read_gyro_y_roll(self):
        """Reads the roll on Y-axis.

        Returns:
        int: Roll"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTY_L_G)
        return self._to_signed(res)

    def read_gyro_z_yaw(self):
        """Reads the yaw on Z-axis.

        Returns:
        int: Yaw"""

        res = self._read_spi_uint16(self.LSM6DS33_OUTZ_L_G)
        return self._to_signed(res)

    def read_gyro_xyz(self):
        """Reads the pitch, roll and yaw.

        Returns:
        int array: Pitch, roll and yaw values respectively"""

        return self._read_spi_int16_array(self.LSM6DS33_OUTX_L_G, 3)

    def read_gyro_accel_xyz(self):
        """Reads the accelerations of X, Y, Z-axes and pitch, roll, yaw values.

        Returns:
        int array: X accel, Y accel, Z accel, pitch, roll and yaw values respectively"""

        return self._read_spi_int16_array(self.LSM6DS33_OUTX_L_G, 6)

    def read_temp(self, fahrenheit = False):
        """Reads the temperature of the IMU.

        Parameters:
        fahrenheit (bool): True for Fahrenheit output, False for Celcius output (False is default)

        Returns:
        float: Temperature of IMU"""

        tmp = self._read_spi_uint16(self.LSM6DS33_OUT_TEMP_L)
        res = 25.0 + (self._to_signed(tmp) / 16.0)

        if (fahrenheit):
            res = (res * 1.8) + 32

        return round(res, 1)

    #Disposal

    def __del__(self):
        """Releases the resources."""

        try:
            if self.is_initialized:
                self._disable_imu()
                self.spi.close()
                GPIO.cleanup()
                del self.is_initialized
        except:
            pass
