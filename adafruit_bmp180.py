# The MIT License (MIT)

# Copyright (c) 2020 BadTigrou

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
`adafruit_bmp180` - Adafruit BMP180 - Temperature & Barometic Pressure Sensor
===============================================================================

CircuitPython driver from BMP180 Temperature and Barometic Pressure sensor

* Author(s): BadTigrou 
"""
import math
from time import sleep
try:
    import struct
except ImportError:
    import ustruct as struct
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/BadTigrou/Adafruit_CircuitPython_BMP180.git"

#    I2C ADDRESS/BITS/SETTINGS
#    -----------------------------------------------------------------------
_CHIP_ID = const(0x55)

_REGISTER_CHIPID    = const(0xD0)
_REGISTER_SOFTRESET = const(0xE0)
_REGISTER_CONTROL   = const(0xF4)
_REGISTER_DATA      = const(0xF6)

"""calibration coefficients register"""
_REGISTER_AC1 = const(0xAA)
_REGISTER_AC2 = const(0xAC)
_REGISTER_AC3 = const(0xAE)
_REGISTER_AC4 = const(0xB0)
_REGISTER_AC5 = const(0xB2)
_REGISTER_AC6 = const(0xB4)
_REGISTER_B1  = const(0xB6)
_REGISTER_B2  = const(0xB8)
_REGISTER_MB  = const(0xBA)
_REGISTER_MC  = const(0xBC)
_REGISTER_MD  = const(0xBE)

_BMP180_PRESSURE_MIN_HPA = const(300)
_BMP180_PRESSURE_MAX_HPA = const(1100)


"""oversampling values for temperature, pressure, and humidity"""
TEMPERATURE_CMD           = const(0x2E)
PRESSURE_OVERSAMPLING_X1  = const(0x01)
PRESSURE_OVERSAMPLING_X2  = const(0x02)
PRESSURE_OVERSAMPLING_X4  = const(0x03)
PRESSURE_OVERSAMPLING_X8  = const(0x04)

_BMP180_PRESSURE_CMD = {PRESSURE_OVERSAMPLING_X1:0x34, PRESSURE_OVERSAMPLING_X2:0x74, PRESSURE_OVERSAMPLING_X4:0xB4, PRESSURE_OVERSAMPLING_X8:0xF4}


"""mode values"""
MODE_ULTRALOWPOWER  = const(0x00)
MODE_STANDARD       = const(0x01)
MODE_HIGHRES        = const(0x02)
MODE_ULTRAHIGHRES   = const(0x03)

_BMP180_MODES = (MODE_ULTRALOWPOWER, MODE_STANDARD, MODE_HIGHRES, MODE_ULTRAHIGHRES)


class Adafruit_BMP180: # pylint: disable=invalid-name
    """Base BMP180 object. Use `Adafruit_BMP180_I2C` or `Adafruit_BMP180_SPI` instead of this. This
       checks the BMP180 was found, reads the coefficients and enables the sensor for continuous
       reads"""
    def __init__(self):
        # Check device ID.
        chip_id = self._read_byte(_REGISTER_CHIPID)
        if _CHIP_ID != chip_id:
            raise RuntimeError('Failed to find BMP180! Chip ID 0x%x' % chip_id)
        #Set some reasonable defaults.
        self._oversampling_setting = PRESSURE_OVERSAMPLING_X8
        self._mode = MODE_HIGHRES

        self._reset()
        
        self._read_coefficients()
       
        self.sea_level_pressure = 1013.25

    @property
    def temperature(self):
        """The compensated temperature in degrees celsius."""
        UT = self._read_raw_temperature()    
        X1 = int(((UT - self._AC6) * self._AC5) >> 15)
        X2 = int((self._MC << 11) / (X1 + self._MD))
        B5 = X1 + X2
        temp = ((B5 + 8) >> 4) / 10.0
        return temp

    def _read_raw_temperature(self):
        self._write_register_byte(_REGISTER_CONTROL,TEMPERATURE_CMD)
        sleep(0.005)  # Wait 5ms
        return self._readU16(_REGISTER_DATA)

    @property
    def altitude(self):
        """The altitude based on the sea level pressure (`sea_level_pressure`) - which you must
           enter ahead of time)"""
        altitude = 44330.0 * (1.0 - pow(self.pressure / self.sea_level_pressure, 0.1903))        
        return round(altitude,1)

    @property
    def pressure(self):
        """The compensated pressure in hectoPascals."""
        UT = self._read_raw_temperature()
        UP = self._read_raw_pressure()

        X1 = int(((UT - self._AC6) * self._AC5) >> 15)
        X2 = int((self._MC << 11) / (X1 + self._MD))
        B5 = X1 + X2

        B6 = B5 - 4000
        X1 = int((self._B2 * (B6 * B6) >> 12) >> 11)
        X2 = int((self._AC2 * B6) >> 11)
        X3 = X1 + X2
        B3 = int((((self._AC1 * 4 + X3) << self._mode) + 2) / 4)

        X1 = int((self._AC3 * B6) >> 13)
        X2 = int((self._B1 * ((B6 * B6) >> 12)) >> 16)
        X3 = int(((X1 + X2) + 2) >> 2)
        B4 = int((self._AC4 * (X3 + 32768)) >> 15)
        B7 = int((UP - B3) * (50000 >> self._mode))

        if (B7 < 0x80000000):
            p = int((B7 * 2) / B4)
        else:
            p = int((B7 / B4) * 2)

        X1 = int((p >> 8) * (p >> 8))
        X1 = int((X1 * 3038) >> 16)
        X2 = int((-7357 * p) >> 16)

        return int(p + ((X1 + X2 + 3791) >> 4)) / 100
  

    def _read_raw_pressure(self):

        self._write_register_byte(_REGISTER_CONTROL, _BMP180_PRESSURE_CMD[self._mode])

        if self._mode == PRESSURE_OVERSAMPLING_X8:
            sleep(0.026)
        elif self._mode == PRESSURE_OVERSAMPLING_X4:
            sleep(0.014)
        elif self._mode == PRESSURE_OVERSAMPLING_X2:
            sleep(0.008)
        else:
            sleep(0.005)

        msb = self._read_byte(_REGISTER_DATA)
        lsb = self._read_byte(_REGISTER_DATA+1)
        xlsb = self._read_byte(_REGISTER_DATA+2)

        return ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self._mode)


    def _reset(self):
        """Soft reset the sensor"""
        self._write_register_byte(_REGISTER_SOFTRESET, 0xB6)
        sleep(0.004)  #Datasheet says 2ms.  Using 4ms just to be safe

    @property
    def mode(self):
        """
        Operation mode
        Allowed values are set in the MODE enum class
        """
        return self._mode

    @mode.setter
    def mode(self, value):
        print("as")
        print(value)
        if not value in _BMP180_MODES:
            raise ValueError('Mode \'%s\' not supported' % (value))
        self._mode = value

    @property
    def oversampling_setting(self):
        """
        Oversampling setting
        Allowed values are set in the OVERSAMPLES enum class
        """
        return self._oversampling_setting

    @oversampling_setting.setter
    def oversampling_setting(self, value):
        print("oss")
        if not value in _BMP180_PRESSURE_CMD:
            raise ValueError('Overscan value \'%s\' not supported' % (value))
        self._overscan_temperature = value



    ####################### Internal helpers ################################
    def _read_coefficients(self):
        """Read & save the calibration coefficients"""

        self._AC1 = self._readS16(_REGISTER_AC1)
        self._AC2 = self._readS16(_REGISTER_AC2) 
        self._AC3 = self._readS16(_REGISTER_AC3)
        self._AC4 = self._readU16(_REGISTER_AC4)
        self._AC5 = self._readU16(_REGISTER_AC5)
        self._AC6 = self._readU16(_REGISTER_AC6)
        self._B1 = self._readS16(_REGISTER_B1)
        self._B2 = self._readS16(_REGISTER_B2)
        self._MB = self._readS16(_REGISTER_MB)
        self._MC = self._readS16(_REGISTER_MC)
        self._MD = self._readS16(_REGISTER_MD)
        
        """print(self._AC1)
        print(self._AC2)
        print(self._AC3)
        print(self._AC4)
        print(self._AC5)
        print(self._AC6)
        print(self._B1)
        print(self._B2)
        print(self._MB)
        print(self._MC)
        print(self._MD)
        """

    def _read_byte(self, register):
        """Read a byte register value and return it"""
        return self._read_register(register, 1)[0]

    def _readS16(self,register):
        msb = self._read_byte(register)
        if msb > 127: msb -= 256
        return  msb * 256 + self._read_byte(register+1)

    def _readU16(self, register):
        return self._read_byte(register) * 256 + self._read_byte(register+1)

    def _read_register(self, register, length):
        """Low level register reading, not implemented in base class"""
        raise NotImplementedError()

    def _write_register_byte(self, register, value):
        """Low level register writing, not implemented in base class"""
        raise NotImplementedError()

class Adafruit_BMP180_I2C(Adafruit_BMP180): # pylint: disable=invalid-name
    """Driver for I2C connected BMP180. Default address is 0x77 but another address can be passed
       in as an argument"""
    def __init__(self, i2c, address=0x77):
        import adafruit_bus_device.i2c_device as i2c_device
        self._i2c = i2c_device.I2CDevice(i2c, address)
        super().__init__()

    def _read_register(self, register, length):
        """Low level register reading over I2C, returns a list of values"""
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF]))
            result = bytearray(length)
            i2c.readinto(result)
            #print("$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write_register_byte(self, register, value):
        """Low level register writing over I2C, writes one 8-bit value"""
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF, value & 0xFF]))
            #print("$%02X <= 0x%02X" % (register, value))

class Adafruit_BMP180_SPI(Adafruit_BMP180):
    """Driver for SPI connected BMP180. Default clock rate is 100000 but can be changed with
      'baudrate'"""
    def __init__(self, spi, cs, baudrate=100000):
        import adafruit_bus_device.spi_device as spi_device
        self._spi = spi_device.SPIDevice(spi, cs, baudrate=baudrate)
        super().__init__()

    def _read_register(self, register, length):
        """Low level register reading over SPI, returns a list of values"""
        register = (register | 0x80) & 0xFF  # Read single, bit 7 high.
        with self._spi as spi:
            # pylint: disable=no-member
            spi.write(bytearray([register]))
            result = bytearray(length)
            spi.readinto(result)
            #print("$%02X => %s" % (register, [hex(i) for i in result]))
            return result

    def _write_register_byte(self, register, value):
        """Low level register writing over SPI, writes one 8-bit value"""
        register &= 0x7F  # Write, bit 7 low.
        with self._spi as spi:
            # pylint: disable=no-member
            spi.write(bytes([register, value & 0xFF]))
