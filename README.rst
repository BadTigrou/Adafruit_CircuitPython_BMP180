Introduction
============
NON-OFFICIAL CircuitPython driver from BMP180 Temperature and Barometic Pressure sensor adapted from `CircuitPython driver for BMP280 <https://github.com/adafruit/Adafruit_CircuitPython_BMP280/>`

NON-TESTED over SPI interface but should work anyway ;)

Installation and Dependencies
=============================

This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
--------------------

Need to create a package on PYPi for easy install.

.. code-block:: bash
	
	git clone https://github.com/BadTigrou/Adafruit_CircuitPython_BMP180.git
	cd Adafruit_CircuitPython_BMP180
	sudo python3 setup.py install

Usage Example
=============

.. code-block:: python

    import board
    import digitalio
    import busio
    import time
    from adafruit_bmp180 import adafruit_bmp180

    # Create library object using our Bus I2C port
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp180 = adafruit_bmp180.Adafruit_BMP180_I2C(i2c)

    # OR create library object using our Bus SPI port
    #spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
    #bmp_cs = digitalio.DigitalInOut(board.D10)
    #bmp180 = adafruit_bmp180.Adafruit_BMP180_SPI(spi, bmp_cs)

    # change this to match the location's pressure (hPa) at sea level
    bmp180.seaLevelhPa = 1013.25

    while True:
        print("\nTemperature: %0.1f C" % bmp180.temperature)
        print("Pressure: %0.1f hPa" % bmp180.pressure)
        print("Altitude = %0.2f meters" % bmp180.altitude)
        time.sleep(2)

