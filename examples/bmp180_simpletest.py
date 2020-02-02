import time
import board
# import digitalio # For use with SPI
import busio
import adafruit_bmp180

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
bmp180 = adafruit_bmp180.Adafruit_BMP180_I2C(i2c)

# OR create library object using our Bus SPI port
#spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
#bmp_cs = digitalio.DigitalInOut(board.D10)
#bmp180 = adafruit_bmp180.Adafruit_BMP180_SPI(spi, bmp_cs)

# change this to match the location's pressure (hPa) at sea level
bmp180.sea_level_pressure = 1013.25

while True:
    print("\nTemperature: %0.1f C" % bmp180.temperature)
    print("Pressure: %0.1f hPa" % bmp180.pressure)
    print("Altitude = %0.2f meters" % bmp180.altitude)
    time.sleep(2)
