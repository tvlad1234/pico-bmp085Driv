# pico-bmp085Driv
BMP085/BMP180 library for pico-sdk\

Based on [Adafruit-BMP085-Library](https://github.com/adafruit/Adafruit-BMP085-Library)
## Usage
Add the _pico-bmp085Driv_ subdirectory to the CMakeLists.txt of your project and include the library in _target_link_libraries_.
### Initializing the sensor
Before reading the sensor, it must be initialized with _BMP085_init_.\
_BMP085_init_ the sensor mode as a parameter\
_Example:_ `BMP085_init(BMP085_STANDARD);`\
Other usable modes are `BMP085_ULTRALOWPOWER`,  `BMP085_HIGHRES`,  `BMP085_ULTRAHIGHRES.` 
###
By default, the library uses pins 4 (SDA) and 5 (SCL). `BMP085_setI2C(i2c_inst_t *i, uint16_t sda, uint16_t scl, uint8_t addr);` can be used to select the I2C peripheral, pins and sensor adress.
### Reading the sensor
`BMP085_readTemperature()` returns the temperature in °C as _float_.\
`BMP085_readPressure()` returns the atmospheric pressure in Pascals (Pa) as _uint32_t_.

