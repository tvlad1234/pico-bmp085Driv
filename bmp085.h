#ifndef _BMP085_H
#define _BMP085_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Modes
#define BMP085_ULTRALOWPOWER 0 //!< Ultra low power mode
#define BMP085_STANDARD 1      //!< Standard mode
#define BMP085_HIGHRES 2       //!< High-res mode
#define BMP085_ULTRAHIGHRES 3  //!< Ultra high-res mode

#define STD_ATM_PRESS 101325

// Sensor Init function
void BMP085_setI2C(i2c_inst_t *i, uint16_t sda, uint16_t scl, uint8_t addr);
uint8_t BMP085_init(uint8_t mode);

// Sensor read functions
float BMP085_readTemperature(void);
int32_t BMP085_readPressure(void);

#endif /* _BMP085_H */
