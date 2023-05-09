#ifndef MPU9250
#define MPU9250
#include <stdint.h> // for integer type definitions
#include "main.h"  // for ADCrange
#include "i2c.h"
//#include "stm32f1xx_hal_i2c.h"

#define    MPU9250_ADDRESS            0x68
void MPU9250_setup();
void MPU9250_read(int16_t [], int16_t [], int16_t []);
#endif // MPU9250
