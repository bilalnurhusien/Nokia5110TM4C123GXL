#include <stdint.h>
#include "inc/tm4c123gh6pm.h"


//********I2C_Init*****************
// Initialize the I2C master module at a rate of 100kHZ
// on GPIOE4(SCL) and GPIOE5(SDA)
// inputs: none
// outputs: none
void I2C_Init();

//********I2C_Read*****************
// Read one byte from a register on an I2C slave address
// inputs: 
//      device_address  - I2C slave address
//      device_register - Register in which to read from
// outputs: one byte of data
uint8_t I2C_Read(uint8_t device_address, uint8_t device_register);

//********I2C_Read2Bytes*****************
// Read two bytes from a register on an I2C slave address
// inputs: 
//      device_address  - I2C slave address
//      device_register - Register in which to read from
// outputs: one word of data
int16_t I2C_Read2Bytes(uint8_t device_address, uint8_t device_register);

//********I2C_Write*****************
// Write one byte from a register on an I2C slave address
// inputs: 
//      device_address  - I2C slave address
//      device_register - Register in which to read from
//      device_data     - Data to write to slave
// outputs: one word of data
void I2C_Write(uint8_t device_address, uint8_t device_register, uint8_t device_data);