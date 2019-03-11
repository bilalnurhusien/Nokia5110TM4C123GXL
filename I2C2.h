#include <stdint.h>
#include "inc/tm4c123gh6pm.h"


void I2C_Init();
uint8_t I2C_Read(uint16_t device_address, uint16_t device_register);
int16_t I2C_Read2Bytes(uint16_t device_address, uint16_t device_register);
void I2C_Write(uint16_t device_address, uint16_t device_register, uint8_t device_data);