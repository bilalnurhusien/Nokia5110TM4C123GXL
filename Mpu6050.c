#include <stdio.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "Mpu6050.h"


void MPU_6050_Init(uint8_t gyroCfg, uint8_t accelCfg)
{
    //
    // I2C Initialization
    // 
    I2C_Init();
      
    //
    // Set clock to internal 8 MHz oscillator
    //    
    I2C_Write(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_POWER_CONFIG, 0);

    I2C_Write(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_ACCEL_CONFIG, accelCfg);
    
    I2C_Write(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_GYRO_CONFIG, gyroCfg);
}

bool MPU_6050_Probe()
{
    const uint8_t ExpectedWhoAmi = 0x72;
  
    uint8_t data = I2C_Read(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_WHO_AM_I);
    
    return (data == ExpectedWhoAmi);
}

void MPU_6050_GetGyroData(int16_t * gyroX, int16_t * gyroY, int16_t * gyroZ)
{
    if ((NULL == gyroX) ||
        (NULL == gyroY) ||
        (NULL == gyroZ))
    {
        return;
    }
    
    *gyroX = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_GYRO_X1);
    *gyroY = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_GYRO_Y1);
    *gyroZ = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_GYRO_Z1);
}

void MPU_6050_GetAccelData(int16_t * accelX, int16_t * accelY, int16_t * accelZ)
{
    if ((NULL == accelX) ||
        (NULL == accelY) ||
        (NULL == accelZ))
    {
        return;
    }
    
    *accelX = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_ACCEL_X1);
    *accelY = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_ACCEL_Y1);
    *accelZ = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_ACCEL_Z1);
}

void MPU_6050_GetTempData(int16_t * tempData)
{
    if (NULL == tempData)
    {
        return;
    }
    
    *tempData = I2C_Read2Bytes(I2C_MPU_6050_I2C_ADDR, I2C_MPU_6050_REG_TEMP1); 
}
