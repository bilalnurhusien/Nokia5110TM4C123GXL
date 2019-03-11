#include <stdint.h>
#include <stdbool.h>
#include "I2C2.h"

// MPU-6050 I2C Address
#define I2C_MPU_6050_I2C_ADDR                  0x68

// Configuration registers
#define I2C_MPU_6050_REG_GYRO_CONFIG           0x1B
#define I2C_MPU_6050_REG_ACCEL_CONFIG          0x1C
#define I2C_MPU_6050_REG_POWER_CONFIG          0x6B
#define I2C_MPU_6050_REG_WHO_AM_I              0x75

// Accelerometer registers
#define I2C_MPU_6050_REG_ACCEL_X1              0x3B
#define I2C_MPU_6050_REG_ACCEL_X2              0x3C
#define I2C_MPU_6050_REG_ACCEL_Y1              0x3D
#define I2C_MPU_6050_REG_ACCEL_Y2              0x3E
#define I2C_MPU_6050_REG_ACCEL_Z1              0x3F
#define I2C_MPU_6050_REG_ACCEL_Z2              0x40

// Temperature registers
#define I2C_MPU_6050_REG_TEMP1                 0x41
#define I2C_MPU_6050_REG_TEMP2                 0x42

// Gyroscope registers
#define I2C_MPU_6050_REG_GYRO_X1               0x43
#define I2C_MPU_6050_REG_GYRO_X2               0x44
#define I2C_MPU_6050_REG_GYRO_Y1               0x45
#define I2C_MPU_6050_REG_GYRO_Y2               0x46
#define I2C_MPU_6050_REG_GYRO_Z1               0x47
#define I2C_MPU_6050_REG_GYRO_Z2               0x48

// Gyroscope config
#define I2C_MPU_6050_GYRO_CONFIG_250_DPS       0x00 // +/- 250 dps
#define I2C_MPU_6050_GYRO_CONFIG_500_DPS       0x08 // +/- 500 dps
#define I2C_MPU_6050_GYRO_CONFIG_1000_DPS      0x10 // +/- 1000 dps
#define I2C_MPU_6050_GYRO_CONFIG_2000_DPS      0x18 // +/- 2000 dps

// Accelerometer config
#define I2C_MPU_6050_ACCEL_CONFIG_2G           0x00 // +/- 2G
#define I2C_MPU_6050_ACCEL_CONFIG_4G           0x08 // +/- 4G
#define I2C_MPU_6050_ACCEL_CONFIG_8G           0x10 // +/- 8G
#define I2C_MPU_6050_ACCEL_CONFIG_16G          0x18 // +/- 16G


//********MPU_6050_Init*****************
// Initialize the MPU-6050 IMU by configuring the power mode,
// accelerometer and gyroscope configuration
// inputs:
//      gyroCfg  - Gyroscope configuration
//      accelCfg - Accelerometer configuration
// outputs: none
void MPU_6050_Init(uint8_t gyroCfg, uint8_t accelCfg);

//********MPU_6050_Probe*****************
// Probe the MPU-6050 IMU
// inputs: none
// outputs: Returns true on success, false on failure
bool MPU_6050_Probe();

//********MPU_6050_GetGyroData*****************
// Get the gyroscope data from MPU-6050 IMU
// inputs: none
// outputs: Retruns X, Y, and Z gyroscope values
void MPU_6050_GetGyroData(int16_t * gyroX, int16_t * gyroY, int16_t * gyroZ);

//********MPU_6050_GetAccelData*****************
// Get the accelerometer data from the MPU-6050 IMU
// inputs: none
// outputs: Retruns X, Y, and Z accelerometer values
void MPU_6050_GetAccelData(int16_t * accelX, int16_t * accelY, int16_t * accelZ);

//********MPU_6050_GetTempData*****************
// Get the temperature data from the MPU-6050 IMU
// inputs: none
// outputs: Returns temperature data
void MPU_6050_GetTempData(int16_t * tempData);