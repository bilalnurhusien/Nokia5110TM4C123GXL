#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <inc/hw_gpio.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>

#include "Nokia5110.h"
#include "Mpu6050.h"
#include "Timer0A.h"

volatile int refreshTimeout = 0;

void Timer0A_Handler(void){
    refreshTimeout = 1;
}

//
// Write static data to LCD
//
void SetUpLcdScreen()
{
    Nokia5110_Clear();
    Nokia5110_OutString("Pitch:");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Roll :");
}

//
// Initialize LCD, IMU, and Timers
//
bool Initialize()
{
    //
    // Nokia 5110 Initialization
    //
    Output_Init();
    
    //
    // MPU-6050 Initialization
    //
    MPU_6050_Init(I2C_MPU_6050_GYRO_CONFIG_500_DPS, I2C_MPU_6050_ACCEL_CONFIG_8G);
    
    int success = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (MPU_6050_Probe())
        {
          success = 1;
          break;
        }
        SysCtlDelay(3);
    }
    
    
    if (success != 1)
    {
        printf("Failed to find MPU-6050 IMU\n");
        return false;
    }
                  
    //
    // Timer initialization for 10ms second timeout
    //
    uint32_t period = SysCtlClockGet()/100;
    
    if (!Timer_0A_Init(&Timer0A_Handler, period))
    {
        printf("Failed to initialize Timer0A");
        return false;     
    }
    
    //
    // Enable the GPIOB peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    //
    // Wait for the GPIOB module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    
    //
    // Set pin 7 output, SW controlled.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);
    
    return true;
}

void CalibrateGyroData(int64_t * gyroXCal, int64_t * gyroYCal, int64_t * gyroZCal)
{
    if (NULL == gyroXCal ||
        NULL == gyroYCal ||
        NULL == gyroZCal)
    {
        printf("Invalid argument\n");
        return;
    }
    
    *gyroXCal = *gyroYCal = *gyroZCal = 0;

    Nokia5110_Clear();
    Nokia5110_SetCursor(0,0);
    
    Nokia5110_OutString("MPU-6050 IMU");

    Nokia5110_SetCursor(0,1);
    
    Nokia5110_OutString("Calibrating");
    
    //
    // Take 500s samples on refresh timeout
    //
    const uint32_t CalibrationCount = 500;
    int16_t gyroX, gyroY, gyroZ, i;
    
    gyroX = gyroY = gyroZ = i = 0;
    
    uint8_t gpioStatus =  (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) & 0xFF);
    
    while (i < CalibrationCount)
    {     
        if (refreshTimeout)
        {
            refreshTimeout = 0;
            gpioStatus ^= GPIO_PIN_7;
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, gpioStatus);
            MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
            *gyroXCal += gyroX;
            *gyroYCal += gyroY;
            *gyroZCal += gyroZ;
            ++i;
            if ((i % 100) == 0)
            {
                Nokia5110_OutChar('.');
            }
        }
    }
    
    *gyroXCal /= CalibrationCount;
    *gyroYCal /= CalibrationCount;
    *gyroZCal /= CalibrationCount;
}

int main(void)
{
    //
    // Enable floating point unit and disable stacking (FPU instructions aren't allowed within interrupt handlers)
    //
    FPUEnable();
    FPUStackingDisable();
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    
    if (!Initialize())
    {
        printf("Failed to initialize\n");
        return false;
    }

    int64_t gyroXCal, gyroYCal, gyroZCal;
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    volatile float anglePitch = 0;
    volatile float angleRoll = 0;
    volatile float accelAnglePitch  = 0;
    volatile float accelAngleRoll = 0;
    volatile float angleTotalVector = 0;

    CalibrateGyroData(&gyroXCal, &gyroYCal, &gyroZCal);
    
    SetUpLcdScreen();

    MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
    MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
    
    gyroX -= gyroXCal;
    gyroY -= gyroYCal;
    gyroZ -= gyroZCal;

    angleTotalVector = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
    
     //57.296 = 1 / (3.142 / 180) The asin function is in radian
    accelAnglePitch = asinf(accelY / angleTotalVector)* 57.296f;                //Calculate the pitch angle

    accelAngleRoll = asinf(accelX/angleTotalVector)* -57.296f;                  //Calculate the roll angle
    
    anglePitch = accelAnglePitch;                                               //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angleRoll = accelAngleRoll;                                                 //Set the gyro roll angle equal to the accelerometer roll angle 
    
    uint8_t gpioB7Status =  (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) & 0xFF);
    int i = 0;
    
    while (1)
    {
      if (refreshTimeout)
      {
        refreshTimeout = 0;
        ++i;
        gpioB7Status ^= GPIO_PIN_7;
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, gpioB7Status);
        
        MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
        MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
        
        gyroX -= gyroXCal;
        gyroY -= gyroYCal;
        gyroZ -= gyroZCal;

        //Gyro angle calculations
        //0.0001527 = 1 / 100Hz / 65.5
        anglePitch += gyroX * 0.0001527f;                                      //Calculate the traveled pitch angle and add this to the angle_pitch variable
        angleRoll += gyroY * 0.0001527f;                                       //Calculate the traveled roll angle and add this to the angle_roll variable
  
        //0.000002665 = 0.0001527 * (3.142(PI) / 180degr) The sin function is in radians
        anglePitch += angleRoll * sinf(gyroZ * 0.000002665f);                   //If the IMU has yawed transfer the roll angle to the pitch angel
        angleRoll -= anglePitch * sinf(gyroZ * 0.000002665f);                   //If the IMU has yawed transfer the pitch angle to the roll angel
 
        angleTotalVector = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ); 

         //57.296 = 1 / (3.142 / 180) The asin function is in radian
        accelAnglePitch = asinf(accelY/angleTotalVector)* 57.296f;       //Calculate the pitch angle
        accelAngleRoll = asinf(accelX/angleTotalVector)* -57.296f;       //Calculate the roll angle

        anglePitch = anglePitch * 0.9996f + accelAnglePitch * 0.0004f;          //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angleRoll = angleRoll * 0.9996f + accelAngleRoll * 0.0004f;             //Correct the drift of the gyro roll angle with the accelerometer roll angle
        
        if (i ==  100)
        {
            i = 0;
            Nokia5110_SetCursor(6,0);
            Nokia5110_OutFloat(&anglePitch);

            Nokia5110_SetCursor(6,1);
            Nokia5110_OutFloat(&angleRoll);
        }
      }
    }
}
