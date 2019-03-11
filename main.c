#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <inc/hw_memmap.h>

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
    Nokia5110_OutString("GyroX:");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("GyroY:");
    Nokia5110_SetCursor(0,2);
    Nokia5110_OutString("GyroZ:");
    Nokia5110_SetCursor(0,3);
    Nokia5110_OutString("AccX :");
    Nokia5110_SetCursor(0,4);
    Nokia5110_OutString("AccY :");
    Nokia5110_SetCursor(0,5);
    Nokia5110_OutString("AccZ :");
}

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
    
    if (!MPU_6050_Probe())
    {
        printf("Failed to find MPU-6050 IMU\n");
        return false;
    }
                  
    //
    // Timer initialization for 4ms second timeout
    //
    uint32_t period = SysCtlClockGet()/250;
    
    if (!Timer_0A_Init(&Timer0A_Handler, period))
    {
        printf("Failed to initialize Timer0A");
        return false;     
    }
    
    //
    // Enable the GPIOA peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    //
    // Wait for the GPIOA module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    
    //
    // Set pins 7 as output, SW controlled.
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
    // Take 2000 samples on refresh timeout
    //
    const uint32_t CalibrationCount = 2000;
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
            
            if ((i % 250) == 0)
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
    SysCtlClockSet(SYSCTL_SYSDIV_1  | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    SysCtlDelay(3);    
    
    if (!Initialize())
    {
        printf("Failed to initialize\n");
        return false;
    }

    int64_t gyroXCal, gyroYCal, gyroZCal;
    CalibrateGyroData(&gyroXCal, &gyroYCal, &gyroZCal);
    
    SetUpLcdScreen();
    
    int16_t gyroX, gyroY, gyroZ, accelX, accelY, accelZ; 
    uint8_t gpioB7Status =  (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) & 0xFF);
    while (1)
    {
      if (refreshTimeout)
      {
        refreshTimeout = 0;
        gpioB7Status ^= GPIO_PIN_7;
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, gpioB7Status);
        
        MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
        MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
 
        gyroX -= gyroXCal;
        gyroY -= gyroYCal;
        gyroZ -= gyroZCal;
        
        Nokia5110_SetCursor(6,0);
        Nokia5110_OutDec(gyroX);

        Nokia5110_SetCursor(6,1);
        Nokia5110_OutDec(gyroY);

        Nokia5110_SetCursor(6,2);
        Nokia5110_OutDec(gyroZ);

        Nokia5110_SetCursor(6,3);
        Nokia5110_OutDec(accelX);
                
        Nokia5110_SetCursor(6,4);
        Nokia5110_OutDec(accelY);
        
        Nokia5110_SetCursor(6,5);
        Nokia5110_OutDec(accelZ);;      
      }
    }
}
