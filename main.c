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

#include "inc/Nokia5110.h"
#include "inc/Mpu6050.h"
#include "inc/Timer0A.h"

//
// Notes:
//  Pitch (inclination)- angle from the x-axis
//  Roll (tilt)- angle from the y-axis
//  Yaw  angle from the z-axis
//

volatile int imuDataRefreshTimeout = 0;

//
// Toggle GPIO B7 which is used for debugging purposes
// when getting data from the IMU
//
void ToggleGpioB7()
{
    uint8_t gpioB7Status =  (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) & 0xFF);
    gpioB7Status ^= GPIO_PIN_7;
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, gpioB7Status);
}

//
// Interrupt handler for Timer0A which is
// used to clear the IMU refresh flag
// every 10 ms
//
void Timer0A_Handler(void){
    imuDataRefreshTimeout = 1;
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
    // Initialize timer to 10ms second timeout
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

//
// Calibrate the IMU by taking several samples and using that value for future use
//
void CalibrateGyroData(int16_t * gyroXCal, int16_t * gyroYCal, int16_t * gyroZCal)
{
    if (NULL == gyroXCal ||
        NULL == gyroYCal ||
        NULL == gyroZCal)
    {
        printf("Invalid argument\n");
        return;
    }
    
    const uint32_t CalibrationCount = 500;
   
    int16_t gyroX, gyroY, gyroZ, i;
    int64_t gyroXCalSum,  gyroYCalSum, gyroZCalSum;

    gyroXCalSum = gyroYCalSum = gyroZCalSum = 0;
    *gyroXCal = *gyroYCal = *gyroZCal = 0;
    gyroX = gyroY = gyroZ = i = 0;

    Nokia5110_Clear();
    Nokia5110_SetCursor(0,0);
    Nokia5110_OutString("MPU-6050 IMU");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Calibrating");
    
    while (i < CalibrationCount)
    {     
        if (imuDataRefreshTimeout)
        {
            //
            // Toggle GPIO B7 for debugging purposes
            //
            ToggleGpioB7();
            imuDataRefreshTimeout = 0;
            MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
            gyroXCalSum += gyroX;
            gyroYCalSum += gyroY;
            gyroZCalSum += gyroZ;
            ++i;
            if ((i % 100) == 0)
            {
                Nokia5110_OutChar('.');
            }
        }
    }
    
    *gyroXCal = (gyroXCalSum / CalibrationCount);
    *gyroYCal = (gyroYCalSum / CalibrationCount);
    *gyroZCal = (gyroZCalSum / CalibrationCount);
}

int main(void)
{
    //
    // Enable floating point unit and disable stacking (FPU instructions aren't allowed within interrupt handlers)
    //
    FPUEnable();
    FPUStackingDisable();
    
    //
    // Set clock to 16 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    
    //
    // Initialize timer and peripherals
    //
    if (!Initialize())
    {
        printf("Failed to initialize\n");
        return false;
    }

    int16_t gyroXCal, gyroYCal, gyroZCal;
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    volatile float anglePitch = 0;
    volatile float accelAnglePitch  = 0;
    volatile float accelAngleRoll = 0;
    volatile float angleIntermCalc = 0;
    
    CalibrateGyroData(&gyroXCal, &gyroYCal, &gyroZCal);
    
    SetUpLcdScreen();

    MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
    MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
    
    gyroX = gyroX;
    gyroY = gyroY;
    gyroZ = gyroZ;
    gyroX -= gyroXCal;
    gyroY -= gyroYCal;
    gyroZ -= gyroZCal;
    
    //
    // 57.296 = 1 / (3.142 / 180) - the asin function returns radians
    //
    angleIntermCalc = accelY / ((float)accelZ);
    accelAnglePitch = atanf(angleIntermCalc)* 57.296f;                // Calculate the pitch angle
     
    anglePitch = accelAnglePitch;                                               // Set the initial gyro pitch angle equal to the accelerometer pitch angle 
    
    int count = 0;
    while (1)
    {
      if (imuDataRefreshTimeout)
      {
        imuDataRefreshTimeout = 0;
        ++count;
        
        //
        // Toggle GPIO B7 for debugging purposes
        //
        ToggleGpioB7();

        MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
        MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
        
        gyroX -= gyroXCal;

        //
        // Gyro angle calculation: Integrate the gyro values every 10 ms
        // and divide by the sensitivity scale factor (65.5 LSB/g). This will
        // give us the current angular position
        // 0.0001527 = 1 / 100Hz / 65.5
        //
        anglePitch += gyroX * 0.0001527f;                                        // Calculate the traveled pitch angle and add this to the angle_pitch variable
  
        //
        // Accelerometer calculations
        //
        if (accelZ < 0)
        {
            accelAnglePitch = (accelY < 0 ? -90.0f : 90.0f);
        }
        else
        {
            angleIntermCalc = accelY / ((float)accelZ);
            // 57.296 = 1 / (3.142 / 180) The asin function is in radians
            accelAnglePitch = atanf(angleIntermCalc)* 57.296f;              // Calculate the pitch angle
        }

        //
        // Complementary filter
        //
        anglePitch = anglePitch * 0.9995f + accelAnglePitch * 0.0005f;              // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        
        //
        // Send pitch and roll data to LCD
        //
        if (count ==  100)
        {
            count = 0;
            Nokia5110_SetCursor(6,0);
            Nokia5110_OutFloat(&anglePitch);
        }
      }
    }
}
