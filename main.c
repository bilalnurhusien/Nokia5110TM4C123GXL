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
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>

#include "inc/Nokia5110.h"
#include "inc/Mpu6050.h"
#include "inc/Timers.h"

//
// Notes:
//  Pitch (inclination)- angle wrt the x-axis
//  Roll (tilt)- angle wrt the y-axis
//  Yaw  angle wrt the z-axis
//

volatile int imuDataRefreshTimeout = 0;
volatile int recordReferencePitchAngle = 0;
volatile int heartBeat = 0;

#define ADC_RESOLUTION  4096.0f
#define ADC_MAX_VOLTAGE 3.3f

//
// Toggle GPIO
//
void ToggleGpio(uint32_t ui32Port, uint8_t ui8Pin)
{
    uint8_t gpioStatus =  (GPIOPinRead(ui32Port, ui8Pin) & 0xFF);
    gpioStatus ^= ui8Pin;
    GPIOPinWrite(ui32Port, ui8Pin, gpioStatus);
}

//
// Interrupt handler that causes main thread to retrieve data from IMU every 10 ms
//
void TimerRefreshImuDataHandler(void){
    imuDataRefreshTimeout = 1;
}

//
// Interrupt handler for the heart beat
//
void TimerHeartBeatHandler(void){
    heartBeat = 1;
}

//
// Write static data to LCD
//
void SetUpLcdScreen()
{
    Nokia5110_Clear();
    Nokia5110_OutString("Pitch:");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Ref  :");
    Nokia5110_SetCursor(0,2);
    Nokia5110_OutString("Volts:");
    Nokia5110_SetCursor(0,3);
    Nokia5110_OutString("Prop :");
    Nokia5110_SetCursor(0,4);
    Nokia5110_OutString("Integ:");
    Nokia5110_SetCursor(0,5);
    Nokia5110_OutString("Deriv:");
}

//
// Interrupt handler for SW1
//
void SW1InterruptHandler() {
    if ((GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) == GPIO_PIN_4) {
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag
        recordReferencePitchAngle = true;
    }
}

//
// Initialize GPIOF (Pins: 1, 2, and 4)
//
void InitializeGPIOF()
{
    //
    // Enable port F
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Wait for the GPIOF module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    
    //
    // Set pin 1 and 2 as outputs, SW controlled.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);    
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
    
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  // Init PF4 as input
    GPIOPadConfigSet(GPIO_PORTF_BASE,
                     GPIO_PIN_4,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);            // Enable weak pullup resistor for PF4
    
    //
    // Interrupt setup
    //
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);        // Disable interrupt for PF4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);          // Clear pending interrupts for PF4
    GPIOIntRegister(GPIO_PORTF_BASE,
                    SW1InterruptHandler);               // Register our handler function for port F
    GPIOIntTypeSet(GPIO_PORTF_BASE,
                   GPIO_PIN_4,
                   GPIO_FALLING_EDGE);                  // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);         // Enable interrupt for PF4
}
  
//
// Initialize LCD, IMU, and Timers
//
bool Initialize()
{
    //
    // Initialize timer for IMU to 10 msec and timer for heart beat to 1 sec 
    //
    uint32_t periodImu = SysCtlClockGet()/100;
    uint32_t periodHeartBeat = SysCtlClockGet();
 
    if (!Timers_Init(&TimerRefreshImuDataHandler, &TimerHeartBeatHandler, periodImu, periodHeartBeat))
    {
        printf("Failed to initialize timers");
        return false;     
    }
     
    //
    // Nokia 5110 Initialization
    //
    Output_Init();
    
    //
    // Initialize GPIOF 
    //
    InitializeGPIOF();
    
    //
    // MPU-6050 Initialization
    //
    MPU_6050_Init(I2C_MPU_6050_GYRO_CONFIG_500_DPS, I2C_MPU_6050_ACCEL_CONFIG_8G);
    
    int successfulProbe = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (MPU_6050_Probe())
        {
          successfulProbe = 1;
          break;
        }
        SysCtlDelay(3);
    }
    
    if (successfulProbe != 1)
    {
        printf("Failed to find MPU-6050 IMU\n");
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

    //
    // Iniitalize ADC0
    //
    ADC_0_Init();
    
    return true;
}

//
// Calibrate the gyroscope data by taking several samples and using those values for future use
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
    Nokia5110_OutString("Gyro");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Calibrating");
    
    while (i < CalibrationCount)
    {     
        if (imuDataRefreshTimeout)
        {
            //
            // Toggle GPIO B7 for debugging purposes
            //
            ToggleGpio(GPIO_PORTB_BASE, GPIO_PIN_7);
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

//
// Calibrate the accelerometer data by taking several samples and using those values for future use
//
void CalibrateAccelData(int16_t * accelXCal, int16_t * accelYCal, int16_t * accelZCal)
{
    if (NULL == accelXCal ||
        NULL == accelYCal ||
        NULL == accelZCal)
    {
        printf("Invalid argument\n");
        return;
    }
    
    const uint32_t CalibrationCount = 500;
   
    int16_t accelX, accelY, accelZ, i;
    int64_t accelXCalSum,  accelYCalSum, accelZCalSum;

    accelXCalSum = accelYCalSum = accelZCalSum = 0;
    *accelXCal = *accelYCal = *accelZCal = 0;
    accelX = accelY = accelZ = i = 0;

    Nokia5110_Clear();
    Nokia5110_SetCursor(0,0);
    Nokia5110_OutString("Accel");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Calibrating");
    
    while (i < CalibrationCount)
    {     
        if (imuDataRefreshTimeout)
        {
            //
            // Toggle GPIO B7 for debugging purposes
            //
            ToggleGpio(GPIO_PORTB_BASE, GPIO_PIN_7);
            imuDataRefreshTimeout = 0;
            MPU_6050_GetGyroData(&accelX, &accelY, &accelZ);
            accelXCalSum += accelX;
            accelYCalSum += accelY;
            accelZCalSum += accelZ;
            ++i;
            if ((i % 100) == 0)
            {
                Nokia5110_OutChar('.');
            }
        }
    }
    
    *accelXCal = (accelXCalSum / CalibrationCount);
    *accelYCal = (accelYCalSum / CalibrationCount);
    *accelZCal = (accelZCalSum / CalibrationCount);
}


int main(void)
{
    int8_t initializeFailed = 0;
  
    //
    // Enable floating point unit and disable stacking (i.e. FPU instructions aren't allowed within interrupt handlers)
    //
    FPUEnable();
    FPUStackingDisable();
    
    //
    // Set clock to 16 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlDelay(3);

    //
    // Initialize timers and peripherals
    //
    if (!Initialize())
    {
        printf("Failed to initialize\n");
        initializeFailed = 1;
    }
    
    while (initializeFailed)
    {
        if (heartBeat)
        {
            heartBeat = 0;
            
            //
            // Toggle Red LED pin
            //
            ToggleGpio(GPIO_PORTF_BASE, GPIO_PIN_1);
        }
    }
    
    int16_t gyroXCal, gyroYCal, gyroZCal;
    int16_t accelXCal, accelYCal, accelZCal;
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    volatile float anglePitch = 0;
    volatile float accelAnglePitch  = 0;
    volatile float angleIntermCalc = 0;
    volatile float anglePitchReference = 90.0f;
    volatile float accelAngleTotal = 0;
    volatile float voltageValue = 0;
    
    CalibrateGyroData(&gyroXCal, &gyroYCal, &gyroZCal);
    CalibrateAccelData(&accelXCal, &accelYCal, &accelZCal);
    
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
    // Note: 57.296 = 1 / (3.142 / 180) - the asin function returns radians
    //
    accelAngleTotal = sqrtf(((float)accelX)*((float)accelX)+((float)accelY)*((float)accelY)+((float)accelZ)*((float)accelZ));  //Calculate the total accelerometer vector
    angleIntermCalc = ((float)accelY) / (accelAngleTotal);    
    accelAnglePitch = asinf(angleIntermCalc)* 57.296f;                          // Calculate the pitch angle
    anglePitch = accelAnglePitch;
        
    int count = 0;
    
    ADC_0_TriggerCapture();
    
    while (1)
    {
      if (recordReferencePitchAngle)
      {
        recordReferencePitchAngle = 0;
        anglePitchReference = anglePitch;
      }
      
      if (heartBeat)
      {
        heartBeat = 0;
        
        //
        // Get value from voltage value from ADC 
        //
        if (ADC_0_IsDataAvailable())
        {
            uint32_t adcValue = ADC_0_GetData();          
            voltageValue = (adcValue / ADC_RESOLUTION) * ADC_MAX_VOLTAGE;

            ADC_0_TriggerCapture();
        }
        
        //
        // Toggle Green LED pin
        //
        ToggleGpio(GPIO_PORTF_BASE, GPIO_PIN_3);
      }
      
      if (imuDataRefreshTimeout)
      {
        imuDataRefreshTimeout = 0;
        ++count;
        
        //
        // Toggle GPIO B7 at start of loop for debugging purposes
        //
        ToggleGpio(GPIO_PORTB_BASE, GPIO_PIN_7);

        MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
        MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);
        
        gyroX -= gyroXCal;
        
        //
        // Gyro angle calculation: Integrate the gyro values (angular velocity) every 10 ms
        // and divide by the sensitivity scale factor (65.5 LSB/g). This will
        // give us the current angular change in position. Note: gyro values will drift 
        // over time, so we'll need to combine this value with acceleromter data later on.
        // Note: 0.0001527 = 1 / 100Hz / 65.5
        //
        anglePitch += gyroX * 0.0001527f;                                       // Calculate the traveled pitch angle and add this to the anglePitch variable

        // Note - the accelerometer data is sensitive to vibration from the motor
        // so we'll need to combine the accelerometer data with the gyroscope using
        // a complemetary filter later on in the code. Calculate the total accelerometer vector       
        accelAngleTotal = sqrtf(((float)accelX)*((float)accelX)+((float)accelY)*((float)accelY)+((float)accelZ)*((float)accelZ));  
        angleIntermCalc = ((float)accelY) / (accelAngleTotal);
        
        //
        // 57.296 = 1 / (3.142 / 180) The asin function is in radians
        //
        accelAnglePitch = asinf(angleIntermCalc)* 57.296f;              // Calculate the pitch angle

        //
        // Complementary filter
        //
        anglePitch = anglePitch * 0.9996f + accelAnglePitch * 0.0004f;              // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        
        //
        // Send pitch and roll data to LCD
        //
        if (count ==  100)
        {
            count = 0;
            Nokia5110_SetCursor(6,0);
            Nokia5110_OutFloat(anglePitch);
            Nokia5110_SetCursor(6,1);
            Nokia5110_OutFloat(anglePitchReference);
            Nokia5110_SetCursor(6,2);
            Nokia5110_OutFloat(voltageValue);           
        }
      }
    }
}
