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
#include <driverlib/interrupt.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>

#include "inc/Nokia5110.h"
#include "inc/Mpu6050.h"
#include "inc/Timers.h"

// These volatile variables are used in the interrupt handlers
volatile int imuDataRefreshTimeout = 0;
volatile int motorForward = 0;

// ADC has 12 bit resolution is 4096
#define ADC_MAX_VALUE  4096.0f

// ADC max voltage tolerance is 3.3V
#define ADC_MAX_VOLTAGE 3.3f

//
// (3.3 kOhm /(3.3 kOhm + 10.0 kOhm))
//
#define RESISTOR_DIV_RATIO (3.3f / 13.3f)

//
// Used to convert ADC voltage value to the battery voltage
//  
//  ADC_MAX_VOLTAGE / (ADC_MAX_VALUE * RESISTOR_DIV_RATIO) = 0.00324707
//
#define ADC_BATTERY_MULTIPLIER 0.00324707f

// Low voltage for batter is 10 Volts
#define MIN_BATTERY_VOLTAGE 10

#define ENABLE_LOW_BATTERY_ERROR 0

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
// Set GPIO
//
void SetGpio(uint32_t ui32Port, uint8_t ui8Pin)
{
    uint8_t gpioStatus =  (GPIOPinRead(ui32Port, ui8Pin) & 0xFF);
    gpioStatus |= ui8Pin;
    GPIOPinWrite(ui32Port, ui8Pin, gpioStatus);
}

//
// Clear GPIO
//
void ClearGpio(uint32_t ui32Port, uint8_t ui8Pin)
{
    uint8_t gpioStatus =  (GPIOPinRead(ui32Port, ui8Pin) & 0xFF);
    gpioStatus &= ~ui8Pin;
    GPIOPinWrite(ui32Port, ui8Pin, gpioStatus);
}

//
// Interrupt handler that causes main thread to retrieve data from IMU every 10 ms
//
void TimerRefreshImuDataHandler(void){
    imuDataRefreshTimeout = 1;
}

//
// Interrupt handler for the motor
//
void TimerMotorHandler(void){
    //
    // Set GPIO B7 for direction of motor and E2 for motor step
    //
   if (motorForward)
   {
      SetGpio(GPIO_PORTB_BASE, GPIO_PIN_7);
      ToggleGpio(GPIO_PORTE_BASE, GPIO_PIN_2);
   }
   else if (motorForward == -1)
   {
      ClearGpio(GPIO_PORTB_BASE, GPIO_PIN_7);
      ToggleGpio(GPIO_PORTE_BASE, GPIO_PIN_2);
   }
}

//
// Write static data to LCD
//
void SetUpLcdScreen()
{
    Nokia5110_Clear();
    Nokia5110_OutString("Pitch:");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("Batt :");
    Nokia5110_SetCursor(0,2);
    Nokia5110_OutString("Prop :");
    Nokia5110_SetCursor(0,3);
    Nokia5110_OutString("Integ:");
    Nokia5110_SetCursor(0,4);
    Nokia5110_OutString("Deriv:");
}

//
// Interrupt handler for SW1
//
void SW1InterruptHandler() {
    if ((GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) == GPIO_PIN_4) {
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag
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
    // Set pin 1, 2, and 3 as outputs, SW controlled.
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
// Initialize GPIOE for stepper motor (Pins: 1, 2)
//
void InitializeGPIOE()
{
    //
    // Enable port F
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Wait for the GPIOF module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    
    //
    // Set pin 1 and 2 as outputs, SW controlled.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);    
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
}

//
// Initialize GPIOB for switch 1
//
void InitializeGPIOB()
{
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
    
    ClearGpio(GPIO_PORTB_BASE, GPIO_PIN_7);
}
  
//
// Initialize LCD, IMU, and Timers
//
bool Initialize()
{
    //
    // Wait for peripherals to fully power up
    //
    SysCtlDelay(SysCtlClockGet());  
  
    //
    // Initialize timer for IMU to 10 msec and timer for motor to 100 us
    //
    uint32_t periodImu = SysCtlClockGet() / 100;
    uint32_t periodMotor = SysCtlClockGet() / 2000;
 
    if (!Timers_Init(&TimerMotorHandler, &TimerRefreshImuDataHandler, periodMotor, periodImu))
    {
        printf("Failed to initialize timers");
        return false;     
    }
     
    //
    // Nokia 5110 Initialization
    //
    Output_Init();
    
    //
    // Initialize GPIOF for LEDs
    //
    InitializeGPIOF();
    
    //
    // Initialize GPIOE for stepper motor DIR and STEP pins
    //
    InitializeGPIOE();
    
    //
    // Initialize GPIOB for switch 1
    //
    InitializeGPIOB();
    
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
    // Iniitalize ADC0
    //
    ADC_0_Init();
    
    return true;
}

//
// Low battery voltage scenario
//
void LowVoltageError(float voltageValue)
{
#if ENABLE_LOW_BATTERY_ERROR == 1

    Nokia5110_Clear();
    Nokia5110_OutString("Low Volt:");
    Nokia5110_OutFloat(voltageValue);
    
    while (1)
    {
       // Do nothing ...
    }
#endif
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
            imuDataRefreshTimeout = 0;
            MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
            gyroXCalSum += gyroX;
            gyroYCalSum += gyroY;
            gyroZCalSum += gyroZ;
            ++i;
            if ((i % 100) == 0)
            {
                Nokia5110_OutChar('.');
                //
                // Toggle F1 for debugging purposes (red LED)
                //
                ToggleGpio(GPIO_PORTF_BASE, GPIO_PIN_1);
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
            imuDataRefreshTimeout = 0;
            MPU_6050_GetGyroData(&accelX, &accelY, &accelZ);
            accelXCalSum += accelX;
            accelYCalSum += accelY;
            accelZCalSum += accelZ;
            ++i;
            if ((i % 100) == 0)
            {
                Nokia5110_OutChar('.');
                ToggleGpio(GPIO_PORTF_BASE, GPIO_PIN_1);
            }
        }
    }
    
    *accelXCal = (accelXCalSum / CalibrationCount);
    *accelYCal = (accelYCalSum / CalibrationCount);
    *accelZCal = (accelZCalSum / CalibrationCount);
}

//
// TODO: Remove magic numbers
//
int main(void)
{
    //
    // Enable floating point unit and disable stacking (i.e. FPU instructions
    // aren't allowed within interrupt handlers)
    //
    FPUEnable();
    FPUStackingDisable();
     
    //
    // Pitch - angle with respect to X-axis
    // Roll - angle with respect to Y-axis not needed for balancing robot
    //
    const float MinAnglePitch = -40.0f;
    const float MaxAnglePitch = 40.0f;
    int8_t initializeFailed = 0;

    
    //
    // Set clock to 16 MHz
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    //
    // Initialize timers and peripherals
    //
    if (!Initialize())
    {
        printf("Failed to initialize\n");
        initializeFailed = 1;
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }
    
    //
    // Loop forever on error
    //
    while (initializeFailed)
    {
    }
    
    int16_t gyroXCal, gyroYCal, gyroZCal;
    int16_t accelXCal, accelYCal, accelZCal;
    int16_t gyroX, gyroY, gyroZ;
    int16_t accelX, accelY, accelZ;
    float anglePitch = 0;
    float accelAnglePitch  = 0;
    float angleIntermCalc = 0;
    float accelAngleTotal = 0;
    float voltageValue = 0;
    
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
    // Calculate the pitch angle in degrees
    // Note: 57.296 = 180 deg / 3.142 rads - the asin function returns radians
    //
    accelAngleTotal = sqrtf(((float)accelX)*((float)accelX)+
                            ((float)accelY)*((float)accelY)+
                            ((float)accelZ)*((float)accelZ));
    angleIntermCalc = ((float)accelY) / (accelAngleTotal);    
    accelAnglePitch = asinf(angleIntermCalc)* 57.296f;

    // Set initial angle provided by accelerometer since robot is at rest initially
    // Accelerometer can provide initial angle when robot is at rest
    anglePitch = accelAnglePitch;
 
    int heartbeatCount = 0;
    
    ADC_0_TriggerCapture();
    
    while (1)
    {          
        if (imuDataRefreshTimeout)
        {
            imuDataRefreshTimeout = 0;
            ++heartbeatCount;

            MPU_6050_GetGyroData(&gyroX, &gyroY, &gyroZ);
            MPU_6050_GetAccelData(&accelX, &accelY, &accelZ);

            gyroX -= gyroXCal;

            //
            // Gyro angle calculation: Calculate the traveled pitch angle
            // and add this to the anglePitch variable. Integrate the gyro values
            // (angular velocity) every 10 ms and divide by the sensitivity
            // scale factor (65.5 LSB/g). This will give us the current angular
            // change in position. The gyro values will drift over time,
            // so we'll need to combine this value with acceleromter data later on.
            // Note: 0.0001527 = 1 / 100Hz / 65.5
            //
            anglePitch += gyroX * 0.0001527f;

            //  Calculate the total accelerometer vector       
            accelAngleTotal = sqrtf(((float)accelX)*((float)accelX)+
                                    ((float)accelY)*((float)accelY)+
                                    ((float)accelZ)*((float)accelZ));
 
            angleIntermCalc = ((float)accelY) / (accelAngleTotal);

            //
            // Calculate the pitch angle
            // 57.296 = 1 / (3.142 / 180) The asin function is in radians
            //
            accelAnglePitch = asinf(angleIntermCalc)* 57.296f;

            //
            // Gyroscope readings over time are susceptible to drift so add the accelerometer
            // pitch angle using a complementary filter. The accelerometer data is sensitive
            // to vibrations from the motor so we'll need to combine the accelerometer data
            // with the gyroscope as follows:
            //
            anglePitch = anglePitch * 0.9996f + accelAnglePitch * 0.0004f;

            // When robot is within reasonable range, operate motor
            if (MinAnglePitch <= anglePitch &&
                MaxAnglePitch >= anglePitch)
            {
                if (anglePitch > 5)
                {                 
                  motorForward = 1; 
                }
                else if (anglePitch < -5)
                {
                  motorForward = -1;
                }
                else
                {
                  motorForward = 0;
                }
            }

            if (heartbeatCount ==  100)
            {
                //
                // Send pitch and roll data to LCD
                //             
                heartbeatCount = 0;
                Nokia5110_SetCursor(6,0);
                Nokia5110_OutFloat(anglePitch);
                Nokia5110_SetCursor(6,1);
                Nokia5110_OutFloat(voltageValue); 
                
                //
                // Get voltage value from ADC 
                //
                if (ADC_0_IsDataAvailable())
                {
                    uint32_t adcValue = ADC_0_GetData();          
                    voltageValue = ((float)adcValue * ADC_BATTERY_MULTIPLIER);

                    if (voltageValue < MIN_BATTERY_VOLTAGE)
                    {
                      LowVoltageError(voltageValue);
                    }

                    ADC_0_TriggerCapture();
                }
                           
                //
                // Toggle Green LED pin (heartbeat)
                //
                ToggleGpio(GPIO_PORTF_BASE, GPIO_PIN_3);
            }
        }
    }
}
