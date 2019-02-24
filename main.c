// main.c
// Runs on LM4F120/TM4C123
// Nokia5110 LCD 
// Daniel Valvano
// May 23, 2014

/* This example accompanies the books
  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014

"Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
 
 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
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
#include <stdio.h>
#include "Nokia5110.h"
void initI2C0(void)
{
	//This function is for eewiki and is to be updated to handle any port

	//enable I2C module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

	//reset I2C module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

	//enable GPIO peripheral that contains I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure the pin muxing for I2C2 functions on port D0 and D1
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

	// Enable and initialize the I2C0 master module.  Use the system clock for
	// the I2C0 module.  The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will
	// be set to 400kbps.
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);

	//clear I2C FIFOs
	HWREG(I2C3_BASE + I2C_O_FIFOCTL) = 80008000;
}

uint8_t readI2C0(uint16_t device_address, uint16_t device_register)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//the register to be read
	I2CMasterDataPut(I2C3_BASE, device_register);

	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//wait for MCU to complete send transaction
	while(I2CMasterBusy(I2C3_BASE));

	//read from the specified slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, true);

	//send control byte and read from the register from the MCU
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

	//wait while checking for MCU to complete the transaction
	while(I2CMasterBusy(I2C3_BASE));

	//Get the data from the MCU register and return to caller
	return( I2CMasterDataGet(I2C3_BASE));
}

uint16_t readI2C0TwoBytes(uint16_t device_address, uint16_t device_register)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//the register to be read
	I2CMasterDataPut(I2C3_BASE, device_register);

	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//wait for MCU to complete send transaction
        while (I2CMasterBusy(I2C3_BASE)); //Wait till end of transaction

	//read from the specified slave device
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, true);

	//send control byte and read from the register from the MCU
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

	//wait while checking for MCU to complete the transaction
        while (!I2CMasterBusy(I2C3_BASE));
        while (I2CMasterBusy(I2C3_BASE)); //Wait till end of transaction
        
     	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

	//wait while checking for MCU to complete the transaction
        while (!I2CMasterBusy(I2C3_BASE));
        while (I2CMasterBusy(I2C3_BASE)); //Wait till end of transaction

	//Get the data from the MCU register and return to caller
	uint8_t data1 = I2CMasterDataGet(I2C3_BASE);
        
        //wait while checking for MCU to complete the transaction
        while (!I2CMasterBusy(I2C3_BASE));
	while(I2CMasterBusy(I2C3_BASE));
       
	//Get the data from the MCU register and return to caller
	uint8_t data2 = I2CMasterDataGet(I2C3_BASE);
        
        uint16_t retVal = (data1 << 8 | data2 << 8);
        
        return retVal;
}

void writeI2C0(uint16_t device_address, uint16_t device_register, uint8_t device_data)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, false);

	//register to be read
	I2CMasterDataPut(I2C3_BASE, device_register);

	//send control byte and register address byte to slave device
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//wait for MCU to finish transaction
	while(I2CMasterBusy(I2C3_BASE));

	I2CMasterSlaveAddrSet(I2C3_BASE, device_address, true);

	//specify data to be written to the above mentioned device_register
	I2CMasterDataPut(I2C3_BASE, device_data);

	//wait while checking for MCU to complete the transaction
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//wait for MCU & device to complete transaction
	while(I2CMasterBusy(I2C3_BASE));
}

#define I2C_MPU_6050_ADDR               0x68
#define I2C_MPU_6050_ACCEL_CONFIG       0x1C
#define I2C_MPU_6050_POWER_CONFIG       0x6B
#define I2C_MPU_6050_GYRO_CONFIG        0x1B
#define I2C_MPU_6050_WHO_AM_I           0x75
#define I2C_MPU_6050_ACCEL_X1           0x3B
#define I2C_MPU_6050_ACCEL_X2           0x3C
#define I2C_MPU_6050_ACCEL_Y1           0x3D
#define I2C_MPU_6050_ACCEL_Y2           0x3E
#define I2C_MPU_6050_ACCEL_Z1           0x3F
#define I2C_MPU_6050_ACCEL_Z2           0x3G

int main(void)
{
    Output_Init();
    initI2C0();
    writeI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_POWER_CONFIG, 0);
    writeI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_CONFIG, 0x10);
    writeI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_GYRO_CONFIG, 0x08);

    Nokia5110_Clear();

    Nokia5110_OutString("AccX:");
    Nokia5110_SetCursor(0,1);
    Nokia5110_OutString("AccY:");
    Nokia5110_SetCursor(0,2);
    Nokia5110_OutString("AccZ:");
    Nokia5110_SetCursor(0,3);
    Nokia5110_OutString("Temp:");
    
    uint8_t who = readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_WHO_AM_I);
    Nokia5110_OutUDec(who);
    who = readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_X1);
    Nokia5110_OutUDec(who);
    int16_t data = (readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_X1) << 8 | readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_X2)) ;
    Nokia5110_SetCursor(5,0);
    Nokia5110_OutUDec(data);

    data = (readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_Y1) << 8 | readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_Y2)) ;
    Nokia5110_SetCursor(5,1);
    Nokia5110_OutUDec(data);

   // data = (readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_Z1) << 8 | readI2C0(I2C_MPU_6050_ADDR, I2C_MPU_6050_ACCEL_Z2)) ;
    Nokia5110_SetCursor(5,2);
    Nokia5110_OutUDec(data);    

    while (1)
    {
    }
}
