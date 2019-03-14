#include <stdint.h>
#include <stdbool.h>
#include "I2C2.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

void I2C_Init(void){
    //
    // Enable I2C module
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    //
    // Reset I2C module
    //
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    //
    // Enable GPIO peripheral that contains I2C
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Configure the pin muxing for I2C2 functions on port E4 and E5
    //
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);

    //
    // Select the I2C function for these pins.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Enable and initialize the I2C2 master module.  Use the system clock for
    // the I2C2 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

    //
    // Clear I2C FIFOs
    //
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}


uint8_t I2C_Read(uint8_t device_address, uint8_t device_register)
{
    //
    // Specify that we want to communicate to device address with an intended write to bus 
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);

    //
    // Specify register in which to start reading
    //
    I2CMasterDataPut(I2C2_BASE, device_register); 
    
    //
    // Send data
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND); 
    

    while (I2CMasterBusy(I2C2_BASE)); 

    //
    // Specify that we want to communicate to device address with an intended read from bus
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, true);

    //
    // Tell master to read data one byte
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); 
    
    //
    // Wait until transfer is done
    //
    while (I2CMasterBusy(I2C2_BASE));
    
    //
    // Get data
    //
    uint8_t data = I2CMasterDataGet(I2C2_BASE);

    return data;
}

int16_t I2C_Read2Bytes(uint8_t device_address, uint8_t device_register)
{
    //
    // Specify that we want to communicate to device address with an intended write to bus 
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);

    //
    // Specify register in which to start reading
    //
    I2CMasterDataPut(I2C2_BASE, device_register); 
    
    //
    // Send data
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    
    while (I2CMasterBusy(I2C2_BASE)); 
  
    //
    // Specify that we want to communicate to device address with an intended read from bus
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, true);
    
    //
    // Specifiy read operation
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); 
    

    while (I2CMasterBusy(I2C2_BASE));
    
    //
    // Get data
    //
    uint8_t data1 = I2CMasterDataGet(I2C2_BASE);
    
    while (I2CMasterBusy(I2C2_BASE)); 

    //
    // Specify that we want to communicate to device address with an intended read from bus
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, true);

    // 
    //
    // Tell slave to read data one more byte
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); 
    
    //
    // Wait until transfer is done
    //
    while (I2CMasterBusy(I2C2_BASE));
    
    //
    // Get data
    //
    uint8_t data2 = I2CMasterDataGet(I2C2_BASE);

    return (int16_t)(data1 << 8 | data2);
}

void I2C_Write(uint8_t device_address, uint8_t device_register, uint8_t device_data)
{
    //
    // Specify that we want to communicate to device address with an intended write to bus
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);

    //
    // Send register in which to write
    //
    I2CMasterDataPut(I2C2_BASE, device_register);

    //
    // Send control byte and register address byte to slave device
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //
    // Wait for MCU to finish transaction
    //
    while(I2CMasterBusy(I2C2_BASE));

    //
    // Specify that we want to communicate to device address with an intended write to bus
    //
    I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false);

    //
    // Specify data to be written to the above mentioned device_register
    //
    I2CMasterDataPut(I2C2_BASE, device_data);

    //
    // Wait while checking for MCU to complete the transaction
    //
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    //
    // Wait for MCU and device to complete transaction
    //
    while(I2CMasterBusy(I2C2_BASE));
}
 