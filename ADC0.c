#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>
#include <driverlib/adc.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>

#include "inc/ADC0.h"

void ADC_0_Init()
{
    //
    // For more information refer to Chpt 13.4 of Initialization and Configuration from
    // TM4C123GH6PM datasheet or Chpt 4 of TivaWare Peripheral Driver Library Datasheet
    //

    //
    // Enable the ADC0 module
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
 
    //
    // Wait for the ADC0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }
    
    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    //
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 1
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 1.
    //
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    //
    // Configure step 0 on sequence 1.  Sample channel 5 (ADC_CTL_CH5) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 1 (ADC_CTL_END).
    //
    ADCSequenceStepConfigure(ADC0_BASE,
                             3,
                             0,
                             ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);

    //
    // Enable sample sequence 1 for ADC0
    //
    ADCSequenceEnable(ADC0_BASE, 3);
    
    //
    // Clear interrupt
    //
    ADCIntClear(ADC0_BASE, 3);
    
    return;
}

void ADC_0_TriggerCapture()
{ 
    ADCProcessorTrigger(ADC0_BASE, 3);
}

bool ADC_0_IsDataAvailable()
{
    return ADCIntStatus(ADC0_BASE, 3, false) > 0;
}

uint32_t ADC_0_GetData()
{
    if (!ADC_0_IsDataAvailable())
    {
        return 0;
    }
    
    uint32_t pUi32Value[1];

    //
    // Clear interrupt
    //
    ADCIntClear(ADC0_BASE, 3);

    ADCSequenceDataGet(ADC0_BASE, 3, pUi32Value);
    
    return *pUi32Value;
}
