#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>
#include <driverlib/adc.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>

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
 
    //
    // Wait for the ADC0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    
    //
    // Disable ADC0 sequencer 1
    //
    ADCSequenceDisable(ADC0_BASE, 1);

    //
    // Enable the first sample sequencer to capture the value of channel 0 (PE3) when
    // the processor trigger occurs using ADCProcessorTrigger() function
    //
    ADCSequenceConfigure(
        ADC0_BASE,
        1,
        ADC_TRIGGER_PROCESSOR,
        0);
    ADCSequenceStepConfigure(
        ADC0_BASE,
        1,
        0,
        ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);

    //
    // Enable ADC0 sequencer 1
    //
    ADCSequenceEnable(ADC0_BASE, 1);
    
    return;
}

void ADC_0_TriggerCapture()
{
    //
    // Clear interrupt
    //
    ADCIntClear(ADC0_BASE, 1);
  
    ADCProcessorTrigger(ADC0_BASE, 1);
}

bool ADC_0_IsDataAvailable()
{
    return ADCIntStatus(ADC0_BASE, 1, false) > 0;
}

uint32_t ADC_0_GetData()
{
    if (!ADC_0_IsDataAvailable())
    {
        return 0;
    }
    
    uint32_t ui32Value;
    
    ADCSequenceDataGet(ADC0_BASE, 1, &ui32Value);
    
    return ui32Value;
}
