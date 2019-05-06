#include <stdint.h>

//********ADC_0_Init*****************
// Initialize a ADC0 using sequencer 1
// inputs: none
// outputs: none
void ADC_0_Init();

//********ADC_0_TriggerCapture*****************
// Trigger ADC sampling to occur
// inputs: none
// outputs: none
void ADC_0_TriggerCapture();

//********ADC_0_IsDataAvailable*****************
// Determine if data is ready to be read from sequencer
// inputs: none
// outputs: true if data is available to be read from FIFO, false otherwise
bool ADC_0_IsDataAvailable();

//********ADC_0_GetData*****************
// Determine if data is ready to be read from sequencer's FIFO
// inputs: none
// outputs: none
uint32_t ADC_0_GetData();