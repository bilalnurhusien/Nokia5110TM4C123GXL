#include <stdint.h>

//********Timers_Init*****************
// Initialize a 16 bit timer (Timer0A) and 16 bit timer (Timer1A) that counts down from specified values.
// When the timer times out, the interrupt is cleared and the function that's registered is called.
// inputs:
//      pfn0Handler  - Pointer to a function that executes periodically when timeout 0 is reached
//      pfn1Handler  - Pointer to a function that executes periodically when timeout 1 is reached
//      startValue0 - Start value for timer 0 in which to count down from
//      startValue1 - Start value for timer 1 in which to count down from
// outputs: none
bool Timers_Init(void (*pfn0Handler)(void), void (*pfn1Handler)(void), uint32_t startValue0, uint32_t startValue1);