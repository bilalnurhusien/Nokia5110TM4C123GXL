#include <stdint.h>

//********Timer0_Init*****************
// Initialize a 16 bit timer (TimerA) and 16 bit timer (TimerB) that counts down from specified values.
// When the timer times out, the interrupt is cleared and the function that's registered is called.
// inputs:
//      pfnHandlerA  - Pointer to a function that executes periodically when timeout A is reached
//      pfnHandlerB  - Pointer to a function that executes periodically when timeout B is reached
//      startValueA - Start value for timer A in which to count down from
//      startValueB - Start value for timer B in which to count down from
// outputs: none
bool Timer_0_Init(void (*pfnAHandler)(void), void (*pfnBHandler)(void), uint32_t startValueA, uint32_t startValueB);