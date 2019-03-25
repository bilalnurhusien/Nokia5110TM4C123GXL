#include <stdint.h>

//********Timer0_Init*****************
// Initialize a 16 bit timer (TimerA) that counts down from a specified value.
// When the timer times out, the interrupt is cleared and the function that's registered is called.
// inputs:
//      pfnHandler  - Pointer to a function that executes periodically when timeout is reached
//      startValue - Start value in which to count down from
// outputs: none
bool Timer_0A_Init(void (*pfnHandler)(void), uint32_t startValue);