#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>

#include "Timer0A.h"

static void (*pTimer0AUserIntHandler)(void);

static void Timer_0A_Handler(void){
    //
    // Acknowledge timer0A timeout
    //
    TimerIntClear(TIMER0_BASE, TimerIntStatus(TIMER0_BASE, false));
    pTimer0AUserIntHandler();
}

bool Timer_0A_Init(void (*pfnHandler)(void), uint32_t startValue)
{
    //
    // For more information refer to Chpt 11.4 of Initialization and Configuration from
    // TM4C123GH6PM datasheet or Chpt 29 of TivaWare Peripheral Driver Library Datasheet
    //
  
    if (NULL == pfnHandler)
    {
        return false;
    }
    
    pTimer0AUserIntHandler = pfnHandler;

    //
    // Enable the Timer0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
 
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    //
    // Configure timer A for periodic mode
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    
    // Load start value
    TimerLoadSet(TIMER0_BASE, TIMER_A, startValue);

    // Enable interrupt for timer A and register handler
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TimerIntStatus(TIMER0_BASE, false));
    TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer_0A_Handler);

    // Enable timer A
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    return true;
}