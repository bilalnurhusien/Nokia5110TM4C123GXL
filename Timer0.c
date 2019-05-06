#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>

#include "inc/Timer0.h"

static void (*pTimer0AUserIntHandler)(void);

static void Timer_0A_Handler(void){
    //
    // Acknowledge timer0A timeout
    //
    TimerIntClear(TIMER0_BASE, TimerIntStatus(TIMER0_BASE, false));
    pTimer0AUserIntHandler();
}

static void (*pTimer0BUserIntHandler)(void);

static void Timer_0B_Handler(void){
    //
    // Acknowledge timer0B timeout
    //
    TimerIntClear(TIMER0_BASE, TimerIntStatus(TIMER0_BASE, false));
    pTimer0BUserIntHandler();
}

bool Timer_0_Init(void (*pfnAHandler)(void), void (*pfnBHandler)(void), uint32_t startValueA, uint32_t startValueB)
{
    //
    // For more information refer to Chpt 11.4 of Initialization and Configuration from
    // TM4C123GH6PM datasheet or Chpt 29 of TivaWare Peripheral Driver Library Datasheet
    //
  
    if (NULL == pfnAHandler || NULL == pfnBHandler)
    {
        return false;
    }
    
    pTimer0AUserIntHandler = pfnAHandler;
    pTimer0BUserIntHandler = pfnBHandler;
       
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
    // Configure timer A and B for periodic mode
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);

    // Load start value
    TimerLoadSet(TIMER0_BASE, TIMER_A, startValueA-1);
    TimerLoadSet(TIMER0_BASE, TIMER_B, startValueB-1);

    // Enable interrupt for timer A and B and register handlers
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer_0A_Handler);
    TimerIntRegister(TIMER0_BASE, TIMER_B, &Timer_0B_Handler);

    // Enable timer A and timer B
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    
    return true;
}