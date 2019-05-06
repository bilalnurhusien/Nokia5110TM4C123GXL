#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>

#include "inc/Timers.h"

static void (*pTimer0UserIntHandler)(void);

static void Timer_0_Handler(void){
    //
    // Acknowledge timer01 timeout
    //
    TimerIntClear(TIMER0_BASE, TimerIntStatus(TIMER0_BASE, false));
    pTimer0UserIntHandler();
}

static void (*pTimer1UserIntHandler)(void);

static void Timer_1_Handler(void){
    //
    // Acknowledge timer0B timeout
    //
    TimerIntClear(TIMER1_BASE, TimerIntStatus(TIMER1_BASE, false));
    pTimer1UserIntHandler();
}

bool Timers_Init(void (*pfn0Handler)(void), void (*pfn1Handler)(void), uint32_t startValue0, uint32_t startValue1)
{
    //
    // For more information refer to Chpt 11.4 of Initialization and Configuration from
    // TM4C123GH6PM datasheet or Chpt 29 of TivaWare Peripheral Driver Library Datasheet
    //
  
    if (NULL == pfn0Handler || NULL == pfn1Handler)
    {
        return false;
    }
    
    pTimer0UserIntHandler = pfn0Handler;
    pTimer1UserIntHandler = pfn1Handler;
       
    //
    // Enable the Timer0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
 
    //
    // Wait for the Timer1 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }
	
   //
    // Enable the Timer0 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
 
    //
    // Wait for the Timer1 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
    {
    }

    //
    // Configure timer 0 and timer 2 for periodic mode
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);
	
    // Load start value
    TimerLoadSet(TIMER0_BASE, TIMER_A, startValue0-1);
    TimerLoadSet(TIMER1_BASE, TIMER_A, startValue1-1);

    // Enable interrupt for timer 0 and timer 1 and register handlers
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer_0_Handler);
    TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer_1_Handler);

    // Enable timer 0 and timer 1
    TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
    
    return true;
}