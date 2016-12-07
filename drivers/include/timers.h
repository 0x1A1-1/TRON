#ifndef __TIMERS_H__
#define __TIMERS_H__

#include <stdbool.h>
#include <stdint.h>

#include "driver_defines.h"

//*****************************************************************************
// Set off a dual 16-bit timer
//
// The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
void gp_timer_start_16(
	TIMER0_Type *timer,
	uint16_t prescaleA,
	uint16_t prescaleB,
	uint16_t ticksA,
	uint16_t ticksB
);

//*****************************************************************************
// Configure a general purpose timer to be two 16-bit timers.  
//
// Paramters
//  timer                 pointer to a general purpose timer
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  count_up              When true, the timer counts up.  When false, it counts
//                        down
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_16(
	TIMER0_Type *timer,    // timer base address
	uint32_t mode,         // one-shot, periodic, or capture
	bool count_up,         // count up or down
	bool enable_interrupts // use interrupts or not
);

//*****************************************************************************
// Configure a general purpose timer to be a 32-bit timer.  
//
// Paramters
//  base_address          The base address of a general purpose timer
//
//  mode                  bit mask for Periodic, One-Shot, or Capture
//
//  count_up              When true, the timer counts up.  When false, it counts
//                        down
//
//  enable_interrupts     When set to true, the timer generates and interrupt
//                        when the timer expires.  When set to false, the timer
//                        does not generate interrupts.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_config_32(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts);


//*****************************************************************************
// Waits for 'ticks' number of clock cycles and then returns.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_wait(uint32_t base_addr, uint32_t ticks);


#endif
