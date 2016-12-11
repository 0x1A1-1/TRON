#include "timers.h"


//*****************************************************************************
// Verifies that the base address is a valid GPIO base address
//*****************************************************************************
static bool verify_base_addr(uint32_t base_addr)
{
   switch( base_addr )
   {
     case TIMER0_BASE:
     case TIMER1_BASE:
     case TIMER2_BASE:
     case TIMER3_BASE:
     case TIMER4_BASE:
     case TIMER5_BASE:
     {
       return true;
     }
     default:
     {
       return false;
     }
   }
}

//*****************************************************************************
// Returns the RCGC and PR masks for a given TIMER base address
//*****************************************************************************
static bool get_clock_masks(
	uint32_t base_addr,
	uint32_t *timer_rcgc_mask,
	uint32_t *timer_pr_mask,
	IRQn_Type *irqn,
	uint8_t num)
{
	// Set the timer_rcgc_mask and timer_pr_mask using the appropriate
	// #defines in ../include/sysctrl.h
	switch(base_addr)
	{
		case TIMER0_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R0;
			*timer_pr_mask = SYSCTL_PRTIMER_R0;
			*irqn = TIMER0A_IRQn;
			if (num > 1) irqn[1] = TIMER0B_IRQn;
			break;
		}
		case TIMER1_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R1;
			*timer_pr_mask = SYSCTL_PRTIMER_R1;
			*irqn = TIMER1A_IRQn;
			if (num > 1) irqn[1] = TIMER1B_IRQn;
			break;
		}
		case TIMER2_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R2;
			*timer_pr_mask = SYSCTL_PRTIMER_R2;
			*irqn = TIMER2A_IRQn;
			if (num > 1) irqn[1] = TIMER2B_IRQn;
			break;
		}
		case TIMER3_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R3;
			*timer_pr_mask = SYSCTL_PRTIMER_R3;
			*irqn = TIMER3A_IRQn;
			if (num > 1) irqn[1] = TIMER3B_IRQn;
			break;
		}
		case TIMER4_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R4;
			*timer_pr_mask = SYSCTL_PRTIMER_R4;
			*irqn = TIMER4A_IRQn;
			if (num > 1) irqn[1] = TIMER4B_IRQn;
			break;
		}
		case TIMER5_BASE:
		{
			*timer_rcgc_mask = SYSCTL_RCGCTIMER_R5;
			*timer_pr_mask = SYSCTL_PRTIMER_R5;
			*irqn = TIMER5A_IRQn;
			if (num > 1) irqn[1] = TIMER5B_IRQn;
			break;
		}
		default:
		{
			return false;
		}
	}
	return true;
}

void gp_timer_start_16(
	TIMER0_Type *timer,
	uint16_t prescaleA,
	uint16_t prescaleB,
	uint16_t ticksA,
	uint16_t ticksB)
{
	timer->TAPR = prescaleA;
	timer->TBPR = prescaleB;
	timer->TAILR = ticksA;
	timer->TBILR = ticksB;
	
	timer->CTL |= TIMER_CTL_TAEN | TIMER_CTL_TBEN;
}

//*****************************************************************************
// Waits for 'ticks' number of clock cycles and then returns.
//
//The function returns true if the base_addr is a valid general purpose timer
//*****************************************************************************
bool gp_timer_wait(uint32_t base_addr, uint32_t ticks)
{
  TIMER0_Type *gp_timer;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }

  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;

  //*********************    
  // ADD CODE
  //*********************
	gp_timer->CTL &= (~TIMER_CTL_TAEN)|(~TIMER_CTL_TBEN);
	gp_timer->TAILR = ticks;
	gp_timer->IMR &= ~TIMER_ICR_TATOCINT;
	gp_timer->CTL |= TIMER_CTL_TAEN;
	while (!(gp_timer->RIS & TIMER_RIS_TATORIS));
  
  return true;
}

bool gp_timer_config_16(
	TIMER0_Type *timer,     // timer base address
	uint32_t mode,          // one-shot, periodic, or capture
	bool count_up,          // count up or down
	bool enable_interrupts) // use interrupts or not
{
	uint32_t rcgc_m;
	uint32_t pr_m;
	IRQn_Type irqn[2];
	
	// verify base address
	if (!verify_base_addr((uint32_t)timer)) return false;
	
	// turn on timer clock
	get_clock_masks((uint32_t)timer, &rcgc_m, &pr_m, irqn, 2);
	SYSCTL->RCGCTIMER |= rcgc_m;
	while(!(SYSCTL->PRTIMER & pr_m));
	
	timer->CTL &= ~(TIMER_CTL_TAEN | TIMER_CTL_TBEN);
	
	timer->CFG = TIMER_CFG_16_BIT;
	timer->TAMR |= mode;
	timer->TBMR |= mode;
	
	// configure count direction
	if (count_up) {
		timer->TAMR |= TIMER_TAMR_TACDIR;
		timer->TBMR |= TIMER_TBMR_TBCDIR;
	} else {
		timer->TAMR &= ~TIMER_TAMR_TACDIR;
		timer->TBMR &= ~TIMER_TBMR_TBCDIR;
	}
	
	// configure interrupts
	if (enable_interrupts) {
		timer->IMR = TIMER_IMR_TATOIM | TIMER_IMR_TBTOIM;
		NVIC_EnableIRQ(irqn[0]);
		NVIC_EnableIRQ(irqn[1]);
	} else {
		timer->IMR &= ~(TIMER_IMR_TATOIM | TIMER_IMR_TBTOIM);
	}
	
	return true;
}

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
bool gp_timer_config_32(uint32_t base_addr, uint32_t mode, bool count_up, bool enable_interrupts)
{
  uint32_t timer_rcgc_mask;
  uint32_t timer_pr_mask;
  TIMER0_Type *gp_timer;
	IRQn_Type irqn;
  
  // Verify the base address.
  if ( ! verify_base_addr(base_addr) )
  {
    return false;
  }
  
  // get the correct RCGC and PR masks for the base address
  get_clock_masks(base_addr, &timer_rcgc_mask, &timer_pr_mask, &irqn, 1);
  
  // Turn on the clock for the timer
  SYSCTL->RCGCTIMER |= timer_rcgc_mask;

  // Wait for the timer to turn on
  while( (SYSCTL->PRTIMER & timer_pr_mask) == 0) {};
  
  // Type cast the base address to a TIMER0_Type struct
  gp_timer = (TIMER0_Type *)base_addr;
    
  //*********************    
  // ADD CODE
  //*********************
	gp_timer->CTL &= (~TIMER_CTL_TAEN)|(~TIMER_CTL_TBEN);
	gp_timer->CFG = TIMER_CFG_32_BIT_TIMER;
	gp_timer->TAMR &= ~TIMER_TAMR_TAMR_M;
	gp_timer->TAMR |= mode;
	if (count_up) {
		gp_timer->TAMR |= TIMER_TAMR_TACDIR;
	} else {
		gp_timer->TAMR &= ~TIMER_TAMR_TACDIR;
	}
	if (enable_interrupts) {
		gp_timer->IMR |= TIMER_IMR_TATOIM;
		NVIC_EnableIRQ(irqn);
	} else {
		gp_timer->IMR &= ~TIMER_IMR_TATOIM;
	}
    
  return true;  
}

bool watchdog_timer_config(
	WATCHDOG0_Type *wd_timer,
	uint32_t ticks,
	bool reset,
	bool interrupt,
	bool int_mask)
{
	if (wd_timer != WATCHDOG0 && wd_timer != WATCHDOG1) return false;
	
	// turn on watchdog clock
	SYSCTL->RCGCWD |= SYSCTL_RCGC0_WDT0;
	while (!SYSCTL->PRWD & SYSCTL_PRWD_R0);
	
	if (reset) {
		wd_timer->CTL |= WATCHDOG_RESET_ON_INTERRUPT;
	} else {
		wd_timer->CTL &= ~WATCHDOG_RESET_ON_INTERRUPT;
	}
	if (interrupt) {
		wd_timer->CTL |= WATCHDOG_INTERRUPT_ENABLE;
	} else {
		wd_timer->CTL &= ~WATCHDOG_INTERRUPT_ENABLE;
	}
	if (int_mask) {
		wd_timer->CTL |= WATCHDOG_MASKED_INTERRUPT;
	} else {
		wd_timer->CTL &= ~WATCHDOG_MASKED_INTERRUPT;
	}
	
	wd_timer->LOAD = ticks;
	
	return true;
}
