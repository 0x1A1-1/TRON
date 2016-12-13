// Copyright (c) 2015-16, Joe Krachey
// All rights reserved.
//
// Redistribution and use in source or binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in source form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in 
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "main.h"
#include "serial_debug.h"
#include "launchpad_io.h"
#include "ps2.h"
#include "lcd.h"
#include "ft6x06.h"
#include "io_expander.h"
#include "wireless.h"

char group[] = "Group25";
char individual_1[] = "Zuodian Hu";
char individual_2[] = "Xiao He";

uint8_t my_id[] = {0,0,0,0,0};
uint8_t dest_id[] = {0,0,0,0,0};

volatile uint16_t x_pos;
volatile uint16_t y_pos;
volatile bool redraw = false;
volatile unsigned int pkts_sent = 0;
volatile unsigned int pkts_rcvd = 0;
volatile unsigned int pkts_drpd = 0;

volatile uint32_t self_position;
volatile uint32_t remote_position;

ADC0_Type *adc = (ADC0_Type *) PS2_ADC_BASE;
TIMER0_Type *timer0 = (TIMER0_Type *)TIMER0_BASE;
TIMER0_Type *timer1 = (TIMER0_Type *)TIMER1_BASE;
WATCHDOG0_Type *wdtimer = (WATCHDOG0_Type *)WATCHDOG0_BASE;

void ADC0SS2_Handler(void) {
	y_pos = adc->SSFIFO2 & ADC_SSFIFO2_DATA_M;
	x_pos = adc->SSFIFO2 & ADC_SSFIFO2_DATA_M;
	
	adc->ISC |= ADC_ISC_IN2;
}

void TIMER0A_Handler(void) {
	// kick off an ADC read
	adc->ACTSS |= ADC_ACTSS_ASEN2;
	adc->PSSI |= ADC_PSSI_SS2;
	
	timer0->ICR |= TIMER_ICR_TATOCINT;
}

void TIMER0B_Handler(void) {
	wireless_com_status_t status;
	static bool send;
	
	// on every interrupt, alternate between two operations
	if (send) {
		// transmit current self position
		status = wireless_send_32(false, false, self_position);
		if (status == NRF24L01_TX_SUCCESS) {
			pkts_sent++;
		} else if (status == NRF24L01_TX_PCK_LOST) {
			pkts_drpd++;
		}
		
		send = false;
	} else {
		// signal the main loop to redraw the frame
		redraw = true;
		
		send = true;
	}
	
	timer0->ICR |= TIMER_ICR_TBTOCINT;
}

void TIMER1A_Handler(void) {
	printf("Packets Sent:     %d\n", pkts_sent);
	printf("Packets Received: %d\n", pkts_rcvd);
	printf("Packets Dropped:  %d\n\n", pkts_drpd);
	
	timer1->ICR |= TIMER_ICR_TATOCINT;
}

// WatchDog timer handler
void WDT0_Handler(void) {
	wdtimer->ICR = 0xFFFFFFFF;
	
	NVIC_DisableIRQ(TIMER0A_IRQn);
	NVIC_DisableIRQ(TIMER0B_IRQn);
	NVIC_DisableIRQ(TIMER1A_IRQn);
	NVIC_DisableIRQ(ADC0SS2_IRQn);
	
	printf("\nWatchDog Triggered\n");
	
	while(1);
}

// radio receive interrupt handler
void GPIOD_Handler(void) {
	// increment received packets count
	pkts_rcvd++;
	
	// store received data
	wireless_get_32(false, (uint32_t *)&remote_position);
	
	// clear interrupt
	GPIOD->ICR = PIN_3;
}

//*****************************************************************************
//*****************************************************************************
void initialize_hardware(void)
{
	// initialize serial debugging
	init_serial_debug(true, true);
	
	// push buttons and RGB LED on LaunchPad
	lp_io_init();
	
	// configure joystick with interrupts
	init_ps2_interrupt(2);
	
	// initialize the LCD to all black
	lcd_config_gpio();
	lcd_config_screen();
	lcd_clear_screen(LCD_COLOR_BLACK);
	
	// I2C
	ft6x06_init();
	init_io_expander();
	
	// SPI radio
	// set up for interrupt on receive
	wireless_initialize();
	wireless_configure_device(
		my_id,
		dest_id,
		true,
		false,
		false
	);
	
	// initialize timers
	// initialize the timers last, since this function also starts the timers
	gp_timer_config_16(
		timer0,
		TIMER_TAMR_TAMR_PERIOD,
		false,
		true
	);
	gp_timer_config_32(
		(uint32_t)timer1,
		TIMER_TAMR_TAMR_PERIOD,
		false,
		true
	);
	timer1->TAILR = 150000000;
	timer1->CTL |= TIMER_CTL_TAEN;
	gp_timer_start_16(
		timer0,
		1,
		10,
		50000,
		50000
	);
	
	// initialize and set off the watchdog timer for 15 seconds
	watchdog_timer_config(WATCHDOG0, 750000000, false, true, false);
}

//*****************************************************************************
//*****************************************************************************
int
main(void)
{
	initialize_hardware();
	
	printf("TRON\n");
	printf("%s\n", group);
	printf("%s\n%s\n\n", individual_1, individual_2);
	
	set_leds(0xAA);
	
	// Reach infinite loop
	while(1){
		// update screen
		if (redraw) {
			redraw = false;
		}
	};
}
