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
#include "eeprom.h"
#include "serial_debug.h"
#include "launchpad_io.h"
#include "ps2.h"
#include "lcd.h"
#include "ft6x06.h"
#include "io_expander.h"
#include "wireless.h"

struct game_info {
	uint64_t boosts_type0;
	uint64_t boosts_type1;
	uint64_t boosts_type2;
	uint64_t boosts_type3;
	uint64_t distance;
	uint64_t turns;
};

struct ECE353_info {
	char group[32];
	char individual_1[32];
	char individual_2[32];
	struct game_info last_game;
	struct game_info lifetime;
};

struct ECE353_info info = {
	"Group25",
	"Zuodian Hu",
	"Xiao He",
	{0,0,0,0,0,0},
	{0,0,0,0,0,0}
};

//two modes
typedef enum
{
	MOV_UP,
	MOV_DOWN,
	MOV_LEFT,
	MOV_RIGHT
} MODES;

static MODES mode_state = MOV_UP;
static MODES player2_mode_state = MOV_DOWN;
static MODES player2_previous_state = MOV_DOWN;

uint8_t dest_id[] = {0,1,8,9,6};
uint8_t my_id[] = {3,1,0,9,5};

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
	
	// feed the dog
	WATCHDOG0->LOAD = 750000000;
	
	// clear interrupt
	GPIOD->ICR = PIN_3;
}

//*****************************************************************************
//*****************************************************************************
void initialize_hardware(void)
{
	// initialize serial debugging
	init_serial_debug(false, false);
	
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
uint32_t bitmap[240][10]={0};

void game_over_check(uint16_t lcd_x, uint16_t lcd_y){

  uint8_t i;

	if ( (lcd_y >= 299 && mode_state==MOV_UP )
					|| (lcd_y <= 1 && mode_state==MOV_DOWN )
					|| (lcd_x >= 219 && mode_state==MOV_LEFT )
					|| (lcd_x <= 1 && mode_state==MOV_RIGHT )				){
					while(1){}}



	if(mode_state==MOV_UP)
  {
    for(i=0; i<10; i++)
    {
       if(bitmap[lcd_x+i][(int)(lcd_y+19)/32] & (1 << ((lcd_y+19)%32)))
       {
         while(1){}
       }
    }
  }
	else if ( mode_state==MOV_DOWN){
    for(i=0; i<10; i++)
    {
       if(bitmap[lcd_x+i][(int)lcd_y/32] & (1 << (lcd_y%32)))
       {
         while(1){}
       }
    }
  }
	else if (mode_state==MOV_LEFT){
    for(i=0; i<10; i++)
    {
       if(bitmap[lcd_x+19][(int)(lcd_y+i)/32] & (1 << ((lcd_y+i)%32)))
       {
         while(1){}
       }
    }
  }
	else
  {
    for(i=0; i<10; i++)
    {
       if(bitmap[lcd_x][(int)(lcd_y+i)/32] & (1 << ((lcd_y+i)%32)))
       {
         while(1){}
       }
    }
  }

}

int
main(void)
{
	i2c_status_t status;
	int i;
	bool start=true;
	uint8_t mov;
	uint16_t prevPix, x_data, y_data, lcd_x = 120, lcd_y = 50, player2_lcd_x = 120,player2_lcd_y = 250 ;
	uint8_t test[60];
	
	uint8_t left = 0,
				right = 0,
				up = 0,
				down = 0;
	
	initialize_hardware();
	
	eeprom_seq_write(EEPROM_I2C_BASE, 0, (uint8_t *)&info, sizeof(info));
	//for (i=0; i<sizeof(info); i++) {
	//	eeprom_byte_read(EEPROM_I2C_BASE, i, &((uint8_t *)(&info))[i]);
	//}
	//for (i=0; i<60; i++) {
	//	test[i] = i;
	//}
	//eeprom_seq_write(EEPROM_I2C_BASE, 512, test, 32);
	//for (i=0; i<60; i++) {
	//	eeprom_byte_read(EEPROM_I2C_BASE, 512+i, &test[i]);
	//}
	
	//eeprom_byte_read(EEPROM_I2C_BASE, 512, test);
	eeprom_seq_read(EEPROM_I2C_BASE, 0, (uint8_t *)(&info), sizeof(info));
	
	printf("\n\nTRON\n");
	printf("%s\n", info.group);
	printf("%s\n%s\n\n", info.individual_1, info.individual_2);
	
	set_leds(0xAA);
	
	// Reach infinite loop
	while(1){
		// update screen
		if (redraw) {
			
			mov = analog_conversion(x_pos, y_pos);
			if(start)
			{
				mov = 0x12;
				start = false;
				up=1;
				lcd_x-=10;
			}
			
			if (mov&0x20 && lcd_x<220){
					left = 1;
					right = 0;
					up = 0;
					down = 0;
				}else if (mov&0x8 && lcd_x>0){
					left = 0;
					right = 1;
					up = 0;
					down = 0;
				}
				else if (mov&0x4 && lcd_y<300){
					left = 0;
					right = 0;
					up = 1;
					down = 0;
				}else if (mov&0x1 && lcd_y>0){
					left = 0;
					right = 0;
					up = 0;
					down = 1;
				}

			 game_over_check(lcd_x, lcd_y);
			 if(left==1)
				 {
				   if(mode_state==MOV_UP){
								lcd_draw_image(
										lcd_x,                 // X Pos
										imageWidthPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageHeightPixels,  // Image Vertical Height
										ver_trail_for_up,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);

								for(i=0; i<16; i++)
								{
									bitmap[lcd_x+4][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
									bitmap[lcd_x+5][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
								}

								lcd_x += 5;
								lcd_y += 10;
					 }
					 if (mode_state==MOV_DOWN){
						lcd_draw_image(
										lcd_x,                 // X Pos
										imageWidthPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageHeightPixels,  // Image Vertical Height
										ver_trail_for_down,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
						   	for(i=4; i<20; i++)
								{
									bitmap[lcd_x+4][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
									bitmap[lcd_x+5][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
								}

							 lcd_x += 5;
						}
					//x_max=0;
          if (lcd_x>220)
          {
            lcd_x=220;
          }

					if (mode_state != MOV_RIGHT){
						mode_state = MOV_LEFT;
						player2_mode_state = MOV_RIGHT;
						//store info
						bitmap[lcd_x][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
						bitmap[lcd_x][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);

						lcd_x++;
						lcd_draw_image(
									lcd_x,                 // X Pos
									imageHeightPixels,   // Image Horizontal Width
									lcd_y,                 // Y Pos
									imageWidthPixels,  // Image Vertical Height
									tron_left,       // Image
									LCD_COLOR_BLUE2,      // Foreground Color
									LCD_COLOR_BLACK     // Background Color
								);
					}else{
						right = 1;
						left =0;
						lcd_x--;
						lcd_draw_image(
									lcd_x,                 // X Pos
									imageHeightPixels,   // Image Horizontal Width
									lcd_y,                 // Y Pos
									imageWidthPixels,  // Image Vertical Height
									tron_right,       // Image
									LCD_COLOR_BLUE2,      // Foreground Color
									LCD_COLOR_BLACK     // Background Color
								);
					}

			 }
			 else if (right==1)
			 {
				  if(mode_state==MOV_UP){
								lcd_draw_image(
										lcd_x,                 // X Pos
										imageWidthPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageHeightPixels,  // Image Vertical Height
										ver_trail_for_up,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);

								for(i=0; i<16; i++)
								{
									bitmap[lcd_x+4][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
									bitmap[lcd_x+5][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
								}

								lcd_x -= 15;
								lcd_y += 10;
					 }
					 if (mode_state==MOV_DOWN){
						lcd_draw_image(
										lcd_x,                 // X Pos
										imageWidthPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageHeightPixels,  // Image Vertical Height
										ver_trail_for_down,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
								for(i=4; i<20; i++)
								{
									bitmap[lcd_x+4][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
									bitmap[lcd_x+5][(int)(lcd_y+i)/32] |= 1 << ((lcd_y+i)%32);
								}

							 lcd_x -= 15;
						}
            if (lcd_x>=240)
            {
              lcd_x=1;
            }

					if (mode_state != MOV_LEFT){
					 mode_state = MOV_RIGHT;
					 player2_mode_state = MOV_LEFT;

					 bitmap[lcd_x+9][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
					 bitmap[lcd_x+9][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);

					 lcd_x--;
					 lcd_draw_image(
									lcd_x,                 // X Pos
									imageHeightPixels,   // Image Horizontal Width
									lcd_y,                 // Y Pos
									imageWidthPixels,  // Image Vertical Height
									tron_right,       // Image
									LCD_COLOR_BLUE2,      // Foreground Color
									LCD_COLOR_BLACK     // Background Color
								);
					}else{
						left = 1 ;
						right = 0;
						lcd_x++;
						lcd_draw_image(
									lcd_x,                 // X Pos
									imageHeightPixels,   // Image Horizontal Width
									lcd_y,                 // Y Pos
									imageWidthPixels,  // Image Vertical Height
									tron_left,       // Image
									LCD_COLOR_BLUE2,      // Foreground Color
									LCD_COLOR_BLACK     // Background Color
								);
					}

			 }
			 else if (up==1)
			 {
					 if(mode_state==MOV_LEFT){
								lcd_draw_image(
										lcd_x,                 // X Pos
										imageHeightPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageWidthPixels,  // Image Vertical Height
										hor_trail_for_left,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);

								for(i=0; i<16; i++)
								{
									bitmap[lcd_x+i][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
									bitmap[lcd_x+i][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);
								}

								lcd_y += 5;
								lcd_x += 10;
					 }
					 if (mode_state==MOV_RIGHT){
						lcd_draw_image(
										lcd_x,                 // X Pos
										imageHeightPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageWidthPixels,  // Image Vertical Height
										hor_trail_for_right,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
								for(i=4; i<20; i++)
								{
									bitmap[lcd_x+i][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
									bitmap[lcd_x+i][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);
								}

							 lcd_y += 5;
						}
            if (lcd_y>300)
            {
              lcd_x=300;
            }


					 if (mode_state != MOV_DOWN){
						 mode_state = MOV_UP;
						 player2_mode_state = MOV_DOWN;

						bitmap[lcd_x+4][(int)lcd_y/32] |= 1 << (lcd_y%32);
						bitmap[lcd_x+5][(int)lcd_y/32] |= 1 << (lcd_y%32);

						 lcd_y++;
						  lcd_draw_image(
										lcd_x,                 // X Pos
										imageWidthPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageHeightPixels,  // Image Vertical Height
										tron_up,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
						}else{
							down = 1;
							up= 0;
							lcd_y--;
							lcd_draw_image(
												lcd_x,                 // X Pos
												imageWidthPixels,   // Image Horizontal Width
												lcd_y,                 // Y Pos
												imageHeightPixels,  // Image Vertical Height
												tron_down,       // Image
												LCD_COLOR_BLUE2,      // Foreground Color
												LCD_COLOR_BLACK     // Background Color
											);
						}

			 }
			 else if (down==1)
					{
						if (mode_state == MOV_LEFT){
							lcd_draw_image(
										lcd_x,                 // X Pos
										imageHeightPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageWidthPixels,  // Image Vertical Height
										hor_trail_for_left,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
								for(i=0; i<16; i++)
								{
									bitmap[lcd_x+i][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
									bitmap[lcd_x+i][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);
								}
								lcd_y -= 15;
								lcd_x += 10;
							}
						if (mode_state==MOV_RIGHT){
						lcd_draw_image(
										lcd_x,                 // X Pos
										imageHeightPixels,   // Image Horizontal Width
										lcd_y,                 // Y Pos
										imageWidthPixels,  // Image Vertical Height
										hor_trail_for_right,       // Image
										LCD_COLOR_BLUE2,      // Foreground Color
										LCD_COLOR_BLACK     // Background Color
									);
								for(i=4; i<20; i++)
								{
									bitmap[lcd_x+i][(int)(lcd_y+4)/32] |= 1 << ((lcd_y+4)%32);
									bitmap[lcd_x+i][(int)(lcd_y+5)/32] |= 1 << ((lcd_y+5)%32);
								}
							 lcd_y -= 15;
						}

            if (lcd_y>320)
            {
              lcd_y=1;
            }

						 if (mode_state != MOV_UP){
							mode_state = MOV_DOWN;
							 player2_mode_state = MOV_UP;


							bitmap[lcd_x+4][(int)(lcd_y+19)/32] |= 1 << ((lcd_y+19)%32);
							bitmap[lcd_x+5][(int)(lcd_y+19)/32] |= 1 << ((lcd_y+19)%32);

						  lcd_y--;
							lcd_draw_image(
												lcd_x,                 // X Pos
												imageWidthPixels,   // Image Horizontal Width
												lcd_y,                 // Y Pos
												imageHeightPixels,  // Image Vertical Height
												tron_down,       // Image
												LCD_COLOR_BLUE2,      // Foreground Color
												LCD_COLOR_BLACK     // Background Color
											);
						}else{
							down = 0;
							up= 1;
							lcd_y++;
							lcd_draw_image(
												lcd_x,                 // X Pos
												imageWidthPixels,   // Image Horizontal Width
												lcd_y,                 // Y Pos
												imageHeightPixels,  // Image Vertical Height
												tron_up,       // Image
												LCD_COLOR_BLUE2,      // Foreground Color
												LCD_COLOR_BLACK     // Background Color
											);
						}
					}
					
					redraw = false;
		}
	};
}
