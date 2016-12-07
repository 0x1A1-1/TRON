// Copyright (c) 2014-16, Joe Krachey
// All rights reserved.
//
// Redistribution and use in binary form, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions in binary form must reproduce the above copyright 
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
#ifndef __PS2_H__
#define __PS2_H__

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "driver_defines.h"
#include "gpio_port.h"
#include "timers.h"

// ADD CODE
// Define the base addresses for the GPIO port that is connected to the PS2 Joystick
#define   PS2_GPIO_BASE    GPIOE_BASE

// ADD CODE
// Define which bit number ( 0 through 7) each signal is connected to on the GPIO port.
// This bit number is NOT the value listed in the Pin Number column of the datasheet.  
// As an example, if the analog input was connected to pin PD2, you would set the bit
// number to 2.
#define   PS2_X_BIT_NUM      3
#define   PS2_Y_BIT_NUM      2

// Macros passed to the gpio functions that are used to configure the GPIO port.
#define   PS2_X_DIR_MASK     (1 << PS2_X_BIT_NUM)
#define   PS2_Y_DIR_MASK     (1 << PS2_Y_BIT_NUM)

// ADD CODE
// Define the base addresses of the the ADC you are going to use to either ADC0 or ADC1.
// Either will work.
#define   PS2_ADC_BASE     ADC0_BASE

// ADD CODE
// Set the analog channel for each direction.  Each analog channel is associated with a
// single GPIO pin.  Make sure to see table 13-1 to see how the GPIO pin maps to an 
// analog channel.  Specifically, look at the column Pin Name.
#define   PS2_X_ADC_CHANNEL  0
#define   PS2_Y_ADC_CHANNEL  1

#define PS2_MAX 0x0fff;
#define PS2_ZERO 0x07ff;

/*******************************************************************************
* Function Name: init_ps2_interrupt
********************************************************************************
* Initializes the GPIO pins connected to the PS2 Joystick.  It also configures
* ADC0 to use the specified Sample Sequencer to generate an interrupt when both
* X and Y positions on the joystick have been read
* Parameters
*   seq_num
*     the sample sequencer to use for the conversion
* Returns:
*   true
*     the sequencer number is valid
*   false
*     the sequencer 3 is selected since it has only a single FIFO entry
*******************************************************************************/
bool init_ps2_interrupt(uint8_t seq_num);

/*******************************************************************************
* Function Name: ps2_initialize
********************************************************************************
* Initializes the GPIO pins connected to the PS2 Joystick.  It also configures
* ADC0 to use Sample Sequencer #3 to convert a programmable channel number.
*******************************************************************************/
void ps2_initialize(void);

/*******************************************************************************
* Function Name: ps2_get_x
********************************************************************************
*Returns the most current reading of the X direction  Only the lower 12-bits
* contain data.
********************************************************************************/
uint16_t ps2_get_x(void);

/*******************************************************************************
* Function Name: ps2_get_y
********************************************************************************
* Returns the most current reading of the Y direction.  Only the lower 12-bits
*  contain data.
********************************************************************************/
uint16_t ps2_get_y(void);

#endif

