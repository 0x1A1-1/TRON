// Copyright (c) 2014, Joe Krachey
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

#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"
#include "driver_defines.h"

/******************************************************************************
 * Initializes ADC to use the specified Sample Sequencer, triggered by a
 * processor event. The ADC generates an interrupt when finished. 
 *
 * Parameters:
 *  adc        The user must pass ADC0_BASE or ADC1_BASE.
 *  seq        The number of the Sample Sequencer to use
 *  chan_cnt   The number of channels to enable in the sequencer
 *  chan_m     Bit mask to select the channels to sample
 *
 * Returns:
 *  true  - if an valid base address was passed in
 *  false - if an invalid base address was passed in
 *****************************************************************************/
bool initialize_adc_interrupt(
	ADC0_Type *adc,   // ADC base
	uint8_t seq,      // sample sequencer to use
	uint8_t chan_cnt, // number of channels
	uint32_t chan_m); // mask specifying which ADC channels to use for each sample

/******************************************************************************
 * Initializes ADC to use Sample Sequencer #3, triggered by software, no IRQs
 *
 * Parameters:
 *  adc_base - The user must pass ADC0_BASE or ADC1_BASE.
 *
 * Returns:
 *  true  - if an valid base address was passed in
 *  false - if an invalid base address was passed in
 *****************************************************************************/
bool initialize_adc(  uint32_t adc_base );


/******************************************************************************
 * Returns the analog reading for a given AIN channel.
 *
 * Parameters:
 *  adc_base - The user must pass ADC0_BASE or ADC1_BASE.
 *  channel  - Which channel to convert
 *
 * Returns:
 *   analog reading
 *****************************************************************************/
uint32_t get_adc_value( uint32_t adc_base, uint8_t channel);

/******************************************************************************
 * Returns the decision of moving dots
 *
 * Returns:
 *   position variable
 *****************************************************************************/
uint8_t analog_conversion(uint16_t ps2_x, uint16_t ps2_y);

#endif
