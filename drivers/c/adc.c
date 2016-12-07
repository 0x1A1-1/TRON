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

#include "adc.h"
#include "driver_defines.h"

bool initialize_adc_interrupt(
	ADC0_Type *adc,   // ADC base
	uint8_t seq,      // sample sequencer to use
	uint8_t chan_cnt, // number of channels
	uint32_t chan_m)  // mask specifying which ADC channels to use for each sample
{
	uint32_t rcgc_m;
	uint32_t pr_m;
	uint32_t asen_m;
	uint32_t emux_m;
	uint32_t interrupt_m;
	IRQn_Type irqn;
	
	// validate ADC base address and set RCGC and PR masks
	switch ((uint32_t)adc) {
		case ADC0_BASE :
			rcgc_m = SYSCTL_RCGCADC_R0;
			pr_m = SYSCTL_PRADC_R0;
			break;
		case ADC1_BASE :
			rcgc_m = SYSCTL_RCGCADC_R1;
			pr_m = SYSCTL_PRADC_R1;
			break;
		default :
			return false;
	}
	
	// turn on ADC clock
	SYSCTL->RCGCADC |= rcgc_m;
	while (!(SYSCTL->PRADC & pr_m));
	
	switch (seq) {
		case 0 :
			if (chan_cnt > 8) {
				SYSCTL->RCGCADC &= !rcgc_m;
				return false;
			}
			
			asen_m = ADC_ACTSS_ASEN0;
			emux_m = ADC_EMUX_EM0_M;
			interrupt_m = ADC_IM_MASK0;
			
			// set the number of samples and where to sample from
			adc->SSCTL0 = (ADC_SSCTL0_IE0 | ADC_SSCTL0_END0) << (4*chan_cnt);
			adc->SSMUX0 = chan_m;
			
			// set IRQ number
			if ((uint32_t)adc == ADC0_BASE) {
				irqn = ADC0SS0_IRQn;
			} else {
				irqn = ADC1SS0_IRQn;
			}
			
			break;
		case 1 :
			if (chan_cnt > 4) {
				SYSCTL->RCGCADC &= !rcgc_m;
				return false;
			}
			
			asen_m = ADC_ACTSS_ASEN1;
			emux_m = ADC_EMUX_EM1_M;
			interrupt_m = ADC_IM_MASK1;
			
			// set the number of samples and where to sample from
			adc->SSCTL1 = (ADC_SSCTL1_IE0 | ADC_SSCTL1_END0) << (4*chan_cnt);
			adc->SSMUX1 = chan_m;
			
			// set IRQ number
			if ((uint32_t)adc == ADC0_BASE) {
				irqn = ADC0SS1_IRQn;
			} else {
				irqn = ADC1SS1_IRQn;
			}
			
			break;
		case 2 :
			if (chan_cnt > 4) {
				SYSCTL->RCGCADC &= !rcgc_m;
				return false;
			}
			
			asen_m = ADC_ACTSS_ASEN2;
			emux_m = ADC_EMUX_EM2_M;
			interrupt_m = ADC_IM_MASK2;
			
			// set the number of samples and where to sample from
			adc->SSCTL2 = (ADC_SSCTL2_IE0 | ADC_SSCTL2_END0) << (4*chan_cnt);
			adc->SSMUX2 = chan_m;
			
			// set IRQ number
			if ((uint32_t)adc == ADC0_BASE) {
				irqn = ADC0SS2_IRQn;
			} else {
				irqn = ADC1SS2_IRQn;
			}
			
			break;
		case 3 :
			if (chan_cnt > 1) {
				SYSCTL->RCGCADC &= !rcgc_m;
				return false;
			}
			
			asen_m = ADC_ACTSS_ASEN3;
			emux_m = ADC_EMUX_EM3_M;
			interrupt_m = ADC_IM_MASK3;
			
			// set the number of samples and where to sample from
			adc->SSCTL3 = (ADC_SSCTL3_IE0 | ADC_SSCTL3_END0) << (4*chan_cnt);
			adc->SSMUX3 = chan_m;
			
			// set IRQ number
			if ((uint32_t)adc == ADC0_BASE) {
				irqn = ADC0SS3_IRQn;
			} else {
				irqn = ADC1SS3_IRQn;
			}
			
			break;
		default :
			SYSCTL->RCGCADC &= !rcgc_m;
			return false;
	}
	
	// configure sample sequencer settings
	adc->ACTSS &= ~asen_m;
	adc->EMUX &= ~emux_m;
	adc->IM |= interrupt_m;
	NVIC_EnableIRQ(irqn);
	
	return true;
}

/******************************************************************************
 * Initializes ADC to use Sample Sequencer #3, triggered by the processor,
 * no IRQs
 *****************************************************************************/
bool initialize_adc(  uint32_t adc_base )
{
  ADC0_Type  *myADC;
  uint32_t rcgc_adc_mask;
  uint32_t pr_mask;
  

  // examine the adc_base.  Verify that it is either ADC0 or ADC1
  // Set the rcgc_adc_mask and pr_mask  
  switch (adc_base) 
  {
    case ADC0_BASE :
    {
      
      // ADD CODE
      // set rcgc_adc_mask
	  rcgc_adc_mask = SYSCTL_RCGCADC_R0;
      
      // ADD CODE
      // Set pr_mask 
	  pr_mask = SYSCTL_PRADC_R0;
      
      break;
    }
    case ADC1_BASE :
    {
      // ADD CODE
      // set rcgc_adc_mask
	  rcgc_adc_mask = SYSCTL_RCGCADC_R1;
      
      // ADD CODE
      // Set pr_mask 
	  pr_mask = SYSCTL_PRADC_R1;
      
      break;
    }
    
    default:
      return false;
  }
  
  // Turn on the ADC Clock
  SYSCTL->RCGCADC |= rcgc_adc_mask;
  
  // Wait for ADCx to become ready
  while( (pr_mask & SYSCTL->PRADC) != pr_mask){}
    
  // Type Cast adc_base and set it to myADC
  myADC = (ADC0_Type *)adc_base;
  
  // ADD CODE
  // disable sample sequencer #3 by writing a 0 to the 
  // corresponding ASENn bit in the ADCACTSS register 
  myADC->ACTSS &= ~ADC_ACTSS_ASEN3;

  // ADD CODE
  // Set the event multiplexer to trigger conversion on a processor trigger
  // for sample sequencer #3.
  myADC->EMUX &= ~ADC_EMUX_EM3_M;
  myADC->EMUX |= ADC_EMUX_EM3_PROCESSOR;

  // ADD CODE
  // Set IE0 and END0 in SSCTL3
  myADC->SSCTL3 |= ADC_SSCTL3_IE0;
  myADC->SSCTL3 |= ADC_SSCTL3_END0;
  
  return true;
}

/******************************************************************************
 * Reads SSMUX3 for the given ADC.  Busy waits until completion
 *****************************************************************************/
uint32_t get_adc_value( uint32_t adc_base, uint8_t channel)
{
  ADC0_Type  *myADC;
  uint32_t result;
  
  if( adc_base == 0)
  {
    return false;
  }
  
  myADC = (ADC0_Type *)adc_base;
  
  myADC->SSMUX3 = channel;          // Set the Channel
  
  myADC->ACTSS |= ADC_ACTSS_ASEN3;  // Enable SS3
  
  myADC->PSSI =   ADC_PSSI_SS3;     // Start SS3
  
  while( (myADC->RIS & ADC_RIS_INR3)  == 0)
  {
    // wait
  }
  
  result = myADC->SSFIFO3 & 0xFFF;    // Read 12-bit data
  
  myADC->ISC  = ADC_ISC_IN3;          // Ack the conversion
  
  return result;
}

