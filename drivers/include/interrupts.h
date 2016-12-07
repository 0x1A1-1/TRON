#ifndef __ECE353_INTERRUPTS_H__
#define __ECE353_INTERRUPTS_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123.h"

#define DisableInterrupts() __asm("CPSID  I")
#define EnableInterrupts() __asm("CPSIE  I")

#endif
