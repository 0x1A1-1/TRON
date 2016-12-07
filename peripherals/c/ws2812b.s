; Copyright (c) 2016, Joe Krachey
; All rights reserved.
;
; Redistribution and use in binary form, with or without modification, 
; are permitted provided that the following conditions are met:
;
; 1. Redistributions in binary form must reproduce the above copyright 
;   notice, this list of conditions and the following disclaimer in 
;    the documentation and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
; THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
; PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
; NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
; EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ONE     EQU         0x80
ZERO    EQU         0x00
    

    export WS2812B_write

GPIO_ADDR        RN R0
LED_ARRAY_ADDR   RN R1
NUM_LEDS         RN R2
COLOR_DATA       RN R3
BYTE_COUNT       RN R4
BIT_COUNT        RN R5
DATA_OUT         RN R6

;**********************************************
; Constant Variables (FLASH) Segment
;**********************************************
    AREA    FLASH, CODE, READONLY
    align

;******************************************************************************** 
; Parameters
; R0 - GPIO Port Address 
; R1 - LED Array Address 
; R2 - Number of LEDs 
;********************************************************************************        
WS2812B_write   PROC
    ; R0 - GPIO Port Address 
    ; R1 - LED Array Address 
    ; R2 - Number of LEDs 
    ; R3 - 24 bit count 

    ; Registers to save to the stack 
    ; R4 - Value to write to GPIO Port 
    ; R5 - Index of next LED 
    ; R6 - Offset into byte array for next data 
    PUSH    {R4-R6}
    
    ; Check to see if the last LED has been sent.
    CMP     NUM_LEDS, #0
    
WS2812B_write_loop_start


    BEQ     WS2812B_write_return
    MOV     BYTE_COUNT, #3

WS2812B_write_get_byte_start  

    ; Load the next LED data 
    LDRB    COLOR_DATA, [LED_ARRAY_ADDR], #1  ; 2 CLK / Total: 02 
    MOV     BIT_COUNT,  #8
    MOV     DATA_OUT, #ONE
    
WS2812B_write_set_pin_start
    ;************************************************
    ; Output a 1 for 20 CLK cycles                   
    ;************************************************
    STR     DATA_OUT, [GPIO_ADDR]                        ; 02 CLK                       
    TST     COLOR_DATA, #ONE                             ; 03 CLK   
    MOVEQ   DATA_OUT, #ZERO                              ; 04 CLK
    MOVNE   DATA_OUT, #ONE                               ; 05 CLK
    LSL     COLOR_DATA, COLOR_DATA, #1                   ; 06 CLK
    NOP                                                  ; 07 CLK
    NOP                                                  ; 08 CLK                                                 
    NOP                                                  ; 09 CLK
    NOP                                                  ; 10 CLK
    NOP                                                  ; 11 CLK
    NOP                                                  ; 12 CLK                                                 
    NOP                                                  ; 13 CLK
    NOP                                                  ; 14 CLK
    NOP                                                  ; 15 CLK
    NOP                                                  ; 16 CLK                                                 
    NOP                                                  ; 17 CLK
    NOP                                                  ; 18 CLK
    ;NOP                                                  ; 19 CLK
    ;NOP                                                  ; 20 CLK                                                 

    ;************************************************
    ; Output data for 20 CLK cycles                   
    ;************************************************
    
    ; Write out the data to the GPIO PORT 
    STR     DATA_OUT, [GPIO_ADDR]                        ; 2 CLK / Total 20 
    MOV     DATA_OUT, #ZERO                              ; 01 CLK
    NOP                                                  ; 02 CLK
    NOP                                                  ; 03 CLK                                                 
    NOP                                                  ; 04 CLK
    NOP                                                  ; 05 CLK
    NOP                                                  ; 06 CLK    
    NOP                                                  ; 07 CLK
    NOP                                                  ; 08 CLK                                                 
    NOP                                                  ; 09 CLK
    NOP                                                  ; 10 CLK
    NOP                                                  ; 11 CLK
    NOP                                                  ; 12 CLK                                                 
    NOP                                                  ; 13 CLK
    NOP                                                  ; 14 CLK
    NOP                                                  ; 15 CLK
    NOP                                                  ; 16 CLK                                                 
    NOP                                                  ; 17 CLK
    NOP                                                  ; 18 CLK
    NOP                                                  ; 19 CLK
    NOP                                                  ; 20 CLK
    
    ;************************************************
    ; Output a 0 for 22 CLK cycles                   
    ;************************************************
    
    ; Write out 0 to the GPIO PORT 
    STR     DATA_OUT, [GPIO_ADDR]                        ; 2 CLK / Total 20 
    SUBS    BIT_COUNT, BIT_COUNT, #1                     ; 01 CLK 
    MOV     DATA_OUT, #ONE                               ; 02 CLK
    NOP                                                  ; 03 CLK
    NOP                                                  ; 04 CLK
    NOP                                                  ; 05 CLK
    NOP                                                  ; 06 CLK    
    NOP                                                  ; 07 CLK
    NOP                                                  ; 08 CLK                                                 
    NOP                                                  ; 09 CLK
    NOP                                                  ; 10 CLK
    NOP                                                  ; 11 CLK
    NOP                                                  ; 12 CLK                                                 
    NOP                                                  ; 13 CLK
    NOP                                                  ; 14 CLK
    NOP                                                  ; 15 CLK
    NOP                                                  ; 16 CLK                                                 
    NOP                                                  ; 17 CLK
    NOP                                                  ; 18 CLK    
    BGT     WS2812B_write_set_pin_start       ; 1 CLK / Total 20 

WS2812B_write_set_pin_end
    SUBS BYTE_COUNT, BYTE_COUNT, #1   
    BGT  WS2812B_write_get_byte_start
    
WS2812B_write_get_byte_end
    SUBS NUM_LEDS, NUM_LEDS, #1
    BGT  WS2812B_write_loop_start
    
    
WS2812B_write_return
    POP     {R4-R6}
    BX LR
    ENDP
    align        
    
    END



