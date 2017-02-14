#include <msp430x20x3.h>

#define DALLAS 002h
#define LED 001h
#define DS1990_ID1 0200h
#define DS1990_ID2 0201h
#define DS1990_ID3 0202h
#define DS1990_ID4 0203h
#define DS1990_ID5 0204h
#define DS1990_ID6 0205h
#define CALCULATED_CRC 0206h


ORG  0F800h
            ; Initialize stackpointer
RESET       mov.w   #0280h,SP        
            ; Stop WDI
StopWDT     mov.w   #WDTPW+WDTHOLD,&WDTCTL  
            ; Set the DCO to 8 MHz
            mov.b   &CALBC1_8MHZ,&BCSCTL1 
            mov.b   &CALDCO_8MHZ,&DCOCTL 
            ; * Set the Dallas pin to be an output and high initially
            bis.b   #DALLAS,&P1DIR   
            bis.b   #DALLAS,&P1OUT
            ; Set the LED pin to be an output and initially low
            bis.b   #LED,&P1DIR   
            bic.b   #LED,&P1OUT      
            
main        call #DS_CHK
            ; If C is set an iButton is present
            jnc main
            ; If C is set then all is well
            call #ds1990rd
            jc match
                    
            jmp main
          
match:      bis.b   #LED,&P1OUT 
            jmp match
          
            ; This subroutine returns with the carry set if an iButton is present
DS_CHK:     bic.b   #DALLAS,&P1OUT          
            bis.b   #DALLAS,&P1DIR  
            mov     #650,r15
            call    #microsecd             
            bic.b   #DALLAS,&P1DIR          
            mov     #14,r15
            call    #microsecd                          
            bit.b   #DALLAS,&P1IN           
            jz      DS_CHK_0                
            mov.w   #15600,r15     
DS_1:       bit.b   #DALLAS,&P1IN           
            jz      DS_2                    
            dec.w   r15                  
            jnz     DS_1                    
            jmp     DS_CHK_0                
DS_2:       mov.w   #62400,r15          
DS_3:       bit.b   #DALLAS,&P1IN           
            jnz     DS_CHK_1                
            dec.w   r15             
            jnz     DS_3                    
DS_CHK_0:   clrc                            
            ret                             
DS_CHK_1:   setc          
            ret  
            
            ; This subroutine sends a byte from r14 to the Dallas device 
touchbyte:  mov #8,r13
            ; Run through all 8 bits
bit_loop:   call #touchbit
            dec r13
            jnz bit_loop
            rlc.b r12
            rrc.b r14
            ret
            ; This subroutine sends a bit and receives a bit
            ; Make the Dallas pin an ouput
touchbit:   bis.b   #DALLAS,&P1DIR  
            ; Delay for 1 us
            mov #1,r15
            call #microsecd
            ; Move the LSB from r14 to the C bit
            ; Get the last received bit from the MSB of r12 and put it in the C bit
            rlc.b r12
            rrc.b r14
            jnc conttouch
            ; The Dallas bit needs to be high so make the line an input
            bic.b   #DALLAS,&P1DIR   
            ; Delay for 15 us 
conttouch:  mov #15,r15
            call #microsecd
            ; Read the Dallas pin
            bit.b   #DALLAS,&P1IN 
            ; Store the result in the MSB of r12
            jnc inbitlow
            ; The pin is high
            setc
            jmp lasttouch
            ; The pin is low
inbitlow:   clrc
lasttouch:  rrc.b r12
            ; Delay for 50us
            mov #50,r15
            call #microsecd     
            bic.b   #DALLAS,&P1DIR   
            ret
            
             ; This subroutine gives a delay of the number of microseconds in R15
microsecd:  dec r15
            nop
            nop
            nop
            nop
            jnz microsecd
            ret
            
            ; This subroutine reads a DS1990
            ; If there is a problem the C bit is cleared
            ; First clear the calculated CRC byte
 ds1990rd:  mov.b #00h,CALCULATED_CRC
            ; Send a ROM search command
            mov.b #0fh,r14
            call #touchbyte
            ; Get the device type
            mov.b #0ffh,r14
            call #touchbyte  
            ; If the device type isn't 01 then return 
            cmp #01h,r14                  
            jeq match1990
            ; This isn't a DS1990
            clrc
            ret
            ; This is a DS1990 so continue
            ; Run the family code through the CRC
match1990:  call #do_crc
            ; Read in the next 6 bytes which make up the DS1990's unique ID
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID1
            call #do_crc
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID2  
            call #do_crc
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID3
            call #do_crc
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID4
            call #do_crc
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID5
            call #do_crc
            mov.b #0ffh,r14
            call #touchbyte
            mov.b r14,DS1990_ID6
            call #do_crc
            ; Finaly get the DS1990's CRC byte
            mov.b #0ffh,r14
            call #touchbyte
            ; Compare the received CRC byte and the calculated CRC byte
            cmp.b CALCULATED_CRC,r14
            ; If not equal there is a problem
            jnz dscfail
            ; Set the C bit to show all is well and return
            setc
            ret
            ; Clear the C bit to show a problem
 dscfail:   clrc
            ret
            
            ;  Calculate the CRC storing the result in CALCULATED_CRC
            ;  and taking r14 as the input also using r15 as a bit counter
do_crc:     push.b r14
            mov.b #08h,r15
lp_crc:     xor.b CALCULATED_CRC,r14
            rrc.b r14
            mov.b CALCULATED_CRC,r14
            jnc zero_crc
            xor.b #018h,r14
            setc
zero_crc:   rrc.b r14     
            mov.b r14,CALCULATED_CRC
            pop.b r14
            rra.b r14
            push.b r14
            dec r15
            jnz lp_crc
            pop.b r14
            ret

;-------------------------------------------------------------------------------
;           Interrupt Vectors
;-------------------------------------------------------------------------------
            ORG     0FFFEh                  ; MSP430 RESET Vector
            DW      RESET                   ;
            END