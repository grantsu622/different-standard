/**
  TMR0 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr0.c

  @Summary
    This is the generated driver implementation file for the TMR0 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for TMR0.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 3.15.0
        Device            :  PIC16F1784
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr0.h"
#include "pin_manager.h"
/**
  Section: Global Variables Definitions
*/

volatile uint8_t timer0ReloadVal;

/**
  Section: TMR0 APIs
*/

void TMR0_Initialize(void)
{
    // Set TMR0 to the options selected in the User Interface

    // PSA assigned; PS 1:4; TMRSE Increment_hi_lo; mask the nWPUEN and INTEDG bits
    OPTION_REG = (OPTION_REG & 0xC0) | 0xD1 & 0x3F; 

    // TMR0 144; 
    TMR0 = 0x0;

    // Load the TMR value to reload variable
    timer0ReloadVal= 144;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 256 - 119;

    // Enabling TMR0 interrupt
    //INTCONbits.TMR0IE = 1;
}


uint8_t TMR0_ReadTimer(void)
{
    uint8_t readVal;

    readVal = TMR0;

    return readVal;
}

void TMR0_WriteTimer(uint8_t timerVal)
{
    // Write to the Timer0 register
    TMR0 = timerVal;
}

void TMR0_Reload(void)
{
    // Write to the Timer0 register
    TMR0 = timer0ReloadVal;
}

void TMR0_InterruptEnable(void)
{
    // clear global interrupt-on-change flag
    INTCONbits.IOCIF = 0;

    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt
    INTCONbits.TMR0IE = 1;
}

void TMR0_InterruptDisable(void)
{
    // Disabling TMR0 interrupt
    INTCONbits.TMR0IE = 0;

}

void TMR0_ISR(void)
{

    // clear the TMR0 interrupt flag
    INTCONbits.TMR0IF = 0;

    TMR0 = timer0ReloadVal;


    // add your TMR0 interrupt custom code
    if(IO_RB5_RPM_GetValue()) tmp = 1;
    else tmp = 0;

    switch(Speed_Work_Status)
    {
        case 0:
            if(tmp == 0)
            {
                Speed_Work_Status++;
            }
            if(error_cnt == 0) TMR0_InterruptDisable();
            else error_cnt--;
            break;
        case 1:
        case 2:
        case 3:
            if(tmp) Speed_U = Speed_U | 0x01;
            Speed_U = Speed_U << 1;
            Speed_Work_Status++;
        case 4:
            if(tmp) Speed_U = Speed_U | 0x01;
            Speed_Work_Status++;
            break;
        case 5 :
		case 6 :
		case 7 :
			if(tmp) Speed_H = Speed_H | 0x01;
			Speed_H = Speed_H << 1;
			Speed_Work_Status++;
		break;
		case 8 :
			if(tmp) Speed_H = Speed_H | 0x01;
			Speed_Work_Status++;
		break;
        case 9:
        case 10:
        case 11:
            if(tmp) Speed_L = Speed_L | 0x01;
            Speed_L = Speed_L << 1;
            Speed_Work_Status++;
            break;
        case 12:
            if(tmp) Speed_L = Speed_L | 0x01;
            Speed_Work_Status++;
            break;
        case 13:
            TMR0_InterruptDisable();
            Speed_Work_Status = 0;
            if(tmp == 1)
            {
                Speed = 30;
            }
            else
            {
                Speed_rd = 1;
            }

            break;
            // Enabling TMR0 interrupt
            INTCONbits.TMR0IE = 0;
    }
}

/**
  End of File
*/
