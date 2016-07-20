/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 3.15.0
        Device            :  PIC16F1784
        Driver Version    :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/

#include <xc.h>
#include "pin_manager.h"
#include "tmr0.h"

void PIN_MANAGER_Initialize(void)
{
    WPUD = 0x0;
    LATB = 0x0;
    WPUE = 0x8;
    LATA = 0x0;
    LATE = 0x0;
    LATD = 0x0;
    LATC = 0x0;
    WPUA = 0x3F;
    WPUB = 0xC0;
    WPUC = 0x0;
    TRISD = 0x33;
    TRISE = 0x8;
    TRISB = 0xE0;
    TRISC = 0x7;
    TRISA = 0x2B;
    ANSELE = 0x7;
    ANSELA = 0xB4;
    ANSELB = 0x5F;
    ANSELD = 0x4;

    OPTION_REGbits.nWPUEN = 0x0;
    APFCON1 = 0x00;
    APFCON2 = 0x00;

	// interrupt on change for group IOCBF - flag
	IOCBFbits.IOCBF5 = 0; // Pin : RB5

	// interrupt on change for group IOCBN - negative
	IOCBNbits.IOCBN5 = 1; // Pin : RB5

	// interrupt on change for group IOCBP - positive
	IOCBPbits.IOCBP5 = 0; // Pin : RB5

    INTCONbits.IOCIE = 1; // Enable IOCI interrupt 


}

void IOCBN5_NegativeSet(void)
{
	// interrupt on change for group IOCBN - negative
	IOCBNbits.IOCBN5 = 1; // Pin : RB5

	// interrupt on change for group IOCBP - positive
	IOCBPbits.IOCBP5 = 0; // Pin : RB5

}
void IOCIE_InterruptEnable(void)
{
	// interrupt on change for group IOCBF - flag
	IOCBFbits.IOCBF5 = 0; // Pin : RB5
    
    INTCONbits.IOCIE = 1; // Enable IOCI interrupt 
}

unsigned char IsIOCIEEnable(void)
{
    return INTCONbits.IOCIE; 
}


void PIN_MANAGER_IOC(void)
{    
	// interrupt on change for group IOCBF
	if(IOCBFbits.IOCBF5 == 1)
	{
		IOCBFbits.IOCBF5 = 0;
		// Add handler code here for Pin - RB5
        
        TMR0_WriteTimer(256-60);
        TMR0_InterruptEnable();

        Speed_U = 0;
        Speed_H = 0;
        Speed_L = 0;
        Speed_Work_Status = 0;
        Seep_256ms_Cnt = 3;
        
        INTCONbits.IOCIE = 0; // Disable IOCI interrupt 
	}
}

/**
 End of File
*/
