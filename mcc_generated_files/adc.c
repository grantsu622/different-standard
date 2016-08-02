/**
  ADC Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    adc.c

  @Summary
    This is the generated driver implementation file for the ADC driver using MPLAB(c) Code Configurator

  @Description
    This source file provides implementations for driver APIs for ADC.
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
#include "adc.h"
#include "mcc.h"

/**
  Section: Macro Declarations
*/

#define ACQ_US_DELAY 5

//ADC define
//#define OverVoltage			0xC8//3920 / ( 5000 / 256 )				//16V
//#define ADC_LowVoltage  0x6C	//7.5VADCъ计
#define FVR_LowVoltage  0x35	//7.5VFVRъ计

#define OverVoltage			0xD6//3920 / ( 5000 / 256 )				//16V
#define ADC_LowVoltage  0x68	//7.5VADCъ计

/**
  Section: ADC Module APIs
*/

void ADC_Initialize(void)
{
    // set the ADC to the options selected in the User Interface
#if 1
    FVRCON = 0b11000011 ;	//ADC Fixed Voltage Reference Peripheral output is 1x (4.096V)
    
    // ADFM sign_magnitude; ADNREF VSS; ADPREF FVR; ADCS Frc; 
	ADCON1 = 0b01110011;								//计綼オ
    
    // ADRMD 10_bit_mode; GO_nDONE stop; ADON enabled; CHS AN4;
	ADCON0 = 0b10010001;
#else
    // ADRMD 10_bit_mode; GO_nDONE stop; ADON enabled; CHS AN4; 
    ADCON0 = 0x91;
    
    // ADFM sign_magnitude; ADNREF VSS; ADPREF VDD; ADCS Frc; 
    //ADCON1 = 0x70;
    
    // ADFM sign_magnitude; ADNREF VSS; ADPREF FVR; ADCS Frc; 
	ADCON1 = 0b01110011;								//计綼オ
#endif
    // TRIGSEL disabled; CHSN ADNREF; 
    ADCON2 = 0xFF;
    
    // ADRESH 0; 
    ADRESH = 0x00;
    
    // ADRESL 0; 
    ADRESL = 0x00;
    
}

void ADC_StartConversion(adc_channel_t channel)
{
    // select the A/D channel
    ADCON0bits.CHS = channel;

    // Turn on the ADC module
    ADCON0bits.ADON = 1;

    // Acquisition time delay
    __delay_us(ACQ_US_DELAY);
    
    // Start the conversion
    ADCON0bits.GO_nDONE = 1;
}

bool ADC_IsConversionDone()
{
    // Start the conversion
    return (!ADCON0bits.GO_nDONE);
}

adc_result_t ADC_GetConversionResult(void)
{
    // Conversion finished, return the result
    return ((ADRESH << 8) + ADRESL);
}

adc_result_t ADC_GetConversion(adc_channel_t channel)
{
    // Select the A/D channel
    ADCON0bits.CHS = channel;

    // Turn on the ADC module
    ADCON0bits.ADON = 1;

    // Acquisition time delay
    __delay_us(ACQ_US_DELAY);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE)
    {
    }
    
    // Conversion finished, return the result
    //return ((ADRESH << 8) + ADRESL);
    return (ADRESH);
}

/*********************************************
* ADC check Voltage
* Low voltage 0x6c
* Over voltage 0xC8
* FVR low voltage 0x35
* Voltage Error = 1
**********************************************/
unsigned char IsVoltageError()
{
	unsigned char Error = 0;
	unsigned int cunt, Normal_Data = 0;	

    for (cunt = 0; cunt < 4; cunt++)
	{
        //FVR_Initialize();
        ADC_Initialize();
        
        // Start the conversion
        Normal_Data += ADC_GetConversion(AN4_VOLT);        
	}
 
	Normal_Data = Normal_Data >> 2;
    
    if ( (Normal_Data <= ADC_LowVoltage) || (Normal_Data >= OverVoltage))
        Error = 1;
    else
        Error = 0;
     //Error = 0;
	return Error;
}
/**
 End of File
*/