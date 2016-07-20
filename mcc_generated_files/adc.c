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
#define OverVoltage			0xC8//3920 / ( 5000 / 256 )				//16V
#define ADC_LowVoltage  0x6C	//7.5V，ADC抓到的數值
#define FVR_LowVoltage  0x35	//7.5V，FVR抓到的數值

/**
  Section: ADC Module APIs
*/

void ADC_Initialize(void)
{
    // set the ADC to the options selected in the User Interface
    
    // ADRMD 10_bit_mode; GO_nDONE stop; ADON enabled; CHS AN4; 
    ADCON0 = 0x91;
    
    // ADFM sign_magnitude; ADNREF VSS; ADPREF VDD; ADCS Frc; 
    ADCON1 = 0x70;
    
    // TRIGSEL disabled; CHSN ADNREF; 
    ADCON2 = 0x0F;
    
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
    return ((ADRESH << 8) + ADRESL);
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
	unsigned int ADC_Data = 0, FVR_Data = 0, Normal_Data = 0, cunt;

	
    ADC_Initialize();
	ADC_StartConversion(AN4_VOLT);

    for (cunt = 0; cunt < 4; cunt++)
	{
        
        //for (cunt = 0; cunt < 200; cunt++);
        while(ADC_IsConversionDone());
		Normal_Data += ADC_GetConversion(AN4_VOLT) >> 8;
        
	}
	Normal_Data = Normal_Data >> 2;
	
	FVR_Initialize();
	
	for (cunt = 0; cunt < 4; cunt++)
	{
        
		while(!FVR_IsOutputReady());
		FVR_Data += ADC_GetConversion(channel_FVR) >> 8;
	}
	FVR_Data = FVR_Data >> 2;
	
	if ((FVR_Data > FVR_LowVoltage) || (Normal_Data < ADC_LowVoltage))											//低於下限值7.5V,此為使用FVR 1.024V狀態下抓到的值，和ADC抓到的值
	{
		Error = 1;
	}
	else if (FVR_Data <= FVR_LowVoltage)
	{
		if (Normal_Data > OverVoltage)
		{
			Error = 1;
		}
		else
			Error = 0;
	}
	
	return Error;
}
/**
 End of File
*/