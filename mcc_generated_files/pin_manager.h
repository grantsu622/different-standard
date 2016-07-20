/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 3.15.0
        Device            :  PIC16F1784
        Version           :  1.01
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


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA0_WG_Signal aliases
#define IO_RA0_WG_Signal_TRIS               TRISA0
#define IO_RA0_WG_Signal_LAT                LATA0
#define IO_RA0_WG_Signal_PORT               RA0
#define IO_RA0_WG_Signal_WPU                WPUA0
#define IO_RA0_WG_Signal_ANS                ANSA0
#define IO_RA0_WG_Signal_SetHigh()    do { LATA0 = 1; } while(0)
#define IO_RA0_WG_Signal_SetLow()   do { LATA0 = 0; } while(0)
#define IO_RA0_WG_Signal_Toggle()   do { LATA0 = ~LATA0; } while(0)
#define IO_RA0_WG_Signal_GetValue()         PORTAbits.RA0
#define IO_RA0_WG_Signal_SetDigitalInput()    do { TRISA0 = 1; } while(0)
#define IO_RA0_WG_Signal_SetDigitalOutput()   do { TRISA0 = 0; } while(0)

#define IO_RA0_WG_Signal_SetPullup()    do { WPUA0 = 1; } while(0)
#define IO_RA0_WG_Signal_ResetPullup()   do { WPUA0 = 0; } while(0)
#define IO_RA0_WG_Signal_SetAnalogMode()   do { ANSA0 = 1; } while(0)
#define IO_RA0_WG_Signal_SetDigitalMode()   do { ANSA0 = 0; } while(0)


// get/set IO_RA1_L_Signal aliases
#define IO_RA1_L_Signal_TRIS               TRISA1
#define IO_RA1_L_Signal_LAT                LATA1
#define IO_RA1_L_Signal_PORT               RA1
#define IO_RA1_L_Signal_WPU                WPUA1
#define IO_RA1_L_Signal_ANS                ANSA1
#define IO_RA1_L_Signal_SetHigh()    do { LATA1 = 1; } while(0)
#define IO_RA1_L_Signal_SetLow()   do { LATA1 = 0; } while(0)
#define IO_RA1_L_Signal_Toggle()   do { LATA1 = ~LATA1; } while(0)
#define IO_RA1_L_Signal_GetValue()         PORTAbits.RA1
#define IO_RA1_L_Signal_SetDigitalInput()    do { TRISA1 = 1; } while(0)
#define IO_RA1_L_Signal_SetDigitalOutput()   do { TRISA1 = 0; } while(0)

#define IO_RA1_L_Signal_SetPullup()    do { WPUA1 = 1; } while(0)
#define IO_RA1_L_Signal_ResetPullup()   do { WPUA1 = 0; } while(0)
#define IO_RA1_L_Signal_SetAnalogMode()   do { ANSA1 = 1; } while(0)
#define IO_RA1_L_Signal_SetDigitalMode()   do { ANSA1 = 0; } while(0)


// get/set IO_RA2_Y_CTRL aliases
#define IO_RA2_Y_CTRL_TRIS               TRISA2
#define IO_RA2_Y_CTRL_LAT                LATA2
#define IO_RA2_Y_CTRL_PORT               RA2
#define IO_RA2_Y_CTRL_WPU                WPUA2
#define IO_RA2_Y_CTRL_ANS                ANSA2
#define IO_RA2_Y_CTRL_SetHigh()    do { LATA2 = 1; } while(0)
#define IO_RA2_Y_CTRL_SetLow()   do { LATA2 = 0; } while(0)
#define IO_RA2_Y_CTRL_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define IO_RA2_Y_CTRL_GetValue()         PORTAbits.RA2
#define IO_RA2_Y_CTRL_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define IO_RA2_Y_CTRL_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define IO_RA2_Y_CTRL_SetPullup()    do { WPUA2 = 1; } while(0)
#define IO_RA2_Y_CTRL_ResetPullup()   do { WPUA2 = 0; } while(0)
#define IO_RA2_Y_CTRL_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define IO_RA2_Y_CTRL_SetDigitalMode()   do { ANSA2 = 0; } while(0)


// get/set IO_RA3_Y_Signal aliases
#define IO_RA3_Y_Signal_TRIS               TRISA3
#define IO_RA3_Y_Signal_LAT                LATA3
#define IO_RA3_Y_Signal_PORT               RA3
#define IO_RA3_Y_Signal_WPU                WPUA3
#define IO_RA3_Y_Signal_ANS                ANSA3
#define IO_RA3_Y_Signal_SetHigh()    do { LATA3 = 1; } while(0)
#define IO_RA3_Y_Signal_SetLow()   do { LATA3 = 0; } while(0)
#define IO_RA3_Y_Signal_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define IO_RA3_Y_Signal_GetValue()         PORTAbits.RA3
#define IO_RA3_Y_Signal_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define IO_RA3_Y_Signal_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define IO_RA3_Y_Signal_SetPullup()    do { WPUA3 = 1; } while(0)
#define IO_RA3_Y_Signal_ResetPullup()   do { WPUA3 = 0; } while(0)
#define IO_RA3_Y_Signal_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define IO_RA3_Y_Signal_SetDigitalMode()   do { ANSA3 = 0; } while(0)


// get/set IO_RA4_Y_CTRL aliases
#define IO_RA4_Y_CTRL_TRIS               TRISA4
#define IO_RA4_Y_CTRL_LAT                LATA4
#define IO_RA4_Y_CTRL_PORT               RA4
#define IO_RA4_Y_CTRL_WPU                WPUA4
#define IO_RA4_Y_CTRL_ANS                ANSA4
#define IO_RA4_Y_CTRL_SetHigh()    do { LATA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetLow()   do { LATA4 = 0; } while(0)
#define IO_RA4_Y_CTRL_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define IO_RA4_Y_CTRL_GetValue()         PORTAbits.RA4
#define IO_RA4_Y_CTRL_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define IO_RA4_Y_CTRL_SetPullup()    do { WPUA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_ResetPullup()   do { WPUA4 = 0; } while(0)
#define IO_RA4_Y_CTRL_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetDigitalMode()   do { ANSA4 = 0; } while(0)


// get/set IO_RA4_Y_CTRL aliases
#define IO_RA4_Y_CTRL_TRIS               TRISA4
#define IO_RA4_Y_CTRL_LAT                LATA4
#define IO_RA4_Y_CTRL_PORT               RA4
#define IO_RA4_Y_CTRL_WPU                WPUA4
#define IO_RA4_Y_CTRL_ANS                ANSA4
#define IO_RA4_Y_CTRL_SetHigh()    do { LATA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetLow()   do { LATA4 = 0; } while(0)
#define IO_RA4_Y_CTRL_Toggle()   do { LATA4 = ~LATA4; } while(0)
#define IO_RA4_Y_CTRL_GetValue()         PORTAbits.RA4
#define IO_RA4_Y_CTRL_SetDigitalInput()    do { TRISA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetDigitalOutput()   do { TRISA4 = 0; } while(0)

#define IO_RA4_Y_CTRL_SetPullup()    do { WPUA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_ResetPullup()   do { WPUA4 = 0; } while(0)
#define IO_RA4_Y_CTRL_SetAnalogMode()   do { ANSA4 = 1; } while(0)
#define IO_RA4_Y_CTRL_SetDigitalMode()   do { ANSA4 = 0; } while(0)


// get/set IO_RA6_WR_CTRL aliases
#define IO_RA6_WR_CTRL_TRIS               TRISA6
#define IO_RA6_WR_CTRL_LAT                LATA6
#define IO_RA6_WR_CTRL_PORT               RA6
#define IO_RA6_WR_CTRL_WPU                WPUA6
#define IO_RA6_WR_CTRL_SetHigh()    do { LATA6 = 1; } while(0)
#define IO_RA6_WR_CTRL_SetLow()   do { LATA6 = 0; } while(0)
#define IO_RA6_WR_CTRL_Toggle()   do { LATA6 = ~LATA6; } while(0)
#define IO_RA6_WR_CTRL_GetValue()         PORTAbits.RA6
#define IO_RA6_WR_CTRL_SetDigitalInput()    do { TRISA6 = 1; } while(0)
#define IO_RA6_WR_CTRL_SetDigitalOutput()   do { TRISA6 = 0; } while(0)

#define IO_RA6_WR_CTRL_SetPullup()    do { WPUA6 = 1; } while(0)
#define IO_RA6_WR_CTRL_ResetPullup()   do { WPUA6 = 0; } while(0)


// get/set IO_RA7_WB_CTRL aliases
#define IO_RA7_WB_CTRL_TRIS               TRISA7
#define IO_RA7_WB_CTRL_LAT                LATA7
#define IO_RA7_WB_CTRL_PORT               RA7
#define IO_RA7_WB_CTRL_WPU                WPUA7
#define IO_RA7_WB_CTRL_ANS                ANSA7
#define IO_RA7_WB_CTRL_SetHigh()    do { LATA7 = 1; } while(0)
#define IO_RA7_WB_CTRL_SetLow()   do { LATA7 = 0; } while(0)
#define IO_RA7_WB_CTRL_Toggle()   do { LATA7 = ~LATA7; } while(0)
#define IO_RA7_WB_CTRL_GetValue()         PORTAbits.RA7
#define IO_RA7_WB_CTRL_SetDigitalInput()    do { TRISA7 = 1; } while(0)
#define IO_RA7_WB_CTRL_SetDigitalOutput()   do { TRISA7 = 0; } while(0)

#define IO_RA7_WB_CTRL_SetPullup()    do { WPUA7 = 1; } while(0)
#define IO_RA7_WB_CTRL_ResetPullup()   do { WPUA7 = 0; } while(0)
#define IO_RA7_WB_CTRL_SetAnalogMode()   do { ANSA7 = 1; } while(0)
#define IO_RA7_WB_CTRL_SetDigitalMode()   do { ANSA7 = 0; } while(0)


// get/set IO_RB0_ECU_4W aliases
#define IO_RB0_ECU_4W_TRIS               TRISB0
#define IO_RB0_ECU_4W_LAT                LATB0
#define IO_RB0_ECU_4W_PORT               RB0
#define IO_RB0_ECU_4W_WPU                WPUB0
#define IO_RB0_ECU_4W_ANS                ANSB0
#define IO_RB0_ECU_4W_SetHigh()    do { LATB0 = 1; } while(0)
#define IO_RB0_ECU_4W_SetLow()   do { LATB0 = 0; } while(0)
#define IO_RB0_ECU_4W_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define IO_RB0_ECU_4W_GetValue()         PORTBbits.RB0
#define IO_RB0_ECU_4W_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define IO_RB0_ECU_4W_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define IO_RB0_ECU_4W_SetPullup()    do { WPUB0 = 1; } while(0)
#define IO_RB0_ECU_4W_ResetPullup()   do { WPUB0 = 0; } while(0)
#define IO_RB0_ECU_4W_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define IO_RB0_ECU_4W_SetDigitalMode()   do { ANSB0 = 0; } while(0)


// get/set IO_RB1_ECU_4WL aliases
#define IO_RB1_ECU_4WL_TRIS               TRISB1
#define IO_RB1_ECU_4WL_LAT                LATB1
#define IO_RB1_ECU_4WL_PORT               RB1
#define IO_RB1_ECU_4WL_WPU                WPUB1
#define IO_RB1_ECU_4WL_ANS                ANSB1
#define IO_RB1_ECU_4WL_SetHigh()    do { LATB1 = 1; } while(0)
#define IO_RB1_ECU_4WL_SetLow()   do { LATB1 = 0; } while(0)
#define IO_RB1_ECU_4WL_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define IO_RB1_ECU_4WL_GetValue()         PORTBbits.RB1
#define IO_RB1_ECU_4WL_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define IO_RB1_ECU_4WL_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define IO_RB1_ECU_4WL_SetPullup()    do { WPUB1 = 1; } while(0)
#define IO_RB1_ECU_4WL_ResetPullup()   do { WPUB1 = 0; } while(0)
#define IO_RB1_ECU_4WL_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define IO_RB1_ECU_4WL_SetDigitalMode()   do { ANSB1 = 0; } while(0)


// get/set IO_RB2_ECU_2WL aliases
#define IO_RB2_ECU_2WL_TRIS               TRISB2
#define IO_RB2_ECU_2WL_LAT                LATB2
#define IO_RB2_ECU_2WL_PORT               RB2
#define IO_RB2_ECU_2WL_WPU                WPUB2
#define IO_RB2_ECU_2WL_ANS                ANSB2
#define IO_RB2_ECU_2WL_SetHigh()    do { LATB2 = 1; } while(0)
#define IO_RB2_ECU_2WL_SetLow()   do { LATB2 = 0; } while(0)
#define IO_RB2_ECU_2WL_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define IO_RB2_ECU_2WL_GetValue()         PORTBbits.RB2
#define IO_RB2_ECU_2WL_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define IO_RB2_ECU_2WL_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define IO_RB2_ECU_2WL_SetPullup()    do { WPUB2 = 1; } while(0)
#define IO_RB2_ECU_2WL_ResetPullup()   do { WPUB2 = 0; } while(0)
#define IO_RB2_ECU_2WL_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define IO_RB2_ECU_2WL_SetDigitalMode()   do { ANSB2 = 0; } while(0)


// get/set IO_RB3_M1R aliases
#define IO_RB3_M1R_TRIS               TRISB3
#define IO_RB3_M1R_LAT                LATB3
#define IO_RB3_M1R_PORT               RB3
#define IO_RB3_M1R_WPU                WPUB3
#define IO_RB3_M1R_ANS                ANSB3
#define IO_RB3_M1R_SetHigh()    do { LATB3 = 1; } while(0)
#define IO_RB3_M1R_SetLow()   do { LATB3 = 0; } while(0)
#define IO_RB3_M1R_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define IO_RB3_M1R_GetValue()         PORTBbits.RB3
#define IO_RB3_M1R_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define IO_RB3_M1R_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define IO_RB3_M1R_SetPullup()    do { WPUB3 = 1; } while(0)
#define IO_RB3_M1R_ResetPullup()   do { WPUB3 = 0; } while(0)
#define IO_RB3_M1R_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define IO_RB3_M1R_SetDigitalMode()   do { ANSB3 = 0; } while(0)


// get/set IO_RB4_M1F aliases
#define IO_RB4_M1F_TRIS               TRISB4
#define IO_RB4_M1F_LAT                LATB4
#define IO_RB4_M1F_PORT               RB4
#define IO_RB4_M1F_WPU                WPUB4
#define IO_RB4_M1F_ANS                ANSB4
#define IO_RB4_M1F_SetHigh()    do { LATB4 = 1; } while(0)
#define IO_RB4_M1F_SetLow()   do { LATB4 = 0; } while(0)
#define IO_RB4_M1F_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define IO_RB4_M1F_GetValue()         PORTBbits.RB4
#define IO_RB4_M1F_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define IO_RB4_M1F_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define IO_RB4_M1F_SetPullup()    do { WPUB4 = 1; } while(0)
#define IO_RB4_M1F_ResetPullup()   do { WPUB4 = 0; } while(0)
#define IO_RB4_M1F_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define IO_RB4_M1F_SetDigitalMode()   do { ANSB4 = 0; } while(0)


// get/set IO_RB5_RPM aliases
#define IO_RB5_RPM_TRIS               TRISB5
#define IO_RB5_RPM_LAT                LATB5
#define IO_RB5_RPM_PORT               RB5
#define IO_RB5_RPM_WPU                WPUB5
#define IO_RB5_RPM_ANS                ANSB5
#define IO_RB5_RPM_SetHigh()    do { LATB5 = 1; } while(0)
#define IO_RB5_RPM_SetLow()   do { LATB5 = 0; } while(0)
#define IO_RB5_RPM_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define IO_RB5_RPM_GetValue()         PORTBbits.RB5
#define IO_RB5_RPM_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define IO_RB5_RPM_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define IO_RB5_RPM_SetPullup()    do { WPUB5 = 1; } while(0)
#define IO_RB5_RPM_ResetPullup()   do { WPUB5 = 0; } while(0)
#define IO_RB5_RPM_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define IO_RB5_RPM_SetDigitalMode()   do { ANSB5 = 0; } while(0)


// get/set IO_RC0_WB_Signal aliases
#define IO_RC0_WB_Signal_TRIS               TRISC0
#define IO_RC0_WB_Signal_LAT                LATC0
#define IO_RC0_WB_Signal_PORT               RC0
#define IO_RC0_WB_Signal_WPU                WPUC0
#define IO_RC0_WB_Signal_SetHigh()    do { LATC0 = 1; } while(0)
#define IO_RC0_WB_Signal_SetLow()   do { LATC0 = 0; } while(0)
#define IO_RC0_WB_Signal_Toggle()   do { LATC0 = ~LATC0; } while(0)
#define IO_RC0_WB_Signal_GetValue()         PORTCbits.RC0
#define IO_RC0_WB_Signal_SetDigitalInput()    do { TRISC0 = 1; } while(0)
#define IO_RC0_WB_Signal_SetDigitalOutput()   do { TRISC0 = 0; } while(0)

#define IO_RC0_WB_Signal_SetPullup()    do { WPUC0 = 1; } while(0)
#define IO_RC0_WB_Signal_ResetPullup()   do { WPUC0 = 0; } while(0)


// get/set IO_RC1_WL_Signal aliases
#define IO_RC1_WL_Signal_TRIS               TRISC1
#define IO_RC1_WL_Signal_LAT                LATC1
#define IO_RC1_WL_Signal_PORT               RC1
#define IO_RC1_WL_Signal_WPU                WPUC1
#define IO_RC1_WL_Signal_SetHigh()    do { LATC1 = 1; } while(0)
#define IO_RC1_WL_Signal_SetLow()   do { LATC1 = 0; } while(0)
#define IO_RC1_WL_Signal_Toggle()   do { LATC1 = ~LATC1; } while(0)
#define IO_RC1_WL_Signal_GetValue()         PORTCbits.RC1
#define IO_RC1_WL_Signal_SetDigitalInput()    do { TRISC1 = 1; } while(0)
#define IO_RC1_WL_Signal_SetDigitalOutput()   do { TRISC1 = 0; } while(0)

#define IO_RC1_WL_Signal_SetPullup()    do { WPUC1 = 1; } while(0)
#define IO_RC1_WL_Signal_ResetPullup()   do { WPUC1 = 0; } while(0)


// get/set IO_RC2_WR_Signal aliases
#define IO_RC2_WR_Signal_TRIS               TRISC2
#define IO_RC2_WR_Signal_LAT                LATC2
#define IO_RC2_WR_Signal_PORT               RC2
#define IO_RC2_WR_Signal_WPU                WPUC2
#define IO_RC2_WR_Signal_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_RC2_WR_Signal_SetLow()   do { LATC2 = 0; } while(0)
#define IO_RC2_WR_Signal_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_RC2_WR_Signal_GetValue()         PORTCbits.RC2
#define IO_RC2_WR_Signal_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_RC2_WR_Signal_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IO_RC2_WR_Signal_SetPullup()    do { WPUC2 = 1; } while(0)
#define IO_RC2_WR_Signal_ResetPullup()   do { WPUC2 = 0; } while(0)


// get/set IO_RC3 aliases
#define IO_RC3_TRIS               TRISC3
#define IO_RC3_LAT                LATC3
#define IO_RC3_PORT               RC3
#define IO_RC3_WPU                WPUC3
#define IO_RC3_SetHigh()    do { LATC3 = 1; } while(0)
#define IO_RC3_SetLow()   do { LATC3 = 0; } while(0)
#define IO_RC3_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define IO_RC3_GetValue()         PORTCbits.RC3
#define IO_RC3_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define IO_RC3_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define IO_RC3_SetPullup()    do { WPUC3 = 1; } while(0)
#define IO_RC3_ResetPullup()   do { WPUC3 = 0; } while(0)


// get/set IO_RC4 aliases
#define IO_RC4_TRIS               TRISC4
#define IO_RC4_LAT                LATC4
#define IO_RC4_PORT               RC4
#define IO_RC4_WPU                WPUC4
#define IO_RC4_SetHigh()    do { LATC4 = 1; } while(0)
#define IO_RC4_SetLow()   do { LATC4 = 0; } while(0)
#define IO_RC4_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define IO_RC4_GetValue()         PORTCbits.RC4
#define IO_RC4_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define IO_RC4_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define IO_RC4_SetPullup()    do { WPUC4 = 1; } while(0)
#define IO_RC4_ResetPullup()   do { WPUC4 = 0; } while(0)


// get/set IO_RC5 aliases
#define IO_RC5_TRIS               TRISC5
#define IO_RC5_LAT                LATC5
#define IO_RC5_PORT               RC5
#define IO_RC5_WPU                WPUC5
#define IO_RC5_SetHigh()    do { LATC5 = 1; } while(0)
#define IO_RC5_SetLow()   do { LATC5 = 0; } while(0)
#define IO_RC5_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define IO_RC5_GetValue()         PORTCbits.RC5
#define IO_RC5_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

#define IO_RC5_SetPullup()    do { WPUC5 = 1; } while(0)
#define IO_RC5_ResetPullup()   do { WPUC5 = 0; } while(0)


// get/set IO_RC6 aliases
#define IO_RC6_TRIS               TRISC6
#define IO_RC6_LAT                LATC6
#define IO_RC6_PORT               RC6
#define IO_RC6_WPU                WPUC6
#define IO_RC6_SetHigh()    do { LATC6 = 1; } while(0)
#define IO_RC6_SetLow()   do { LATC6 = 0; } while(0)
#define IO_RC6_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define IO_RC6_GetValue()         PORTCbits.RC6
#define IO_RC6_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define IO_RC6_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define IO_RC6_SetPullup()    do { WPUC6 = 1; } while(0)
#define IO_RC6_ResetPullup()   do { WPUC6 = 0; } while(0)


// get/set IO_RC7 aliases
#define IO_RC7_TRIS               TRISC7
#define IO_RC7_LAT                LATC7
#define IO_RC7_PORT               RC7
#define IO_RC7_WPU                WPUC7
#define IO_RC7_SetHigh()    do { LATC7 = 1; } while(0)
#define IO_RC7_SetLow()   do { LATC7 = 0; } while(0)
#define IO_RC7_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define IO_RC7_GetValue()         PORTCbits.RC7
#define IO_RC7_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define IO_RC7_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define IO_RC7_SetPullup()    do { WPUC7 = 1; } while(0)
#define IO_RC7_ResetPullup()   do { WPUC7 = 0; } while(0)


// get/set IO_RD0_HAND_4WL aliases
#define IO_RD0_HAND_4WL_TRIS               TRISD0
#define IO_RD0_HAND_4WL_LAT                LATD0
#define IO_RD0_HAND_4WL_PORT               RD0
#define IO_RD0_HAND_4WL_WPU                WPUD0
#define IO_RD0_HAND_4WL_ANS                ANSD0
#define IO_RD0_HAND_4WL_SetHigh()    do { LATD0 = 1; } while(0)
#define IO_RD0_HAND_4WL_SetLow()   do { LATD0 = 0; } while(0)
#define IO_RD0_HAND_4WL_Toggle()   do { LATD0 = ~LATD0; } while(0)
#define IO_RD0_HAND_4WL_GetValue()         PORTDbits.RD0
#define IO_RD0_HAND_4WL_SetDigitalInput()    do { TRISD0 = 1; } while(0)
#define IO_RD0_HAND_4WL_SetDigitalOutput()   do { TRISD0 = 0; } while(0)

#define IO_RD0_HAND_4WL_SetPullup()    do { WPUD0 = 1; } while(0)
#define IO_RD0_HAND_4WL_ResetPullup()   do { WPUD0 = 0; } while(0)
#define IO_RD0_HAND_4WL_SetAnalogMode()   do { ANSD0 = 1; } while(0)
#define IO_RD0_HAND_4WL_SetDigitalMode()   do { ANSD0 = 0; } while(0)


// get/set IO_RD1_HAND_4W aliases
#define IO_RD1_HAND_4W_TRIS               TRISD1
#define IO_RD1_HAND_4W_LAT                LATD1
#define IO_RD1_HAND_4W_PORT               RD1
#define IO_RD1_HAND_4W_WPU                WPUD1
#define IO_RD1_HAND_4W_ANS                ANSD1
#define IO_RD1_HAND_4W_SetHigh()    do { LATD1 = 1; } while(0)
#define IO_RD1_HAND_4W_SetLow()   do { LATD1 = 0; } while(0)
#define IO_RD1_HAND_4W_Toggle()   do { LATD1 = ~LATD1; } while(0)
#define IO_RD1_HAND_4W_GetValue()         PORTDbits.RD1
#define IO_RD1_HAND_4W_SetDigitalInput()    do { TRISD1 = 1; } while(0)
#define IO_RD1_HAND_4W_SetDigitalOutput()   do { TRISD1 = 0; } while(0)

#define IO_RD1_HAND_4W_SetPullup()    do { WPUD1 = 1; } while(0)
#define IO_RD1_HAND_4W_ResetPullup()   do { WPUD1 = 0; } while(0)
#define IO_RD1_HAND_4W_SetAnalogMode()   do { ANSD1 = 1; } while(0)
#define IO_RD1_HAND_4W_SetDigitalMode()   do { ANSD1 = 0; } while(0)


// get/set IO_RD2 aliases
#define IO_RD2_TRIS               TRISD2
#define IO_RD2_LAT                LATD2
#define IO_RD2_PORT               RD2
#define IO_RD2_WPU                WPUD2
#define IO_RD2_ANS                ANSD2
#define IO_RD2_SetHigh()    do { LATD2 = 1; } while(0)
#define IO_RD2_SetLow()   do { LATD2 = 0; } while(0)
#define IO_RD2_Toggle()   do { LATD2 = ~LATD2; } while(0)
#define IO_RD2_GetValue()         PORTDbits.RD2
#define IO_RD2_SetDigitalInput()    do { TRISD2 = 1; } while(0)
#define IO_RD2_SetDigitalOutput()   do { TRISD2 = 0; } while(0)

#define IO_RD2_SetPullup()    do { WPUD2 = 1; } while(0)
#define IO_RD2_ResetPullup()   do { WPUD2 = 0; } while(0)
#define IO_RD2_SetAnalogMode()   do { ANSD2 = 1; } while(0)
#define IO_RD2_SetDigitalMode()   do { ANSD2 = 0; } while(0)


// get/set IO_RD3 aliases
#define IO_RD3_TRIS               TRISD3
#define IO_RD3_LAT                LATD3
#define IO_RD3_PORT               RD3
#define IO_RD3_WPU                WPUD3
#define IO_RD3_SetHigh()    do { LATD3 = 1; } while(0)
#define IO_RD3_SetLow()   do { LATD3 = 0; } while(0)
#define IO_RD3_Toggle()   do { LATD3 = ~LATD3; } while(0)
#define IO_RD3_GetValue()         PORTDbits.RD3
#define IO_RD3_SetDigitalInput()    do { TRISD3 = 1; } while(0)
#define IO_RD3_SetDigitalOutput()   do { TRISD3 = 0; } while(0)

#define IO_RD3_SetPullup()    do { WPUD3 = 1; } while(0)
#define IO_RD3_ResetPullup()   do { WPUD3 = 0; } while(0)


// get/set IO_RD4_HAND_2WL aliases
#define IO_RD4_HAND_2WL_TRIS               TRISD4
#define IO_RD4_HAND_2WL_LAT                LATD4
#define IO_RD4_HAND_2WL_PORT               RD4
#define IO_RD4_HAND_2WL_WPU                WPUD4
#define IO_RD4_HAND_2WL_SetHigh()    do { LATD4 = 1; } while(0)
#define IO_RD4_HAND_2WL_SetLow()   do { LATD4 = 0; } while(0)
#define IO_RD4_HAND_2WL_Toggle()   do { LATD4 = ~LATD4; } while(0)
#define IO_RD4_HAND_2WL_GetValue()         PORTDbits.RD4
#define IO_RD4_HAND_2WL_SetDigitalInput()    do { TRISD4 = 1; } while(0)
#define IO_RD4_HAND_2WL_SetDigitalOutput()   do { TRISD4 = 0; } while(0)

#define IO_RD4_HAND_2WL_SetPullup()    do { WPUD4 = 1; } while(0)
#define IO_RD4_HAND_2WL_ResetPullup()   do { WPUD4 = 0; } while(0)


// get/set IO_RD5_HAND_2W aliases
#define IO_RD5_HAND_2W_TRIS               TRISD5
#define IO_RD5_HAND_2W_LAT                LATD5
#define IO_RD5_HAND_2W_PORT               RD5
#define IO_RD5_HAND_2W_WPU                WPUD5
#define IO_RD5_HAND_2W_SetHigh()    do { LATD5 = 1; } while(0)
#define IO_RD5_HAND_2W_SetLow()   do { LATD5 = 0; } while(0)
#define IO_RD5_HAND_2W_Toggle()   do { LATD5 = ~LATD5; } while(0)
#define IO_RD5_HAND_2W_GetValue()         PORTDbits.RD5
#define IO_RD5_HAND_2W_SetDigitalInput()    do { TRISD5 = 1; } while(0)
#define IO_RD5_HAND_2W_SetDigitalOutput()   do { TRISD5 = 0; } while(0)

#define IO_RD5_HAND_2W_SetPullup()    do { WPUD5 = 1; } while(0)
#define IO_RD5_HAND_2W_ResetPullup()   do { WPUD5 = 0; } while(0)


// get/set IO_RD6_RELAY aliases
#define IO_RD6_RELAY_TRIS               TRISD6
#define IO_RD6_RELAY_LAT                LATD6
#define IO_RD6_RELAY_PORT               RD6
#define IO_RD6_RELAY_WPU                WPUD6
#define IO_RD6_RELAY_SetHigh()    do { LATD6 = 1; } while(0)
#define IO_RD6_RELAY_SetLow()   do { LATD6 = 0; } while(0)
#define IO_RD6_RELAY_Toggle()   do { LATD6 = ~LATD6; } while(0)
#define IO_RD6_RELAY_GetValue()         PORTDbits.RD6
#define IO_RD6_RELAY_SetDigitalInput()    do { TRISD6 = 1; } while(0)
#define IO_RD6_RELAY_SetDigitalOutput()   do { TRISD6 = 0; } while(0)

#define IO_RD6_RELAY_SetPullup()    do { WPUD6 = 1; } while(0)
#define IO_RD6_RELAY_ResetPullup()   do { WPUD6 = 0; } while(0)


// get/set IO_RD7_ECU_2W aliases
#define IO_RD7_ECU_2W_TRIS               TRISD7
#define IO_RD7_ECU_2W_LAT                LATD7
#define IO_RD7_ECU_2W_PORT               RD7
#define IO_RD7_ECU_2W_WPU                WPUD7
#define IO_RD7_ECU_2W_SetHigh()    do { LATD7 = 1; } while(0)
#define IO_RD7_ECU_2W_SetLow()   do { LATD7 = 0; } while(0)
#define IO_RD7_ECU_2W_Toggle()   do { LATD7 = ~LATD7; } while(0)
#define IO_RD7_ECU_2W_GetValue()         PORTDbits.RD7
#define IO_RD7_ECU_2W_SetDigitalInput()    do { TRISD7 = 1; } while(0)
#define IO_RD7_ECU_2W_SetDigitalOutput()   do { TRISD7 = 0; } while(0)

#define IO_RD7_ECU_2W_SetPullup()    do { WPUD7 = 1; } while(0)
#define IO_RD7_ECU_2W_ResetPullup()   do { WPUD7 = 0; } while(0)


// get/set IO_RE0 aliases
#define IO_RE0_TRIS               TRISE0
#define IO_RE0_LAT                LATE0
#define IO_RE0_PORT               RE0
#define IO_RE0_WPU                WPUE0
#define IO_RE0_ANS                ANSE0
#define IO_RE0_SetHigh()    do { LATE0 = 1; } while(0)
#define IO_RE0_SetLow()   do { LATE0 = 0; } while(0)
#define IO_RE0_Toggle()   do { LATE0 = ~LATE0; } while(0)
#define IO_RE0_GetValue()         PORTEbits.RE0
#define IO_RE0_SetDigitalInput()    do { TRISE0 = 1; } while(0)
#define IO_RE0_SetDigitalOutput()   do { TRISE0 = 0; } while(0)

#define IO_RE0_SetPullup()    do { WPUE0 = 1; } while(0)
#define IO_RE0_ResetPullup()   do { WPUE0 = 0; } while(0)
#define IO_RE0_SetAnalogMode()   do { ANSE0 = 1; } while(0)
#define IO_RE0_SetDigitalMode()   do { ANSE0 = 0; } while(0)


// get/set IO_RE1 aliases
#define IO_RE1_TRIS               TRISE1
#define IO_RE1_LAT                LATE1
#define IO_RE1_PORT               RE1
#define IO_RE1_WPU                WPUE1
#define IO_RE1_ANS                ANSE1
#define IO_RE1_SetHigh()    do { LATE1 = 1; } while(0)
#define IO_RE1_SetLow()   do { LATE1 = 0; } while(0)
#define IO_RE1_Toggle()   do { LATE1 = ~LATE1; } while(0)
#define IO_RE1_GetValue()         PORTEbits.RE1
#define IO_RE1_SetDigitalInput()    do { TRISE1 = 1; } while(0)
#define IO_RE1_SetDigitalOutput()   do { TRISE1 = 0; } while(0)

#define IO_RE1_SetPullup()    do { WPUE1 = 1; } while(0)
#define IO_RE1_ResetPullup()   do { WPUE1 = 0; } while(0)
#define IO_RE1_SetAnalogMode()   do { ANSE1 = 1; } while(0)
#define IO_RE1_SetDigitalMode()   do { ANSE1 = 0; } while(0)


// get/set IO_RE2 aliases
#define IO_RE2_TRIS               TRISE2
#define IO_RE2_LAT                LATE2
#define IO_RE2_PORT               RE2
#define IO_RE2_WPU                WPUE2
#define IO_RE2_ANS                ANSE2
#define IO_RE2_SetHigh()    do { LATE2 = 1; } while(0)
#define IO_RE2_SetLow()   do { LATE2 = 0; } while(0)
#define IO_RE2_Toggle()   do { LATE2 = ~LATE2; } while(0)
#define IO_RE2_GetValue()         PORTEbits.RE2
#define IO_RE2_SetDigitalInput()    do { TRISE2 = 1; } while(0)
#define IO_RE2_SetDigitalOutput()   do { TRISE2 = 0; } while(0)

#define IO_RE2_SetPullup()    do { WPUE2 = 1; } while(0)
#define IO_RE2_ResetPullup()   do { WPUE2 = 0; } while(0)
#define IO_RE2_SetAnalogMode()   do { ANSE2 = 1; } while(0)
#define IO_RE2_SetDigitalMode()   do { ANSE2 = 0; } while(0)

extern unsigned char Speed,Speed_U, Speed_H,Speed_L;
extern unsigned char Speed_Work_Status, Seep_256ms_Cnt, Speed_rd;

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

void IOCIE_InterruptEnable(void);
unsigned char IsIOCIEEnable(void);
void IOCBN5_NegativeSet(void);

#endif // PIN_MANAGER_H
/**
 End of File
*/