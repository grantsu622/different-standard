/*
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB(c) Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"

#define LED_EN  1
#define  AUTORUN   0 
#define MOTORRETURN     0 

//差速器馬達狀態
#define	Motor1_Status_2WD_1		0b00001001		//前差RA0/WG,RA1/L,RA3/Y
#define	Motor1_Status_4WD_1 	0b00001000		//前差RA0/WG,RA1/L,RA3/Y
#define	Motor1_Status_4WD_2 	0b00000000		//前差RA0/WG,RA1/L,RA3/Y
#define	Motor1_Status_4WL_1 	0b00000001		//前差RA0/WG,RA1/L,RA3/Y

//檔位狀態
#define Position_Status_2WD 0b00000000  //RC0/WB, RC1/WL, RC2/WR
#define Position_Status_4WD 0b00000100  //RC0/WB, RC1/WL, RC2/WR
#define Position_Status_4WDL 0b00000110  //RC0/WB, RC1/WL, RC2/WR

//差速器馬達狀態(前4bits) + 檔位狀態(後3bits) + MSB(Lock(0) or UNLOCK(1))
#define Status_2WD 0b10001001   //0x19
#define Status_2WDL 0b00001001  //0x09
#define Status_4WD_1 0b01001000 //0x48
#define Status_4WD_2 0b01000000 //0x40
#define Status_4WDL 0b01100001  //0x31

//把手訊號
#define _4WDLOCK_1				0b00110010 			    //0b00010010                   
#define _4WDLOCK_2				0b00110010 			    //0b00110010                   								
#define _4WD_1					0b00110001  			//0b00010001                  
#define _4WD_2					0b00110001			    //0b00110001                    
#define _2WDLOCK				0b00100011			    //0b00000011                    
#define _2WD					0b00010011 			    //0b00100011                   

//Error
unsigned char Work_status = 0;
unsigned char Error_Flag = 0;	//5秒Time out旗標
unsigned char Error_Mode = 0;
unsigned char Over_Speed_Error = 0;
unsigned char Front_Error	= 0;
unsigned char Back_Error = 0;
unsigned char Compare_Error = 0;
unsigned char IsFistLEDFlash = 0;   //1:is fist, 0:not fist
unsigned char IsLEDErrorChang = 0;   //1:is Error Change, 0:not Change
unsigned char LEDError_Status = 0;

//Pull 
#define Pull_Count_Val  39	//5秒
#define PULL_VALUE  3   // 拉動次數
unsigned char Pull_Error = 0;
unsigned char Pull = 0;
unsigned char Pull_5S_CNT = Pull_Count_Val;
unsigned char Pull_Count = 0;


//RPM
unsigned char RPM_Zero = 0 ;
unsigned char RPM_Flag = 0;

//Motor 
unsigned char Motor_Front_Status = 0;
unsigned char Motor_Back_Status	= 0;
unsigned char Motor_Status_Now = 0;
unsigned char Motor_OLD_Status = 0;
unsigned char Motor_Temp = 0;
unsigned char Motor_Remove = 0;
unsigned char Special = 0; //Motor error check
unsigned char Moving_Status;
unsigned char Is2WDLOCK = 0;

//Gear (齒輪) Hand status
unsigned int Hand_Status_NEW;
unsigned int Hand_Status_OLD = 0;

//Position (檔位)
unsigned int Position_Status_NEW;
unsigned int Position_Status_OLD = 0;
unsigned char Position_Temp = 0;
unsigned char Error_Position = 0;
unsigned char Position_Status = 0;

//Hand feedback
unsigned char	Handback_Error = 0;			//把手失效錯誤

// Voltage
unsigned char Voltage_Error = 0;

//LED 
unsigned int LED1_Count = 0;
unsigned int LED2_Count = 0;

//timer
#define _1S_Val	8
#define _1_5S_Val	12
unsigned char _5S_CNT = _1S_Val;								

//Speed
unsigned char Speed = 30,Speed_U = 0,Speed_H = 0,Speed_L = 0;
unsigned char Speed_Work_Status = 0,Seep_256ms_Cnt = 2,Speed_rd = 0;
unsigned char tmp,error_cnt = 3;


//sub-function
void Check_Motor_Status(void);
void Check_Hand_Status(void);
void LED1_Flash(unsigned int Time);
void ECU_2WD_4WD_Flash(unsigned int Time);
void ECU_2WD_LOCK_Flash(unsigned int Time);
void ECU_4WD_LOCK_Flash(unsigned int Time);
void ECU_2WD_2WL_Flash(unsigned int Time);
void ECU_4WD_2WL_Flash(unsigned int Time);
void ECU_LOCK_2WL_Flash(unsigned int Time);
void Error_Mode_Func(unsigned char New_Status,unsigned char Old_Status);
void Change_Func(unsigned char Goto,unsigned char Old_Status);
void Output_ECU(void);
void Check_Status(void);
void position_2WD_Error_Flash(void);
void position_4WD_Error_Flash(void);
void position_4WDL_Error_Flash(void);
void Error_Mode_ReturnMotor(unsigned char Old_Status, unsigned char New_Status);

//================================================================================================
//  寫入燒入程式的版本  
//  EEPROM data  [NOP]  [年]  [年]  [月]  [日] [版次] [NOP]  [NOP]
//================================================================================================
//__EEPROM_DATA(0xAB, 0x20, 0x15, 0x03, 0x24, 0x01, 0xFF, 0xFF);

/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    IO_RC7_SetHigh();

#if 0
    Voltage_Error = IsVoltageError();
    while(1);
#endif
    Check_Motor_Status();
#if(!AUTORUN)
	Check_Hand_Status();
	switch (Hand_Status_NEW)
	{
		case _4WDLOCK_1:
				Handback_Error = 0;
		case _2WDLOCK:
				Handback_Error = 0;
		case _4WD_1:
				Handback_Error = 0;
		case _2WD:
				Handback_Error = 0;
		default:
				Handback_Error = 1;
	}
#endif	
	if ( Error_Mode == 1)
	{
		Special = 1;
		Hand_Status_NEW = _2WD;
	}
    
    while (1)
    {
        // Add your application code
        IOCBN5_NegativeSet();
        
#if(!AUTORUN)
        //LED1_Flash(8);
#endif
        if (Special == 1)	//開機第一次會做
		{
			Special = 0;
		}
		else
		{	
#if(!AUTORUN)
			Check_Hand_Status();
#else
            if (LED1_Count >= 24)
            {
                LED1_Count = 0;
                switch(Hand_Status_OLD)
                {
                    case _4WDLOCK_1:
                        Hand_Status_NEW = _4WD_1;
                        break;
                    case _4WD_1:
                        Hand_Status_NEW = _2WDLOCK;
                        break;
                    case _2WDLOCK:
                        Hand_Status_NEW = _2WD;
                        break;
                    case _2WD:
                        Hand_Status_NEW = _4WDLOCK_1;
                        break;
                }
            }
            
#endif // end of AUTORUN
			Check_Motor_Status();
            //Check_Position_Status();
            
		}

        if (Speed_rd)
        {
            Speed_rd = 0;
            TMR0_InterruptDisable();
            Speed = (Speed_U * 100) + (Speed_H * 10) + Speed_L;
            NOP();
            NOP();

        }

		Voltage_Error = IsVoltageError();
        
        while (Voltage_Error == 1) //等待電壓正常
        {
		    Voltage_Error = IsVoltageError();
        }
        //Speed=3;
        //if (Speed < 30)
        //    while(1);
        if((Speed < 15)&&(Voltage_Error == 0) )	
        {	
            if (Pull_Error == 1 && Pull_Count == PULL_VALUE)
            {
                if (Pull == 1)
                {
#if(MOTORRETURN)
                    Error_Mode_ReturnMotor(Moving_Status, Hand_Status_NEW);
#endif
                    Pull_Count ++;
                    Pull_5S_CNT = Pull_Count_Val;
                }
                
            }
            if(Pull_Error == 1 && Pull_Count < PULL_VALUE)												//錯誤模式下
			{	
                LED1_Flash(8);
                if( Pull ==1)
				{	
					Pull_Count ++;
#if(MOTORRETURN)
					switch(Hand_Status_NEW)
					{
						case _4WDLOCK_1:
								 Error_Mode_Func(_4WDLOCK_1, Moving_Status);	
						break;
						case _2WDLOCK:
								 Error_Mode_Func(_2WDLOCK, Moving_Status);	
						break;
						case _4WD_1:
								 Error_Mode_Func(_4WD_1, Moving_Status);	
						break;
						case _2WD:
								 Error_Mode_Func(_2WD, Moving_Status);	
						break;		
					}
#endif
					Pull_5S_CNT = Pull_Count_Val;
				}
				
            }
			
            if (Hand_Status_NEW != Hand_Status_OLD)			 //把手狀態
            {
                Pull_Error = 0;
                Pull_Count = 0;
                Pull_5S_CNT = Pull_Count_Val;
                
                switch (Hand_Status_NEW)
                {
                    case _4WDLOCK_1:
                        if (Speed < 3)
                        {
                            Change_Func(_4WDLOCK_1,Hand_Status_OLD);
                            Hand_Status_OLD = Hand_Status_NEW;
                        }
                        break;
                    case _2WDLOCK:
                        if (Speed < 3)
                        {
                            Change_Func(_2WDLOCK, Hand_Status_OLD);
                            Hand_Status_OLD = Hand_Status_NEW;
                        }
                        break;
                    case _4WD_1:
                        if (Speed < 15)
                        {
                            Change_Func(_4WD_1, Hand_Status_OLD);
                            Hand_Status_OLD = Hand_Status_NEW;
                        }
                        break;
                    case _2WD:
                        if (Speed < 15)
                        {
                            Change_Func(_2WD, Hand_Status_OLD);
                            Hand_Status_OLD = Hand_Status_NEW;
                        }
                        break;
                }
            }
            //Compare_Motor_Position();
            Check_Status();
            Output_ECU();
        } 
        
    }
}

/******************************************************************************
*    Check_Motor_Status and Position status
*    IO_RA1_L_Signal_GetValue = LOW
*    IO_RC0_WB_Signal_GetValue = LOW
******************************************************************************/
void Check_Motor_Status(void)
{	
    Motor_Temp = 0;

    Motor_Front_Status = (IO_RA0_WG_Signal_GetValue() << 0) | (IO_RA1_L_Signal_GetValue() << 1) 
                            | (IO_RA3_Y_Signal_GetValue() << 3);
    
    Position_Status = (IO_RC0_WB_Signal_GetValue() << 0) | (IO_RC1_WL_Signal_GetValue() << 1) 
                            | (IO_RC2_WR_Signal_GetValue() << 2);
    
    Motor_Temp = Motor_Front_Status | (Position_Status << 4) | (IO_RD6_RELAY_GetValue() << 7); 
    
    if (Pull_Error) 
    {
        Error_Mode = 1;
        return;
    }

    switch( Motor_Temp)
    {
        case Status_2WD:	
            Hand_Status_OLD = _2WD;
            Error_Mode = 0;
            break;
        case Status_4WD_1:
            Hand_Status_OLD = _4WD_1;
            Error_Mode = 0;
            break;
        case Status_4WD_2:
            Hand_Status_OLD = _4WD_1;
            Error_Mode = 0;
            break;
        case Status_4WDL:
            Hand_Status_OLD = _4WDLOCK_1;
            Error_Mode = 0;
            break;
        case Status_2WDL:
            Hand_Status_OLD = _2WDLOCK;
            Error_Mode = 0;
            break;
        default:
            Error_Mode = 1;
    }
}


/******************************************************************************
*    Check_Position_Status (檔位)
******************************************************************************/
void Check_Position_Status(void)
{
    Position_Temp = 0;

    Position_Temp = (IO_RC0_WB_Signal_GetValue() << 0) || (IO_RC1_WL_Signal_GetValue() << 1) 
                            || (IO_RC2_WR_Signal_GetValue() << 3); 

    switch( Position_Temp)
    {
        case Position_Status_2WD:	
            Position_Status_OLD = _2WD;
            Error_Position = 0;
            break;
        case Position_Status_4WD:
            Position_Status_OLD = _4WD_1;
            Error_Position = 0;
            break;
        case Position_Status_4WDL:
            Position_Status_OLD = _4WDLOCK_1;
            Error_Position = 0;
            break;
        default:
            Error_Position = 1;
    }
}

/******************************************************************************
*    Check_Hand_Status
******************************************************************************/
void Check_Hand_Status(void)
{
    unsigned char Loop = 1, k = 3;
	do
	{	
        Delay_128msec(1);
		Hand_Status_NEW = (IO_RD0_HAND_4WL_GetValue() << 0) | (IO_RD1_HAND_4W_GetValue() << 1) 
							| (IO_RD4_HAND_2WL_GetValue() << 4) | (IO_RD5_HAND_2W_GetValue() << 5);
		if(Hand_Status_NEW == _4WDLOCK_2)
			Hand_Status_NEW = _4WDLOCK_1;
		if(Hand_Status_NEW == _4WD_2)
			Hand_Status_NEW = _4WD_1;
			
		switch(Hand_Status_NEW)
		{
					case _4WDLOCK_1:
					case _2WDLOCK:
					case _4WD_1:
					case _2WD:
							 Handback_Error = 0;
							 Loop = 0;
							 break;
					default:
							 Handback_Error = 1;
							 k--;
							 if( k== 0)
							 {	
							   Loop = 0;
							 }	
							 
		}
	}
	while(Loop == 1);
}


/******************************************************************************
*    LED1_FLASH
******************************************************************************/
void LED1_Flash(unsigned int Time)
{	
	if(LED1_Count >= Time)
	{	
        LED1_Count =0;
        IO_RC7_Toggle();
	}
}

/******************************************************************************
*   2WD_4WD flash 
******************************************************************************/
void ECU_2WD_4WD_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RD7_ECU_2W_Toggle();
        IO_RB0_ECU_4W_Toggle();
	}
}

/******************************************************************************
*   2WD_4WDLOCK flash 
******************************************************************************/
void ECU_2WD_LOCK_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RD7_ECU_2W_Toggle();
        IO_RB1_ECU_4WL_Toggle();
	}
}

/******************************************************************************
*   2WD_2WL flash 
******************************************************************************/
void ECU_2WD_2WL_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RD7_ECU_2W_Toggle();
        IO_RB2_ECU_2WL_Toggle(); 
	}
}
/******************************************************************************
*   4WD_4WDLOCK flash 
******************************************************************************/
void ECU_4WD_LOCK_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RB0_ECU_4W_Toggle();
        IO_RB1_ECU_4WL_Toggle();
	}
}

/******************************************************************************
*   4WD_2WL flash 
******************************************************************************/
void ECU_4WD_2WL_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RB0_ECU_4W_Toggle();
        IO_RB2_ECU_2WL_Toggle(); 
	}
}

/******************************************************************************
*   4WDLOCK_2WL flash 
******************************************************************************/
void ECU_LOCK_2WL_Flash(unsigned int Time)
{	
	if(LED2_Count >= Time)
	{	
        LED2_Count =0;
        IO_RB1_ECU_4WL_Toggle();
        IO_RB2_ECU_2WL_Toggle(); 
	}
}

/******************************************************************************
*    Motor1_F 馬達正轉
******************************************************************************/
void Motor1_F(void)
{
    IO_RB4_M1F_SetHigh();
}

/******************************************************************************
*    Motor1_R 馬達反轉
******************************************************************************/
void Motor1_R(void)
{
    IO_RB3_M1R_SetHigh();
}

/******************************************************************************
*    Motor1_S 馬達停止
******************************************************************************/
void Motor1_S(void)
{
    IO_RB4_M1F_SetLow();
    IO_RB3_M1R_SetLow(); 
}

/******************************************************************************
*    Error Exit
******************************************************************************/
void Error_Exit_Func(void)
{
    Motor1_S();
    Work_status = 0;
    Error_Flag = 0;
}

/******************************************************************************
*   Error model return motor status
******************************************************************************/
void Error_Mode_ReturnMotor(unsigned char Old_Status, unsigned char New_Status)
{
    switch (Old_Status)
    {
        case _2WD:
            if ((New_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
                Front_Error = 0;
                Error_Exit_Func();
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetHigh(); //後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            
            break;
        case _2WDLOCK:
            if ((New_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
                Front_Error = 0;
                Error_Exit_Func();
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetLow(); //後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            
            break;
        case _4WD_1:
            if(IO_RA3_Y_Signal_GetValue() == 1)
            {
                while(IO_RA3_Y_Signal_GetValue() == 1)
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                    }
                }
                IO_RD6_RELAY_SetLow(); //relay後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            // 4WD Lock -> 4WD        
            else if (IO_RA3_Y_Signal_GetValue() == 0)
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                    }
                }
                IO_RD6_RELAY_SetLow(); //後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            break;
        case _4WDLOCK_1:
            if ((New_Status == _2WD) || (New_Status == _2WDLOCK) )
            {
                while((IO_RA3_Y_Signal_GetValue() == 1) )
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }
                Front_Error = 0;
                Error_Exit_Func();
                 
            }
            
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_F();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetLow(); //後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
    }
}
/******************************************************************************
*   Error model 
******************************************************************************/
void Error_Mode_Func(unsigned char New_Status, unsigned char Old_Status)
{
    //Moving_Status = Old_Status;
    //_5S_CNT = _1S_Val;															
    _5S_CNT = _1_5S_Val;															
    Work_status = 1;
    Voltage_Error = IsVoltageError();
    Front_Error = 0 ;

#if(MOTORRETURN)
    Error_Mode_ReturnMotor(Moving_Status, New_Status);
#endif //end of MOTOR RETURN

    switch (New_Status)
    {
        case _4WDLOCK_1:
            if ((Old_Status == _2WD) || (Old_Status == _2WDLOCK) )
            {
                while((IO_RA3_Y_Signal_GetValue() == 1) )
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }
                Front_Error = 0;
                Error_Exit_Func();
                 
            }
            
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_F();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetLow(); //relay後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
        case _4WD_1:
            // 2WD -> 4WD        
            if(IO_RA3_Y_Signal_GetValue() == 1)
            {
                while(IO_RA3_Y_Signal_GetValue() == 1)
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                    }
                }
                IO_RD6_RELAY_SetLow(); //relay後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            // 4WD Lock -> 4WD        
            else if (IO_RA3_Y_Signal_GetValue() == 0)
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                    }
                }
                IO_RD6_RELAY_SetLow(); //後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            break;
        case _2WDLOCK:
            if ((Old_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
                Front_Error = 0;
                Error_Exit_Func();
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetLow(); //後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
        case _2WD:
            if ((Old_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
                Front_Error = 0;
                Error_Exit_Func();
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                }
            }
            IO_RD6_RELAY_SetHigh(); //後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
    }
}

/******************************************************************************
*   Normal model 
*   Goto: 把手切換位置
*   Status: 馬達位置
******************************************************************************/
void Change_Func(unsigned char Goto,unsigned char Old_Status)
{
    Moving_Status = Old_Status;
    LEDError_Status = Old_Status;

    //_5S_CNT = _1S_Val;
    _5S_CNT = _1_5S_Val;														
    Work_status = 1;
    Voltage_Error = IsVoltageError();
    Front_Error = 0 ;

    switch (Goto)
    {
        case _4WDLOCK_1:
            if ((Old_Status == _2WD) || (Old_Status == _2WDLOCK) )
            {
                while((IO_RA3_Y_Signal_GetValue() == 1) )
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }
                 
            }
           
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_F();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                    return;
                }
            }
            
            IO_RD6_RELAY_SetLow(); //relay後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
        case _4WD_1:
            // 2WD -> 4WD        
            if(IO_RA3_Y_Signal_GetValue() == 1)
            {
                while(IO_RA3_Y_Signal_GetValue() == 1)
                {
                    Motor1_F();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }
                IO_RD6_RELAY_SetLow(); //relay後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            // 4WD Lock -> 4WD        
            else if (IO_RA3_Y_Signal_GetValue() == 0)
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }
                IO_RD6_RELAY_SetLow(); //relay後差訊號
                Front_Error = 0;
                Error_Exit_Func();
            }
            break;
        case _2WDLOCK:
            if ((Old_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                    return;
                }
            }
            IO_RD6_RELAY_SetLow(); //relay後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
        case _2WD:
            if ((Old_Status == _4WDLOCK_1) )
            {
                while(IO_RA3_Y_Signal_GetValue() == 0)
                {
                    Motor1_R();
                    if (Error_Flag == 1)
                    {
                        Front_Error = 1;
                        Error_Exit_Func();
                        return;
                    }
                }   
            }
            while(IO_RA0_WG_Signal_GetValue() == 0)
            {
                Motor1_R();
                if (Error_Flag == 1)
                {
                    Front_Error = 1;
                    Error_Exit_Func();
                    return;
                }
            }
            IO_RD6_RELAY_SetHigh(); //relay後差訊號
            Front_Error = 0;
            Error_Exit_Func();
            break;
    }
}

/******************************************************************************
*   compare Motor_Status and Position
******************************************************************************/
void Compare_Motor_Position(void)
{
    if ((Motor_Temp == Motor1_Status_2WD_1) && 
            (Position_Temp == Position_Status_2WD))
    {
        Compare_Error = 0;
    }
    else if (((Motor_Temp == Motor1_Status_4WD_1) 
                || (Motor_Temp == Motor1_Status_4WD_2))
                && (Position_Temp == Position_Status_4WD))
    {
        Compare_Error = 0;
    }
    else if ((Motor_Temp == Motor1_Status_4WL_1) && 
            (Position_Temp == Position_Status_4WDL))
    {
        Compare_Error = 0;
    }
    else
    {
        Compare_Error = 1;
    }
}

/******************************************************************************
*   Output gpio to ECU 
******************************************************************************/
void Output_ECU(void)
{
    if (Error_Mode == 0)
    {
        IsFistLEDFlash = 1;
        //IsLEDErrorChange = 0;
    }

    if(Handback_Error == 1)
    {
        IO_RD7_ECU_2W_SetHigh(); 
        IO_RB0_ECU_4W_SetHigh(); 
        IO_RB1_ECU_4WL_SetHigh(); 
        IO_RB2_ECU_2WL_SetHigh(); 

    }
    else if (Error_Mode == 1)
    {
        switch(Position_Status)
        {
            case Position_Status_2WD:
                position_2WD_Error_Flash();
           
                break;
            case Position_Status_4WD:
                position_4WD_Error_Flash();
               break;
            case Position_Status_4WDL:
                position_4WDL_Error_Flash();
                
                break;
        }

    }
    else if(Motor_Temp == Status_2WDL)
    {
        IO_RD7_ECU_2W_SetHigh(); 
        IO_RB0_ECU_4W_SetHigh(); 
        IO_RB1_ECU_4WL_SetHigh(); 
        IO_RB2_ECU_2WL_SetLow(); 
        Is2WDLOCK = 1;
    }
    else if(Motor_Temp == Status_2WD)
    {
        IO_RD7_ECU_2W_SetLow(); 
        IO_RB0_ECU_4W_SetHigh(); 
        IO_RB1_ECU_4WL_SetHigh(); 
        IO_RB2_ECU_2WL_SetHigh(); 
        Is2WDLOCK = 0;
    }
    else if((Motor_Temp == Status_4WD_1) || (Motor_Temp == Status_4WD_2))
    {
        IO_RD7_ECU_2W_SetHigh(); 
        IO_RB0_ECU_4W_SetLow(); 
        IO_RB1_ECU_4WL_SetHigh(); 
        IO_RB2_ECU_2WL_SetHigh(); 
    }
    else if(Motor_Temp == Status_4WDL)
    {
        IO_RD7_ECU_2W_SetHigh(); 
        IO_RB0_ECU_4W_SetHigh(); 
        IO_RB1_ECU_4WL_SetLow(); 
        IO_RB2_ECU_2WL_SetHigh(); 
    }
}

/******************************************************************************
*   Check status for Motor and Hand
******************************************************************************/
void Check_Status(void)
{
    
    switch(Hand_Status_NEW)
    {
        case _4WDLOCK_1:
            if((Motor_Temp == Status_4WDL))
            {
                Error_Mode = 0;
                Pull_Error = 0;
                //Hand_Status_OLD = Hand_Status_NEW;
                //Pull_Count = 0;
                return;
            }
            Error_Mode = 1;
            Pull_Error = 1;
            break;
        case _2WDLOCK: 
            if(Motor_Temp == Status_2WDL) 
            {
                Error_Mode = 0;
                Pull_Error = 0;
                //Hand_Status_OLD = Hand_Status_NEW;
                //Pull_Count = 0;
                return;
            }
            Error_Mode = 1;
            Pull_Error = 1;
            break;
        case _4WD_1:
            if((Motor_Temp == Status_4WD_1) || (Motor_Temp == Status_4WD_2))
            {
                Error_Mode = 0;
                Pull_Error = 0;
                //Hand_Status_OLD = Hand_Status_NEW;
                //Pull_Count = 0;
                return;
            }
            Error_Mode = 1;
            Pull_Error = 1;
            break;
        case _2WD:
            if(Motor_Temp == Status_2WD)
            {
                Error_Mode = 0;
                Pull_Error = 0;
                //Hand_Status_OLD = Hand_Status_NEW;
                //Pull_Count = 0;
                return;
            }
            Error_Mode = 1;
            Pull_Error = 1;
            break;
        default:
            Error_Mode = 1;
            Pull_Error = 1;
    }
}
/******************************************************************************
*  Position 2WD Error Flash
*  檔位-2WD與馬達位置不符合
*  
******************************************************************************/
void position_2WD_Error_Flash(void)
{
    //2WD->4WDL, 2WD->4WD<->4WDL
    if (( Hand_Status_NEW == _4WDLOCK_1 && (Moving_Status == _2WD || (Moving_Status == _4WD_1 && Is2WDLOCK == 0))))
    {
        if (IsFistLEDFlash)
        {
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            
            IO_RD7_ECU_2W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WDLOCK || LEDError_Status == _4WD_1 || LEDError_Status == _4WD_2)
        {
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            IO_RD7_ECU_2W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            LEDError_Status = 0; 
        }
        ECU_2WD_LOCK_Flash(4);
    }
    //2WDLOCK->4WDL, 2WDLOCK->4WD<->4WDL
    else if (( Hand_Status_NEW == _4WDLOCK_1 && (Moving_Status == _2WDLOCK || (Moving_Status == _4WD_1 && Is2WDLOCK == 1))))
    {
        if (IsFistLEDFlash)
        {
            IO_RD7_ECU_2W_SetHigh();
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            
            IO_RB1_ECU_4WL_SetHigh(); 
            IO_RB2_ECU_2WL_SetLow(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WD || LEDError_Status == _4WD_1 || LEDError_Status == _4WD_2)
        {
            IO_RD7_ECU_2W_SetHigh();
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            IO_RB1_ECU_4WL_SetHigh(); 
            IO_RB2_ECU_2WL_SetLow(); 
            LEDError_Status = 0; 
        }
        ECU_LOCK_2WL_Flash(4);
    }
    //2WD->4WD, 2WD->4WD<->4WDL
    else if ( Hand_Status_NEW == _4WD_1 && (Moving_Status == _2WD || (Moving_Status == _4WDLOCK_1 && Is2WDLOCK == 0)))
    {
        if (IsFistLEDFlash)
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            
            IO_RD7_ECU_2W_SetLow(); 
            IO_RB0_ECU_4W_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WDLOCK || LEDError_Status == _4WD_1 
                || LEDError_Status == _4WD_2 || LEDError_Status == _4WDLOCK_1)
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            
            IO_RD7_ECU_2W_SetLow(); 
            IO_RB0_ECU_4W_SetHigh(); 
            LEDError_Status = 0;
        }
        ECU_2WD_4WD_Flash(4);
    }
    //2WDLOCK->4WD, 2WDLOCK->4WD<->4WDL
    else if ( Hand_Status_NEW == _4WD_1 && (Moving_Status == _2WDLOCK || (Moving_Status == _4WDLOCK_1 && Is2WDLOCK == 1)))
    {
        if (IsFistLEDFlash)
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RD7_ECU_2W_SetHigh();
            
            IO_RB2_ECU_2WL_SetLow();
            IO_RB0_ECU_4W_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _4WDLOCK_1 || LEDError_Status == _4WD_1 
                || LEDError_Status == _4WD_2 || LEDError_Status == _2WDLOCK)
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RD7_ECU_2W_SetHigh();
            
            IO_RB2_ECU_2WL_SetLow();
            IO_RB0_ECU_4W_SetHigh(); 
            LEDError_Status = 0;
        }
        ECU_4WD_2WL_Flash(4);
    }
}
/******************************************************************************
*  Position 4WD Error Flash
*  檔位-4WD與馬達位置不符合
******************************************************************************/
void position_4WD_Error_Flash(void)
{
    //4WD->2WD
    //if((Motor_Front_Status == Motor1_Status_2WD_1) && 
    //        (Hand_Status_NEW == _2WD))//M:2WD,P:4WD,H:2WD
    if(Hand_Status_NEW == _2WD)
    {
        if (IsFistLEDFlash)
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
                    
            IO_RD7_ECU_2W_SetHigh(); 
            IO_RB0_ECU_4W_SetLow(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WDLOCK || LEDError_Status == _4WDLOCK_1 )
        {
            IO_RB1_ECU_4WL_SetHigh(); //初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            IO_RD7_ECU_2W_SetHigh(); 
            IO_RB0_ECU_4W_SetLow(); 
            LEDError_Status = 0;
        }
        
        ECU_2WD_4WD_Flash(4);
    }
    //4WD->4WDL
    //else if (Motor_Front_Status == Motor1_Status_4WL_1 && ( Hand_Status_NEW == _4WDLOCK_1))//M:4WDL,P:4WD,H:4WDL
    else if (Hand_Status_NEW == _4WDLOCK_1)
    {
        if (IsFistLEDFlash)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WD || LEDError_Status == _2WDLOCK || LEDError_Status == _4WDLOCK_1)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            LEDError_Status = 0;
        }
        ECU_4WD_LOCK_Flash(4);
    }
    //4WD->2WDLOCK
    else if ( Hand_Status_NEW == _2WDLOCK)//M:2WD,P:4WD,H:2WL
    {
        if (IsFistLEDFlash)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB1_ECU_4WL_SetHigh();
            
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB2_ECU_2WL_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WD || LEDError_Status == _2WDLOCK || LEDError_Status == _4WDLOCK_1)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB1_ECU_4WL_SetHigh();
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB2_ECU_2WL_SetHigh(); 
            LEDError_Status = 0;
        }
        
        ECU_4WD_2WL_Flash(4);
    }
} 
/******************************************************************************
*  Position 4WDL Error Flash
*  檔位-4WDL與馬達位置不符合
******************************************************************************/
void position_4WDL_Error_Flash(void)
{
    //4WDL->2WD
    if ( Hand_Status_NEW == _2WD)
    {
        if (IsFistLEDFlash)
        {
            IO_RB2_ECU_2WL_SetHigh(); //初始化其他燈號
            IO_RB0_ECU_4W_SetHigh(); 
            
            IO_RD7_ECU_2W_SetHigh(); 
            IO_RB1_ECU_4WL_SetLow(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WDLOCK 
                || (LEDError_Status == _4WD_1 || LEDError_Status == _4WD_2 ))
        {
            IO_RB2_ECU_2WL_SetHigh(); //初始化其他燈號
            IO_RB0_ECU_4W_SetHigh(); 
            IO_RD7_ECU_2W_SetHigh(); 
            IO_RB1_ECU_4WL_SetLow(); 
            LEDError_Status = 0;
        }
        ECU_2WD_LOCK_Flash(4);
    }
    //4WDL->4WD
    //else if(((Motor_Front_Status == Motor1_Status_4WD_1) 
    //            || (Motor_Front_Status == Motor1_Status_4WD_2))
    //            && (Hand_Status_NEW == _4WD_1))
    else if (Hand_Status_NEW == _4WD_1)
    {
        if (IsFistLEDFlash)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WD || LEDError_Status == _2WDLOCK)
        {
            IO_RD7_ECU_2W_SetHigh();//初始化其他燈號
            IO_RB2_ECU_2WL_SetHigh();
            IO_RB0_ECU_4W_SetLow(); 
            IO_RB1_ECU_4WL_SetHigh(); 
            LEDError_Status = 0;
        }

        ECU_4WD_LOCK_Flash(4);
    }
    //4WDL->2WDL
    else if ( Hand_Status_NEW == _2WDLOCK)//M:2WD,P:4WDL,H:2WL
    {
        if (IsFistLEDFlash)
        {
            IO_RD7_ECU_2W_SetHigh();
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            
            IO_RB1_ECU_4WL_SetLow(); 
            IO_RB2_ECU_2WL_SetHigh(); 
            IsFistLEDFlash = 0;
        }
        if ( LEDError_Status == _2WD || (LEDError_Status == _4WD_1 || LEDError_Status == _4WD_2))
        {
            IO_RD7_ECU_2W_SetHigh();
            IO_RB0_ECU_4W_SetHigh(); //初始化其他燈號
            IO_RB1_ECU_4WL_SetLow(); 
            IO_RB2_ECU_2WL_SetHigh(); 
            LEDError_Status = 0;
        }
        ECU_LOCK_2WL_Flash(4);
    }
}
/**
 End of File
*/
