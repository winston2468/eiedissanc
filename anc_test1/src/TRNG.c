/*************************************************************************************************************************
									File Details
**************************************************************************************************************************
	File name 							: TRNG.c
	Purpose								: Has definitions for various functions required to configure the TRNG
	Author								: Harshit Gaharwar
	VDSP version used for testing		: CCES Cygnus M3
	Hardware used						: N/A
	Connection details 					: N/A
	Guidelines for change				: None
**************************************************************************************************************************/

/*************************************************************************************************************************
									Include Files
*************************************************************************************************************************/
#include <sys/platform.h>
#include "TRNG.h"
#include "stdio.h"

/*************************************************************************************************************************
									Function Definitions
**************************************************************************************************************************/
void Read_TRNG_Output1(int *destination,int SIZE)
{
	int i=0;
		volatile int *temp, *temp1, temp2;
		temp=destination;
		temp1=(volatile int *)REG_TRNG0_OUTPUT0;
		temp2=SIZE;
		for(i=0;i<temp2/4; i++)
		{
			*temp=*temp1;
			temp1++;
			temp++;
		}
}

void Write_TRNG_Input1(int *source, int SIZE)
{
	int i=0;
	volatile int *temp, *temp1, temp2;
	temp=source;
	temp1=(volatile int *)REG_TRNG0_INPUT0;
	temp2=SIZE;
	for(i=0;i<temp2; i++)
	{
		*temp1=*temp;
		temp1++;
		temp++;
	}
}

void Write_TRNG_Input(int *iInput)
{
	int *iTemp;
	iTemp= iInput;
	*pREG_TRNG0_INPUT0=*iTemp;
#if defined(__ADSPBF70x__)
	iTemp++;
	*pREG_TRNG0_INPUT1=*iTemp;
#endif
}
void Read_TRNG_Output(int *iOutput)
{
	int *iTemp;
	iTemp= iOutput;
	*iTemp=*pREG_TRNG0_OUTPUT0;
	iTemp++;
	*iTemp=*pREG_TRNG0_OUTPUT1;
	iTemp++;
	*iTemp=*pREG_TRNG0_OUTPUT2;
	iTemp++;
	*iTemp=*pREG_TRNG0_OUTPUT3;
}

int Read_TRNG_Stat()
{
	int iTemp;
	iTemp= *pREG_TRNG0_STAT;
	return iTemp;
}

void Acknowledge_Interrupt(int iValue)
{
	*pREG_TRNG0_INTACK= (iValue & 0xFF);
}

void Startup_Cycle_Number(int iValue)
{
	*pREG_TRNG0_CTL |= iValue<<16;
}

void Enable_TRNG()
{
	*pREG_TRNG0_CTL |= 0x1<<10;
}

void Disable_TRNG()
{
	*pREG_TRNG0_CTL &= ~(0x1<<10);
}

void Enable_Post_Processing()
{
	*pREG_TRNG0_CTL |= 0x1<<12;
}


void Mask_Interrupt(int iValue)
{
	*pREG_TRNG0_CTL |= iValue;
}

void Enable_Test_Mode()
{
	*pREG_TRNG0_CTL |= 0x1<<8;
}

void Re_seed_TRNG()
{
	*pREG_TRNG0_CTL |=0x1<<15;
}

void Min_Refill_Cycle(int iValue)
{
	*pREG_TRNG0_CFG |= 	(iValue & 0xFF);
}

void Max_Refill_Cycle(int iValue)
{
	*pREG_TRNG0_CFG |= 	iValue<<16;
}

void Sample_division(int iValue)
{
	*pREG_TRNG0_CFG |= iValue<<8;  // iVlaue can be 0-15
}

void Disbale_Read_Timeout()
{
	*pREG_TRNG0_CFG &= ~(0x1<<12);
}

void Set_Secure_Read_Timeout(int iValue)
{
	*pREG_TRNG0_CFG |= iValue<<12;  //iVlaue can be 1-15
}

void Set_Alarm_Threshold(int iValue)
{
	*pREG_TRNG0_ALMCNT |= iValue; // any value from 1 to 255
}

void Stall_Run_poker()
{
	*pREG_TRNG0_ALMCNT |= 0x1<<15;
}

void Set_Shutdown_Threshold(int iValue)
{
	*pREG_TRNG0_ALMCNT |= iValue<<16; // any value between 1 to 32
}

int Read_Shutdown_Count()
{
	int iTemp;
	iTemp= *pREG_TRNG0_ALMCNT;
	iTemp= iTemp>>24;
	return iTemp;   //should return 1 to 8
}

void Enable_FRO(int iValue)
{
	*pREG_TRNG0_FROEN |= iValue;
}

void Disbale_FRO(int iValue)
{
	*pREG_TRNG0_FROEN &= ~ iValue;
}

void Detune_FRO(int iValue)
{
	*pREG_TRNG0_FRODETUNE |= iValue;
}

int Read_Alarm_Mask()
{
	int iTemp;
	iTemp= *pREG_TRNG0_ALMMSK;
	return iTemp;
}

int Read_Alarm_Stop()
{
	int iTemp;
	iTemp= *pREG_TRNG0_ALMSTP;
	return iTemp;
}


void Init_Key(int *Key)
{
	int *iTemp;
	iTemp= Key;
#if defined(__ADSPBF70x__)
	*pREG_TRNG0_KEY0=*iTemp;
	iTemp++;
	*pREG_TRNG0_KEY1=*iTemp;
	iTemp++;
	*pREG_TRNG0_KEY2=*iTemp;
	iTemp++;
	*pREG_TRNG0_KEY3=*iTemp;
	iTemp++;
	*pREG_TRNG0_KEY4=*iTemp;
	iTemp++;
	*pREG_TRNG0_KEY5=*iTemp;
#endif
}

void Init_V_Value(int *IV)
{
	int *iTemp;
	iTemp= IV;
	*pREG_TRNG1_V0=*iTemp;
	iTemp++;
	*pREG_TRNG1_V1=*iTemp;
}
//  TRNG0 Test register
void FRO_Test_Select(int iValue)
{
	*pREG_TRNG0_TEST |= iValue<<8;
}

void Test_Enable_Output()
{
	*pREG_TRNG0_TEST |= 0x1;
}

void Enable_Test_Pattern_FRO()
{
	*pREG_TRNG0_TEST |= 0x1<<1;
}

void Enable_Test_Pattern_Detect()
{
	*pREG_TRNG0_TEST |= 0x1<<2;
}

void Continue_Poker_Test()
{
	*pREG_TRNG0_TEST |= 0x1<<4;
}

void Test_Run_Poker()
{
	*pREG_TRNG0_TEST |= 0x1<<5;
}

void Test_Post_Processor()
{
	*pREG_TRNG0_TEST |= 0x1<<6;
}

void Clear_Test_Post_Processor()
{
	*pREG_TRNG0_TEST |= 0x1<<6;
}

void Set_Test_Pattern(int iValue)
{
	*pREG_TRNG0_TEST |= iValue<<16;
}






