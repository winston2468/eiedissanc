#ifndef __TRNG_H__
#define __TRNG_H__


/*! \file    TRNG.h
    \date    12:00
	\author  hgaharwa
	\brief    Contains include files, macros, and function declarations corresponding to the "SPE.c" file.
	\details  VDSP version used for testing		: CCES Cygnus M3 \n
			  Hardware used						: N/A \n
			  Connection details 				: N/A \n
			  Guidelines for change				: none
*/


/*************************************************************************************************************************
									Include Files
*************************************************************************************************************************/
#include <sys/platform.h>

/* macros to be used for configuring the mode of operation */

#define ready 			0x0
#define shutdown 		0x1<<1
#define stuck_out 		0x1<<2
#define noise_fail 		0x1<<3
#define run_fail 		0x1<<4
#define long_run_fail 	0x1<<5
#define poker_fail 		0x1<<6
#define monobit_fail 	0x1<<7

#define REG_TRNG1_V0                    0x200F0060         /* TRNG0 TRNG Post-Process aVa Value Registers */
#define REG_TRNG1_V1                    0x200F0064         /* TRNG0 TRNG Post-Process aVa Value Registers */
#define pREG_TRNG1_V0                    ((volatile uint32_t *)REG_TRNG1_V0)                     /* TRNG0 TRNG Post-Process aVa Value Registers */
#define pREG_TRNG1_V1                    ((volatile uint32_t *)REG_TRNG1_V1)                     /* TRNG0 TRNG Post-Process aVa Value Registers */

/*************************************************************************************************************************
									Function Declarations
**************************************************************************************************************************/

void Read_TRNG_Output1(int *destination,int SIZE);


void Write_TRNG_Input1(int *source, int SIZE);
void Write_TRNG_Input(int *iInput);
void Read_TRNG_Output(int *iOutput);


int Read_TRNG_Stat();


void Acknowledge_Interrupt(int iValue);


void Startup_Cycle_Number(int iValue);


void Enable_TRNG();


void Disable_TRNG();


void Enable_Post_Processing();



void Mask_Interrupt(int iValue);


void Enable_Test_Mode();


void Re_seed_TRNG();


void Min_Refill_Cycle(int iValue);


void Max_Refill_Cycle(int iValue);


void Sample_division(int iValue);


void Disbale_Read_Timeout();


void Set_Secure_Read_Timeout(int iValue);


void Set_Alarm_Threshold(int iValue);


void Stall_Run_poker();


void Set_Shutdown_Threshold(int iValue);


int Read_Shutdown_Count();


void Enable_FRO(int iValue);


void Disbale_FRO(int iValue);


void Detune_FRO(int iValue);


int Read_Alarm_Mask();


int Read_Alarm_Stop();



void Init_Key(int *Key);


void Init_V_Value(int *IV);

//  TRNG0 Test register
void FRO_Test_Select(int iValue);


void Test_Enable_Output();


void Enable_Test_Pattern_FRO();


void Enable_Test_Pattern_Detect();


void Continue_Poker_Test();


void Test_Run_Poker();


void Test_Post_Processor();
void Clear_Test_Post_Processor();


void Set_Test_Pattern(int iValue);




#endif


