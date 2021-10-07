#ifndef __PKIC_H__
#define __PKIC_H__

#include <sys/platform.h>

/*************************************************************************************************************************
									Function Declarations
**************************************************************************************************************************/

void Set_PKIC_Polarity(int iValue);

void Set_PKIC_Level_type(int iValue);


void Enable_TRNG_Interrupt();


void Disable_TRNG_Interrupt();

void Enable_PKIC_Interrupt(int iValue);


void Disable_PKIC_Interrupt(int iValue);



int Read_PKIC_Unmasked_Interrupt_Source();


int Read_PKIC_Masked_Interrupt_Source();


void PKIC_Interrupt_ACK(int iValue);


#endif

