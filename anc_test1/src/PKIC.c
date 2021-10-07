
#include <sys/platform.h>
#include "PKIC.h"



void Set_PKIC_Polarity(int iValue)
{
	*pREG_PKIC0_POL_CTL= iValue;
}

void Set_PKIC_Level_type(int iValue)
{
	*pREG_PKIC0_TYPE_CTL= iValue;
}

void Enable_TRNG_Interrupt()
{
	*pREG_PKIC0_EN_SET = 0x1<<3;
}

void Disable_TRNG_Interrupt()
{
	*pREG_PKIC0_EN_CLR |= 0x1<<3;
}

void Enable_PKIC_Interrupt(int iValue)
{
	*pREG_PKIC0_EN_SET |= iValue;
}

void Disable_PKIC_Interrupt(int iValue)
{
	*pREG_PKIC0_EN_CLR |= iValue;
}

int Read_PKIC_Unmasked_Interrupt_Source()
{
	int iTemp;
	iTemp= *pREG_PKIC0_RAW_STAT;
	return iTemp;
}

int Read_PKIC_Masked_Interrupt_Source()
{
	int iTemp;
	iTemp= *pREG_PKIC0_EN_STAT;
	return iTemp;
}

void PKIC_Interrupt_ACK(int iValue)
{
	*pREG_PKIC0_ACK= iValue;
}



