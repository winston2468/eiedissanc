/*
 * Auau1979Interface.c
 *
 *  Created on: 12 Oct 2021
 *      Author: Winston
 */


#include <stdio.h>
#include <drivers/dac/adau1962a/adi_adau1962a.h>
#include "anc_test1.h"



/* Twi  */
static uint32_t TwiMemory[ADI_TWI_MEMORY_SIZE];

/* ADAU1962A DAC DATA */
static ADI_ADAU1962A_HANDLE phAdau1962a;
uint32_t Adau1962aMemory[ADI_ADAU1962A_MEMORY_SIZE];
/* ADAU1962A Sport */
uint32_t Adau1962aSportMemory[ADI_SPORT_DMA_MEMORY_SIZE];


/* Dac linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align(4)
int32_t DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 2];


/* DAC callback */
extern void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg);


uint32_t Adau1962aInit(void) {
	/*
	 * Opens and initializes ADAU1962A DAC Device.
	 *
	 * Parameters
	 *  None
	 *
	 * Returns
	 *  0 if success, other values for error
	 *
	 */

	ADI_ADAU1962A_RESULT eResult;
	ADI_ADAU1962A_TWI_CONFIG TwiConfig;
	ADI_ADAU1962A_SPORT_CONFIG SportConfig;

	/* Open ADAU1962A device instance */
	if ((eResult = adi_adau1962a_Open(0u,

#ifdef TDM_MODE
            						 ADI_ADAU1962A_SERIAL_MODE_TDM8,
#else
             					     ADI_ADAU1962A_SERIAL_MODE_STEREO,
#endif
			&Adau1962aMemory,
			ADI_ADAU1962A_MEMORY_SIZE, &phAdau1962a))
			!= ADI_ADAU1962A_SUCCESS) {
		printf(
				"ADAU1962A: Failed to open ADAU1962A device instance, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum = 0u;
	TwiConfig.eTwiAddr = ADI_ADAU1962A_TWI_ADDR_04;
	TwiConfig.TwiDevMemSize = ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem = &TwiMemory;

	/* Configure TWI */
	if ((eResult = adi_adau1962a_ConfigTwi(phAdau1962a, &TwiConfig))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to configure TWI, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	SportConfig.SportDevNum 	= 4u;
	SportConfig.eSportChnl	    = ADI_ADAU1962A_SPORT_B;
	SportConfig.eSportPri	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.SportDevMemSize	= ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem 	= &Adau1962aSportMemory;


	/* Configure SPORT */
	if ((eResult = adi_adau1962a_ConfigSport(phAdau1962a, &SportConfig))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to configure SPORT, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/* DAC Master Power-up */
	if ((eResult = adi_adau1962a_ConfigDacPwr(phAdau1962a,
			ADI_ADAU1962A_CHNL_DAC_MSTR, ADI_ADAU1962A_DAC_PWR_LOW, true))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to configure DAC power, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/*
	 * Configure PLL clock - DAC is clock master and drives SPORT clk and FS
	 * MCLK 24.576 MHz and PLL uses MCLK
	 */
	if ((eResult = adi_adau1962a_ConfigPllClk(phAdau1962a,
	ADAU1962A_MCLK_IN, ADI_ADAU1962A_MCLK_SEL_PLL,
			ADI_ADAU1962A_PLL_IN_MCLKI_XTALI)) != ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to configure PLL clock, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/*
	 * Configure serial data clock
	 * DAC as clock master, External BCLK, Latch on raising edge
	 * LRCLK at 50% duty cycle, MSB first, Left channel at LRCLK low
	 */
	if ((eResult = adi_adau1962a_ConfigSerialClk(phAdau1962a,
	LR_B_CLK_MASTER_1962, false,
	BCLK_RISING_1962,
	/* pulse mode - true */
	#ifdef TDM_MODE
		                                              true,
	#else
		                                              false,
	#endif

	false,
	LRCLK_HI_LO_1962)) != ADI_ADAU1962A_SUCCESS) {
		printf(
				"ADAU1962A: Failed to configure serial data clock, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/* Power-up PLL */
	if ((eResult = adi_adau1962a_ConfigBlockPwr(phAdau1962a, false, true, true))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to Power-up PLL, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/* Configure Sample rate */
	if ((eResult = adi_adau1962a_SetSampleRate(phAdau1962a, SAMPLE_RATE_A))
			!= ADI_ADAU1962A_SUCCESS) {
		printf(
				"ADAU1962A: Failed to configure Sample rate, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	/* Configure Word width */
	if ((eResult = adi_adau1962a_SetWordWidth(phAdau1962a,
			ADI_ADAU1962A_WORD_WIDTH_24)) != ADI_ADAU1962A_SUCCESS) {
		printf(
				"ADAU1962A: Failed to configure word width, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}
	// Register callback
	if ((eResult = adi_adau1962a_RegisterCallback(phAdau1962a, DacCallback,
	NULL)) != ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to register callback, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}
	//VOL
	if ((eResult = adi_adau1962a_SetVolume(phAdau1962a,
			ADI_ADAU1962A_CHNL_DAC_MSTR, DacMasterVolume))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to set volume, Error Code: 0x%08X\n",
				eResult);
		// return error
		return 1u;
	}
	if ((eResult = adi_adau1962a_ConfigDeEmphasis(phAdau1962a, false,
			true,  false, false))
			!= ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to set volume, Error Code: 0x%08X\n",
				eResult);
		// return error
		return 1u;
	}



	return 0u;
}

uint32_t Adau1962aSubmitBuffers(void) {
	/*
	 * Submits ping-pong buffers to DAC and enables ADC data flow.
	 *
	 * Parameters
	 *  None
	 *
	 * Returns
	 *  0 if success, other values for error
	 *
	 */
	uint32_t Result = 0u;

	/* submit ping buffer */
	if ((uint32_t) adi_adau1962a_SubmitBuffer(phAdau1962a,
			&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 0u],
			AUDIO_BUFFER_SIZE_DAC) != 0u) {
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if ((uint32_t) adi_adau1962a_SubmitBuffer(phAdau1962a,
			&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 1u],
			AUDIO_BUFFER_SIZE_DAC) != 0u) {
		/* return error */
		return 1u;
	}

	return Result;
}


uint32_t Adau1962aEnable( void )
{
	if((uint32_t)adi_adau1962a_Enable(phAdau1962a, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	return 0;
}



uint32_t Adau1962aDoneWithBuffer( volatile void *pBuffer )
{
	ADI_ADAU1962A_RESULT    eResult2;
	void 					*pDAC;
	/*
	if(pBuffer == &DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 0])
	{
		pDAC = (void *)&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 2];
	}
	else if(pBuffer == &DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 1])
	{
		pDAC = (void *)&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 0];
	}
	else if(pBuffer == &DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 2])
	{
		pDAC = (void *)&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 1];
	}

	if(pBuffer == &DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 0])
	{
		pDAC = (void *)&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 1];
	}
	else if(pBuffer == &DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 1])
	{
		pDAC = (void *)&DacBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL * NUM_DAC_CHANNELS * 0];
	}


	else
	{
		// there has been an error
		return 1u;
	}
*/
	/* submit the DAC buffer */
   // eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC, AUDIO_BUFFER_SIZE_DAC);
	 eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pBuffer, AUDIO_BUFFER_SIZE_DAC);
	return 0;
}



