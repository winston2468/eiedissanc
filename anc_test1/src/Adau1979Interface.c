/*
 * Auau1979Interface.c
 *
 *  Created on: 12 Oct 2021
 *      Author: Winston
 */


#include <stdio.h>

#include <drivers/adc/adau1979/adi_adau1979.h>
#include "anc_test1.h"



/* Twi  */
static uint32_t TwiMemory[ADI_TWI_MEMORY_SIZE];

/* ADAU1979 ADC */
static ADI_ADAU1979_HANDLE phAdau1979;
uint32_t Adau1979Memory[ADI_ADAU1979_MEMORY_SIZE];
/* ADAU1979 Sport */
uint32_t Adau1979SportMemory[ADI_SPORT_DMA_MEMORY_SIZE];


#pragma align(4)
int32_t AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 2];


/* ADC callback */
extern void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg);




/*
 * Opens and initializes ADAU1979 ADC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979Init(void) {
	uint32_t Result = 0u;
	/* Instance to submit SPORT configuration */
	ADI_ADAU1979_SPORT_CONFIG SportConfig;
	/* Instance to submit TWI configuration */
	ADI_ADAU1979_TWI_CONFIG TwiConfig;


	/* open ADAU1979 instance */
	if ((uint32_t) adi_adau1979_Open(0u,
#ifdef TDM_MODE
			ADI_ADAU1979_SERIAL_MODE_TDM8,
#else
			ADI_ADAU1979_SERIAL_MODE_STEREO,
#endif
			&Adau1979Memory,
			ADI_ADAU1979_MEMORY_SIZE, &phAdau1979) != 0u) {
		printf("ADAU1979: adi_adau1979_Open failed\n");
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum = 0u;
	TwiConfig.TwiDevMemSize = ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem = &TwiMemory;
	TwiConfig.eTwiAddr = ADI_ADAU1979_TWI_ADDR_11;

	if ((uint32_t) adi_adau1979_ConfigTwi(phAdau1979, &TwiConfig) != 0u) {
		printf("ADAU1979: adi_adau1979_ConfigTwi failed\n");
		/* return error */
		return 1u;
	}

	/* SPORT parameters required to open/configure TWI */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	SportConfig.SportDevNum = 4u;
#elif defined(__ADSPSC573_FAMILY__)
	SportConfig.SportDevNum = 2u;
#else
#error "processor not defined"
#endif
	SportConfig.SportDevMemSize = ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem = &Adau1979SportMemory;
	SportConfig.eSportChnl = ADI_ADAU1979_SPORT_A;
	SportConfig.eSportPri = ADI_ADAU1979_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec = ADI_ADAU1979_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec = ADI_ADAU1979_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.bLsbFirst = true;

	if ((uint32_t) adi_adau1979_ConfigSport(phAdau1979, &SportConfig) != 0u) {
		printf("ADAU1979: adi_adau1979_ConfigSport failed\n");
		/* return error */
		return 1u;
	}

	/* ADC is a master source of SPORT clk and FS, MCLK 25.576 MHz and PLL used MCLK */
	if ((uint32_t) adi_adau1979_ConfigPllClk(phAdau1979,
	LR_B_CLK_MASTER_1979, ADI_ADAU1979_MCLK_IN_FREQ_24576000HZ,
			ADI_ADAU1979_PLL_SOURCE_MCLK) != 0u) {
		printf("ADAU1979: adi_adau1979_ConfigPllClk failed\n");
		/* return error */
		return 1u;
	}

	if ((uint32_t) adi_adau1979_ConfigSerialClk(phAdau1979,
	BCLK_RISING_1979,
	LRCLK_HI_LO_1979) != 0u) {
		printf("ADAU1979: adi_adau1979_ConfigSerialClk failed\n");
		/* return error */
		return 1u;
	}

	if ((uint32_t) adi_adau1979_SetSampleRate(phAdau1979,
			ADI_ADAU1979_SAMPLE_RATE_192000HZ) != 0u) {
		printf("ADAU1979: adi_adau1979_SetSampleRate failed\n");
		/* return error */
		return 1u;
	}
	if ((uint32_t) adi_adau1979_SetWordWidth(phAdau1979,
			ADI_ADAU1979_WORD_WIDTH_24) != 0u) {
		printf("ADAU1979: adi_adau1979_SetWordWidth failed\n");
		/* return error */
		return 1u;
	}
/*
	if ((uint32_t) adi_adau1979_SetVolume(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL1, (uint8_t)0xF0)
			!= 0u) {
		printf("ADAU1979: adi_adau1979_SetVolume1 failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_SetVolume(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL2, (uint8_t)0xF0)
			!= 0u) {
		printf("ADAU1979: adi_adau1979_SetVolume2 failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_SetVolume(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL3, (uint8_t)0xF0)
			!= 0u) {
		printf("ADAU1979: adi_adau1979_SetVolume3 failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_SetVolume(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL4, (uint8_t)0xF0)
			!= 0u) {
		printf("ADAU1979: adi_adau1979_SetVolume4 failed\n");
		// return error
		return 1u;
	}
*/

	if ((uint32_t) adi_adau1979_RegisterCallback(phAdau1979, AdcCallback, NULL)
			!= 0u) {
		printf("ADAU1979: adi_adau1979_RegisterCallback failed\n");
		/* return error */
		return 1u;
	}


	if ((uint32_t) adi_adau1979_HighPassChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL1, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_HighPassChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL2, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_HighPassChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL3, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	if ((uint32_t) adi_adau1979_HighPassChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL4, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	/*
	//MODIFIED FUNCTION!
	if ((uint32_t) adi_adau1979_CalibrateChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL1, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	//MODIFIED FUNCTION!
	if ((uint32_t) adi_adau1979_CalibrateChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL2, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	//MODIFIED FUNCTION!
	if ((uint32_t) adi_adau1979_CalibrateChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL3, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
	//MODIFIED FUNCTION!
	if ((uint32_t) adi_adau1979_CalibrateChannel(phAdau1979, ADI_ADAU1979_AUDIO_CHANNEL4, true)
			!= 0u) {
		printf("ADAU1979: highpass failed\n");
		// return error
		return 1u;
	}
*/
	return Result;
}


/*
 * Submits ping-pong buffers to ADC and enables ADC data flow.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979SubmitBuffers(void) {
	uint32_t Result = 0u;

	/* submit ping buffer */
	if ((uint32_t) adi_adau1979_SubmitBuffer(phAdau1979,
			&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL* NUM_ADAU1979_CHANNELS * 0u],
			AUDIO_BUFFER_SIZE_ADC_1979) != 0u) {
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if ((uint32_t) adi_adau1979_SubmitBuffer(phAdau1979,
			&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL* NUM_ADAU1979_CHANNELS * 1u],
			AUDIO_BUFFER_SIZE_ADC_1979) != 0u) {
		/* return error */
		return 1u;
	}

	return Result;
}


uint32_t Adau1979Enable( void )
{
	if((uint32_t)adi_adau1979_Enable(phAdau1979, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	return 0;
}



uint32_t Adau1979DoneWithBuffer( volatile  void *pBuffer )
{
    ADI_ADAU1979_RESULT     eResult1;
    void 					*pADC;

	/* Determine which is the next sub buffer to be submitted.
	 * The three sub-buffers must re-submitted in a cyclic order.
	 * 1-> 2 -> 3 -> 1....etc
	 * Since the DMA has an inbuilt ping-pong buffer queue when a callback is detected,
	 * the next buffer for submission is two buffers ahead
	 */
	 /*
	if(pBuffer == &AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 0])
	{
		pADC = (void *)&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 2];
	}
	else if(pBuffer == &AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 1])
	{
		pADC = (void *)&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 0];
	}
	else if(pBuffer == &AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 2])
	{
		pADC = (void *)&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 1];
	}


	if(pBuffer == &AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 0])
	{
		pADC = (void *)&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 1];
	}
	else if(pBuffer == &AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 1])
	{
		pADC = (void *)&AdcBuf[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS * 0];
	}

	else
	{
		return 1u;
	}
*/

	/* submit the ADC buffer */
   // eResult1 = adi_adau1979_SubmitBuffer(phAdau1979, (void *) pADC, AUDIO_BUFFER_SIZE_ADC_1979);
    eResult1 = adi_adau1979_SubmitBuffer(phAdau1979, (void *) pBuffer, AUDIO_BUFFER_SIZE_ADC_1979);
    return 0;
}



