/*********************************************************************************

Copyright(c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/

/*
 * This example demonstrates how to record audio data from the MIC IN or LINE IN
 * to Blackfin memory using the ADAU1761 codec driver.  Recording stops when the
 * audio buffer is full (approx. 5 seconds).  The recorded audio data
 * can then be played back from Blackfin memory. *
 *
 * This example operates in Callback mode.
 *
 *
 * Press Push Button 1 to start/stop recording.
 * Press Push Button 2 to play recording.
 *
 * Connect an audio source to Audio EZ-Extender board LINE IN (J4) or use MICROPHONE.
 * Connect headphones to HP (J2) or speakers LINE OUT (J3)
 */

#include <sys/platform.h>
#include <sys/adi_core.h>
#include <services/spu/adi_spu.h>
#include <services/gpio/adi_gpio.h>
#include <services/int/adi_int.h>

#include <drivers/dac/adau1962a/adi_adau1962a.h>
#include <drivers/twi/adi_twi.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Audio_EI3/drivers/codec/adau1761/adi_adau1761.h"
#include "adi_initialize.h"
#include "Audio_EI3/drivers/codec/adau1761/export_IC_1.h"
#include <SRU.h>
#include <sys/platform.h>
#include "anc_test1.h"


/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE

/* select output */
#define USE_HEADPHONE
//#define USE_LINE_OUT

/* the ADAU1761 Rec Mixer Left 0 register */
#define REC_MIX_LEFT_REG    (0x400A)
/* the ADAU1761 Rec Mixer Right 0 register */
#define REC_MIX_RIGHT_REG   (0x400C)

/* select sample rate */
#define SAMPLE_RATE  (ADI_ADAU1761_SAMPLE_RATE_48KHZ)

/* 5 seconds of audio data @ 12 KHz */
#define AUDIO_MEM_SIZE          (48000*2)

/* codec device instance to be tested */
#define ADAU1761_DEV_NUM          0

#define NUM_AUDIO_SAMPLES       256u

#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)

#define BUFFER_SIZE      (NUM_AUDIO_SAMPLES*sizeof(uint32_t))

/* used for exit timeout */
#define MAXCOUNT (50000000u)

#pragma align(4)
static uint8_t sportRxMem1[ADI_SPORT_DMA_MEMORY_SIZE];

#pragma align(4)
static int32_t RxBuffer1Ping[NUM_AUDIO_SAMPLES];

#pragma align(4)
static int32_t RxBuffer1Pong[NUM_AUDIO_SAMPLES];

#pragma align(4)
static uint8_t sportTxMem[ADI_SPORT_DMA_MEMORY_SIZE];

#pragma align(4)
static int32_t TxBuffer1[NUM_AUDIO_SAMPLES];

#pragma align(4)
static int32_t TxBuffer2[NUM_AUDIO_SAMPLES];

static int32_t AudioBuffer[AUDIO_MEM_SIZE];

typedef enum
{
	NONE,
	START_RECORDING,
	STOP_RECORDING,
	START_PLAYBACK,
	STOP_PLAYBACK,
	SUBMIT_RX_BUFFER,
	SUBMIT_TX_BUFFER
} MODE;

static uint32_t nAudioCount = 0;
static uint32_t bAudioFull = false;
static uint32_t *pAudioData;
static MODE eMode = NONE;

static uint8_t gpioMemory[GPIO_MEMORY_SIZE];

/* Memory required for codec driver */
static int32_t codecmem1[ADI_ADAU1761_MEMORY_SIZE];

/* adau1761 device handle */
static ADI_ADAU1761_HANDLE  hADAU1761_1;

static bool bError;
static uint32_t count;

static bool bEvent;
static int32_t *pRxBuffer;
static int32_t *pTxBuffer;




static bool bError;
static uint32_t count;

static bool bEvent;

/* Twi  */
uint32_t TwiMemory[ADI_TWI_MEMORY_SIZE];
/* ADAU1962A DAC DATA */
static ADI_ADAU1962A_HANDLE phAdau1962a;
uint32_t Adau1962aMemory[ADI_ADAU1962A_MEMORY_SIZE];
/* ADAU1962A Sport */
uint32_t Adau1962aSportMemory[ADI_SPORT_DMA_MEMORY_SIZE];

/* Counter to keep track of number of ADC/DAC buffers processed */
volatile uint32_t AdcCount = 0u;
volatile uint32_t DacCount = 0u;

/* ADC/DAC buffer pointer */
volatile void *pGetDAC = NULL;
uint32_t *temp = NULL;
void *pDAC;

/* Flag to register callback error */
volatile bool bEventError = false;

/* Dac linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align(4)
int32_t DacBuf[AUDIO_BUFFER_SIZE * 2];

/* Memory required for the SPU operation */
static uint32_t SpuMemory[ADI_SPU_MEMORY_SIZE];
/* SPU handle */
static ADI_SPU_HANDLE hSpu;

volatile int32_t ch1[(NUM_AUDIO_SAMPLES / 2)];
volatile int32_t ch2[(NUM_AUDIO_SAMPLES / 2)];
static int32_t sRxBuffer1[NUM_AUDIO_SAMPLES];
/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/
/* Initialize GPIO and reset peripherals */
uint32_t GpioInit(void);
/* Initializes DAC */
uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
uint32_t Adau1962aSubmitBuffers(void);
uint32_t ProcessBuffers(void);
void VolControl(void);
/* DAC callback */
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg);



extern void ConfigSoftSwitches(void);

static void CheckResult(ADI_ADAU1761_RESULT result)
{
	if (result != ADI_ADAU1761_SUCCESS) {
		printf("Codec failure\n");
		bError = true;
	}
}

static void CheckGpioResult(ADI_GPIO_RESULT result)
{
	if (result != ADI_GPIO_SUCCESS) {
		printf("GPIO failure\n");
		bError = true;
	}
}

/* codec callback */
static void ADAU1761Callback(void *pCBParam, uint32_t Event, void *pArg)
{
    switch(Event)
    {
    	case (uint32_t)ADI_ADAU1761_EVENT_RX_BUFFER_PROCESSED:
			/* re-sumbit the buffer for processing */
    			AdcCount++;
    			pRxBuffer = pArg;
    			memcpy(&sRxBuffer1, pRxBuffer, BUFFER_SIZE);
    			eMode = SUBMIT_RX_BUFFER;
    			bEvent = true;
    		break;

    	case (uint32_t)ADI_ADAU1761_EVENT_TX_BUFFER_PROCESSED:
        		/* re-sumbit the buffer for processing */
        		pTxBuffer = pArg;
    			eMode = SUBMIT_RX_BUFFER;
    			bEvent = true;
    		break;

        default:
        	printf("test");
            break;
    }
}

void pinIntCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t PinIntData,  void *pCBParam)
{
	if (ePinInt == PUSH_BUTTON1_PINT)
	{
		//push button 1
		if (PinIntData & PUSH_BUTTON1_PIN)
		{
			if (eMode == NONE)
			{
				//start recording
				nAudioCount = 0;
				bAudioFull = false;

				//LED on
				adi_gpio_Set(LED2_PORT, LED2_PIN);

				eMode = START_RECORDING;
				bEvent = true;
			}
		}
	}
	else if (ePinInt == PUSH_BUTTON2_PINT)
	{
		//push button 2
		if (PinIntData & PUSH_BUTTON2_PIN)
		{
			if (eMode == NONE)
			{
				//LED on
				adi_gpio_Set(LED1_PORT, LED1_PIN);

				//start playback
				nAudioCount = 0;
				bAudioFull = false;

				//copy initial data to Tx buffer
				memcpy(TxBuffer1, &AudioBuffer[nAudioCount], NUM_AUDIO_SAMPLES*sizeof(uint32_t));
				nAudioCount += NUM_AUDIO_SAMPLES;

				eMode = START_PLAYBACK;
				bEvent = true;
			}
		}
	}

	/* reset the exit counter */
	count = 0u;

}

void configGpio(void)
{
	int i;
	uint32_t gpioMaxCallbacks;
	ADI_INT_STATUS  eIntResult;

    // init the GPIO service
	ADI_GPIO_RESULT result = adi_gpio_Init(
			(void*)gpioMemory,
			GPIO_MEMORY_SIZE,
			&gpioMaxCallbacks);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 1
	 */

	// set GPIO input

	result = adi_gpio_SetDirection(
		PUSH_BUTTON1_PORT,
		PUSH_BUTTON1_PIN,
	    ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT B is connected to Pin Interrupt 0)
	result = adi_gpio_SetPinIntEdgeSense(
		PUSH_BUTTON1_PINT,
		PUSH_BUTTON1_PIN,
	    ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint callback
	result = adi_gpio_RegisterCallback(
		PUSH_BUTTON1_PINT,
		PUSH_BUTTON1_PIN,
	    pinIntCallback,
	    (void*)0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
		PUSH_BUTTON1_PINT,
		PUSH_BUTTON1_PIN,
	    true);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 2
	 */

	// set GPIO input

	result = adi_gpio_SetDirection(
		PUSH_BUTTON2_PORT,
		PUSH_BUTTON2_PIN,
	    ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT E is connected to Pin Interrupt 3)
	result = adi_gpio_SetPinIntEdgeSense(
		PUSH_BUTTON2_PINT,
		PUSH_BUTTON2_PIN,
	    ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint 3 callback
	result = adi_gpio_RegisterCallback(
		PUSH_BUTTON2_PINT,
		PUSH_BUTTON2_PIN,
	    pinIntCallback,
	    (void*)0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
		PUSH_BUTTON2_PINT,
		PUSH_BUTTON2_PIN,
	    true);
	CheckGpioResult(result);

	/*
	 * Setup LED's
	 */

	// set GPIO output LED
	result = adi_gpio_SetDirection(
		LED1_PORT,
		LED1_PIN,
	    ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	// set GPIO output LED
	result = adi_gpio_SetDirection(
		LED2_PORT,
		LED2_PIN,
	    ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);


}

void MixerEnable(bool bEnable)
{
	ADI_ADAU1761_RESULT result;
	uint8_t value;

#ifdef USE_LINE_IN
	if (bEnable)
	{
		/* enable the record mixer (left) */
		result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_LEFT_REG, 0x5B); /* 0 dB */
		CheckResult(result);

		/* enable the record mixer (right) */
		result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, 0x5B);  /* 0 dB */
		CheckResult(result);

		/* disable the output mixer (left) */
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4011, 0xFB); /* MUTE */
		//CheckResult(result);
		/* disable the output mixer (left) */
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4012, 0x3B); /* MUTE */
		//CheckResult(result);
		/* disable the output mixer (left) */
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4013, 0x68); /* MUTE */
		//CheckResult(result);
		/* disable the output mixer (left) */
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4014, 0x31); /* MUTE */
		//CheckResult(result);




	}
	else
	{
		/* disable the record mixer (left) */
		result = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_LEFT_REG, &value);
		result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_LEFT_REG, value & 0xFE);
		CheckResult(result);

		/* disable the record mixer (right) */
		result = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, &value);
		result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, value & 0xFE);
		CheckResult(result);
	}
#else
	/* disable the record mixer (left) */
	result = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_LEFT_REG, &value);
	result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_LEFT_REG, value & 0xFE);
	CheckResult(result);

	/* disable the record mixer (right) */
	result = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, &value);
	result = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, value & 0xFE);
	CheckResult(result);
#endif
}

int main(void)
{
	ADI_ADAU1761_RESULT result;
	ADI_ADAU1761_SPORT_INFO sportRxInfo1;
	ADI_ADAU1761_SPORT_INFO sportTxInfo1;
	bool bExit;
	uint32_t Result = 0u;
	bExit = false;

    adi_initComponents(); /* auto-generated code */

	/* Software Switch Configuration for the EZ-BOARD */
	ConfigSoftSwitches();

    configGpio();

	/* Open the codec driver */
	result = adi_adau1761_Open(ADAU1761_DEV_NUM,
			codecmem1,
			ADI_ADAU1761_MEMORY_SIZE,
			ADI_ADAU1761_COMM_DEV_TWI,
			&hADAU1761_1 );
	CheckResult(result);

	/* Configure TWI */
	result = adi_adau1761_ConfigTWI(hADAU1761_1, TWI_DEV_NUM, ADI_ADAU1761_TWI_ADDR0);
	CheckResult(result);

	/* load Sigma Studio program exported from *_IC_1.h */
	result = adi_adau1761_SigmaStudioLoad(hADAU1761_1, default_download_IC_1);
	CheckResult(result);

	//LRCLK PULSE MODE, TDM 8 , RISING EDGE BCLK POLARITY
	//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x2D);
	//CheckResult(result);
	//LRCLK PULSE MODE
	//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x31);
	//CheckResult(result);


	result = adi_adau1761_SetRegister (hADAU1761_1, 0x4016, 0x01);
	CheckResult(result);


	//divide by 2
	//uint8_t pllclockconfig = {0x00, 0x01, 0x00,0x00,0x22,0x01};
	//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4002,pllclockconfig ); /* MUTE */
	//		CheckResult(result);
	//DISABLE MIXER OUTPUT
	result = adi_adau1761_SetRegister (hADAU1761_1, 0x401C, 0x21); /* MUTE */
	CheckResult(result);
	/* disable the output mixer (left) */
	result = adi_adau1761_SetRegister (hADAU1761_1, 0x401D, 0x00); /* MUTE */
	//CheckResult(result);
	/* enable the output mixer (right) */
	result = adi_adau1761_SetRegister (hADAU1761_1, 0x401E, 0x41);  /* MUTE */
	CheckResult(result);
	/* enable the output mixer (right) */
	result = adi_adau1761_SetRegister (hADAU1761_1, 0x401F, 0x00);  /* MUTE */
	CheckResult(result);
	result = adi_adau1761_SetRegister (hADAU1761_1, 0x4019, 0x33);  /* MUTE */
	CheckResult(result);
	/* config SPORT for Rx data transfer */
	sportRxInfo1.nDeviceNum = SPORT_RX_DEVICE;
	sportRxInfo1.eChannel = ADI_HALF_SPORT_B;
	sportRxInfo1.eMode = ADI_ADAU1761_SPORT_I2S;
	sportRxInfo1.hDevice = NULL;
	sportRxInfo1.pMemory = sportRxMem1;
	sportRxInfo1.bEnableDMA = true;
	sportRxInfo1.eDataLen = ADI_ADAU1761_SPORT_DATA_24;
	sportRxInfo1.bEnableStreaming = false;

	result = adi_adau1761_ConfigSPORT (hADAU1761_1,
			ADI_ADAU1761_SPORT_INPUT, &sportRxInfo1);
	CheckResult(result);

	/* register a Rx callback */
	result = adi_adau1761_RegisterRxCallback (hADAU1761_1,
			ADAU1761Callback, NULL);
	CheckResult(result);

	/* config SPORT for Tx data transfer */
	sportTxInfo1.nDeviceNum = SPORT_TX_DEVICE;
	sportTxInfo1.eChannel = ADI_HALF_SPORT_A;
	sportTxInfo1.eMode = ADI_ADAU1761_SPORT_I2S;
	sportTxInfo1.hDevice = NULL;
	sportTxInfo1.pMemory = sportTxMem;
	sportTxInfo1.bEnableDMA = true;
	sportTxInfo1.eDataLen = ADI_ADAU1761_SPORT_DATA_24;
	sportTxInfo1.bEnableStreaming = false;

	result = adi_adau1761_ConfigSPORT (hADAU1761_1,
			ADI_ADAU1761_SPORT_OUTPUT, &sportTxInfo1);
	CheckResult(result);

	/* register a Tx callback */
	result = adi_adau1761_RegisterTxCallback (hADAU1761_1,
			ADAU1761Callback, NULL);
	CheckResult(result);

#if defined(USE_LINE_IN)
	result = adi_adau1761_SelectInputSource(hADAU1761_1, ADI_ADAU1761_INPUT_ADC);
	CheckResult(result);
#else
	result = adi_adau1761_SelectInputSource(hADAU1761_1, ADI_ADAU1761_INPUT_MIC);
	CheckResult(result);
#endif

	/* disable mixer */
	MixerEnable(false);

	//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x01);
	//CheckResult(result);

#if defined(USE_HEADPHONE)
	result = adi_adau1761_SetVolume (hADAU1761_1,
			ADI_ADAU1761_VOL_HEADPHONE,
			ADI_ADAU1761_VOL_CHAN_BOTH,
			true,
			0);
	CheckResult(result);

	result = adi_adau1761_SetVolume (hADAU1761_1,
			ADI_ADAU1761_VOL_LINE_OUT,
			ADI_ADAU1761_VOL_CHAN_BOTH,
			false,
			0);
	CheckResult(result);
#else
	result = adi_adau1761_SetVolume (hADAU1761_1,
			ADI_ADAU1761_VOL_HEADPHONE,
			ADI_ADAU1761_VOL_CHAN_BOTH,
			false,
			0);
	CheckResult(result);

	result = adi_adau1761_SetVolume (hADAU1761_1,
			ADI_ADAU1761_VOL_LINE_OUT,
			ADI_ADAU1761_VOL_CHAN_BOTH,
			true,
			0);
	CheckResult(result);
#endif

	result = adi_adau1761_SetSampleRate (hADAU1761_1, SAMPLE_RATE);
	CheckResult(result);

	count = 0u;

	//Initialize SPU Service
	if (adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to initialize SPU service\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 4A to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 4B to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 4A DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 4B DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
		return (ADI_SPU_FAILURE);
	}

	// Initialize ADAU1962A
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	printf("\nPress Push Button 1 to start recording.\n");
	printf("Press Push Button 2 to play recording.\n\n");
	eMode=START_RECORDING;
	bEvent=true;
	while(!bExit)
	{
        if (bEvent)
        {
			switch(eMode)
			{
				case SUBMIT_RX_BUFFER:
					/*
					if (pRxBuffer != NULL)
					{
						result = adi_adau1761_SubmitRxBuffer(hADAU1761_1, pRxBuffer, BUFFER_SIZE);
						CheckResult(result);
					}
					if (pTxBuffer != NULL)
					{
						result = adi_adau1761_SubmitTxBuffer(hADAU1761_1, pTxBuffer, BUFFER_SIZE);
						CheckResult(result);
					}
					if (pRxBuffer != NULL&&pTxBuffer != NULL){
						memcpy(pTxBuffer, pRxBuffer, NUM_AUDIO_SAMPLES*sizeof(uint32_t));
					}
					*/
					ProcessBuffers();
					//printf("test");
					break;
				case SUBMIT_TX_BUFFER:
					/* re-submit processed buffer from callback */
					if (pTxBuffer != NULL)
					{
						result = adi_adau1761_SubmitTxBuffer(hADAU1761_1, pTxBuffer, BUFFER_SIZE);
						CheckResult(result);
					}
					break;
				case START_RECORDING:
#ifdef USE_MICROPHONE
				printf("Microphone ");
#else
				printf("Line In ");
#endif
					printf("Recording started.\n");

					/* enable mixer */
					MixerEnable(true);

					/* stop current playback */
					result = adi_adau1761_EnableInput (hADAU1761_1, false);
					CheckResult(result);

					/* submit buffer1 */
					result = adi_adau1761_SubmitRxBuffer(hADAU1761_1, RxBuffer1Ping, BUFFER_SIZE);
					CheckResult(result);

					/* submit buffer2 */
					result = adi_adau1761_SubmitRxBuffer(hADAU1761_1,	RxBuffer1Pong, BUFFER_SIZE);
					CheckResult(result);




					result = adi_adau1761_EnableInput (hADAU1761_1, true);
					CheckResult(result);
					//1962a ping pong buffers
					if (Result == 0u) {
						Result = Adau1962aSubmitBuffers();
					}

					// Enable data flow for the DAC
					if ((uint32_t) adi_adau1962a_Enable(phAdau1962a, true) != 0u) {
						// return error
						return 1u;
					}
					break;
				case STOP_RECORDING:
					result = adi_adau1761_EnableInput (hADAU1761_1, false);
					CheckResult(result);

					/* disable mixer */
					MixerEnable(false);

					printf("Recording done.\n");
					eMode = NONE;
					break;
				case START_PLAYBACK:
					printf("Playback started.\n");
					/* stop current playback */
					result = adi_adau1761_EnableOutput (hADAU1761_1, false);
					CheckResult(result);

					/* submit buffer1 */
					result = adi_adau1761_SubmitTxBuffer(hADAU1761_1, TxBuffer1, BUFFER_SIZE);
					CheckResult(result);

					/* submit buffer2 */
					result = adi_adau1761_SubmitTxBuffer(hADAU1761_1,	TxBuffer2, BUFFER_SIZE);
					CheckResult(result);

					result = adi_adau1761_EnableOutput (hADAU1761_1, true);
					CheckResult(result);


					break;
				case STOP_PLAYBACK:
					printf("Playback done.\n");
					adi_adau1761_EnableOutput (hADAU1761_1, false);
					eMode = NONE;
					break;
				default:
					break;
			}

			bEvent = false;
		}

		/* wait for push button interrupts - exit the loop after a while */
		if (count > MAXCOUNT)
		{
			bExit = true;
		}
		count++;
	}

	result = adi_adau1761_Close(hADAU1761_1);
	CheckResult(result);

	if (!bError) {
		printf("All done\n");
	} else {
		printf("Audio error\n");
	}

	return 0;
}


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
	if ((eResult = adi_adau1962a_Open(0u, ADI_ADAU1962A_SERIAL_MODE_TDM8,
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

	SportConfig.SportDevNum = 4u;

	SportConfig.eSportChnl = ADI_ADAU1962A_SPORT_B;
	SportConfig.eSportPri = ADI_ADAU1962A_SERIAL_PORT_DSDATA1;
	SportConfig.eSportSec = ADI_ADAU1962A_SERIAL_PORT_NONE;
	SportConfig.SportDevMemSize = ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem = &Adau1962aSportMemory;

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
	BCLK_RISING_1962, true, false,
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

	/* Register callback */
	if ((eResult = adi_adau1962a_RegisterCallback(phAdau1962a, DacCallback,
	NULL)) != ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to register callback, Error Code: 0x%08X\n",
				eResult);
		/* return error */
		return 1u;
	}

	//VOL
	if ((eResult = adi_adau1962a_SetVolume(phAdau1962a,
			ADI_ADAU1962A_CHNL_DAC_MSTR, 0)) != ADI_ADAU1962A_SUCCESS) {
		printf("ADAU1962A: Failed to set volume, Error Code: 0x%08X\n",
				eResult);
		// return error
		return 1u;
	}
	/*
	 //De Emphasis
	 if ((eResult = adi_adau1962a_ConfigDeEmphasis (phAdau1962a,
	 false,
	 true,
	 true,
	 true
	 )) != ADI_ADAU1962A_SUCCESS)
	 {
	 printf ("ADAU1962A: Failed to set _ConfigDeEmphasis, Error Code: 0x%08X\n", eResult);
	 // return error
	 return 1u;
	 }

	 */

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
			&DacBuf[AUDIO_BUFFER_SIZE * 0u], AUDIO_BUFFER_SIZE) != 0u) {
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if ((uint32_t) adi_adau1962a_SubmitBuffer(phAdau1962a,
			&DacBuf[AUDIO_BUFFER_SIZE * 1u], AUDIO_BUFFER_SIZE) != 0u) {
		/* return error */
		return 1u;
	}

	return Result;
}

uint32_t ProcessBuffers(void) {
	/*
	 * Processes audio buffers that are processed by ADC and DAC driver.
	 *
	 * Parameters
	 *  None
	 *
	 * Returns
	 *  0 if success, other values for error
	 *
	 */
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_RESULT result2;
	ADI_ADAU1962A_RESULT eResult2;

	int32_t i = 0u;
	int32_t *pSrc;
	int32_t pSrcL1[NUM_AUDIO_SAMPLES / 2];
	int32_t pSrcR1[NUM_AUDIO_SAMPLES / 2];
	int32_t pSrcL2[NUM_AUDIO_SAMPLES / 2];
	int32_t pSrcR2[NUM_AUDIO_SAMPLES / 2];
	int32_t *pDst;
	int32_t * pSrcL_temp = 0;
	int32_t * pSrcR_temp = 0;

	int32_t *pDstL;
	int32_t *pDstR;

	// re-submit processed buffer from callback
	if (pRxBuffer != NULL) {

		// re-submit the ADC buffer
		result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1, pRxBuffer,
		BUFFER_SIZE);
		CheckResult(result1);

	}
#ifdef USE_ADAU1761_2
	if (pRxBuffer2 != NULL) {

		// re-submit the ADC buffer
		result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2, pRxBuffer2,
				BUFFER_SIZE_1761);
		CheckResult(result2);

	}
#endif

	if (pGetDAC != NULL) {
		//printf("test2");

		/* re-submit the DAC buffer */
		eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC,
		AUDIO_BUFFER_SIZE);

		if (eResult2) {
			printf("ERR");
			return 1u;

		}
		//pGetDAC = NULL;

	}
		printf("test1");
	// process ADC to DAC buffer
	if (sRxBuffer1 != NULL && pDAC != NULL) {

		//pDAC = pADC;
		pDst = (int32_t *) pDAC;

		uint32_t k = 0;
		uint32_t j = 0;
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES; i++) {

			if (i % 2 == 0) {
				pSrcL1[i - k] = sRxBuffer1[i];
				j += 1;
			} else {
				pSrcR1[i - j] = sRxBuffer1[i];
				k += 1;
			}
		}
		for (uint32_t j = 0; j < NUM_AUDIO_SAMPLES / 2; j++) {
			*pDst++ = pSrcL1[j];
			*pDst++ = pSrcR1[j];
			*pDst++ = pSrcL1[j];
			*pDst++ = pSrcR1[j];
			*pDst++ = pSrcL1[j];
			*pDst++ = pSrcR1[j];
			*pDst++ = pSrcL1[j];
			*pDst++ = pSrcR1[j];
		}

	}

	return 0u;
}

void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg) {
	/*
	 * DAC Callback.
	 *
	 * Parameters
	 *  None
	 *
	 * Returns
	 *  None
	 *
	 */
	switch (nEvent) {
	case ADI_SPORT_EVENT_TX_BUFFER_PROCESSED:

		/* Update callback count */
		DacCount++;
		/* store pointer to the processed buffer that caused the callback */
		pGetDAC = pArg;
		pDAC = (void *) pGetDAC;
		break;
	default:
		if (bEventError == false)
			printf("dead", nEvent);

		bEventError = true;
		break;
	}
}

