/*****************************************************************************
 * anc_test1.c
 *****************************************************************************/



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
#include "anc_test1.h"
#define INSTANT_START
#define USE_ADAU1761_2
/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE

/* the ADAU1761 Rec Mixer Left 0 register */
#define REC_MIX_LEFT_REG    (0x400A)
/* the ADAU1761 Rec Mixer Right 0 register */
#define REC_MIX_RIGHT_REG   (0x400C)

/* select sample rate */
#define SAMPLE_RATE  (ADI_ADAU1761_SAMPLE_RATE_48KHZ)

/* codec device instance to be tested */
#define ADAU1761_DEV_NUM          0


#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)


/* used for exit timeout */
#define MAXCOUNT (50000000000u)
/*=============  D A T A  =============*/

#pragma align(4)
static uint8_t sportRxMem1[ADI_SPORT_DMA_MEMORY_SIZE];



/* Memory required for codec driver */
static uint8_t codecMem1[ADI_ADAU1761_MEMORY_SIZE];

/* adau1761 device handle */
static ADI_ADAU1761_HANDLE hADAU1761_1;

int32_t AdcBuf1[BUFFER_SIZE_1761*2 ];

static int32_t *pRxBuffer1;

static int32_t sRxBuffer1[NUM_AUDIO_SAMPLES];

bool bAvailable;


#ifdef USE_ADAU1761_2

static ADI_ADAU1761_HANDLE hADAU1761_2;

#define ADAU1761_DEV_NUM1          1

int32_t AdcBuf2[BUFFER_SIZE_1761*2 ];

#pragma align(4)
static uint8_t sportRxMem2[ADI_SPORT_DMA_MEMORY_SIZE];

static uint8_t codecMem2[ADI_ADAU1761_MEMORY_SIZE];

static int32_t *pRxBuffer2;

static int32_t sRxBuffer2[NUM_AUDIO_SAMPLES];

bool bAvailable1;


#endif

bool bAvailableDAC;

int32_t *pSrc;
int32_t pSrcL1[NUM_AUDIO_SAMPLES / 2];
int32_t pSrcR1[NUM_AUDIO_SAMPLES / 2];
int32_t pSrcL2[NUM_AUDIO_SAMPLES / 2];
int32_t pSrcR2[NUM_AUDIO_SAMPLES / 2];
int32_t *pDst;

static uint8_t DacMasterVolume = 0; //Master volume control, uint8_t 0 to 255 = 0 dB to -95.625 dB



typedef enum {
	NONE,
	START_RECORDING,
	ENABLE_DAC,
	STOP_RECORDING,
	START_PLAYBACK,
	STOP_PLAYBACK,
	SUBMIT_RX_BUFFER,
	SUBMIT_TX_BUFFER,
} MODE;

typedef enum {
	VOL_SAME,
	VOL_UP,
	VOL_DOWN
} VOL_CONTROL;

static VOL_CONTROL eVolControl = VOL_SAME;

static MODE eMode = NONE;

static uint32_t gpioMemory[GPIO_MEMORY_SIZE];



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

static void CheckResult(ADI_ADAU1761_RESULT result) {
	if (result != ADI_ADAU1761_SUCCESS) {
		printf("Codec failure\n");
		bError = true;
	}
}

static void CheckGpioResult(ADI_GPIO_RESULT result) {
	if (result != ADI_GPIO_SUCCESS) {
		printf("GPIO failure\n");
		bError = true;
	}
}


/* codec callback */
static void ADAU1761Callback_1(void *pCBParam, uint32_t Event, void *pArg) {
	switch (Event) {
	case (uint32_t) ADI_ADAU1761_EVENT_RX_BUFFER_PROCESSED:
		/* re-sumbit the buffer for processing */
		//pGetADC = pArg;
		//pADC = (void *)pGetADC;
		//AdcCount++;
		pRxBuffer1 = pArg;
	memcpy(&sRxBuffer1[0], pArg, BUFFER_SIZE_1761);
		eMode = SUBMIT_RX_BUFFER;
		bEvent = true;
		//printf("test1");
		break;
	default:
		printf("test0 %d \n", (int)Event );
		pRxBuffer1 = pArg;
		eMode = SUBMIT_RX_BUFFER;
		bEvent = true;
		break;
	}
}

#ifdef USE_ADAU1761_2
/* codec callback */
static void ADAU1761Callback_2(void *pCBParam, uint32_t Event, void *pArg) {
	switch (Event) {
		case (uint32_t) ADI_ADAU1761_EVENT_RX_BUFFER_PROCESSED:
		//AdcCount++;
		pRxBuffer2 = pArg;
		memcpy(&sRxBuffer2[0], pRxBuffer2, BUFFER_SIZE_1761);
		eMode = SUBMIT_RX_BUFFER;
		bEvent = true;
		break;
		default:

			printf("test1 %d \n", (int)Event );
			pRxBuffer2 = pArg;
			eMode = SUBMIT_RX_BUFFER;
			bEvent = true;
		break;

	}
}
#endif



void pinIntCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t PinIntData,
		void *pCBParam) {
	if (ePinInt == PUSH_BUTTON1_PINT) {
		//push button 1
		if (PinIntData & PUSH_BUTTON1_PINT_PIN) {
				//LED on
				adi_gpio_Set(LED1_PORT, LED1_PIN);

				eVolControl = VOL_UP;
				bEvent = true;
		}
	}
	if (ePinInt == PUSH_BUTTON2_PINT) {
		//push button 2
		if (PinIntData & PUSH_BUTTON2_PINT_PIN) {
				//LED on
				adi_gpio_Set(LED2_PORT, LED2_PIN);
				eVolControl = VOL_DOWN;
				bEvent = true;
		}
	}

	/* reset the exit counter */
	count = 0u;

}

void configGpio(void) {
	int i;
	uint32_t gpioMaxCallbacks;
	ADI_INT_STATUS eIntResult;

	// init the GPIO service
	ADI_GPIO_RESULT result = adi_gpio_Init((void*) gpioMemory,
	GPIO_MEMORY_SIZE, &gpioMaxCallbacks);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 1
	 */

	// set GPIO input
	result = adi_gpio_SetDirection(
	PUSH_BUTTON1_PORT,
	PUSH_BUTTON1_PIN, ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT B is connected to Pin Interrupt 0)
	result = adi_gpio_SetPinIntEdgeSense(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint callback
	result = adi_gpio_RegisterCallback(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, pinIntCallback, (void*) 0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, true);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 2
	 */

	// set GPIO input
	result = adi_gpio_SetDirection(
	PUSH_BUTTON2_PORT,
	PUSH_BUTTON2_PIN, ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT E is connected to Pin Interrupt 3)
	result = adi_gpio_SetPinIntEdgeSense(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint 3 callback
	result = adi_gpio_RegisterCallback(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, pinIntCallback, (void*) 0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, true);
	CheckGpioResult(result);

	/*
	 * Setup LED's
	 */

	// set GPIO output LED
	result = adi_gpio_SetDirection(
	LED1_PORT,
	LED1_PIN, ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	// set GPIO output LED
	result = adi_gpio_SetDirection(
	LED2_PORT,
	LED2_PIN, ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	//ADAU1979 (Not used) & ADAU1962a GPIO setup

	result = adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14,
			ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	/* bring reset low */
	result = adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14);
	CheckGpioResult(result);

	/* delay */
	for (i = DELAY_COUNT; i; i--)
		;

	/* bring reset high */
	result = adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14);
	CheckGpioResult(result);
	/* delay */
	for (i = DELAY_COUNT; i; i--)
		;
}

void MixerEnable(bool bEnable) {
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_RESULT result2;
	uint8_t value;

#ifdef USE_LINE_IN
	if (bEnable) {
		/* enable the record mixer (left) */
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_LEFT_REG, 0x5B); /* 0 dB */
		CheckResult(result1);

		/* enable the record mixer (right) */
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_RIGHT_REG, 0x5B); /* 0 dB */
		CheckResult(result1);
		//RISING EDGE BCLK POLARITY
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x11); /* MUTE */
		//CheckResult(result);
#ifdef USE_ADAU1761_2
		/* enable the record mixer (left) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_LEFT_REG, 0x5B); /* 0 dB */
		CheckResult(result2);

		/* enable the record mixer (right) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_RIGHT_REG, 0x5B); /* 0 dB */
		CheckResult(result2);
#endif
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
	} else {
		/* disable the record mixer (left) */
		result1 = adi_adau1761_GetRegister(hADAU1761_1, REC_MIX_LEFT_REG,
				&value);
		CheckResult(result1);
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_LEFT_REG,
				value & 0xFE);
		CheckResult(result1);

		/* disable the record mixer (right) */
		result1 = adi_adau1761_GetRegister(hADAU1761_1, REC_MIX_RIGHT_REG,
				&value);
		CheckResult(result1);
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_RIGHT_REG,
				value & 0xFE);
		CheckResult(result1);

#ifdef USE_ADAU1761_2
		/* disable the record mixer (left) */
		result2 = adi_adau1761_GetRegister(hADAU1761_2, REC_MIX_LEFT_REG,
				&value);
		CheckResult(result2);
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_LEFT_REG,
				value & 0xFE);
		CheckResult(result2);

		/* disable the record mixer (right) */
		result2 = adi_adau1761_GetRegister(hADAU1761_2, REC_MIX_RIGHT_REG,
				&value);
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_RIGHT_REG,
				value & 0xFE);
		CheckResult(result2);
#endif

	}
#else
	/* disable the record mixer (left) */
	result1 = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_LEFT_REG, &value);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_LEFT_REG, value & 0xFE);
	CheckResult(result1);

	/* disable the record mixer (right) */
	result1 = adi_adau1761_GetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, &value);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister (hADAU1761_1, REC_MIX_RIGHT_REG, value & 0xFE);
	CheckResult(result1);
#ifdef USE_ADAU1761_2
	/* disable the record mixer (left) */
	result2 = adi_adau1761_GetRegister (hADAU1761_2, REC_MIX_LEFT_REG, &value);
	CheckResult(result2);
	result2 = adi_adau1761_SetRegister (hADAU1761_2, REC_MIX_LEFT_REG, value & 0xFE);
	CheckResult(result2);

	/* disable the record mixer (right) */
	result2 = adi_adau1761_GetRegister (hADAU1761_2, REC_MIX_RIGHT_REG, &value);
	CheckResult(result2);
	result2 = adi_adau1761_SetRegister (hADAU1761_2, REC_MIX_RIGHT_REG, value & 0xFE);
	CheckResult(result2);
#endif

#endif
}

int main(void) {
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_SPORT_INFO sportRxInfo1;
	ADI_ADAU1761_SPORT_INFO sportTxInfo1;
#ifdef USE_ADAU1761_2
	ADI_ADAU1761_RESULT result2;
	ADI_ADAU1761_SPORT_INFO sportRxInfo2;
	ADI_ADAU1761_SPORT_INFO sportTxInfo2;
#endif

	bool bExit;
	uint32_t Result = 0u;
	//uint32_t i;
	bExit = false;

	adi_initComponents(); /* auto-generated code */

	/* Software Switch Configuration for the EZ-BOARD */
	ConfigSoftSwitches();

	configGpio();

	//Initialize SPU Service
	if (adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to initialize SPU service\n");
		return (ADI_SPU_FAILURE);
	}
/*
	// Make SPORT 4A to generate secure transactions
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
		return (ADI_SPU_FAILURE);
	}
*/

	/* Make SPORT 4B to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
		return (ADI_SPU_FAILURE);
	}

/*
	// Make SPORT 4A DMA to generate secure transactions
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
		return (ADI_SPU_FAILURE);
	}
*/

	/* Make SPORT 4B DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
		return (ADI_SPU_FAILURE);
	}


	/* Make SPORT 2B to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_2B_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 2B\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 2B DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_2B_DMA5_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 2B DMA\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 0B to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_0B_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 0B\n");
		return (ADI_SPU_FAILURE);
	}

	/* Make SPORT 0B DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_0B_DMA1_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 0B DMA\n");
		return (ADI_SPU_FAILURE);
	}


	// Initialize ADAU1962A
	if (Result == 0u) {
		Result = Adau1962aInit();
	}




	//DEVICE 1
	// Open the codec driver
	result1 = adi_adau1761_Open(ADAU1761_DEV_NUM, codecMem1,
	ADI_ADAU1761_MEMORY_SIZE, ADI_ADAU1761_COMM_DEV_TWI, &hADAU1761_1);
	CheckResult(result1);

	/* Configure TWI */
	result1 = adi_adau1761_ConfigTWI(hADAU1761_1, TWI_DEV_NUM,
			ADI_ADAU1761_TWI_ADDR0);
	CheckResult(result1);

	/* load Sigma Studio program exported from *_IC_1.h */
	result1 = adi_adau1761_SigmaStudioLoad(hADAU1761_1, default_download_IC_1);
	CheckResult(result1);

	/* config SPORT for Rx data transfer */
	sportRxInfo1.nDeviceNum = SPORT_RX_DEVICE1;
	sportRxInfo1.eChannel = ADI_HALF_SPORT_B;
	sportRxInfo1.eMode = ADI_ADAU1761_SPORT_I2S;
	sportRxInfo1.hDevice = NULL;
	sportRxInfo1.pMemory = sportRxMem1;
	sportRxInfo1.bEnableDMA = true;
	sportRxInfo1.eDataLen = ADI_ADAU1761_SPORT_DATA_24;
	sportRxInfo1.bEnableStreaming = true;

	result1 = adi_adau1761_ConfigSPORT(hADAU1761_1, ADI_ADAU1761_SPORT_INPUT,
			&sportRxInfo1);
	CheckResult(result1);
	/* register a Rx callback */
	result1 = adi_adau1761_RegisterRxCallback(hADAU1761_1, ADAU1761Callback_1,
	NULL);
	CheckResult(result1);
	//DISABLE MIXER BYPASS OUTPUT
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401C, 0x21); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401D, 0x00); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401E, 0x41); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401F, 0x00); // MUTE
	CheckResult(result1);
	//result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4019, 0x33);
	//CheckResult(result1);

	result1 = adi_adau1761_SetRegister (hADAU1761_1, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x01);
	CheckResult(result1);

#if defined(USE_LINE_IN)
	result1 = adi_adau1761_SelectInputSource(hADAU1761_1,
			ADI_ADAU1761_INPUT_ADC);
	CheckResult(result1);
#else
	result1 = adi_adau1761_SelectInputSource(hADAU1761_1, ADI_ADAU1761_INPUT_MIC);
	CheckResult(result1);
#endif

	// disable mixer
	//MixerEnable(false);

	result1 = adi_adau1761_SetVolume(hADAU1761_1, ADI_ADAU1761_VOL_HEADPHONE,
			ADI_ADAU1761_VOL_CHAN_BOTH, false, 0);
	CheckResult(result1);

	result1 = adi_adau1761_SetVolume(hADAU1761_1, ADI_ADAU1761_VOL_LINE_OUT,
			ADI_ADAU1761_VOL_CHAN_BOTH, false, 0);
	CheckResult(result1);

	result1 = adi_adau1761_SetSampleRate(hADAU1761_1, SAMPLE_RATE);
	CheckResult(result1);

#ifdef USE_ADAU1761_2
	//DEVICE 2
	// Open the codec driver
	result2 = adi_adau1761_Open(ADAU1761_DEV_NUM1, codecMem2,
			ADI_ADAU1761_MEMORY_SIZE, ADI_ADAU1761_COMM_DEV_TWI, &hADAU1761_2);
	CheckResult(result2);

	// Configure TWI
	//The TWI address when ADD1 pin is high and ADD0 pin is low
	result2 = adi_adau1761_ConfigTWI(hADAU1761_2, TWI_DEV_NUM,
			ADI_ADAU1761_TWI_ADDR2);
	CheckResult(result2);

	// load Sigma Studio program exported from *_IC_1.h
	result2 = adi_adau1761_SigmaStudioLoad(hADAU1761_2, default_download_IC_1);
	CheckResult(result2);
	// config SPORT for Rx data transfer
	sportRxInfo2.nDeviceNum = SPORT_RX_DEVICE2;
	sportRxInfo2.eChannel = ADI_HALF_SPORT_B;
	sportRxInfo2.eMode = ADI_ADAU1761_SPORT_I2S;
	sportRxInfo2.hDevice = NULL;
	sportRxInfo2.pMemory = sportRxMem2;
	sportRxInfo2.bEnableDMA = true;
	sportRxInfo2.eDataLen = ADI_ADAU1761_SPORT_DATA_24;
	sportRxInfo2.bEnableStreaming = true;

	result2 = adi_adau1761_ConfigSPORT(hADAU1761_2, ADI_ADAU1761_SPORT_INPUT,
			&sportRxInfo2);
	CheckResult(result2);

	// register a Rx callback
	result1 = adi_adau1761_RegisterRxCallback(hADAU1761_2, ADAU1761Callback_2,
			NULL);
	CheckResult(result2);

	//DISABLE MIXER BYPASS OUTPUT
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401C, 0x21); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401D, 0x00); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401E, 0x41); // MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401F, 0x00); // MUTE
	CheckResult(result1);
	//result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4019, 0x33); // MUTE
	//CheckResult(result1);

	result1 = adi_adau1761_SetRegister (hADAU1761_1, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x01);
	CheckResult(result1);
#if defined(USE_LINE_IN)
	result2 = adi_adau1761_SelectInputSource(hADAU1761_2,
			ADI_ADAU1761_INPUT_ADC);
	CheckResult(result2);
#else
	result2 = adi_adau1761_SelectInputSource(hADAU1761_2, ADI_ADAU1761_INPUT_MIC);
	CheckResult(result2);
#endif

	// disable mixer
	MixerEnable(false);

	result2 = adi_adau1761_SetVolume(hADAU1761_2, ADI_ADAU1761_VOL_HEADPHONE,
			ADI_ADAU1761_VOL_CHAN_BOTH, false, 0);
	CheckResult(result2);

	result2 = adi_adau1761_SetVolume(hADAU1761_2, ADI_ADAU1761_VOL_LINE_OUT,
			ADI_ADAU1761_VOL_CHAN_BOTH, false, 0);
	CheckResult(result2);

	result2 = adi_adau1761_SetSampleRate(hADAU1761_2, SAMPLE_RATE);
	CheckResult(result2);
#endif



	count = 0u;
#ifdef INSTANT_START
	bEvent = true;
	eMode = START_RECORDING;
#endif

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case SUBMIT_RX_BUFFER:
				//VolControl();
				Result = ProcessBuffers();
					//printf("ERR %d \n", Result);
				break;
			case START_RECORDING:
#ifdef USE_MICROPHONE
				printf("Microphone ");
#else
				printf("Line In ");
#endif
				printf("Recording started.\n");

				//enable mixer
				MixerEnable(true);
				//result1 = adi_adau1761_EnableOutput(hADAU1761_1, false);
				//CheckResult(result1);
				result1 = adi_adau1761_EnableInput(hADAU1761_1, false);
				CheckResult(result1);
#ifdef USE_ADAU1761_2
				//result2 = adi_adau1761_EnableOutput(hADAU1761_2, false);
				//CheckResult(result2);
				result2 = adi_adau1761_EnableInput(hADAU1761_2, false);
				CheckResult(result2);
#endif

				//submit ping pong buffer
				result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1,
						&AdcBuf1[BUFFER_SIZE_1761 * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result1);
				result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1,
						&AdcBuf1[BUFFER_SIZE_1761 * 1u],
						BUFFER_SIZE_1761);
				CheckResult(result1);

#ifdef USE_ADAU1761_2
				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2, &AdcBuf2[BUFFER_SIZE_1761 * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result2);
				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2, &AdcBuf2[BUFFER_SIZE_1761 * 1u],
						BUFFER_SIZE_1761);
				CheckResult(result2);
#endif

				//Start recording
 				result1 = adi_adau1761_EnableInput(hADAU1761_1, true);
				CheckResult(result1);
#ifdef USE_ADAU1761_2
				result1 = adi_adau1761_EnableInput(hADAU1761_2, true);
				CheckResult(result2);
#endif

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
			default:
				break;
			}
			bEvent = false;
		}

		/* wait for push button interrupts - exit the loop after a while */
		if (count > MAXCOUNT) {
			bExit = true;
		}
		count++;
	}

	//Close devices
	result1 = adi_adau1761_Close(hADAU1761_1);
	CheckResult(result1);
#ifdef USE_ADAU1761_2
	result2 = adi_adau1761_Close(hADAU1761_2);
	CheckResult(result2);
#endif
	// Disable data flow for the DAC
	if ((uint32_t) adi_adau1962a_Enable(phAdau1962a, false) != 0u) {
		// return error
		return 1u;
	}
	adi_adau1962a_Close(phAdau1962a);

	printf("%d a %d b ", (int) DacCount, (int) AdcCount);
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
			ADI_ADAU1962A_CHNL_DAC_MSTR, DacMasterVolume)) != ADI_ADAU1962A_SUCCESS) {
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


	// re-submit processed buffer from callback
	if (pRxBuffer1 != NULL) {


		// re-submit the ADC buffer
		adi_adau1761_SubmitRxBuffer(hADAU1761_1, pRxBuffer1,
		BUFFER_SIZE_1761);

	}
#ifdef USE_ADAU1761_2
	if (pRxBuffer2 != NULL) {

		// re-submit the ADC buffer
		adi_adau1761_SubmitRxBuffer(hADAU1761_2, pRxBuffer2,
				BUFFER_SIZE_1761);

	}
#endif
	if (pGetDAC != NULL) {
		/* re-submit the DAC buffer */
		adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC,
		AUDIO_BUFFER_SIZE);
	}





#ifdef USE_ADAU1761_2
	// process ADC to DAC buffer
	if ((sRxBuffer1 != NULL && sRxBuffer2 != NULL && (pDAC != NULL))) {

		pDst = (int32_t *) pDAC;
		uint32_t k = 0;
		uint32_t j = 0;
		/*
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES/8; i+=8) {

			pSrcL1[k] = sRxBuffer1[i];
			k++;
			pSrcR2[j] = sRxBuffer1[i+1];
			j++;
		}
*/

		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES; i++) {

			if (i % 2 == 0) {
				pSrcL1[i - k] = sRxBuffer1[i];
				pSrcL2[i - k] = sRxBuffer2[i];
				j ++;
			} else {
				pSrcR1[i - j] = sRxBuffer1[i];
				pSrcR2[i - j] = sRxBuffer2[i];
				k ++;
			}
		}

		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES / 2; i++) {
			//TDM8 SHIFT <<8
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
		}


	    pDAC = NULL;
#else
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
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES / 2; i++) {
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
			*pDst++ = pSrcL1[i]<<8;
			*pDst++ = pSrcR1[i]<<8;
		}
#endif
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
		//DacCount++;
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

void VolControl(){
	ADI_ADAU1962A_RESULT eResult;
	switch (eVolControl){
	case VOL_UP:
		//VOL
		if ((eResult = adi_adau1962a_SetVolume(phAdau1962a,
				ADI_ADAU1962A_CHNL_DAC_MSTR, DacMasterVolume)) != ADI_ADAU1962A_SUCCESS) {
			printf("ADAU1962A: Failed to set volume, Error Code: 0x%08X\n",
					eResult);
		}
		eVolControl= VOL_SAME;
		break;
	case VOL_DOWN:
		//VOL
		if ((eResult = adi_adau1962a_SetVolume(phAdau1962a,
				ADI_ADAU1962A_CHNL_DAC_MSTR, DacMasterVolume)) != ADI_ADAU1962A_SUCCESS) {
			printf("ADAU1962A: Failed to set volume, Error Code: 0x%08X\n",
					eResult);
		}
		eVolControl= VOL_SAME;
		break;
	default:
		//LED off
		adi_gpio_Clear(LED1_PORT, LED1_PIN);
		adi_gpio_Clear(LED2_PORT, LED2_PIN);
		break;
	}

}

