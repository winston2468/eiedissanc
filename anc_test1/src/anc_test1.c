/*****************************************************************************
 * anc_test1.c
 *****************************************************************************/
/*

 fid2 = fopen('lowpass_filter_500_1000.dat','w');
 Character2add = ','; % here put the character to be added

 for k = 1 : length(Num)
 fprintf(fid2,'%s%s\n',Num(k),Character2add);
 end

 fclose(fid2);
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
#include <matrix.h>
#include "adi_initialize.h"

#include <stdlib.h>
#include <adi_types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <drivers/fir/adi_fir.h>
#include <drivers/asrc/adi_asrc.h>
#include <services/pcg/adi_pcg.h>
#ifdef __ADSPARM__
#include <runtime/cache/adi_cache.h>

/* Caches are enabled by default on the ARM core, and buffers are placed in
 ** cached memory.
 */
#define USING_CACHE 1
#else
//#define  DO_CYCLE_COUNTS
#include <cycles.h>
#include <sys/cache.h>
/* On SHARC core we do not define USING_CACHE as in this example
 ** the buffers are allocated explicitly in L1 memory.
 */

#endif
/*

 #define DSTALIGN ADI_CACHE_ALIGN_MIN_4
 #define SRCALIGN _Pragma("align 4")
*/

#include <filter.h>
#include "cycle_count.h"
#include "anc_test1.h"

#define INSTANT_START
#define USE_ADAU1761_2
/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE
uint8_t AsrcMemory4[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc4;
uint8_t AsrcMemory5[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc5;
uint8_t AsrcMemory6[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc6;
uint8_t AsrcMemory7[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc7;

/* PCG - C */
uint8_t PcgMemory[ADI_PCG_MEMORY_SIZE];
static ADI_PCG_HANDLE phPcg;


/* DMA Stream Handle */
static ADI_DMA_STREAM_HANDLE   hMemDmaStream;
/* Source DMA Handle */
static ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel;
/* Destination DMA Handle */
static ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel;

static volatile bool bMemCopyInProgress;
static volatile bool bLowPassFIRInProgress;
static volatile bool bControlFIRInProgress;
static volatile bool bOSPMFIRInProgress;


#pragma alignment_region (4)
/* Memory to handle DMA Stream */
static uint8_t MemDmaStreamMem[ADI_DMA_STREAM_REQ_MEMORY];
static ADI_DMA_2D_MEM_TRANSFER Src_2DMemXfer;
static ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXfer;

/* ----------------------------   FIR Configuration ------------------------------------------- */

float Left[NUM_AUDIO_SAMPLES_ADC_SINGLE] = { 0 };
float Right[NUM_AUDIO_SAMPLES_ADC_SINGLE] = { 0 };
float refSignal[NUM_AUDIO_SAMPLES_ADC_SINGLE] = { 0 };
float refSignalPastFuture[refInputSize] = { 0 };
float refSignalControlPastFuture[controlInputSize] = { 0 };
float refSignalOSPMPastFuture[OSPMInputSize] = { 0 };
float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_ADC / 2] = { 0 };
float OSPMAWGNSignalPastFuture[numControlSignal][OSPMInputSize] = { 0 };

/* Channel Configurations parameters */
//Max FIR channels = 32
#ifdef LowPassFilter
uint8_t ConfigMemoryRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryRef[ADI_FIR_CHANNEL_MEMORY_SIZE];
ADI_FIR_CHANNEL_PARAMS channelRef;
float refInputBuff[refLength] = { 0 };
float refCoeffBuff[refLength] = {
#include "data/lowpass_filter_2000_2500.dat"
		};
#endif

uint8_t ConfigMemoryControl[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryControl[numControlSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
ADI_FIR_CHANNEL_PARAMS channelControl[numControlSignal];
float controlInputBuff[numControlSignal][refInputSize] = { 0 };
float controlCoeffBuff[numControlSignal][controlLength] = { 0 };

uint8_t ConfigMemoryOSPM[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOSPM[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
ADI_FIR_CHANNEL_PARAMS channelOSPM[numControlSignal][numErrorSignal];
float OSPMInputBuff[numControlSignal][numErrorSignal][refInputSize] = { 0 };
float OSPMCoeffBuff[numControlSignal][numErrorSignal][OSPMLength] = { 0 };

float OSPMRef[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };
//float OSPMAux[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };

float outputSignal[numControlSignal][NUM_AUDIO_SAMPLES_DAC] = { 0 };

float OSPMAWGNGain[numControlSignal][OSPMLength] = { 0 };
float OSPMAWGNSignal[numControlSignal][OSPMLength] = { 0 };
float powerOSPMAWGNSignal[numControlSignal][OSPMLength] = { 0 };
float filteredErrorSignal[numErrorSignal][OSPMLength] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength];
float powerIndirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength] = {
		0 };
float powerFilteredErrorSignal[numErrorSignal][OSPMLength] = { 0 };
float stepSizeS[numControlSignal][numErrorSignal][OSPMLength] = { 0 };
float stepSizeW[numControlSignal] = { 0 };

float controlOutputBuff[numControlSignal][controlOutputSize];
float OSPMOutputBuff[numControlSignal][numErrorSignal][OSPMOutputSize];

#pragma alignment_region_end

#ifdef LowPassFilter
ADI_CACHE_ALIGN float refOutputBuff[ADI_CACHE_ROUND_UP_SIZE(refOutputSize, float)];;
#endif



 ADI_FIR_RESULT res;
 ADI_FIR_HANDLE hFir;
 ADI_FIR_CONFIG_HANDLE hConfigControl, hConfigOSPM;

ADI_FIR_CHANNEL_HANDLE hChannelOSPM[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelControl[numControlSignal];

#ifdef LowPassFilter
ADI_FIR_CONFIG_HANDLE hConfigRef;
ADI_FIR_CHANNEL_HANDLE hChannelRef;


#endif



float forgettingFactor = 0.6;
float stepSizeSMin = 0.001;

/* the ADAU1761 Rec Mixer Left 0 register */
#define REC_MIX_LEFT_REG    (0x400A)
/* the ADAU1761 Rec Mixer Right 0 register */
#define REC_MIX_RIGHT_REG   (0x400C)

/* codec device instance to be tested */
#define ADAU1761_DEV_NUM          0

#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)

/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

#pragma align(4)
static uint8_t sportRxMem1[ADI_SPORT_DMA_MEMORY_SIZE];

/* Memory required for codec driver */
static uint8_t codecMem1[ADI_ADAU1761_MEMORY_SIZE];

/* adau1761 device handle */
static ADI_ADAU1761_HANDLE hADAU1761_1;
#pragma align 4
int32_t AdcBuf1[NUM_AUDIO_SAMPLES_ADC * 2];
static int32_t *pRxBuffer1;

#ifdef USE_ADAU1761_2

static ADI_ADAU1761_HANDLE hADAU1761_2;

#define ADAU1761_DEV_NUM1          1
#pragma align 4
int32_t AdcBuf2[NUM_AUDIO_SAMPLES_ADC * 2];

#pragma align(4)
static uint8_t sportRxMem2[ADI_SPORT_DMA_MEMORY_SIZE];

static uint8_t codecMem2[ADI_ADAU1761_MEMORY_SIZE];

static int32_t *pRxBuffer2;

#endif

int32_t *pSrc;

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

static MODE eMode = NONE;

static uint32_t gpioMemory[GPIO_MEMORY_SIZE];

static bool bError;
static uint32_t count;

volatile bool bEvent;

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
int32_t DacBuf[NUM_AUDIO_SAMPLES_DAC * NUM_DAC_CHANNELS * 2];

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
int32_t FIR_init(void);
void reverseArray(int32_t*, uint32_t);
void reverseArrayf(float*, uint32_t);


float AWGN_generator(void);
uint32_t SpuInit(void);
uint32_t DMAInit(void);
uint32_t PcgInit(void);
uint32_t AsrcDacInit(void);
int32_t ANCALG_1(void);
int32_t ANCALG_2(void);
int8_t DisableAllFIRChannels(void);
static void LowPassFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void ControlFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OSPMFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void MemDmaCallback(void *pCBParam, uint32_t Event, void *pArg);

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
		eMode = SUBMIT_RX_BUFFER;
		bEvent = true;
		break;
	default:
		printf("test1");
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
		eMode = SUBMIT_RX_BUFFER;
		bEvent = true;

		break;
	default:
		printf("test2");
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

			bEvent = true;
		}
	}
	if (ePinInt == PUSH_BUTTON2_PINT) {
		//push button 2
		if (PinIntData & PUSH_BUTTON2_PINT_PIN) {
			//LED on
			adi_gpio_Set(LED2_PORT, LED2_PIN);
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
		//0x5B 0 db
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_LEFT_REG, 0x7F); /* +6 dB */
		CheckResult(result1);

		/* enable the record mixer (right) */
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_RIGHT_REG,
				0x7F); /* 0 dB */
		CheckResult(result1);
		//RISING EDGE BCLK POLARITY
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x11); /* MUTE */
		//CheckResult(result);
#ifdef USE_ADAU1761_2
		/* enable the record mixer (left) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_LEFT_REG, 0x7F); /* 6 dB */
		CheckResult(result2);

		/* enable the record mixer (right) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_RIGHT_REG,
				0x7F); /* 0 dB */
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

int32_t FIR_init() {

	//FIR stuff
	ADI_FIR_RESULT res;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			powerOSPMAWGNSignal[j][i] = 0.0001;
		}
	}
	for (uint8_t k = 0; k < numErrorSignal; k++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			powerFilteredErrorSignal[k][i] = 1.0;
		}
	}
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OSPMLength; i++) {
				powerIndirectErrorSignal[j][k][i] = 1.0;
			}
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			OSPMAWGNGain[j][i] = 0.00001;
		}
	}

	/*
	 for (int32_t j = 0; j < numControlSignal; j++) {
	 for (int32_t k = 0; k < numErrorSignal; k++) {
	 for (int32_t i = 0; i < OSPMLength; i++) {
	 stepSizeS[j][k][i] = 0.001;
	 }
	 }
	 }
	 */
	for (uint8_t j = 0; j < numControlSignal; j++) {
		stepSizeW[j] = 0.0001;
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < controlLength; i++) {
			controlCoeffBuff[j][i] = 0.0000001;		//0.0000001;
		}
	}
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OSPMLength; i++) {
				OSPMCoeffBuff[j][k][i] = 0.0000001;
			}
		}
	}
#ifdef LowPassFilter
	reverseArrayf(refCoeffBuff, refLength);
	channelRef.nTapLength = refLength;
	channelRef.nWindowSize = refWindowSize;
	channelRef.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelRef.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelRef.nGroupNum = 0u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelRef.pInputBuffBase = (void *) refInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelRef.pInputBuffIndex = (void *) refInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelRef.nInputBuffCount = refInputSize; /*!< Number of elements in the input circular buffer */
	channelRef.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelRef.pCoefficientBase = (void *) refCoeffBuff; /*!< Pointer to the start of the coefficient buffer */
	channelRef.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelRef.pCoefficientIndex = (void *) refCoeffBuff; /*!< Pointer to the start of the coefficient buffer */

	channelRef.pOutputBuffBase = (void *) refOutputBuff; /*!< Pointer to the base of the output circular buffer */
	channelRef.pOutputBuffIndex = (void *) refOutputBuff; /*!< Pointer to the current index of the output circular buffer */
	channelRef.nOutputBuffCount = refWindowSize; /*!< Number of elements in the output circular buffer */
	channelRef.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */
#endif

	channelControl[0].nTapLength = controlLength;
	channelControl[0].nWindowSize = controlWindowSize;
	channelControl[0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelControl[0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelControl[0].nGroupNum = 0u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelControl[0].pInputBuffBase = (void *) controlInputBuff[0]; /*!< Pointer to the base of the input circular buffer */
	channelControl[0].pInputBuffIndex = (void *) controlInputBuff[0]; /*!< Pointer to the current index of the input circular buffer */
	channelControl[0].nInputBuffCount = controlInputSize; /*!< Number of elements in the input circular buffer */
	channelControl[0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelControl[0].pCoefficientBase = (void *) controlCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */
	channelControl[0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl[0].pCoefficientIndex = (void *) controlCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */

	channelControl[0].pOutputBuffBase = (void *) controlOutputBuff[0]; /*!< Pointer to the base of the output circular buffer */
	channelControl[0].pOutputBuffIndex = (void *) controlOutputBuff[0]; /*!< Pointer to the current index of the output circular buffer */
	channelControl[0].nOutputBuffCount = controlWindowSize; /*!< Number of elements in the output circular buffer */
	channelControl[0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t j = 1; j < numControlSignal; j++) {
		channelControl[j] = channelControl[0];

		channelControl[j].pCoefficientBase = (void *) controlCoeffBuff[j]; /*!< Pointer to the start of the coefficient buffer */
		channelControl[j].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelControl[j].pCoefficientIndex = (void *) controlCoeffBuff[j]; /*!< Pointer to the start of the coefficient buffer */

		channelControl[j].pOutputBuffBase = (void *) controlOutputBuff[j]; /*!< Pointer to the base of the output circular buffer */
		channelControl[j].pOutputBuffIndex = (void *) controlOutputBuff[j]; /*!< Pointer to the current index of the output circular buffer */
		channelControl[j].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */
	}

	channelOSPM[0][0].nTapLength = OSPMLength;
	channelOSPM[0][0].nWindowSize = OSPMWindowSize;
	channelOSPM[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOSPM[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOSPM[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOSPM[0][0].pInputBuffBase = (void *) OSPMInputBuff[0][0]; /*!< Pointer to the base of the input circular buffer */
	channelOSPM[0][0].pInputBuffIndex = (void *) OSPMInputBuff[0][0]; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM[0][0].nInputBuffCount = OSPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOSPM[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM[0][0].pCoefficientBase = (void *) OSPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM[0][0].pCoefficientIndex = (void *) OSPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM[0][0].pOutputBuffBase = (void *) OSPMOutputBuff[0][0]; /*!< Pointer to the base of the output circular buffer */
	channelOSPM[0][0].pOutputBuffIndex = (void *) OSPMOutputBuff[0][0]; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM[0][0].nOutputBuffCount = OSPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOSPM[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOSPM[0][k] = channelOSPM[0][0];

		channelOSPM[0][k].pInputBuffBase = (void *) OSPMInputBuff[0][k]; /*!< Pointer to the base of the input circular buffer */
		channelOSPM[0][k].pInputBuffIndex = (void *) OSPMInputBuff[0][k]; /*!< Pointer to the current index of the input circular buffer */
		channelOSPM[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOSPM[0][k].pCoefficientBase = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOSPM[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOSPM[0][k].pCoefficientIndex = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOSPM[0][k].pOutputBuffBase = (void *) OSPMOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOSPM[0][k].pOutputBuffIndex = (void *) OSPMOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOSPM[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOSPM[j][k] = channelOSPM[0][0];

			channelOSPM[j][k].pInputBuffBase = (void *) OSPMInputBuff[j][k]; /*!< Pointer to the base of the input circular buffer */
			channelOSPM[j][k].pInputBuffIndex = (void *) OSPMInputBuff[j][k]; /*!< Pointer to the current index of the input circular buffer */
			channelOSPM[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOSPM[j][k].pCoefficientBase = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOSPM[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOSPM[j][k].pCoefficientIndex = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOSPM[j][k].pOutputBuffBase = (void *) OSPMOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOSPM[j][k].pOutputBuffIndex = (void *) OSPMOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOSPM[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}

	res = adi_fir_Open(0u, &hFir);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_Open failed\n");
		return -1;
	}

	// ------------------------------- Create Configurations --------------------------------------------
#ifdef LowPassFilter
	res = adi_fir_CreateConfig(hFir, ConfigMemoryRef,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback (hConfigRef, LowPassFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_RegisterCallback failed\n");
				return -1;
	}
#endif
	res = adi_fir_CreateConfig(hFir, ConfigMemoryControl,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigControl);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback (hConfigControl, ControlFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_RegisterCallback failed\n");
				return -1;
	}
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOSPM,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOSPM);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback (hConfigOSPM, OSPMFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_RegisterCallback failed\n");
				return -1;
	}

	// ----------------------------------  Add Channels ---------------------------------------------------
#ifdef LowPassFilter
	res = adi_fir_AddChannel(hConfigRef, ChannelMemoryRef,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelRef, &hChannelRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel Ref failed\n");
		return -1;
	}
#endif
	for (int8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl[j],
		ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl[j], &hChannelControl[j]);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_AddChannel Control %d failed\n", j);
			return -1;
		}

	}
	for (int8_t j = 0; j < numControlSignal; j++) {
		for (int8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM[j][k],
					&hChannelOSPM[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OSPM %d%d failed\n");
				return -1;
			}
			/*
			 res = adi_fir_EnableChannel (hChannelOSPM33, true);
			 if( res != ADI_FIR_RESULT_SUCCESS) {
			 printf("adi_fir_EnableChannel %d%d failed\n", j, k);
			 return -1;
			 }

			 */
		}
	}

	res = adi_fir_ChannelInterruptEnable(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	res = adi_fir_ChannelInterruptEnable(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
#ifdef LowPassFilter
	res = adi_fir_ChannelInterruptEnable(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
#endif

	return 0;
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
	int8_t firResult = 0;
	bool bExit;
	uint32_t Result = 0u;
	//uint32_t i;
	bExit = false;

	adi_initComponents(); /* auto-generated code */
	//FIR driver stuff
#if defined (__ADSPSC589_FAMILY__)
	*pREG_SPU0_SECUREP155 = 2u;
#elif defined (__ADSPSC573_FAMILY__)
	*pREG_SPU0_SECUREP110 = 2u;
#endif

	/* Software Switch Configuration for the EZ-BOARD */
	ConfigSoftSwitches();
	if (Result == 0u) {
		Result = SpuInit();
	}
	if (Result == 0u) {
		Result = DMAInit();
	}

	if (Result == 0u) {
		Result = PcgInit();
	}
	if (Result == 0u) {
		Result = AsrcDacInit();
	}

	configGpio();
	/* Enable the PCG */
	if (Result == 0u) {
		Result = (uint32_t) adi_pcg_Enable(phPcg, true);
	}
	/* Enable the ASRC */
	if (Result == 0u) {
		Result = (uint32_t) adi_asrc_Enable(phAsrc4, true);
	}
	/* Enable the ASRC */
	if (Result == 0u) {
		Result = (uint32_t) adi_asrc_Enable(phAsrc5, true);
	}
	/* Enable the ASRC */
	if (Result == 0u) {
		Result = (uint32_t) adi_asrc_Enable(phAsrc6, true);
	}
	/* Enable the ASRC */
	if (Result == 0u) {
		Result = (uint32_t) adi_asrc_Enable(phAsrc7, true);
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
	sportRxInfo1.eDataLen = ADI_ADAU1761_SPORT_DATA_32;
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

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4015, 0x01);
	CheckResult(result1);
	//high pass 2 hz filter
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4019, 0x33);

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
	sportRxInfo2.eDataLen = ADI_ADAU1761_SPORT_DATA_32;
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

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4015, 0x01);
	CheckResult(result1);
	//high pass 2 hz filter
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4019, 0x33);
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
	firResult = FIR_init();
	if (firResult == 0) {
		printf("FIR init success\n");
	}
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
						&AdcBuf1[NUM_AUDIO_SAMPLES_ADC * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result1);
				result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1,
						&AdcBuf1[NUM_AUDIO_SAMPLES_ADC * 1u],
						BUFFER_SIZE_1761);
				CheckResult(result1);

#ifdef USE_ADAU1761_2
				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
						&AdcBuf2[NUM_AUDIO_SAMPLES_ADC * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result2);
				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
						&AdcBuf2[NUM_AUDIO_SAMPLES_ADC * 1u],
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
		/*
		 // wait for push button interrupts - exit the loop after a while
		 if (count > MAXCOUNT) {
		 bExit = true;
		 }
		 count++;
		 */
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
	printf("output1 %f OSPMAWGNSignal1 %f pSrcL1 %f yo %d\n",
			outputSignal[0][0], OSPMAWGNSignal[0][0], refSignal[0],
			conv_fix_by(outputSignal[0][0], 10));
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
			ADI_ADAU1962A_CHNL_DAC_MSTR, DacMasterVolume))
			!= ADI_ADAU1962A_SUCCESS) {
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
			&DacBuf[NUM_AUDIO_SAMPLES_DAC * NUM_DAC_CHANNELS * 0u],
			AUDIO_BUFFER_SIZE) != 0u) {
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if ((uint32_t) adi_adau1962a_SubmitBuffer(phAdau1962a,
			&DacBuf[NUM_AUDIO_SAMPLES_DAC * NUM_DAC_CHANNELS * 1u],
			AUDIO_BUFFER_SIZE) != 0u) {
		/* return error */
		return 1u;
	}

	return Result;
}

float AWGN_generator() {/* Generates additive white Gaussian Noise samples with zero mean and a standard deviation of 1. */

	float temp1;
	float temp2;
	float result;
	int p;

	p = 1;

	while (p > 0) {
		temp2 = (rand() / ((float) RAND_MAX)); //  rand() function generates an integer between 0 and  RAND_MAX, which is defined in stdlib.h.

		if (temp2 == 0) {				// temp2 is >= (RAND_MAX / 2)
			p = 1;
		}				// end if
		else {				// temp2 is < (RAND_MAX / 2)
			p = -1;
		}				// end else

	}				// end while()

	temp1 = cosf((2.0 * 3.1415926536) * rand() / ((float) RAND_MAX));
	result = sqrtf(-2.0 * logf(temp2)) * temp1;
	return result;	// return the generated random sample to the caller

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
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;
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
		// re-submit the DAC buffer
		adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC,
		AUDIO_BUFFER_SIZE);
	}

#ifdef USE_ADAU1761_2
	// process ADC to DAC buffer

	if ((pRxBuffer1 != NULL && pRxBuffer2 != NULL && (pDAC != NULL))) {
		//DacCount++;
		pDst = (int32_t *) pDAC;
		while (bMemCopyInProgress);
#pragma vector_for(4)
		for (int32_t i = 0; i < NUM_AUDIO_SAMPLES_ADC_SINGLE; i++) {
			refSignal[i] = conv_float_by((pRxBuffer1[2 * i] << 8), -20);
			errorSignal[0][i] = conv_float_by((pRxBuffer1[2 * i + 1] << 8), -20);
			errorSignal[1][i] = conv_float_by((pRxBuffer2[2 * i] << 8), -20);
			errorSignal[2][i] = conv_float_by((pRxBuffer2[2 * i + 1] << 8), -20);


		}


		    /* Initialize flag */
		    bMemCopyInProgress = true;
			eResult = adi_mdma_Copy1D (hMemDmaStream,
				                                   (void *)&refSignalPastFuture[(refLength/2)],
				                                   (void *)&refSignal[0],
				                                   MEMCOPY_MSIZE,
												   (refLength -1));

		    /* IF (Failure) */
		    if (eResult != ADI_DMA_SUCCESS)
		    {
		    	DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
		    }


			while (bMemCopyInProgress);


		ANCALG_1();



		for (int32_t i=0,j=NUM_AUDIO_SAMPLES_DAC -1; i < NUM_AUDIO_SAMPLES_DAC, j>-1;i++, j--) {
			//TDM8 SHIFT <<8

			*pDst++ = (conv_fix_by(refSignal[j], 7)) ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 7)) ;
			*pDst++ = (conv_fix_by(refSignal[i], 7)) ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 7));
			*pDst++ = (conv_fix_by(refSignal[i], 7));
			*pDst++ = (conv_fix_by(errorSignal[0][i], 7));
			*pDst++ = (conv_fix_by(refSignal[i], 7)) ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 7));

		}
/*

		for (int32_t i=0,j=NUM_AUDIO_SAMPLES_DAC -1; i < NUM_AUDIO_SAMPLES_DAC, j>-1;i++, j--) {
			//TDM8 SHIFT <<8

			*pDst++ = (conv_fix_by(outputSignal[0][i], 7)) ;
			*pDst++ = (conv_fix_by(outputSignal[1][i], 7)) ;
			*pDst++ = (conv_fix_by(outputSignal[2][i], 7)) ;
			*pDst++ = (conv_fix_by(outputSignal[3][i], 7)) ;
			*pDst++ = (conv_fix_by(outputSignal[4][i], 7)) ;
			*pDst++ = 0 ;
			*pDst++ = (conv_fix_by(outputSignal[5][i], 7)) ;
			*pDst++ = 0 ;

		}
*/

		ANCALG_2();


	    /* Initialize flag */
	    bMemCopyInProgress = true;
		eResult = adi_mdma_Copy1D (hMemDmaStream,
			                                   (void *)&refSignalPastFuture[0],
			                                   (void *)&refSignal[0],
			                                   MEMCOPY_MSIZE,
											   refLength);

	    /* IF (Failure) */
	    if (eResult != ADI_DMA_SUCCESS)
	    {
	    	DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
	    }





	}

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
			*pDst++ = (int32_t) pSrcL1[i]<<8;
			*pDst++ = (int32_t) pSrcR1[i]<<8;
			*pDst++ = (int32_t) pSrcL1[i]<<8;
			*pDst++ = (int32_t) pSrcR1[i]<<8;
			*pDst++ = (int32_t) pSrcL1[i]<<8;
			*pDst++ = (int32_t) pSrcR1[i]<<8;
			*pDst++ = (int32_t) pSrcL1[i]<<8;
			*pDst++ = (int32_t) pSrcR1[i]<<8;
		}
	}
#endif

	return 0u;
}

void reverseArray(int32_t arr[], uint32_t size) {
#pragma vector_for
	for (uint32_t i = 0; i < (size / 2); i++) {
		int32_t temp = arr[i];
		arr[i] = arr[size - i - 1];
		arr[size - i - 1] = temp;
	}

}

void reverseArrayf(float arr[], uint32_t arrLength) {
#pragma vector_for
	for (uint32_t i = 0; i < (arrLength / 2); i++) {
		float temp = arr[i];
		arr[i] = arr[arrLength - i - 1];
		arr[arrLength - i - 1] = temp;
	}
}

int8_t DisableAllFIRChannels() {
	ADI_FIR_RESULT res;
	// ---------------------------------------   Enable Channels -----------------------------------------------
#ifdef LowPassFilter
	res = adi_fir_EnableChannel(hChannelRef, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelRef\n");
		return -1;
	}
#endif
	for (uint8_t j = 0; j < numControlSignal; j++) {

		res = adi_fir_EnableChannel(hChannelControl[j], false);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel failed hChannelControl%d\n", j);
			return -1;
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hChannelOSPM[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel failed hChannelOSPM%d%d\n", j, k);
				return -1;
			}
		}
	}
	return 0;

}

int32_t ANCALG_1(void) {
	ADI_FIR_RESULT res;
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;
#ifdef LowPassFilter
	res = adi_fir_SubmitInputCircBuffer(hChannelRef, refSignalPastFuture, refSignalPastFuture,
	refInputSize, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	bLowPassFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	while(bLowPassFIRInProgress);
	/*
	res = adi_fir_WaitForEvent(hConfigRef, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed Ref\n");
		return -1;
	}
	*/
	//DMACopy_4Bytes_1D(refSignalControlPastFuture, refOutputBuff, (controlLength/2), 0, (controlLength -1));

    bMemCopyInProgress = true;
	eResult = adi_mdma_Copy1D (hMemDmaStream,
		                                   (void *)&refSignalControlPastFuture[(controlLength/2)],
										   (void *)&refOutputBuff[0],
		                                   MEMCOPY_MSIZE,
										   (controlLength -1));

    /* IF (Failure) */
    if (eResult != ADI_DMA_SUCCESS)
    {
    	DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
    }


	while (bMemCopyInProgress);


/*
	memcpy(&refSignalControlPastFuture[(controlInputSize + 1) / 2], refOutputBuff,
			((controlInputSize + 1) / 2 - 1) * 4u);

*/

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {

		res = adi_fir_SubmitInputCircBuffer(hChannelControl[j], refSignalControlPastFuture,
				refSignalControlPastFuture, controlInputSize, 1);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_SubmitInputCircBuffer hChannelControl[%d] failed\n",
					j);
			return -1;
		}
	}

#else
#pragma vector_for
	for(uint8_t j = 0; j < numControlSignal; j++) {

		res = adi_fir_SubmitInputCircBuffer(hChannelControl[j],
				refSignalPastFuture, refSignalPastFuture, controlInputSize, 1);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_SubmitInputCircBuffer hChannelControl[%d] failed\n", j);
			return -1;
		}
	}
#endif


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {

		res = adi_fir_EnableChannel(hChannelControl[j], true);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel hChannelControl[%d] failed\n", j);
			return -1;
		}
	}
	bControlFIRInProgress = true;

	res = adi_fir_EnableConfig(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	//OSPM
#ifdef LowPassFilter
	//DMACopy_4Bytes_1D(refSignalOSPMPastFuture, refOutputBuff, (OSPMLength/2), 0, (OSPMLength -1));

    bMemCopyInProgress = true;
	eResult = adi_mdma_Copy1D (hMemDmaStream,
		                                   (void *)&refSignalOSPMPastFuture[(OSPMLength/2)],
										   (void *)&refOutputBuff[0],
		                                   MEMCOPY_MSIZE,
										   (OSPMLength -1));

    /* IF (Failure) */
    if (eResult != ADI_DMA_SUCCESS)
    {
    	DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
    }


	while (bMemCopyInProgress);


/*
	memcpy(&refSignalOSPMPastFuture[(OSPMInputSize + 1) / 2], refOutputBuff,
			((OSPMInputSize + 1) / 2 - 1) * 4u);
			*/
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hChannelOSPM[j][k],
					refSignalOSPMPastFuture, refSignalOSPMPastFuture, OSPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hChannelOSPM[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}
#else
#pragma vector_for
	for(uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for(uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hChannelOSPM[j][k], refSignalPastFuture,
					refSignalPastFuture, OSPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_SubmitInputCircBuffer hChannelOSPM[%d][%d] failed\n", j ,k);
				return -1;
			}
		}
	}
#endif
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hChannelOSPM[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hChannelOSPM[%d][%d] failed\n", j,
						k);
				return -1;
			}

		}
	}

	bOSPMFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}



	//do stuff while OSPM FIR
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for

		for (uint32_t i = 0; i < OSPMLength; i++) {
			OSPMAWGNSignal[j][i] = (float) OSPMAWGNGain[j][i]
					* (float) AWGN_generator();
		}



		//DMACopy_4Bytes_1D(OSPMAWGNSignalPastFuture[j], OSPMAWGNSignal[j], (OSPMLength/2), 0, (OSPMLength -1));

	    bMemCopyInProgress = true;
		eResult = adi_mdma_Copy1D (hMemDmaStream,
			                                   (void *)&OSPMAWGNSignalPastFuture[j][(OSPMLength/2)],
											   (void *)&OSPMAWGNSignal[j][0],
			                                   MEMCOPY_MSIZE,
											   (OSPMLength -1));

	    /* IF (Failure) */
	    if (eResult != ADI_DMA_SUCCESS)
	    {
	    	DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
	    }


		while (bMemCopyInProgress);

		/*
		memcpy(&OSPMAWGNSignalPastFuture[j][(OSPMInputSize + 1) / 2], OSPMAWGNSignal[j],
				((OSPMInputSize + 1) / 2 - 1) * 4u);
				*/
	}



	while(bControlFIRInProgress);

	/*

		res = adi_fir_WaitForEvent(hConfigControl, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
				hChannelControl[5]);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_WaitForEvent failed hChannel1_1\n");
			return -1;
		}
	*/

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_DAC; i++) {
			outputSignal[j][i] = controlOutputBuff[j][i] + OSPMAWGNSignal[j][i];
		}

	}
	/*
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		reverseArrayf(outputSignal[j], NUM_AUDIO_SAMPLES_DAC);
	}
	*/
	return 0;

}

int32_t ANCALG_2(void) {
	ADI_FIR_RESULT res;
	int8_t disableAllFirChannelsResult = 0;
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;
	while(bOSPMFIRInProgress);
	/*
	res = adi_fir_WaitForEvent(hConfigOSPM, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelOSPM[1][1]);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed hChannelOSPM ANCALG_2\n");
		return -1;
	}
*/
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hChannelOSPM[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_EnableChannel disable hChannelOSPM[%d][%d] failed\n",
						j, k);
				return -1;
			}
		}
	}

//#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		//DMACopy_4Bytes_2D(OSPMRef[j], OSPMOutputBuff[j], 0, 0, 0, 0, OSPMWindowSize, numErrorSignal);



		    /* Initialize flag */
		    bMemCopyInProgress = true;
		    /* Populate 2D Memory transfer instance for source channel */
		    Src_2DMemXfer.pStartAddress    = &OSPMOutputBuff[j][0][0];
		    Src_2DMemXfer.YCount           = numErrorSignal;                    /* Configure YCount for 2D transfer */
		    Src_2DMemXfer.YModify          = 4;
		    Src_2DMemXfer.XCount           = OSPMWindowSize;
		    Src_2DMemXfer.XModify          = 4;

		    /* Populate 2D Memory transfer instance for destination channel */
		    Dest_2DMemXfer.pStartAddress   = &OSPMRef[j][0][0];
		    Dest_2DMemXfer.YCount          = numErrorSignal;                    /* Configure YCount for 2D transfer */
		    Dest_2DMemXfer.YModify         = 4;
		    Dest_2DMemXfer.XCount          = OSPMWindowSize;
		    Dest_2DMemXfer.XModify         = 4;

		    eResult = adi_mdma_Copy2D (hMemDmaStream,
		    							MEMCOPY_MSIZE,
		                               &Dest_2DMemXfer,
		                               &Src_2DMemXfer);

		    /* IF (Failure) */
		    if (eResult != ADI_DMA_SUCCESS)
		    {
		    	DBG_MSG("Failed initialize MDMA 2D Copy \n", eResult);
		    }


			while (bMemCopyInProgress);



		/*
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			memcpy(&OSPMRef[j][k], OSPMOutputBuff[j][k], OSPMWindowSize * 4u);
		}
		*/
	}



#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_SubmitInputCircBuffer(hChannelOSPM[j][k],
					OSPMAWGNSignal[j], OSPMAWGNSignal[j], OSPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hChannelOSPM[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}

#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hChannelOSPM[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel ChannelOSPM[%d][%d] failed\n", j,
						k);
				return -1;
			}

		}
	}
	bOSPMFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}
	while(bOSPMFIRInProgress);
/*
	res = adi_fir_WaitForEvent(hConfigOSPM, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelOSPM[0][0]);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed hChannel1_1\n");
		return -1;
	}
	*/

	//cycle_t start_count=0;

	// cycle_t final_count=0;
	//START_CYCLE_COUNT(start_count);
#pragma vector_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
		for (int32_t i = 0; i < OSPMOutputSize; i++) {

			float OSPMAuxSumTemp = 0;
			for (uint8_t j = 0; j < numControlSignal; j++) {
				OSPMAuxSumTemp += OSPMOutputBuff[j][k][i];
			}
			filteredErrorSignal[k][i] = errorSignal[k][i] - OSPMAuxSumTemp;

		}
	}

	//INDIRECT ERROR SIGNAL
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				indirectErrorSignal[j][k][i] = filteredErrorSignal[k][i]
						+ OSPMOutputBuff[j][k][i];
			}
		}
	}

	//	power of OSPMAWGNSignal
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(OSPMOutputSize)
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			powerOSPMAWGNSignal[j][i] = forgettingFactor
					* powerOSPMAWGNSignal[j][i]
					+ (1.0 - forgettingFactor) * OSPMAWGNSignal[j][i]
							* OSPMAWGNSignal[j][i];
		}
	}

	//	power of filteredErrorSignal
#pragma vector_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			powerFilteredErrorSignal[k][i] = forgettingFactor
					* powerFilteredErrorSignal[k][i]
					+ (1.0 - forgettingFactor) * filteredErrorSignal[k][i]
							* filteredErrorSignal[k][i];
		}
	}

	//power of indirectErrorSignal
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				powerIndirectErrorSignal[j][k][i] = forgettingFactor
						* powerIndirectErrorSignal[j][k][i]
						+ (1.0 - forgettingFactor)
								* indirectErrorSignal[j][k][i]
								* indirectErrorSignal[j][k][i];
			}
		}
	}

	//stepSizeS
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				stepSizeS[j][k][i] = (powerOSPMAWGNSignal[j][i] * stepSizeSMin)
						/ powerIndirectErrorSignal[j][k][i];
			}
		}
	}

	//OSPMAWGNGain
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			float powerFilteredErrorSignalSumTemp = 0;
			float powerpowerIndirectErrorSignalTemp = 0;

			for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerFilteredErrorSignalSumTemp +=
						powerFilteredErrorSignal[k][i];
				powerpowerIndirectErrorSignalTemp +=
						powerIndirectErrorSignal[j][k][i];
			}
			OSPMAWGNGain[j][i] = powerFilteredErrorSignalSumTemp
					/ powerpowerIndirectErrorSignalTemp;
		}
	}

	disableAllFirChannelsResult = DisableAllFIRChannels();
	if (disableAllFirChannelsResult != 0) {
		printf("error disabling FIR channels");
	}

	//OSPM Coeff

#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (int32_t i = 0, l=OSPMOutputSize-1; i < OSPMOutputSize, l>(-1); i++,l--) {
				OSPMCoeffBuff[j][k][l] = OSPMCoeffBuff[j][k][l]
						+ (stepSizeS[j][k][i] * OSPMAWGNSignal[j][i]
								* filteredErrorSignal[k][i]);

			}
		}
	}
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (uint8_t k = 0; k < numErrorSignal; k++) {
			adi_fir_SetChannelCoefficientBuffer(hChannelOSPM[j][k],
					OSPMCoeffBuff[j][k], OSPMCoeffBuff[j][k], 1);
		}
	}

	//Control Coeff
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (int32_t i = 0, l=controlOutputSize-1; i < controlOutputSize, l>(-1); i++,l--) {
			float filteredErrorSignal_OSPMRef_SumTemp = 0;

			for (uint8_t k = 0; k < numErrorSignal; k++) {
				filteredErrorSignal_OSPMRef_SumTemp += filteredErrorSignal[k][i]
						* OSPMRef[j][k][i];
			}
			controlCoeffBuff[j][l] = controlCoeffBuff[j][l]
					+ stepSizeW[j] * filteredErrorSignal_OSPMRef_SumTemp;

		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		adi_fir_SetChannelCoefficientBuffer(hChannelControl[j],
				controlCoeffBuff[j], controlCoeffBuff[j], 1);
	}
/*
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			memcpy(&OSPMAWGNSignalPastFuture[j][0], OSPMAWGNSignal[j],
					(controlInputSize + 1) / 2 * 4u);
		}
*/
	//DMACopy_4Bytes_2D(OSPMAWGNSignalPastFuture, OSPMAWGNSignal, 0, 0 ,0 ,0, OSPMLength, numControlSignal);
    /* Initialize flag */
    bMemCopyInProgress = true;
    /* Populate 2D Memory transfer instance for source channel */
    Src_2DMemXfer.pStartAddress    = &OSPMAWGNSignal[0][0];
    Src_2DMemXfer.YCount           = numControlSignal;                    /* Configure YCount for 2D transfer */
    Src_2DMemXfer.YModify          = 4;
    Src_2DMemXfer.XCount           = OSPMLength;
    Src_2DMemXfer.XModify          = 4;

    /* Populate 2D Memory transfer instance for destination channel */
    Dest_2DMemXfer.pStartAddress   = &OSPMAWGNSignalPastFuture[0][0];
    Dest_2DMemXfer.YCount          = numControlSignal;                    /* Configure YCount for 2D transfer */
    Dest_2DMemXfer.YModify         = 4;
    Dest_2DMemXfer.XCount          = OSPMLength;
    Dest_2DMemXfer.XModify         = 4;

    eResult = adi_mdma_Copy2D (hMemDmaStream,
    							MEMCOPY_MSIZE,
                               &Dest_2DMemXfer,
                               &Src_2DMemXfer);

    /* IF (Failure) */
    if (eResult != ADI_DMA_SUCCESS)
    {
    	DBG_MSG("Failed initialize MDMA 2D Copy \n", eResult);
    }


	while (bMemCopyInProgress);




	return 0;
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
		//eMode=SUBMIT_RX_BUFFER;
		//bEvent=true;
		break;
	default:

		//eMode = SUBMIT_RX_BUFFER;
		//bEvent = true;
		break;
	}
}



/*
 * Opens and initializes PCG Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t PcgInit(void) {
	uint32_t Result = 0u;

	/* Init PCG
	 *
	 * Using ext clk 24.576MHz
	 *
	 * CLK = numASRC * 64 * Fs Hz
	 * FS = 24 kHz
	 */

	uint32_t DeviceNum;
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	DeviceNum = 2u; /* PCG C */
#elif defined(__ADSPSC573_FAMILY__)
	DeviceNum = 0u; /* PCG A */
#else
#error "processor not supported"
#endif

	/* Open PCG */
	if (adi_pcg_Open(DeviceNum, &PcgMemory[0],
	ADI_PCG_MEMORY_SIZE, &phPcg) != ADI_PCG_SUCCESS) {
		Result = 1u;
	}

	/* Select FS and clock outputs for PCG */
	if ((uint32_t) adi_pcg_SelectOutput(phPcg, ADI_PCG_OUT_CLK_FS) != 0u) {
		/* return error */
		return 1u;
	}

	/* Set the PCG input clock source to external*/
	if ((uint32_t) adi_pcg_ClockSource(phPcg, ADI_PCG_CLK_EXT) != 0u) {
		/* return error */
		return 1u;
	}

	/* Set the PCG input fs to external */
	if ((uint32_t) adi_pcg_FrameSyncSource(phPcg, ADI_PCG_FS_EXT) != 0u) {
		/* return error */
		return 1u;
	}

	/* Clock C numASRC * 64 * Fs Hz */
	if ((uint32_t) adi_pcg_ClockDivide(phPcg, pcgCLKDIV) != 0u) {
		/* return error */
		return 1u;
	}
	/* FS C kHz */
	if ((uint32_t) adi_pcg_FrameSyncDivide(phPcg, pcgFSDIV) != 0u) {
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Opens and initializes ASRC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t AsrcDacInit(void) {
	uint32_t Result = 0u;
	ADI_ASRC_SPORT_CONFIG SportConfig;

	uint32_t nBlockNum;
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	nBlockNum = 1u;
#elif defined(__ADSPSC573_FAMILY__)
	nBlockNum = 0u;
#else
#error "processor not supported"
#endif
	/* For ADSP-SC573 family processors, open ASRC 0.  Otherwise open ASRC 4 */
	if (adi_asrc_Open(nBlockNum, 0u, &AsrcMemory4[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc4) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	/* For ADSP-SC573 family processors, open ASRC 1.  Otherwise open ASRC 5 */
	if (adi_asrc_Open(nBlockNum, 1u, &AsrcMemory5[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc5) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	/* For ADSP-SC573 family processors, open ASRC 2.  Otherwise open ASRC 6 */
	if (adi_asrc_Open(nBlockNum, 2u, &AsrcMemory6[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc6) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}

	/* For ADSP-SC573 family processors, open ASRC 3.  Otherwise open ASRC 7 */
	if (adi_asrc_Open(nBlockNum, 3u, &AsrcMemory7[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc7) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}

	//adi_asrc_SetMatchPhaseMode(phAsrc6, true);
	if (adi_asrc_SetSerialFormat(phAsrc4, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc5, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc6, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc7, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}

	return Result;
}

uint32_t SpuInit(void) {

	uint32_t Result = 0u;

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

    /* Make MDMA0 Source to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA0_SRC_DMA8_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Source\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA0 Destination to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA0_DST_DMA9_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

	return (Result);
}

uint32_t DMAInit(void){
	ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;
    eResult = adi_mdma_Open (MEMCOPY_STREAM_ID,
                               &MemDmaStreamMem[0],
                               &hMemDmaStream,
                               &hSrcDmaChannel,
                               &hDestDmaChannel,
                               NULL,
                               NULL);

      /* IF (Failure) */
      if (eResult != ADI_DMA_SUCCESS)
      {
      	DBG_MSG("Failed to open MDMA stream, Error Code: 0x%08X\n", eResult);
      }

      /* IF (Success) */
      if (eResult == ADI_DMA_SUCCESS)
      {
          /* Set DMA Callback Function. No need to set for source channel since callback is not supported for it */
          eResult = adi_dma_UpdateCallback (hDestDmaChannel, MemDmaCallback, hMemDmaStream);

          /* IF (Failure) */
          if (eResult != ADI_DMA_SUCCESS)
          {
              DBG_MSG("Failed to set DMA callback, Error Code: 0x%08X\n", eResult);
          }
      }

      return 0;
}



/*
 *  Callback from MDMA Manager
 *
 * Parameters
 *  - [in]  pCBParam    Callback parameter supplied by application
 *  - [in]  Event       Callback event
 *  - [in]  pArg        Callback argument
 *
 * Returns  None
 *
 */
static void MemDmaCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    /* CASEOF (Event) */
    switch ((ADI_DMA_EVENT)Event)
    {
        /* CASE (Processed a one-shot/circular buffer) */
        case (ADI_DMA_EVENT_BUFFER_PROCESSED):
            /* Update memory copy status flag */
            bMemCopyInProgress = false;
            break;

        default:
            break;
    }
}






static void LowPassFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg)
{
    /* CASEOF (Event) */
    switch ((ADI_FIR_EVENT)eEvent)
    {
        /* CASE (Processed a one-shot/circular buffer) */
        case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
            /* Update memory copy status flag */
            bLowPassFIRInProgress = false;
            break;

        default:
            break;
    }
}






static void ControlFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg)
{
    /* CASEOF (Event) */
    switch ((ADI_FIR_EVENT)eEvent)
    {
        /* CASE (Processed a one-shot/circular buffer) */
        case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
            /* Update memory copy status flag */
            bControlFIRInProgress = false;
            break;

        default:
            break;
    }
}


static void OSPMFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg)
{
    /* CASEOF (Event) */
    switch ((ADI_FIR_EVENT)eEvent)
    {
        /* CASE (Processed a one-shot/circular buffer) */
        case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
            /* Update memory copy status flag */
            bOSPMFIRInProgress = false;
            break;

        default:
            break;
    }
}












