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


/* ----------------------------   FIR Configuration ------------------------------------------- */

uint8_t cc=0;
uint32_t sampleCount=0;
#pragma align 4
static float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_ADC / 2];
static float errorSignalOSPM[numErrorSignal][OSPMLength];

#pragma align 4
static float conv_float_control_temp[controlLength];
#pragma align 4
static float conv_float_OSPM_temp[controlLength];

#pragma align 4
uint8_t ConfigMemoryOSPM[ADI_FIR_CONFIG_MEMORY_SIZE];
#pragma align 4
uint8_t ConfigMemoryControl[ADI_FIR_CONFIG_MEMORY_SIZE];

#ifdef LowPassFilter
#pragma align 4
uint8_t ConfigMemoryRef[ADI_FIR_CONFIG_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryRef[ADI_FIR_CHANNEL_MEMORY_SIZE];
#endif

#pragma align 4
uint8_t ChannelMemoryOSPM11[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM12[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM13[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM21[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM22[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM23[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM31[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM32[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM33[ADI_FIR_CHANNEL_MEMORY_SIZE];

#pragma align 4
uint8_t ChannelMemoryControl1[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryControl2[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryControl3[ADI_FIR_CHANNEL_MEMORY_SIZE];

/* Channel Configurations parameters */
#ifdef LowPassFilter
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelRef;
#endif

#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM11;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM12;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM13;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM21;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM22;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM23;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM31;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM32;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM33;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl1;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl2;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl3;

static float AWGN_generator_temp[numControlSignal] = { 0 };

//Max FIR channels = 32

#pragma align 4
//static float outputSignal[numControlSignal][NUM_AUDIO_SAMPLES]={0};
#pragma align 4
static float outputSignal1[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static float outputSignal2[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static float outputSignal3[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static float outputSignal4[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static float outputSignal5[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static float outputSignal6[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal1_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal2_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal3_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal4_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal5_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };
#pragma align 4
static int32_t outputSignal6_temp[NUM_AUDIO_SAMPLES_DAC] = { 0 };

static ADI_FIR_RESULT res;
static ADI_FIR_HANDLE hFir;
static ADI_FIR_CONFIG_HANDLE hConfigControl, hConfigOSPM;

static ADI_FIR_CHANNEL_HANDLE hChannelOSPM11, hChannelOSPM12, hChannelOSPM13,
		hChannelOSPM21, hChannelOSPM22, hChannelOSPM23, hChannelOSPM31,
		hChannelOSPM32, hChannelOSPM33;
static ADI_FIR_CHANNEL_HANDLE hChannelControl1, hChannelControl2,
		hChannelControl3;

#ifdef LowPassFilter
static ADI_FIR_CONFIG_HANDLE  hConfigRef;
static ADI_FIR_CHANNEL_HANDLE hChannelRef;


#pragma align 4
static float refInputBuff[refLength] = { 0 };

float refCoeffBuff[refLength] = {
#include "data/lowpass_filter_500_1000.dat"
		};

ADI_CACHE_ALIGN float refOutputBuff[ADI_CACHE_ROUND_UP_SIZE(refOutputSize,
		float)];
#endif


#pragma align 4
static float controlInputBuff[controlLength] = { 0 };

ADI_CACHE_ALIGN float controlOutputBuff1[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];
ADI_CACHE_ALIGN float controlOutputBuff2[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];
ADI_CACHE_ALIGN float controlOutputBuff3[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];
ADI_CACHE_ALIGN float controlOutputBuff4[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];
ADI_CACHE_ALIGN float controlOutputBuff5[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];
ADI_CACHE_ALIGN float controlOutputBuff6[ADI_CACHE_ROUND_UP_SIZE(
		controlOutputSize, float)];

static float controlOutputBuff1_temp[controlOutputSize] = { 0 };
static float controlOutputBuff2_temp[controlOutputSize] = { 0 };
static float controlOutputBuff3_temp[controlOutputSize] = { 0 };
static float controlOutputBuff4_temp[controlOutputSize] = { 0 };
static float controlOutputBuff5_temp[controlOutputSize] = { 0 };
static float controlOutputBuff6_temp[controlOutputSize] = { 0 };

static float controlCoeffBuff1[controlLength] = { 0 };
static float controlCoeffBuff2[controlLength] = { 0 };
static float controlCoeffBuff3[controlLength] = { 0 };

#pragma align 4
static float OSPMInputBuff11[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff12[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff13[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff21[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff22[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff23[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff31[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff32[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff33[OSPMLength] = { 0 };

static float OSPMCoeffBuff11[OSPMLength] = { 0 };
static float OSPMCoeffBuff12[OSPMLength] = { 0 };
static float OSPMCoeffBuff13[OSPMLength] = { 0 };
static float OSPMCoeffBuff21[OSPMLength] = { 0 };
static float OSPMCoeffBuff22[OSPMLength] = { 0 };
static float OSPMCoeffBuff23[OSPMLength] = { 0 };
static float OSPMCoeffBuff31[OSPMLength] = { 0 };
static float OSPMCoeffBuff32[OSPMLength] = { 0 };
static float OSPMCoeffBuff33[OSPMLength] = { 0 };

ADI_CACHE_ALIGN float OSPMOutputBuff11[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff12[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff13[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff21[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff22[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff23[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff31[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff32[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff33[ADI_CACHE_ROUND_UP_SIZE(OSPMOutputSize,
		float)];

static float AWGNSample[OSPMLength] = { 0 };
static float OSPMAWGNGain[numControlSignal][OSPMLength] = { 0 };
#pragma align 4
static float OSPMAWGNSignal1[OSPMLength] = { 0 };
#pragma align 4
static float OSPMAWGNSignal2[OSPMLength] = { 0 };
#pragma align 4
static float OSPMAWGNSignal3[OSPMLength] = { 0 };

static float powerOSPMAWGNSignal[numControlSignal][OSPMLength] = { 0 };


static float OSPMRef[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };

static float OSPMAux[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };

float forgettingFactor = 0.6;
static float filteredErrorSignal[numErrorSignal][OSPMLength] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength];
float powerIndirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength] = {
		1.0 };
float powerFilteredErrorSignal[numErrorSignal][OSPMLength] = { 1.0 };
float stepSizeS[numControlSignal][numErrorSignal][OSPMLength] = { 0 };
float stepSizeSMin = 0.001;
float stepSizeW[numControlSignal] = { 0.0001, 0.0001, 0.0001, 0.0001, 0.0001,
		0.0001 };

#if (numControlSignal == 6)

static ADI_FIR_CHANNEL_HANDLE hChannelOSPM14, hChannelOSPM15, hChannelOSPM16,
		hChannelOSPM24, hChannelOSPM25, hChannelOSPM26, hChannelOSPM34,
		hChannelOSPM35, hChannelOSPM36;
static ADI_FIR_CHANNEL_HANDLE hChannelControl4, hChannelControl5,
		hChannelControl6;

#pragma align 4
uint8_t ChannelMemoryOSPM14[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM15[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM16[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM24[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM25[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM26[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM34[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM35[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryOSPM36[ADI_FIR_CHANNEL_MEMORY_SIZE];

#pragma align 4
uint8_t ChannelMemoryControl4[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryControl5[ADI_FIR_CHANNEL_MEMORY_SIZE];
#pragma align 4
uint8_t ChannelMemoryControl6[ADI_FIR_CHANNEL_MEMORY_SIZE];

#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl4;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl5;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelControl6;

static float controlCoeffBuff4[controlLength] = { 0 };
static float controlCoeffBuff5[controlLength] = { 0 };
static float controlCoeffBuff6[controlLength] = { 0 };

#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM14;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM15;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM16;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM24;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM25;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM26;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM34;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM35;
#pragma align 4
ADI_FIR_CHANNEL_PARAMS channelOSPM36;

#pragma align 4
static float OSPMInputBuff14[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff15[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff16[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff24[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff25[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff26[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff34[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff35[OSPMLength] = { 0 };
#pragma align 4
static float OSPMInputBuff36[OSPMLength] = { 0 };

static float OSPMCoeffBuff14[OSPMLength] = { 0 };
static float OSPMCoeffBuff15[OSPMLength] = { 0 };
static float OSPMCoeffBuff16[OSPMLength] = { 0 };
static float OSPMCoeffBuff24[OSPMLength] = { 0 };
static float OSPMCoeffBuff25[OSPMLength] = { 0 };
static float OSPMCoeffBuff26[OSPMLength] = { 0 };
static float OSPMCoeffBuff34[OSPMLength] = { 0 };
static float OSPMCoeffBuff35[OSPMLength] = { 0 };
static float OSPMCoeffBuff36[OSPMLength] = { 0 };

static float OSPMAux14[OSPMLength] = { 0 };
static float OSPMAux15[OSPMLength] = { 0 };
static float OSPMAux16[OSPMLength] = { 0 };
static float OSPMAux24[OSPMLength] = { 0 };
static float OSPMAux25[OSPMLength] = { 0 };
static float OSPMAux26[OSPMLength] = { 0 };
static float OSPMAux34[OSPMLength] = { 0 };
static float OSPMAux35[OSPMLength] = { 0 };
static float OSPMAux36[OSPMLength] = { 0 };

ADI_CACHE_ALIGN float OSPMOutputBuff14[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff15[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff16[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff24[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff25[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff26[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff34[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff35[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];
ADI_CACHE_ALIGN float OSPMOutputBuff36[ADI_CACHE_ROUND_UP_SIZE(OSPMWindowSize,
		float)];

#pragma align 4
float OSPMAWGNSignal4[OSPMLength] = { 0 };
#pragma align 4
float OSPMAWGNSignal5[OSPMLength] = { 0 };
#pragma align 4
float OSPMAWGNSignal6[OSPMLength] = { 0 };



#endif

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
#pragma align 4
int32_t pSrcL1[NUM_AUDIO_SAMPLES_ADC / 2];
#pragma align 4
int32_t pSrcR1[NUM_AUDIO_SAMPLES_ADC / 2];
#pragma align 4
int32_t pSrcL2[NUM_AUDIO_SAMPLES_ADC / 2];
#pragma align 4
int32_t pSrcR2[NUM_AUDIO_SAMPLES_ADC / 2];


static float refSignal[refLength] = {0};
static float refSignalLP[refLength] = {0};
static float refSignalControl[controlLength] = {0};

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
int32_t DacBuf[NUM_AUDIO_SAMPLES_DAC *NUM_DAC_CHANNELS * 2];

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
uint32_t PcgInit(void);
uint32_t AsrcDacInit(void);
int32_t ANCALG(void);
int8_t DisableAllFIRChannels(void);
void    Block_Fixed_To_Float( int * , float * , float *  );

void    Block_Float_To_Fixed( int * , float * , float *  );
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
	for (int32_t j = 0; j < numControlSignal; j++) {
	for (int32_t i = 0; i < OSPMLength; i++) {
		powerOSPMAWGNSignal[j][i] = 0.0001;
	}
	}
	for (int32_t k = 0; k < numErrorSignal; k++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			powerFilteredErrorSignal[k][i] = 1;
		}
	}
	for (int32_t j = 0; j < numControlSignal; j++) {
		for (int32_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OSPMLength; i++) {
				powerIndirectErrorSignal[j][k][i] = 1;
			}
		}
	}

	for (int32_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			OSPMAWGNGain[j][i] = 1;
		}
	}
	reverseArrayf(refCoeffBuff,sizeof(refCoeffBuff));
	/*
	 for (int32_t j = 0; j < numControlSignal; j++) {
	 for (int32_t k = 0; k < numErrorSignal; k++) {
	 for (int32_t i = 0; i < OSPMLength; i++) {
	 stepSizeS[j][k][i] = 0.001;
	 }
	 }
	 }
	 */

	for (int32_t i = 0; i < controlLength; i++) {
		controlCoeffBuff1[i] = 0;//0.0000001;
		controlCoeffBuff2[i] =0;// 0.0000001;
		controlCoeffBuff3[i] = 0;//0.0000001;
#if (numControlSignal==6)
		controlCoeffBuff4[i] =0;// 0.0000001;
		controlCoeffBuff5[i] =0;// 0.0000001;
		controlCoeffBuff6[i] = 0;//0.0000001;
#endif
	}

	for (int32_t i = 0; i < OSPMLength; i++) {
		OSPMCoeffBuff11[i] = 0.0000001;
		OSPMCoeffBuff12[i] = 0.0000001;
		OSPMCoeffBuff13[i] = 0.0000001;

		OSPMCoeffBuff21[i] = 0.0000001;
		OSPMCoeffBuff22[i] = 0.0000001;
		OSPMCoeffBuff23[i] = 0.0000001;

		OSPMCoeffBuff31[i] = 0.0000001;
		OSPMCoeffBuff32[i] = 0.0000001;
		OSPMCoeffBuff33[i] = 0.0000001;

#if (numControlSignal==6)
		OSPMCoeffBuff14[i] = 0.0000001;
		OSPMCoeffBuff15[i] = 0.0000001;
		OSPMCoeffBuff16[i] = 0.0000001;
		OSPMCoeffBuff24[i] = 0.0000001;
		OSPMCoeffBuff25[i] = 0.0000001;
		OSPMCoeffBuff26[i] = 0.0000001;
		OSPMCoeffBuff34[i] = 0.0000001;
		OSPMCoeffBuff35[i] = 0.0000001;
		OSPMCoeffBuff36[i] = 0.0000001;
#endif
	}
#ifdef LowPassFilter
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

	channelControl1.nTapLength = controlLength;
	channelControl1.nWindowSize = controlWindowSize;
	channelControl1.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelControl1.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelControl1.nGroupNum = 0u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelControl1.pInputBuffBase = (void *) controlInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelControl1.pInputBuffIndex = (void *) controlInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelControl1.nInputBuffCount = controlInputSize; /*!< Number of elements in the input circular buffer */
	channelControl1.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelControl1.pCoefficientBase = (void *) controlCoeffBuff1; /*!< Pointer to the start of the coefficient buffer */
	channelControl1.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl1.pCoefficientIndex = (void *) controlCoeffBuff1; /*!< Pointer to the start of the coefficient buffer */

	channelControl1.pOutputBuffBase = (void *) controlOutputBuff1; /*!< Pointer to the base of the output circular buffer */
	channelControl1.pOutputBuffIndex = (void *) controlOutputBuff1; /*!< Pointer to the current index of the output circular buffer */
	channelControl1.nOutputBuffCount = controlWindowSize; /*!< Number of elements in the output circular buffer */
	channelControl1.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelControl2 = channelControl1;

	channelControl2.pCoefficientBase = (void *) controlCoeffBuff2; /*!< Pointer to the start of the coefficient buffer */
	channelControl2.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl2.pCoefficientIndex = (void *) controlCoeffBuff2; /*!< Pointer to the start of the coefficient buffer */

	channelControl2.pOutputBuffBase = (void *) controlOutputBuff2; /*!< Pointer to the base of the output circular buffer */
	channelControl2.pOutputBuffIndex = (void *) controlOutputBuff2; /*!< Pointer to the current index of the output circular buffer */
	channelControl2.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelControl3 = channelControl1;

	channelControl3.pCoefficientBase = (void *) controlCoeffBuff3; /*!< Pointer to the start of the coefficient buffer */
	channelControl3.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl3.pCoefficientIndex = (void *) controlCoeffBuff3; /*!< Pointer to the start of the coefficient buffer */

	channelControl3.pOutputBuffBase = (void *) controlOutputBuff3; /*!< Pointer to the base of the output circular buffer */
	channelControl3.pOutputBuffIndex = (void *) controlOutputBuff3; /*!< Pointer to the current index of the output circular buffer */
	channelControl3.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM11.nTapLength = OSPMLength;
	channelOSPM11.nWindowSize = OSPMWindowSize;
	channelOSPM11.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOSPM11.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOSPM11.nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOSPM11.pInputBuffBase = (void *) OSPMInputBuff11; /*!< Pointer to the base of the input circular buffer */
	channelOSPM11.pInputBuffIndex = (void *) OSPMInputBuff11; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM11.nInputBuffCount = OSPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOSPM11.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM11.pCoefficientBase = (void *) OSPMCoeffBuff11; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM11.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM11.pCoefficientIndex = (void *) OSPMCoeffBuff11; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM11.pOutputBuffBase = (void *) OSPMOutputBuff11; /*!< Pointer to the base of the output circular buffer */
	channelOSPM11.pOutputBuffIndex = (void *) OSPMOutputBuff11; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM11.nOutputBuffCount = OSPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOSPM11.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM12 = channelOSPM11;

	channelOSPM12.pInputBuffBase = (void *) OSPMInputBuff12; /*!< Pointer to the base of the input circular buffer */
	channelOSPM12.pInputBuffIndex = (void *) OSPMInputBuff12; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM12.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM12.pCoefficientBase = (void *) OSPMCoeffBuff12; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM12.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM12.pCoefficientIndex = (void *) OSPMCoeffBuff12; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM12.pOutputBuffBase = (void *) OSPMOutputBuff12; /*!< Pointer to the base of the output circular buffer */
	channelOSPM12.pOutputBuffIndex = (void *) OSPMOutputBuff12; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM12.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM13 = channelOSPM11;

	channelOSPM13.pInputBuffBase = (void *) OSPMInputBuff13; /*!< Pointer to the base of the input circular buffer */
	channelOSPM13.pInputBuffIndex = (void *) OSPMInputBuff13; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM13.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM13.pCoefficientBase = (void *) OSPMCoeffBuff13; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM13.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM13.pCoefficientIndex = (void *) OSPMCoeffBuff13; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM13.pOutputBuffBase = (void *) OSPMOutputBuff13; /*!< Pointer to the base of the output circular buffer */
	channelOSPM13.pOutputBuffIndex = (void *) OSPMOutputBuff13; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM13.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM21 = channelOSPM11;

	channelOSPM21.pInputBuffBase = (void *) OSPMInputBuff21; /*!< Pointer to the base of the input circular buffer */
	channelOSPM21.pInputBuffIndex = (void *) OSPMInputBuff21; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM21.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM21.pCoefficientBase = (void *) OSPMCoeffBuff21; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM21.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM21.pCoefficientIndex = (void *) OSPMCoeffBuff21; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM21.pOutputBuffBase = (void *) OSPMOutputBuff21; /*!< Pointer to the base of the output circular buffer */
	channelOSPM21.pOutputBuffIndex = (void *) OSPMOutputBuff21; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM21.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM22 = channelOSPM11;

	channelOSPM22.pInputBuffBase = (void *) OSPMInputBuff22; /*!< Pointer to the base of the input circular buffer */
	channelOSPM22.pInputBuffIndex = (void *) OSPMInputBuff22; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM22.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM22.pCoefficientBase = (void *) OSPMCoeffBuff22; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM22.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM22.pCoefficientIndex = (void *) OSPMCoeffBuff22; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM22.pOutputBuffBase = (void *) OSPMOutputBuff22; /*!< Pointer to the base of the output circular buffer */
	channelOSPM22.pOutputBuffIndex = (void *) OSPMOutputBuff22; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM22.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM23 = channelOSPM11;

	channelOSPM23.pInputBuffBase = (void *) OSPMInputBuff23; /*!< Pointer to the base of the input circular buffer */
	channelOSPM23.pInputBuffIndex = (void *) OSPMInputBuff23; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM23.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM23.pCoefficientBase = (void *) OSPMCoeffBuff23; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM23.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM23.pCoefficientIndex = (void *) OSPMCoeffBuff23; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM23.pOutputBuffBase = (void *) OSPMOutputBuff23; /*!< Pointer to the base of the output circular buffer */
	channelOSPM23.pOutputBuffIndex = (void *) OSPMOutputBuff23; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM23.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM31 = channelOSPM11;

	channelOSPM31.pInputBuffBase = (void *) OSPMInputBuff31; /*!< Pointer to the base of the input circular buffer */
	channelOSPM31.pInputBuffIndex = (void *) OSPMInputBuff31; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM31.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM31.pCoefficientBase = (void *) OSPMCoeffBuff31; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM31.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM31.pCoefficientIndex = (void *) OSPMCoeffBuff31; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM31.pOutputBuffBase = (void *) OSPMOutputBuff31; /*!< Pointer to the base of the output circular buffer */
	channelOSPM31.pOutputBuffIndex = (void *) OSPMOutputBuff31; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM31.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM32 = channelOSPM11;

	channelOSPM32.pInputBuffBase = (void *) OSPMInputBuff32; /*!< Pointer to the base of the input circular buffer */
	channelOSPM32.pInputBuffIndex = (void *) OSPMInputBuff32; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM32.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM32.pCoefficientBase = (void *) OSPMCoeffBuff32; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM32.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM32.pCoefficientIndex = (void *) OSPMCoeffBuff32; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM32.pOutputBuffBase = (void *) OSPMOutputBuff32; /*!< Pointer to the base of the output circular buffer */
	channelOSPM32.pOutputBuffIndex = (void *) OSPMOutputBuff32; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM32.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM33 = channelOSPM11;

	channelOSPM33.pInputBuffBase = (void *) OSPMInputBuff33; /*!< Pointer to the base of the input circular buffer */
	channelOSPM33.pInputBuffIndex = (void *) OSPMInputBuff33; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM33.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM33.pCoefficientBase = (void *) OSPMCoeffBuff33; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM33.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM33.pCoefficientIndex = (void *) OSPMCoeffBuff33; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM33.pOutputBuffBase = (void *) OSPMOutputBuff33; /*!< Pointer to the base of the output circular buffer */
	channelOSPM33.pOutputBuffIndex = (void *) OSPMOutputBuff33; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM33.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

#if (numControlSignal == 6)

	channelControl4 = channelControl1;

	channelControl4.pCoefficientBase = (void *) controlCoeffBuff4; /*!< Pointer to the start of the coefficient buffer */
	channelControl4.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl4.pCoefficientIndex = (void *) controlCoeffBuff4; /*!< Pointer to the start of the coefficient buffer */

	channelControl4.pOutputBuffBase = (void *) controlOutputBuff4; /*!< Pointer to the base of the output circular buffer */
	channelControl4.pOutputBuffIndex = (void *) controlOutputBuff4; /*!< Pointer to the current index of the output circular buffer */
	channelControl4.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelControl5 = channelControl1;

	channelControl5.pCoefficientBase = (void *) controlCoeffBuff5; /*!< Pointer to the start of the coefficient buffer */
	channelControl5.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl5.pCoefficientIndex = (void *) controlCoeffBuff5; /*!< Pointer to the start of the coefficient buffer */

	channelControl5.pOutputBuffBase = (void *) controlOutputBuff5; /*!< Pointer to the base of the output circular buffer */
	channelControl5.pOutputBuffIndex = (void *) controlOutputBuff5; /*!< Pointer to the current index of the output circular buffer */
	channelControl5.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelControl6 = channelControl1;

	channelControl6.pCoefficientBase = (void *) controlCoeffBuff6; /*!< Pointer to the start of the coefficient buffer */
	channelControl6.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelControl6.pCoefficientIndex = (void *) controlCoeffBuff6; /*!< Pointer to the start of the coefficient buffer */

	channelControl6.pOutputBuffBase = (void *) controlOutputBuff6; /*!< Pointer to the base of the output circular buffer */
	channelControl6.pOutputBuffIndex = (void *) controlOutputBuff6; /*!< Pointer to the current index of the output circular buffer */
	channelControl6.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM14 = channelOSPM11;

	channelOSPM14.pInputBuffBase = (void *) OSPMInputBuff14; /*!< Pointer to the base of the input circular buffer */
	channelOSPM14.pInputBuffIndex = (void *) OSPMInputBuff14; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM14.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM14.pCoefficientBase = (void *) OSPMCoeffBuff14; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM14.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM14.pCoefficientIndex = (void *) OSPMCoeffBuff14; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM14.pOutputBuffBase = (void *) OSPMOutputBuff14; /*!< Pointer to the base of the output circular buffer */
	channelOSPM14.pOutputBuffIndex = (void *) OSPMOutputBuff14; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM14.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM15 = channelOSPM11;

	channelOSPM15.pInputBuffBase = (void *) OSPMInputBuff15; /*!< Pointer to the base of the input circular buffer */
	channelOSPM15.pInputBuffIndex = (void *) OSPMInputBuff15; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM15.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM15.pCoefficientBase = (void *) OSPMCoeffBuff15; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM15.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM15.pCoefficientIndex = (void *) OSPMCoeffBuff15; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM15.pOutputBuffBase = (void *) OSPMOutputBuff15; /*!< Pointer to the base of the output circular buffer */
	channelOSPM15.pOutputBuffIndex = (void *) OSPMOutputBuff15; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM15.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM16 = channelOSPM11;

	channelOSPM16.pInputBuffBase = (void *) OSPMInputBuff16; /*!< Pointer to the base of the input circular buffer */
	channelOSPM16.pInputBuffIndex = (void *) OSPMInputBuff16; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM16.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM16.pCoefficientBase = (void *) OSPMCoeffBuff16; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM16.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM16.pCoefficientIndex = (void *) OSPMCoeffBuff16; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM16.pOutputBuffBase = (void *) OSPMOutputBuff16; /*!< Pointer to the base of the output circular buffer */
	channelOSPM16.pOutputBuffIndex = (void *) OSPMOutputBuff16; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM16.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM24 = channelOSPM11;

	channelOSPM24.pInputBuffBase = (void *) OSPMInputBuff24; /*!< Pointer to the base of the input circular buffer */
	channelOSPM24.pInputBuffIndex = (void *) OSPMInputBuff24; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM24.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM24.pCoefficientBase = (void *) OSPMCoeffBuff24; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM24.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM24.pCoefficientIndex = (void *) OSPMCoeffBuff24; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM24.pOutputBuffBase = (void *) OSPMOutputBuff24; /*!< Pointer to the base of the output circular buffer */
	channelOSPM24.pOutputBuffIndex = (void *) OSPMOutputBuff24; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM24.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM25 = channelOSPM11;

	channelOSPM25.pInputBuffBase = (void *) OSPMInputBuff25; /*!< Pointer to the base of the input circular buffer */
	channelOSPM25.pInputBuffIndex = (void *) OSPMInputBuff25; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM25.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM25.pCoefficientBase = (void *) OSPMCoeffBuff25; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM25.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM25.pCoefficientIndex = (void *) OSPMCoeffBuff25; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM25.pOutputBuffBase = (void *) OSPMOutputBuff25; /*!< Pointer to the base of the output circular buffer */
	channelOSPM25.pOutputBuffIndex = (void *) OSPMOutputBuff25; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM25.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM26 = channelOSPM11;

	channelOSPM26.pInputBuffBase = (void *) OSPMInputBuff26; /*!< Pointer to the base of the input circular buffer */
	channelOSPM26.pInputBuffIndex = (void *) OSPMInputBuff26; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM26.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM26.pCoefficientBase = (void *) OSPMCoeffBuff26; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM26.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM26.pCoefficientIndex = (void *) OSPMCoeffBuff26; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM26.pOutputBuffBase = (void *) OSPMOutputBuff26; /*!< Pointer to the base of the output circular buffer */
	channelOSPM26.pOutputBuffIndex = (void *) OSPMOutputBuff26; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM26.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM34 = channelOSPM11;

	channelOSPM34.pInputBuffBase = (void *) OSPMInputBuff34; /*!< Pointer to the base of the input circular buffer */
	channelOSPM34.pInputBuffIndex = (void *) OSPMInputBuff34; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM34.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM34.pCoefficientBase = (void *) OSPMCoeffBuff34; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM34.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM34.pCoefficientIndex = (void *) OSPMCoeffBuff34; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM34.pOutputBuffBase = (void *) OSPMOutputBuff34; /*!< Pointer to the base of the output circular buffer */
	channelOSPM34.pOutputBuffIndex = (void *) OSPMOutputBuff34; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM34.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM35 = channelOSPM11;

	channelOSPM35.pInputBuffBase = (void *) OSPMInputBuff35; /*!< Pointer to the base of the input circular buffer */
	channelOSPM35.pInputBuffIndex = (void *) OSPMInputBuff35; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM35.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM35.pCoefficientBase = (void *) OSPMCoeffBuff35; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM35.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM35.pCoefficientIndex = (void *) OSPMCoeffBuff35; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM35.pOutputBuffBase = (void *) OSPMOutputBuff35; /*!< Pointer to the base of the output circular buffer */
	channelOSPM35.pOutputBuffIndex = (void *) OSPMOutputBuff35; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM35.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	channelOSPM36 = channelOSPM11;

	channelOSPM36.pInputBuffBase = (void *) OSPMInputBuff36; /*!< Pointer to the base of the input circular buffer */
	channelOSPM36.pInputBuffIndex = (void *) OSPMInputBuff36; /*!< Pointer to the current index of the input circular buffer */
	channelOSPM36.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPM36.pCoefficientBase = (void *) OSPMCoeffBuff36; /*!< Pointer to the start of the coefficient buffer */
	channelOSPM36.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPM36.pCoefficientIndex = (void *) OSPMCoeffBuff36; /*!< Pointer to the start of the coefficient buffer */

	channelOSPM36.pOutputBuffBase = (void *) OSPMOutputBuff36; /*!< Pointer to the base of the output circular buffer */
	channelOSPM36.pOutputBuffIndex = (void *) OSPMOutputBuff36; /*!< Pointer to the current index of the output circular buffer */
	channelOSPM36.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

#endif

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
#endif
	res = adi_fir_CreateConfig(hFir, ConfigMemoryControl,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigControl);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOSPM,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOSPM);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}

	// ----------------------------------  Add Channels ---------------------------------------------------
#ifdef LowPassFilter
	res = adi_fir_AddChannel(hConfigRef, ChannelMemoryRef,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelRef, &hChannelRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
#endif
	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl1,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl1, &hChannelControl1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl2,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl2, &hChannelControl2);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl3,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl3, &hChannelControl3);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM11,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM11, &hChannelOSPM11);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM12,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM12, &hChannelOSPM12);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM13,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM13, &hChannelOSPM13);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM21,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM21, &hChannelOSPM21);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM22,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM22, &hChannelOSPM22);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM23,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM23, &hChannelOSPM23);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM31,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM31, &hChannelOSPM31);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM32,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM32, &hChannelOSPM32);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM33,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM33, &hChannelOSPM33);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	// ---------------------------------------   Enable Channels -----------------------------------------------
	/*
	 res = adi_fir_EnableChannel (hChannelRef, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelControl1, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }

	 res = adi_fir_EnableChannel (hChannelControl2, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelControl3, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelOSPM11, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM12, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM13, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelOSPM21, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM22, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM23, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }



	 res = adi_fir_EnableChannel (hChannelOSPM31, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM32, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM33, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }

	 */

#if (numControlSignal == 6)

	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl4,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl4, &hChannelControl4);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl5,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl5, &hChannelControl5);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl6,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl6, &hChannelControl6);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM14,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM14, &hChannelOSPM14);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM15,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM15, &hChannelOSPM15);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM16,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM16, &hChannelOSPM16);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM24,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM24, &hChannelOSPM24);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM25,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM25, &hChannelOSPM25);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM26,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM26, &hChannelOSPM26);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM34,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM34, &hChannelOSPM34);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM35,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM35, &hChannelOSPM35);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}
	res = adi_fir_AddChannel(hConfigOSPM, ChannelMemoryOSPM36,
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPM36, &hChannelOSPM36);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel failed\n");
		return -1;
	}

	// ---------------------------------------   Enable Channels -----------------------------------------------
	/*
	 res = adi_fir_EnableChannel (hChannelControl4, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelControl5, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelControl6, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelOSPM14, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM15, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM16, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }


	 res = adi_fir_EnableChannel (hChannelOSPM24, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM25, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM26, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }



	 res = adi_fir_EnableChannel (hChannelOSPM34, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM35, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 res = adi_fir_EnableChannel (hChannelOSPM36, true);
	 if( res != ADI_FIR_RESULT_SUCCESS) {
	 printf("adi_fir_EnableChannel failed\n");
	 return -1;
	 }
	 */

#endif

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
		Result = PcgInit();
	}
	if (Result == 0u) {
		Result = AsrcDacInit();
	}



	configGpio();
	 /* Enable the PCG */
		    if(Result == 0u)
		    {
		        Result = (uint32_t)adi_pcg_Enable(phPcg, true);
		    }
		    /* Enable the ASRC */
		    if(Result == 0u)
		    {
		        Result = (uint32_t)adi_asrc_Enable(phAsrc4, true);
		    }
		    /* Enable the ASRC */
		    if(Result == 0u)
		    {
		        Result = (uint32_t)adi_asrc_Enable(phAsrc5, true);
		    }
		    /* Enable the ASRC */
		    if(Result == 0u)
		    {
		        Result = (uint32_t)adi_asrc_Enable(phAsrc6, true);
		    }
		    /* Enable the ASRC */
		    if(Result == 0u)
		    {
		        Result = (uint32_t)adi_asrc_Enable(phAsrc7, true);
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
	printf(
			"outputSignal1_temp %d output1 %f OSPMAWGNSignal1 %f pSrcL1 %f yo %d\n",
			outputSignal1_temp[0], outputSignal1[0], OSPMAWGNSignal1[0],
			refSignal[0], conv_fix_by(outputSignal1[0], 10));
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
			&DacBuf[NUM_AUDIO_SAMPLES_DAC*NUM_DAC_CHANNELS * 0u], AUDIO_BUFFER_SIZE) != 0u) {
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if ((uint32_t) adi_adau1962a_SubmitBuffer(phAdau1962a,
			&DacBuf[NUM_AUDIO_SAMPLES_DAC*NUM_DAC_CHANNELS * 1u], AUDIO_BUFFER_SIZE) != 0u) {
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
		//memcpy(&sRxBuffer1[0], pRxBuffer1, BUFFER_SIZE_1761);
		//memcpy(&sRxBuffer2[0], pRxBuffer2, BUFFER_SIZE_1761);
		pDst = (int32_t *) pDAC;

#pragma vector_for(4)

		for (int32_t i = 0,  j = NUM_AUDIO_SAMPLES_ADC/2 -1; i < NUM_AUDIO_SAMPLES_ADC/2, j>-1; i++,j--) {
		refSignal[i]  = conv_float_by((pRxBuffer1[2*i]<<8), -1)  ;
		errorSignal[0][i]  = conv_float_by((pRxBuffer1[2*i+1]<<8), -1)  ;
		errorSignal[1][i]  = conv_float_by((pRxBuffer2[2*i]<<8), -1)  ;
		errorSignal[2][i]  = conv_float_by((pRxBuffer2[2*i+1]<<8), -1)  ;
		}

		/*
#pragma vector_for
		for (int32_t i = 0,  j = 0; i < refLength, j<NUM_AUDIO_SAMPLES/2; i++,j+=((NUM_AUDIO_SAMPLES/2)/(refLength))) {
		refSignalLP[i]  = refSignal[j];
		errorSignalOSPM[0][j]  = errorSignal[0][j];
		errorSignalOSPM[1][j]  = errorSignal[0][j];
		errorSignalOSPM[2][j]  = errorSignal[0][j];
		}


*/
/*
		for (uint32_t i = 0, j=0,k=0; i < NUM_AUDIO_SAMPLES; i++) {

			if (i % 2 == 0) {
				pSrcL1[i - k] = pRxBuffer1[i] <<8;
				pSrcL2[i - k] = pRxBuffer2[i] <<8;
				j++;
			} else {
				//original pSrcR1[i - j] = sRxBuffer1[i] << 8;
				pSrcR1[i - j] = pRxBuffer1[i] <<8;
				pSrcR2[i - j] = pRxBuffer2[i] <<8;
				k++;
			}
		}
		*/
/*

			for (int32_t i = 0; i < NUM_AUDIO_SAMPLES / 2; i++) {
				refSignal[i] = conv_float_by(pSrcR1[i], -23);
			}


			//conv_float_temp[controlLength];
			for (int32_t i = 0; i < NUM_AUDIO_SAMPLES / 2; i++) {
				errorSignal[0][i] = conv_float_by(pSrcL1[i], -23);
				errorSignal[1][i] = conv_float_by(pSrcL2[i], -23);
				errorSignal[2][i] = conv_float_by(pSrcR2[i], -23);

			}
*/
		/*
		Block_Fixed_To_Float(sRxBuffer1, errorSignal[0],refSignal);
		Block_Fixed_To_Float(sRxBuffer2, errorSignal[1],errorSignal[2]);
		*/
			/*
		reverseArrayf(refSignal, NUM_AUDIO_SAMPLES/2);
		reverseArrayf(errorSignal[0], NUM_AUDIO_SAMPLES/2);
		reverseArrayf(errorSignal[1], NUM_AUDIO_SAMPLES/2);
		reverseArrayf(errorSignal[2], NUM_AUDIO_SAMPLES/2);
		*/

		//for( int i=0; i<5000000; i++);


		ANCALG();

		reverseArrayf(conv_float_control_temp,sizeof(conv_float_control_temp));


#pragma vector_for
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_DAC; i++) {
			//TDM8 SHIFT <<8
			/*
			 outputSignal1[i]=outputSignal1[i];
			 outputSignal2[i]=outputSignal2[i];
			 outputSignal3[i]=outputSignal3[i];
			 outputSignal4[i]=outputSignal4[i];
			 outputSignal5[i]=outputSignal5[i];
			 outputSignal6[i]=outputSignal6[i];*/
			/*
			outputSignal1_temp[i] =  conv_fix_by(outputSignal1[i], 12);
			outputSignal2_temp[i] =  conv_fix_by(outputSignal2[i],
					12);
			outputSignal3_temp[i] = conv_fix_by(outputSignal3[i],
					12);
			outputSignal4_temp[i] =  conv_fix_by(outputSignal4[i],
					12);
			outputSignal5_temp[i] =  conv_fix_by(outputSignal5[i],
					12);
			outputSignal6_temp[i] = conv_fix_by(outputSignal6[i],
					12);
					*/
			/*
			*pDst++ =  conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ =  conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ =  conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ =  conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ = conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ =  0;	//conv_fix_by( outputSignal1[i],-10) << 8;
			*pDst++ =  conv_fix_by(outputSignal1[i], 12) <<8;// >> 16;
			*pDst++ = (int32_t) 0; //conv_fix_by( outputSignal1[i],-10) << 8;
			*/
/*
			*pDst++ = (conv_fix_by(refSignal[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 1)) >>8;
			*pDst++ = (conv_fix_by(refSignal[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 1)) >>8;
			*pDst++ = (conv_fix_by(refSignal[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 1)) >>8;
			*pDst++ = (conv_fix_by(refSignal[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(errorSignal[0][i], 1)) >>8;
*/

			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1)) >>8;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1)) >>8;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1)) >>8;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1))>>8 ;
			*pDst++ = (conv_fix_by(conv_float_control_temp[i], 1)) >>8;

		}

		/*
		 for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES / 2; i++) {
		 //TDM8 SHIFT <<8
		 *pDst++ = (int32_t) pSrcL1[i] << 8;
		 *pDst++ = (int32_t) pSrcR1[i] << 8;
		 *pDst++ = (int32_t) pSrcL1[i] << 8;
		 *pDst++ = (int32_t) pSrcR1[i] << 8;
		 *pDst++ = (int32_t) pSrcL1[i] << 8;
		 *pDst++ = (int32_t) pSrcR1[i] << 8;
		 *pDst++ = (int32_t) pSrcL1[i] << 8;
		 *pDst++ = (int32_t) pSrcR1[i] << 8;
		 }
		 */

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

void reverseArrayf(float arr[], uint32_t size) {
#pragma vector_for
	for (uint32_t i = 0; i < (size / 2); i++) {
		float temp = arr[i];
		arr[i] = arr[size - i - 1];
		arr[size - i - 1] = temp;
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

	res = adi_fir_EnableChannel(hChannelControl1, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl1\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelControl2, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl2\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl3, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl3\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM11, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM11\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM12, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM12\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM13, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM13\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM21, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM21\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM22, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM22\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM23, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM23\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM31, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM31\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM32, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM32\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM33, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM33\n");
		return -1;
	}

#if (numControlSignal == 6 )

	// ---------------------------------------   Disable Channels -----------------------------------------------

	res = adi_fir_EnableChannel(hChannelControl4, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl4\n ");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelControl5, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl5\n ");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl6, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl6\n ");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM14, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM14\n ");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM15, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM15\n ");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM16, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM16\n ");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM24, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM24\n ");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM25, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM25\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM26, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM26\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM34, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM34\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM35, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM35\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM36, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelOSPM36\n");
		return -1;
	}

#endif
	return 0;
}

int32_t ANCALG(void) {
	ADI_FIR_RESULT res;
	int8_t disableAllFirChannelsResult = 0;
	int8_t refResult = 0;

#ifdef LowPassFilter
	res = adi_fir_SubmitInputCircBuffer(hChannelRef, refSignal,
			refSignal, NUM_AUDIO_SAMPLES_ADC / 2, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

	res = adi_fir_EnableConfig(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}
	res = adi_fir_WaitForEvent(hConfigRef, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed Ref\n");
		return -1;
	}

	//memcpy(&conv_float_control_temp, refOutputBuff, sizeof(refOutputBuff));
	//memcpy(&conv_float_OSPM_temp, refOutputBuff, sizeof(refOutputBuff));

	memcpy(&conv_float_control_temp, refOutputBuff,
			refOutputSize*4u);
	memcpy(&conv_float_OSPM_temp, refOutputBuff, refOutputSize*4u);
#else
	memcpy(&conv_float_control_temp, refSignal,
			refOutputSize*4u);
	memcpy(&conv_float_OSPM_temp, refSignal, refOutputSize*4u);
#endif


	res = adi_fir_SubmitInputCircBuffer(hChannelControl1,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelControl2,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}
	res = adi_fir_SubmitInputCircBuffer(hChannelControl3,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}
#if (numControlSignal==6)
	res = adi_fir_SubmitInputCircBuffer(hChannelControl4,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}
	res = adi_fir_SubmitInputCircBuffer(hChannelControl5,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}
	res = adi_fir_SubmitInputCircBuffer(hChannelControl6,
			conv_float_control_temp, conv_float_control_temp, controlLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}
#endif

	res = adi_fir_EnableChannel(hChannelControl1, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl2, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl3, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
#if (numControlSignal==6)
	res = adi_fir_EnableChannel(hChannelControl4, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed hChannelControl41\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl5, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelControl6, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

#endif

	res = adi_fir_EnableConfig(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

//OSPM

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM11, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM12, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM13, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM21, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM22, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM23, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM31, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM32, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM33, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM11, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM12, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM13, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM21, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM22, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM23, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM31, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM32, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM33, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

#if (numControlSignal == 6)
	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM14, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM15, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM16, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM24, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM25, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM26, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM34, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM35, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM36, conv_float_OSPM_temp,
			conv_float_OSPM_temp, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM14, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM15, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM16, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM24, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM25, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM26, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM34, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM35, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM36, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
#endif

	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}


	res = adi_fir_WaitForEvent(hConfigOSPM, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelOSPM33);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed hChannel1_1\n");
		return -1;
	}
	res = adi_fir_WaitForEvent(hConfigControl, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelControl3);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed hChannel1_1\n");
		return -1;
	}

	memcpy(&controlOutputBuff1_temp, controlOutputBuff1,
			controlOutputSize*4u);
	memcpy(&controlOutputBuff2_temp, controlOutputBuff2,
			controlOutputSize*4u);
	memcpy(&controlOutputBuff3_temp, controlOutputBuff3,
			controlOutputSize*4u);
	memcpy(&controlOutputBuff4_temp, controlOutputBuff4,
			controlOutputSize*4u);
	memcpy(&controlOutputBuff5_temp, controlOutputBuff5,
			controlOutputSize*4u);
	memcpy(&controlOutputBuff6_temp, controlOutputBuff6,
			controlOutputSize*4u);
	memcpy(&OSPMRef[0][0], OSPMOutputBuff11, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[0][1], OSPMOutputBuff21, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[0][2], OSPMOutputBuff31, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[1][0], OSPMOutputBuff12, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[1][1], OSPMOutputBuff22, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[1][2], OSPMOutputBuff32, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[2][0], OSPMOutputBuff13, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[2][1], OSPMOutputBuff23, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[2][2], OSPMOutputBuff33, OSPMWindowSize * 4u);
#if (numControlSignal == 6)
	memcpy(&OSPMRef[3][0], OSPMOutputBuff14, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[3][1], OSPMOutputBuff24, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[3][2], OSPMOutputBuff34, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[4][0], OSPMOutputBuff15, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[4][1], OSPMOutputBuff25, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[4][2], OSPMOutputBuff35, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[5][0], OSPMOutputBuff16, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[5][1], OSPMOutputBuff26, OSPMWindowSize * 4u);
	memcpy(&OSPMRef[5][2], OSPMOutputBuff36, OSPMWindowSize * 4u);
#endif

	//do stuff while OSPM FIR
	for (uint32_t i = 0; i < OSPMLength; i++) {
		//AWGN_generator_temp[i]= AWGN_generator();

		OSPMAWGNSignal1[i] = (float) OSPMAWGNGain[0][i]
		* (float) AWGN_generator() ;
		 //*AWGN_generator_temp[i];
		OSPMAWGNSignal2[i] = (float) OSPMAWGNGain[1][i]
				* (float) (AWGN_generator()) ;
		OSPMAWGNSignal3[i] = (float) OSPMAWGNGain[2][i]
				* (float) (AWGN_generator());
#if (numControlSignal == 6 )
		OSPMAWGNSignal4[i] = (float) OSPMAWGNGain[3][i]
				* (float) (AWGN_generator()) ;
		OSPMAWGNSignal5[i] = (float) OSPMAWGNGain[4][i]
				* (float) (AWGN_generator());
		OSPMAWGNSignal6[i] = (float) OSPMAWGNGain[5][i]
				* (float) (AWGN_generator()) ;
#endif
	}
	res = adi_fir_EnableChannel(hChannelOSPM11, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM12, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM13, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM21, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM22, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM23, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM31, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM32, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM33, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

#if (numControlSignal == 6)

	// ---------------------------------------   Disable Channels -----------------------------------------------

	res = adi_fir_EnableChannel(hChannelOSPM14, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM15, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM16, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM24, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM25, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM26, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM34, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM35, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM36, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

#endif

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM11, OSPMAWGNSignal1,
			OSPMAWGNSignal1, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM12, OSPMAWGNSignal2,
			OSPMAWGNSignal2, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM13, OSPMAWGNSignal3,
			OSPMAWGNSignal3, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM21, OSPMAWGNSignal1,
			OSPMAWGNSignal1, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM22, OSPMAWGNSignal2,
			OSPMAWGNSignal2, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM23, OSPMAWGNSignal3,
			OSPMAWGNSignal3, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM31, OSPMAWGNSignal1,
			OSPMAWGNSignal1, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM32, OSPMAWGNSignal2,
			OSPMAWGNSignal2, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM33, OSPMAWGNSignal3,
			OSPMAWGNSignal3, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM11, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM12, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM13, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM21, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM22, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM23, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM31, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM32, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM33, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}

#if (numControlSignal == 6)
	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM14, OSPMAWGNSignal4,
			OSPMAWGNSignal4, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM15, OSPMAWGNSignal5,
			OSPMAWGNSignal5, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM16, OSPMAWGNSignal6,
			OSPMAWGNSignal6, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM24, OSPMAWGNSignal4,
			OSPMAWGNSignal4, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM25, OSPMAWGNSignal5,
			OSPMAWGNSignal5, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM26, OSPMAWGNSignal6,
			OSPMAWGNSignal6, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM34, OSPMAWGNSignal4,
			OSPMAWGNSignal4, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM35, OSPMAWGNSignal5,
			OSPMAWGNSignal5, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_SubmitInputCircBuffer(hChannelOSPM36, OSPMAWGNSignal6,
			OSPMAWGNSignal6, OSPMLength, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_SubmitInputCircBuffer failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOSPM14, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM15, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM16, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM24, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM25, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM26, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM34, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM35, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
	res = adi_fir_EnableChannel(hChannelOSPM36, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel failed\n");
		return -1;
	}
#endif

	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	res = adi_fir_WaitForEvent(hConfigOSPM, ADI_FIR_EVENT_ALL_CHANNEL_DONE,
			hChannelOSPM33);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_WaitForEvent failed hChannel1_1\n");
		return -1;
	}


	memcpy(&OSPMAux[0][0], OSPMOutputBuff11, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[0][1], OSPMOutputBuff21, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[0][2], OSPMOutputBuff31, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[1][0], OSPMOutputBuff12, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[1][1], OSPMOutputBuff22, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[1][2], OSPMOutputBuff32, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[2][0], OSPMOutputBuff13, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[2][1], OSPMOutputBuff23, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[2][2], OSPMOutputBuff33, OSPMWindowSize * 4u);
#if (numControlSignal == 6)
	memcpy(&OSPMAux[3][0], OSPMOutputBuff14, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[3][1], OSPMOutputBuff24, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[3][2], OSPMOutputBuff34, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[4][0], OSPMOutputBuff15, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[4][1], OSPMOutputBuff25, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[4][2], OSPMOutputBuff35, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[5][0], OSPMOutputBuff16, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[5][1], OSPMOutputBuff26, OSPMWindowSize * 4u);
	memcpy(&OSPMAux[5][2], OSPMOutputBuff36, OSPMWindowSize * 4u);
#endif
	  //cycle_t start_count=0;

	 // cycle_t final_count=0;
	//START_CYCLE_COUNT(start_count);

#pragma vector_for
	for (int32_t i = 0; i < OSPMOutputSize; i++) {
#pragma SIMD_for
		for (int32_t k = 0; k < numErrorSignal; k++) {

			filteredErrorSignal[k][i] = errorSignal[k][i]-
					(OSPMAux[0][k][i]+OSPMAux[1][k][i]+OSPMAux[2][k][i]+OSPMAux[3][k][i]+OSPMAux[4][k][i]+OSPMAux[5][k][i]);
		}
	}

//INDIRECT ERROR SIGNAL
#pragma vector_for
	for (uint32_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				indirectErrorSignal[j][k][i] = filteredErrorSignal[k][i]
						+ OSPMAux[j][k][i];
			}
		}
	}

	//	power of OSPMAWGNSignal
#pragma vector_for
	for (uint32_t i = 0; i < OSPMOutputSize; i++) {
		powerOSPMAWGNSignal[0][i] = forgettingFactor * powerOSPMAWGNSignal[0][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal1[i]
						* OSPMAWGNSignal1[i];
		powerOSPMAWGNSignal[1][i] = forgettingFactor * powerOSPMAWGNSignal[1][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal2[i]
						* OSPMAWGNSignal2[i];
		powerOSPMAWGNSignal[2][i] = forgettingFactor * powerOSPMAWGNSignal[2][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal3[i]
						* OSPMAWGNSignal3[i];

#if (numControlSignal ==6)
		powerOSPMAWGNSignal[3][i] = forgettingFactor * powerOSPMAWGNSignal[3][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal4[i]
						* OSPMAWGNSignal4[i];
		powerOSPMAWGNSignal[4][i] = forgettingFactor * powerOSPMAWGNSignal[4][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal5[i]
						* OSPMAWGNSignal5[i];
		powerOSPMAWGNSignal[5][i] = forgettingFactor * powerOSPMAWGNSignal[5][i]
				+ (1.0 - forgettingFactor) * OSPMAWGNSignal6[i]
						* OSPMAWGNSignal6[i];

#endif

	}
	//	power of filteredErrorSignal
#pragma vector_for
	for (uint32_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			powerFilteredErrorSignal[k][i] = forgettingFactor
					*  powerFilteredErrorSignal[k][i]
					+  (1.0 - forgettingFactor)
							*  filteredErrorSignal[k][i]
							*  filteredErrorSignal[k][i];
		}
	}

	//power of indirectErrorSignal
#pragma vector_for
	for (uint32_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				powerIndirectErrorSignal[j][k][i] = forgettingFactor
						* powerIndirectErrorSignal[j][k][i]
						+ ((1.0 - forgettingFactor)
								*  indirectErrorSignal[j][k][i]
								* indirectErrorSignal[j][k][i]);
			}
		}
	}
	//stepSizeS
#pragma vector_for
	for (uint32_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint32_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			stepSizeS[j][k][i] = ( powerOSPMAWGNSignal[j][i]
					*  stepSizeSMin)
					/  powerIndirectErrorSignal[j][k][i];
		}
	}
	}

	//OSPMAWGNGain
#pragma vector_for
	for (uint32_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			OSPMAWGNGain[j][i] =
					( powerFilteredErrorSignal[0][i]
							+ powerFilteredErrorSignal[1][i]
							+  powerFilteredErrorSignal[2][i])
							/ ( powerIndirectErrorSignal[j][0][i]
									+  powerIndirectErrorSignal[j][1][i]
									+  powerIndirectErrorSignal[j][2][i]);

		}
	}

	disableAllFirChannelsResult = DisableAllFIRChannels();
	if (disableAllFirChannelsResult != 0) {
		printf("error disabling FIR channels");
	}

	//OSPM Coeff
#pragma vector_for
	for (uint32_t i = 0; i < OSPMOutputSize; i++) {
		OSPMCoeffBuff11[i] = OSPMCoeffBuff11[i]
				+ (stepSizeS[0][0][i] * OSPMAux[0][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff21[i] = OSPMCoeffBuff21[i]
				+ (stepSizeS[0][1][i] * OSPMAux[0][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff31[i] = OSPMCoeffBuff31[i]
				+ (stepSizeS[0][2][i] * OSPMAux[0][2][i]
						* filteredErrorSignal[2][i]);
		OSPMCoeffBuff12[i] = OSPMCoeffBuff12[i]
				+ (stepSizeS[1][0][i] * OSPMAux[1][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff22[i] = OSPMCoeffBuff22[i]
				+ (stepSizeS[1][1][i] * OSPMAux[1][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff32[i] = OSPMCoeffBuff32[i]
				+ (stepSizeS[1][2][i] * OSPMAux[1][2][i]
						* filteredErrorSignal[2][i]);
		OSPMCoeffBuff13[i] = OSPMCoeffBuff13[i]
				+ (stepSizeS[2][0][i] * OSPMAux[2][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff23[i] = OSPMCoeffBuff23[i]
				+ (stepSizeS[2][1][i] * OSPMAux[2][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff33[i] = OSPMCoeffBuff33[i]
				+ (stepSizeS[2][2][i] * OSPMAux[2][2][i]
						* filteredErrorSignal[2][i]);
#if (numControlSignal ==6)
		OSPMCoeffBuff14[i] = OSPMCoeffBuff14[i]
				+ (stepSizeS[3][0][i] * OSPMAux[3][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff24[i] = OSPMCoeffBuff24[i]
				+ (stepSizeS[3][1][i] * OSPMAux[3][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff34[i] = OSPMCoeffBuff34[i]
				+ (stepSizeS[3][2][i] * OSPMAux[3][2][i]
						* filteredErrorSignal[2][i]);
		OSPMCoeffBuff15[i] = OSPMCoeffBuff15[i]
				+ (stepSizeS[4][0][i] * OSPMAux[4][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff25[i] = OSPMCoeffBuff25[i]
				+ (stepSizeS[4][1][i] * OSPMAux[4][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff35[i] = OSPMCoeffBuff35[i]
				+ (stepSizeS[4][2][i] * OSPMAux[4][2][i]
						* filteredErrorSignal[2][i]);
		OSPMCoeffBuff16[i] = OSPMCoeffBuff16[i]
				+ (stepSizeS[5][0][i] * OSPMAux[5][0][i]
						* filteredErrorSignal[0][i]);
		OSPMCoeffBuff26[i] = OSPMCoeffBuff26[i]
				+ (stepSizeS[5][1][i] * OSPMAux[5][1][i]
						* filteredErrorSignal[1][i]);
		OSPMCoeffBuff36[i] = OSPMCoeffBuff36[i]
				+ (stepSizeS[5][2][i] * OSPMAux[5][2][i]
						* filteredErrorSignal[2][i]);

#endif

	}

	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM11, OSPMCoeffBuff11,
			OSPMCoeffBuff11, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM12, OSPMCoeffBuff12,
			OSPMCoeffBuff12, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM13, OSPMCoeffBuff13,
			OSPMCoeffBuff13, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM21, OSPMCoeffBuff21,
			OSPMCoeffBuff21, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM22, OSPMCoeffBuff22,
			OSPMCoeffBuff22, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM23, OSPMCoeffBuff23,
			OSPMCoeffBuff23, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM31, OSPMCoeffBuff31,
			OSPMCoeffBuff31, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM32, OSPMCoeffBuff32,
			OSPMCoeffBuff32, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM33, OSPMCoeffBuff33,
			OSPMCoeffBuff33, 1);

#if (numControlSignal ==6)
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM14, OSPMCoeffBuff14,
			OSPMCoeffBuff14, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM15, OSPMCoeffBuff15,
			OSPMCoeffBuff15, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM16, OSPMCoeffBuff16,
			OSPMCoeffBuff16, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM24, OSPMCoeffBuff24,
			OSPMCoeffBuff24, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM25, OSPMCoeffBuff25,
			OSPMCoeffBuff25, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM26, OSPMCoeffBuff26,
			OSPMCoeffBuff26, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM34, OSPMCoeffBuff34,
			OSPMCoeffBuff34, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM35, OSPMCoeffBuff35,
			OSPMCoeffBuff35, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelOSPM36, OSPMCoeffBuff36,
			OSPMCoeffBuff36, 1);

#endif

	//Control Coeff
#pragma vector_for
	for (uint32_t i = 0; i < controlOutputSize; i++) {

		controlCoeffBuff1[i] =  controlCoeffBuff1[i]
				+ stepSizeW[0]
						*  (( filteredErrorSignal[0][i]
								*  OSPMRef[0][0][i])
								+ ( filteredErrorSignal[1][i]
										*  OSPMRef[0][1][i])
								+ ( filteredErrorSignal[2][i]
										*  OSPMRef[0][2][i]));
		controlCoeffBuff2[i] =
				controlCoeffBuff2[i]
						+ stepSizeW[1]
								* ((filteredErrorSignal[0][i] * OSPMRef[1][0][i])
										+ (filteredErrorSignal[1][i]
												* OSPMRef[1][1][i])
										+ (filteredErrorSignal[2][i]
												* OSPMRef[1][2][i]));
		controlCoeffBuff3[i] =
				controlCoeffBuff3[i]
						+ stepSizeW[2]
								* ((filteredErrorSignal[0][i] * OSPMRef[2][0][i])
										+ (filteredErrorSignal[1][i]
												* OSPMRef[2][1][i])
										+ (filteredErrorSignal[2][i]
												* OSPMRef[2][2][i]));
#if (numControlSignal ==6)
		controlCoeffBuff4[i] =
				controlCoeffBuff4[i]
						+ stepSizeW[3]
								* ((filteredErrorSignal[0][i] * OSPMRef[3][0][i])
										+ (filteredErrorSignal[1][i]
												* OSPMRef[3][1][i])
										+ (filteredErrorSignal[2][i]
												* OSPMRef[3][2][i]));
		controlCoeffBuff5[i] =
				controlCoeffBuff5[i]
						+ stepSizeW[4]
								* ((filteredErrorSignal[0][i] * OSPMRef[4][0][i])
										+ (filteredErrorSignal[1][i]
												* OSPMRef[4][1][i])
										+ (filteredErrorSignal[2][i]
												* OSPMRef[4][2][i]));
		controlCoeffBuff6[i] =
				controlCoeffBuff6[i]
						+ stepSizeW[5]
								* ((filteredErrorSignal[0][i] * OSPMRef[5][0][i])
										+ (filteredErrorSignal[1][i]
												* OSPMRef[5][1][i])
										+ (filteredErrorSignal[2][i]
												* OSPMRef[5][2][i]));
#endif

	}
	adi_fir_SetChannelCoefficientBuffer(hChannelControl1, controlCoeffBuff1,
			controlCoeffBuff1, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelControl2, controlCoeffBuff2,
			controlCoeffBuff2, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelControl3, controlCoeffBuff3,
			controlCoeffBuff3, 1);

#if (numControlSignal ==6)
	adi_fir_SetChannelCoefficientBuffer(hChannelControl4, controlCoeffBuff4,
			controlCoeffBuff4, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelControl5, controlCoeffBuff5,
			controlCoeffBuff5, 1);
	adi_fir_SetChannelCoefficientBuffer(hChannelControl6, controlCoeffBuff6,
			controlCoeffBuff6, 1);

#endif
/*
#pragma vector_for
	for (uint32_t i = 0, j=0; i < NUM_AUDIO_SAMPLES / 2, j <controlLength; i+=4, j++) {
		outputSignal1[i] = controlOutputBuff1_temp[j] +OSPMAWGNSignal1[j];
		outputSignal2[i] = controlOutputBuff2_temp[j] +OSPMAWGNSignal2[j];
		outputSignal3[i] =  controlOutputBuff3_temp[j] +OSPMAWGNSignal3[j];

#if (numControlSignal ==6)
		outputSignal4[i] = controlOutputBuff4_temp[j] +OSPMAWGNSignal4[j];
		outputSignal5[i] =  controlOutputBuff5_temp[j] +OSPMAWGNSignal5[j];
		outputSignal6[i] =  controlOutputBuff6_temp[j] +OSPMAWGNSignal6[j];
#endif
*/
#pragma vector_for
	for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_DAC; i++) {
		outputSignal1[i] = controlOutputBuff1_temp[i] +OSPMAWGNSignal1[i];
		outputSignal2[i] = controlOutputBuff2_temp[i] +OSPMAWGNSignal2[i];
		outputSignal3[i] =  controlOutputBuff3_temp[i] +OSPMAWGNSignal3[i];

#if (numControlSignal ==6)
		outputSignal4[i] = controlOutputBuff4_temp[i] +OSPMAWGNSignal4[i];
		outputSignal5[i] =  controlOutputBuff5_temp[i] +OSPMAWGNSignal5[i];
		outputSignal6[i] =  controlOutputBuff6_temp[i] +OSPMAWGNSignal6[i];
#endif
	}
	//STOP_CYCLE_COUNT(final_count,start_count);
	//PRINT_CYCLES("\nNumber of cycles:",final_count);
	/*
	reverseArrayf(outputSignal1, NUM_AUDIO_SAMPLES / 2);
	reverseArrayf(outputSignal2, NUM_AUDIO_SAMPLES / 2);
	reverseArrayf(outputSignal3, NUM_AUDIO_SAMPLES / 2);
#if(numControlSignal==6)
	reverseArrayf(outputSignal4, NUM_AUDIO_SAMPLES / 2);
	reverseArrayf(outputSignal5, NUM_AUDIO_SAMPLES / 2);
	reverseArrayf(outputSignal6, NUM_AUDIO_SAMPLES / 2);

#endif
*/
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
void    Block_Fixed_To_Float( int * Fixed_In, float * Float_Out_L, float * Float_Out_R )
{
    int i;
    #pragma SIMD_for
    for (i=0;i<NUM_AUDIO_SAMPLES/2;i++) //AUDIO_BLOCKSIZE
    {
        Float_Out_L[i]  = ((float) (Fixed_In[2*i]<<8))   * (1.0/2147483648.0);
        Float_Out_R[i]  = ((float) (Fixed_In[2*i+1]<<8)) * (1.0/2147483648.0);
    }
}

void    Block_Float_To_Fixed( int * Fixed_Out, float * Float_In_L, float * Float_In_R )
{
    int i;
    #pragma SIMD_for
    for (i=0;i<NUM_AUDIO_SAMPLES/2;i++)
    {
        Fixed_Out[2*i]   = ((int) (2147483648.0*Float_In_L[i]))>>8;
        Fixed_Out[2*i+1] = ((int) (2147483648.0*Float_In_R[i]))>>8;
    }
}*/

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
uint32_t PcgInit(void)
{
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
    if( adi_pcg_Open(DeviceNum,
                     &PcgMemory[0],
                     ADI_PCG_MEMORY_SIZE,
                     &phPcg) != ADI_PCG_SUCCESS)
    {
        Result = 1u;
    }


    /* Select FS and clock outputs for PCG */
    if((uint32_t)adi_pcg_SelectOutput(phPcg, ADI_PCG_OUT_CLK_FS) != 0u)
    {
        /* return error */
        return 1u;
    }

    /* Set the PCG input clock source to external*/
    if((uint32_t)adi_pcg_ClockSource(phPcg, ADI_PCG_CLK_EXT) != 0u)
    {
        /* return error */
        return 1u;
    }

    /* Set the PCG input fs to external */
    if((uint32_t)adi_pcg_FrameSyncSource(phPcg, ADI_PCG_FS_EXT) != 0u)
    {
        /* return error */
        return 1u;
    }

    /* Clock C numASRC * 64 * Fs Hz */
    if((uint32_t)adi_pcg_ClockDivide(phPcg, pcgCLKDIV) != 0u)
    {
        /* return error */
        return 1u;
    }
    /* FS C kHz */
    if((uint32_t)adi_pcg_FrameSyncDivide(phPcg, pcgFSDIV) != 0u)
    {
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
uint32_t AsrcDacInit(void)
{
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
    if( adi_asrc_Open(nBlockNum,
    				  0u,
                      &AsrcMemory4[0],
                      ADI_ASRC_MEMORY_SIZE,
                      &phAsrc4) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }
    /* For ADSP-SC573 family processors, open ASRC 1.  Otherwise open ASRC 5 */
    if( adi_asrc_Open(nBlockNum,
    				  1u,
                      &AsrcMemory5[0],
                      ADI_ASRC_MEMORY_SIZE,
                      &phAsrc5) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }
    /* For ADSP-SC573 family processors, open ASRC 2.  Otherwise open ASRC 6 */
    if( adi_asrc_Open(nBlockNum,
    				  2u,
                      &AsrcMemory6[0],
                      ADI_ASRC_MEMORY_SIZE,
                      &phAsrc6) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }

    /* For ADSP-SC573 family processors, open ASRC 3.  Otherwise open ASRC 7 */
    if( adi_asrc_Open(nBlockNum,
    				  3u,
                      &AsrcMemory7[0],
                      ADI_ASRC_MEMORY_SIZE,
                      &phAsrc7) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }


    //adi_asrc_SetMatchPhaseMode(phAsrc6, true);
    if( adi_asrc_SetSerialFormat(phAsrc4, ADI_ASRC_INPUT_TDM, ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }
    if( adi_asrc_SetSerialFormat(phAsrc5, ADI_ASRC_INPUT_TDM, ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }
    if( adi_asrc_SetSerialFormat(phAsrc6, ADI_ASRC_INPUT_TDM, ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }
    if( adi_asrc_SetSerialFormat(phAsrc7, ADI_ASRC_INPUT_TDM, ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS)
    {
        Result = 1u;
    }


    return Result;
}


uint32_t SpuInit(void)
{
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
return (Result);
}

