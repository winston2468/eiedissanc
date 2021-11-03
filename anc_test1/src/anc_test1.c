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
#include <SRU.h>

#include <services/int/adi_sec.h>
#include <drivers/adc/adau1979/adi_adau1979.h>
#include <drivers/twi/adi_twi.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Audio_EI3/drivers/codec/adau1761/adi_adau1761.h"
#include "adi_initialize.h"
#include "Audio_EI3/drivers/codec/adau1761/export_IC_1.h"
#include <filter.h>

#include <stdlib.h>
#include <adi_types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <drivers/fir/adi_fir.h>

#include <vector.h>
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

#include "cycle_count.h"
#include "anc_test1.h"

/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE

static ADI_DMA_2D_MEM_TRANSFER Src_2DMemXfer;
static ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXfer;

volatile bool bEnableOutput = true;
volatile bool bEnableOCPM = true;
static volatile bool bMemCopyInProgress = false;

#ifdef RefFilter
static volatile bool bRefFIRInProgress = false;
#endif
static volatile bool bControlFIRInProgress = false;
static volatile bool bOCPMRefFIRInProgress = false;
static volatile bool bOCPMAuxFIRInProgress = false;
volatile bool controlFirstIt = true;
volatile bool controlW = false;
#ifdef OCPMExtendedFilter
static volatile bool bOCPMExtendedFIRInProgress = false;
#endif

#ifdef OFPMFilter
static volatile bool bOFPMFIRInProgress = false;
static volatile bool bOFPMErrorFIRInProgress = false;
#endif
//*****************************************************************************
uint8_t DacStarted = 0u;
/* ADC/DAC buffer pointer */
volatile void *pGetDAC = NULL;
volatile void *pDAC = NULL;
 int countaa =0;
#pragma alignment_region (4)
float controlOutputBuff[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };

#pragma alignment_region_end

#ifdef USE_ASRC
extern uint32_t PcgDacInit(void);
extern uint32_t AsrcDacInit(void);
extern uint32_t PcgDacEnable(void);
extern uint32_t AsrcDacEnable(void);
#endif

/* Initializes DAC */
extern uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
extern uint32_t Adau1962aSubmitBuffers(void);
extern uint32_t Adau1962aEnable(void);
extern uint32_t Adau1962aDoneWithBuffer(volatile void *pBuffer);
uint8_t ControlFIR(void);
uint8_t GenControlSignal(void);
uint8_t PushControlSignal(void);

int32_t randBuff[4] = { 0 };

#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)

#ifdef USE_ADAU1761
/* the ADAU1761 Rec Mixer Left 0 register */
#define REC_MIX_LEFT_REG    (0x400A)
/* the ADAU1761 Rec Mixer Right 0 register */
#define REC_MIX_RIGHT_REG   (0x400C)

/* codec device instance to be tested */
#define ADAU1761_DEV_NUM          0

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
#pragma align(4)
int32_t AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL* 2 * 2];
volatile void *pGetADC1 = NULL;
void *pADC1;
int32_t *pADC1Buffer;

volatile bool ADC1Flag =false;

static ADI_ADAU1761_HANDLE hADAU1761_2;

#define ADAU1761_DEV_NUM1          1
#pragma align 4
int32_t AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL*2 * 2];

#pragma align(4)
static uint8_t sportRxMem2[ADI_SPORT_DMA_MEMORY_SIZE];

static uint8_t codecMem2[ADI_ADAU1761_MEMORY_SIZE];

volatile void *pGetADC2 = NULL;
void *pADC2;
int32_t *pADC2Buffer;
volatile bool ADC2Flag =false;

#endif

volatile int32_t DacCount = 0;
 bool OCPMUpdate = true;
#pragma alignment_region (4)
int32_t * pADCBuffer;

/* ----------------------------   FIR Configuration ------------------------------------------- */
//int32_t test[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS]={0};
float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL*2-1] = { 0 };
//float errorSignal_temp[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
/* Channel Configurations parameters */
//Max FIR channels = 32

uint8_t ConfigMemoryRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryRef[ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryOCPMRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMRef[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
uint8_t ConfigMemoryOCPMAux[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMAux[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryControl[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryControl[numControlSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

//float refSignal[NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
float refInputBuff[refInputSize] = { 0 };
float controlOutputSignal[numControlSignal][OFPMInputSize] = { 0 };
float controlInputBuff[controlInputSize] = { 0 };
float OCPMAuxInputBuff[numControlSignal][OCPMInputSize] = { 0 };
float OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength] = { 0 };
float controlCoeffBuff[numControlSignal][controlLength] = { 0 };
float controlCoeffBuff_temp[numControlSignal][controlLength] = { 0 };
float OCPMRefOutputBuff[numControlSignal][numErrorSignal][OCPMOutputSize] =
		{ 0 };
float OCPMRefOutputBuff_pp[numControlSignal][numErrorSignal][OCPMOutputSize*2-1] =
		{ 0 };
float OCPMAuxOutputBuff[numControlSignal][numErrorSignal][OCPMOutputSize] =
		{ 0 };

#ifdef OCPMExtendedFilter
uint8_t ConfigMemoryOCPMExtended[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMExtended[numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
float OCPMExtendedCoeffBuff[numErrorSignal][OCPMLength] = { 0 };
float OCPMExtendedOutputBuff[numErrorSignal][OCPMOutputSize] = { 0 };
#endif

float WNSignal[WNLength] = { 0 };
#ifdef OCPMExtendedFilter
float OCPMExtendedError[numErrorSignal][OCPMLength*2-1] = { 0 };
#endif
float uncorruptedErrorSignal[numErrorSignal][OCPMLength*2-1] = { 0 };
//float uncorruptedRefSignal[OFPMInputSize] = { 0 };

float OCPMWNGain[numControlSignal] = { 0 };
float powerOCPMWNSignal[numControlSignal] = { 0 };
float powerResidualErrorSignal[numErrorSignal] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal][OCPMLength]={0};
float powerIndirectErrorSignal[numControlSignal][numErrorSignal] = { 0 };
float powerUncorruptedErrorSignal[numErrorSignal] = { 0 };
//float powerUncorruptedRefSignal = 0;

float forgettingFactorOCPM = 0.6;

float stepSizeW = 0;
float stepSizeE = 0;
float stepSizeSMin = 1; // 0.0005;//0.00001;
float stepSizeS[numControlSignal][numErrorSignal] = { 0 };
#ifdef OFPMFilter
uint8_t ConfigMemoryOFPM[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOFPM[numControlSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryOFPMError[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOFPMError[ADI_FIR_CHANNEL_MEMORY_SIZE];
float OFPMErrorSignal[OCPMLength] = {0};
float OFPMInputBuff[numControlSignal][OFPMInputSize] = {0};
float OFPMCoeffBuff[numControlSignal][OFPMLength] = {0};

float OFPMErrorInputBuff[OFPMInputSize] = {0};
float OFPMErrorCoeffBuff[OFPMLength] = {0};

float powerOFPMErrorSignal = 0;
float OFPMPowerRatio = 0;
float forgettingFactorOFPM = 0.9;
float stepSizeH = 0;
float stepSizeHMin = 0.001; //0.00001;
float stepSizeHMax = 0.000001;//0.0001;
float stepSizeFMin = 0.000001;//0.00001;
float stepSizeFMax = 0.001;//0.0001;
float stepSizeF = 0;
float OFPMLeak = 0.999999;
float OFPMErrorLeak = 0.999999;
float OFPMOutputBuff[numControlSignal][OFPMOutputSize] = {0};
float OFPMErrorOutputBuff[OFPMOutputSize]= {0};
#endif
float controlLeak = 0.0001;
float OCPMLeak = 0.0001;
float OCPMExtendedLeak = 0.0001;

#ifdef RefFilter
float OCPMRefInputBuff[OCPMInputSize] = {0};
float refCoeffBuff[refLength] = {
#include "data/lowpass_filter_2000_2500.dat"
};
float refOutputBuff[refOutputSize] = {0};
#endif

#pragma alignment_region_end

ADI_FIR_RESULT res;
ADI_FIR_HANDLE hFir;
ADI_FIR_CHANNEL_PARAMS channelOCPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_PARAMS channelOCPMAux[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_PARAMS channelControl[numControlSignal];
ADI_FIR_CONFIG_HANDLE hConfigOCPMRef;
ADI_FIR_CONFIG_HANDLE hConfigOCPMAux;
ADI_FIR_CONFIG_HANDLE hConfigControl;
ADI_FIR_CHANNEL_HANDLE hChannelOCPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelOCPMAux[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelControl[numControlSignal];
#ifdef OCPMExtendedFilter
ADI_FIR_CHANNEL_PARAMS channelOCPMExtended[numErrorSignal];
ADI_FIR_CONFIG_HANDLE hConfigOCPMExtended;
ADI_FIR_CHANNEL_HANDLE hChannelOCPMExtended[numErrorSignal];
#endif

#ifdef OFPMFilter
ADI_FIR_CHANNEL_PARAMS channelOFPM[numControlSignal];
ADI_FIR_CHANNEL_PARAMS channelOFPMError;
ADI_FIR_CONFIG_HANDLE hConfigOFPM;
ADI_FIR_CONFIG_HANDLE hConfigOFPMError;
ADI_FIR_CHANNEL_HANDLE hChannelOFPM[numControlSignal];
ADI_FIR_CHANNEL_HANDLE hChannelOFPMError;
#endif
#ifdef RefFilter
ADI_FIR_CHANNEL_PARAMS channelRef;
ADI_FIR_CONFIG_HANDLE hConfigRef;
ADI_FIR_CHANNEL_HANDLE hChannelRef;
#endif

volatile bool TRNGFlag = false;

/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;

int32_t *pSrc;

int32_t *pDst;
volatile bool DACFlag = false;

typedef enum {
	NONE, START, RECIEVE
} MODE;

static MODE eMode = NONE;

static bool bError;
static uint32_t count;

volatile bool bEvent;
volatile uint32_t OCPMWNSignal_X = 0;
//volatile uint32_t OCPMWNSignal_X =OCPMWindowSize-1;
//volatile uint32_t OCPMWNSignal_Y =0;

/* ADC buffer pointer */
volatile void *pGetADC = NULL;
volatile void *pADC = NULL;
volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;

extern int32_t MemDma0Copy1D(void *pMemDest, void *pMemSrc,
		uint8_t nBytesInElement, uint32_t ElementCount);

extern int32_t MemDma0Copy2D(void *pDestBuffer, void *pSrcBuffer,
		uint8_t nBytesInElement, uint32_t Xcount, uint32_t YCount);

/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/

/* Initializes ADC */
extern uint32_t Adau1979Init(void);

/* Submit buffers to ADC */
extern uint32_t Adau1979SubmitBuffers(void);

extern uint32_t Adau1979Enable(void);

extern uint32_t Adau1979DoneWithBuffer(volatile void *pBuffer);
extern void configGpio(void);
float constrain(float input, float low, float high);

//void ProcessBufferADC(uint32_t iid, void* handlerArg);
void ProcessBufferADC(void);
void ProcessBufferDAC(uint32_t iid, void* handlerArg);
#ifdef RefFilter
void RefFIR(void);
#endif
int32_t FIR_init(void);
void reverseArrayf(float*, uint32_t);
int32_t ControlWeightUpdate(void);
int32_t OCPMRefFIR(void);
int32_t OCPMAuxFIR(void);
int32_t OCPMWeightUpdate(void);
#ifdef OCPMExtendedFilter
int32_t OCPMExtendedFIR(void);
int32_t OCPMExtendedWeightUpdate(void);
#endif
#ifdef OFPMFilter
int32_t OFPMFIR(void);
int32_t OFPMErrorFIR(void);
#endif
int32_t ANCALG_1(void);
int32_t ANCALG_2(void);
int32_t ANCALG_3(void);
int32_t ANCALG_4(void);
int32_t ANCALG_5(void);

int32_t WN_Gen(void);

extern uint32_t DMAInit(void);
#ifdef RefFilter
static void RefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
#endif
static void ControlFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OCPMRefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OCPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
#ifdef OCPMExtendedFilter
static void OCPMExtendedFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
#endif
static void OFPMFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OFPMErrorFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);

void *DMASlaveDestinationAddress;

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";

extern void PKIC_Interrupt_ACK(int iValue);
extern void Enable_TRNG_Interrupt(void);
extern int Read_TRNG_Stat(void);
extern int Read_Alarm_Mask(void);
extern int Read_Alarm_Stop(void);
extern int Read_PKIC_Masked_Interrupt_Source(void);
extern void Mask_Interrupt(int);

extern void Startup_Cycle_Number(int iValue);
extern void Min_Refill_Cycle(int iValue);

extern void Max_Refill_Cycle(int iValue);
extern void Sample_division(int iValue);

extern void Set_Alarm_Threshold(int iValue);
extern void Set_Shutdown_Threshold(int iValue);

extern void Disbale_FRO(int iValue);
extern void Detune_FRO(int iValue);
extern void Enable_FRO(int iValue);
extern void Acknowledge_Interrupt(int iValue);
extern void Disable_TRNG(void);
extern void Enable_TRNG(void);
extern void Set_PKIC_Polarity(int iValue);
extern void Set_PKIC_Level_type(int iValue);
extern void Disable_PKIC_Interrupt(int iValue);

extern int Read_PKIC_Unmasked_Interrupt_Source(void);

extern void Read_TRNG_Output(int *iOutput);
volatile uint32_t iida = 0;

void Read_TRNG_Output_Imp(float *iOutput);

static void PKIC_ISR(uint32_t iid, void* handlerArg);

static void TRNG_ISR(void);
static void TRNG_ACK(void);
float VecSumf(const void* x, uint32_t length);

unsigned int *randSeed;
unsigned int randSeedInt = 0;
/*
 int rand_r_imp(unsigned int *seed);
 */
void aluHandler(uint32_t iid, void* handlerArg);

void aluHandler(uint32_t iid, void* handlerArg) {
	printf("%d", iid);
	printf("ERR");
	iida = iid;
}

/*
 int rand_r_imp(unsigned int *seed) {
 unsigned int next = *seed;
 int result;
 next *= 1103515245;
 next += 12345;
 result = (unsigned int) (next / 65536) % 2048;
 next *= 1103515245;
 next += 12345;
 result <<= 10;
 result ^= (unsigned int) (next / 65536) % 1024;
 next *= 1103515245;
 next += 12345;
 result <<= 10;
 result ^= (unsigned int) (next / 65536) % 1024;
 *seed = next;
 return result;
 }
 */
#ifdef USE_ADAU1761
static void CheckResult(ADI_ADAU1761_RESULT result);
void MixerEnable(bool bEnable);
static void CheckResult(ADI_ADAU1761_RESULT result) {
	if (result != ADI_ADAU1761_SUCCESS) {
		printf("Codec failure\n");
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
		pADC1 = pArg;

		if(ANCInProgress) {
			printf("ANC ERR");
		}

		ADC1Flag= true;
		if(ADC1Flag && ADC2Flag){
			eMode = RECIEVE;
		bEvent = true;
		}
		break;
		default:
		printf("test1");
		break;
	}
}

/* codec callback */
static void ADAU1761Callback_2(void *pCBParam, uint32_t Event, void *pArg) {
	switch (Event) {
		case (uint32_t) ADI_ADAU1761_EVENT_RX_BUFFER_PROCESSED:
		//AdcCount++;
		pADC2 = pArg;

		ADC2Flag= true;
		if(ADC1Flag && ADC2Flag){
			eMode = RECIEVE;
		bEvent = true;
		}
		break;
		default:
		printf("test2");
		break;

	}
}

void MixerEnable(bool bEnable) {
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_RESULT result2;
	uint8_t value;

#ifdef USE_LINE_IN
	if (bEnable) {
		/* enable the record mixer (left) */
		//0x5B 0 db
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_LEFT_REG, 0x13); /* -12 dB */
		CheckResult(result1);

		/* enable the record mixer (right) */
		result1 = adi_adau1761_SetRegister(hADAU1761_1, REC_MIX_RIGHT_REG,
				0x13); /* 0 dB */
		CheckResult(result1);
		//RISING EDGE BCLK POLARITY
		//result = adi_adau1761_SetRegister (hADAU1761_1, 0x4015, 0x11); /* MUTE */
		//CheckResult(result);

		/* enable the record mixer (left) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_LEFT_REG,0x13); /* -12 dB */
		CheckResult(result2);

		/* enable the record mixer (right) */
		result2 = adi_adau1761_SetRegister(hADAU1761_2, REC_MIX_RIGHT_REG,
				0x13); /* 0 dB */
		CheckResult(result2);

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
}

#endif

uint8_t ControlFIR() {
#ifdef RefFilter
	bMemCopyInProgress=true;
	MemDma0Copy1D((void*) &OCPMRefInputBuff[NUM_AUDIO_SAMPLES_PER_CHANNEL - 1], (void *)&refOutputBuff[0], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL);
	while(bMemCopyInProgress);

#endif

	for (uint32_t j = 0; j < numControlSignal; j++) {

		res = adi_fir_SubmitInputCircBuffer(hChannelControl[j],
#ifdef RefFilter
				OCPMRefInputBuff, OCPMRefInputBuff,
#else
#ifdef OFPMFilter
				uncorruptedRefSignal, uncorruptedRefSignal,
#else
				(void*)&refInputBuff, (void*)&refInputBuff,
#endif
#endif
				controlInputSize, 1);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_SubmitInputCircBuffer hChannelControl %d failed\n",
					j);
		}

	}

	for (uint32_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_EnableChannel(hChannelControl[j], true);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel hChannelControl failed\n");
		}
	}

	bControlFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig hConfigControl failed\n");
	}

	return 0;
}

uint8_t GenControlSignal() {
	if(OCPMUpdate){
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {

				controlOutputSignal[j][i + NUM_AUDIO_SAMPLES_PER_CHANNEL - 1] =
						controlOutputBuff[j][i]
								+ OCPMAuxInputBuff[j][i + OCPMWindowSize - 1];

			}
	}
	}
	else{
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {


					controlOutputSignal[j][i + NUM_AUDIO_SAMPLES_PER_CHANNEL - 1] =
							controlOutputBuff[j][i];

			}
	}

	}
	return 0;
}

uint8_t PushControlSignal() {
	if (pGetDAC != NULL) {
		pDAC = (void *) pGetDAC;
#ifndef USE_ADAU1761
		Adau1979DoneWithBuffer(pGetADC);
		pGetADC = NULL;
#endif
		Adau1962aDoneWithBuffer(pGetDAC);

		pGetDAC = NULL;
		int32_t *pDst;
		pDst = (int32_t *) pDAC;
		for (uint32_t i = NUM_AUDIO_SAMPLES_PER_CHANNEL - 1; i < OFPMInputSize;
				i++) {
			//TDM8 SHIFT <<8
			/*
			 for(int32_t j =0; j< numControlSignal;j++){
			 *pDst++ = (conv_fix_by(controlSignal[j][i], 10)) ;
			 }

			 for(int32_t m =0; m< NUM_DAC_CHANNELS - numControlSignal;m++){
			 *pDst++ = 0;
			 }
			 */
			/*
			 *pDst++ = (conv_fix_by(controlOutputSignal[0][i], 18));
			 *pDst++ = (conv_fix_by(controlOutputSignal[1][i], 18));
			 *pDst++ = 0;
			 *pDst++ = 0;
			 *pDst++ = 0;
			 *pDst++ = 0;
			 *pDst++ = 0;
			 *pDst++ = 0;

			 */

			//*pDst++ = (conv_fix_by(refInputBuff[i], 25));
			//*pDst++ = (conv_fix_by(OCPMAuxInputBuff[0][i], 25));
			*pDst++ = (conv_fix_by(controlOutputSignal[0][i], 15));
			*pDst++ = (conv_fix_by(controlOutputSignal[1][i], 15));
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;

			/*
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 *pDst++ = 0;
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 *pDst++ = 0;
			 *pDst++ = (conv_fix_by(WNSignal[0][i]*10000000, 1));
			 */

		}
	}
	else{
		printf("error");
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
// DAC callback - Called when the DAC requires data
//				  The MDMA A1 callback MUST occur before this callback.
//				  The user must make sure the filters do not take more time
//				  than is available in the frame.
//////////////////////////////////////////////////////////////////////////////
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg) {

	switch (nEvent) {
	case ADI_SPORT_EVENT_TX_BUFFER_PROCESSED:
		// store pointer to the processed buffer that caused the callback
		// We can still copy to the buffer after it is submitted to the driver
		pGetDAC = pArg;


		/*
		 if (ANCInProgress){
		 ANCERR = true;
		 printf("anc err\n");
		 }
		 */

		//DacCount++;

		break;
	default:
		printf("err");
		break;
	}
}

float VecSumf(const void* x, uint32_t length)

{
	float sum = 0.0f;
	float* y = (float*) x;
#pragma vector_for
	for (uint32_t i = 0; i < length; i++) {
		sum += *(y + i);
	}
	return sum;
}

//convert to float [-10,10]
void Read_TRNG_Output_Imp(float *iOutput) {
	//int range [-2147483647,2147483647]
	float *iTemp;
	iTemp = iOutput;
	*iTemp = (((float) (*pREG_TRNG0_OUTPUT0)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float) (*pREG_TRNG0_OUTPUT1)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float) (*pREG_TRNG0_OUTPUT2)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float) (*pREG_TRNG0_OUTPUT3)) / (2147483647)) - 1;
}

void PKIC_ISR(uint32_t iid, void* handlerArg) {
	int iTemp, iTemp2;
	iTemp = Read_PKIC_Masked_Interrupt_Source();
	iTemp2 = iTemp && 0x8;
	if ((iTemp && 0x8) == 1) {
		PKIC_Interrupt_ACK(0x8);
		TRNG_ISR();
	} else
		PKIC_Interrupt_ACK(iTemp);
}

void TRNG_ISR() {
	int iTemp, iTemp2, iTemp3;
	iTemp = Read_TRNG_Stat();
	if ((iTemp & 0x2) == 1) {

		iTemp2 = Read_Alarm_Mask();
		iTemp3 = Read_Alarm_Stop();		//Fro will be disbaled automatically.
		if (iTemp2 == iTemp3) {
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if ((iTemp & 0x1) == 1) {
		/*
		 if(OCPMWNSignal_Y < numControlSignal ){
		 if(OCPMWNSignal_X < OCPMInputSize ){
		 //Read_TRNG_Output(randBuff);
		 Read_TRNG_Output_Imp(&OCPMAuxInputBuff[OCPMWNSignal_Y][OCPMWNSignal_X]);
		 Read_TRNG_Output_Imp(&WNSignal[OCPMWNSignal_Y][OCPMWNSignal_X-OCPMWindowSize-1]);
		 OCPMWNSignal_X +=4;
		 Acknowledge_Interrupt(iTemp);
		 }
		 else{
		 OCPMWNSignal_Y +=1;
		 OCPMWNSignal_X = OCPMWindowSize - 1;
		 Acknowledge_Interrupt(iTemp);

		 }


		 }
		 else{
		 TRNGFlag = true;
		 }
		 */
		/*
		 Read_TRNG_Output(randBuff);
		 TRNGFlag = true;

		 */

		if (OCPMWNSignal_X < WNLength) {
			//Read_TRNG_Output(randBuff);
			Read_TRNG_Output_Imp(&WNSignal[OCPMWNSignal_X]);
			OCPMWNSignal_X += 4;
			Acknowledge_Interrupt(iTemp);
		} else {
			TRNGFlag = true;

		}

	}
}

void TRNG_ACK() {
	int iTemp, iTemp2, iTemp3;
	iTemp = Read_TRNG_Stat();
	if ((iTemp & 0x2) == 1) {

		iTemp2 = Read_Alarm_Mask();
		iTemp3 = Read_Alarm_Stop();		//Fro will be disbaled automatically.
		if (iTemp2 == iTemp3) {
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if ((iTemp & 0x1) == 1) {
		/*
		 OCPMWNSignal_Y =0;
		 OCPMWNSignal_X=OCPMWindowSize - 1;
		 */
		OCPMWNSignal_X = 0;

		Acknowledge_Interrupt(iTemp);
	}

	asm("nop;");
}

float constrain(float input, float low, float high) {
	//printf("ERRORCONST");
	//while(1);

	if (input > high) {
		return high;
	} else if (input < low) {
		return low;
	} else {
		return input;
	}

}

void pinIntCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t PinIntData,
		void *pCBParam) {
	if (ePinInt == PUSH_BUTTON1_PINT) {
		//push button 1
		if (PinIntData & PUSH_BUTTON1_PINT_PIN) {
			if (bEnableOutput) {
				//LED on
				adi_gpio_Clear(LED1_PORT, LED1_PIN);
				bEnableOutput = false;
			} else {
				//LED off
				adi_gpio_Set(LED1_PORT, LED1_PIN);
				bEnableOutput = true;
			}

		}
	}
	if (ePinInt == PUSH_BUTTON2_PINT) {
		//push button 2
		if (PinIntData & PUSH_BUTTON2_PINT_PIN) {
			if (bEnableOCPM) {
				//LED on bEnableOCPM
				adi_gpio_Clear(LED2_PORT, LED2_PIN);
				bEnableOCPM = false;
			} else {
				//LED off
				adi_gpio_Set(LED2_PORT, LED2_PIN);
				bEnableOCPM = true;
			}
		}
	}

	/* reset the exit counter */
	count = 0u;

}

int32_t FIR_init() {

	//FIR stuff
	ADI_FIR_RESULT res;

	//reverseArrayf(refCoeffBuff, sizeof(refCoeffBuff));
	/*
	 #pragma vector_for
	 for (uint32_t i = 0; i < (sizeof(refCoeffBuff) / 2); i++) {
	 float temp = refCoeffBuff[i];
	 refCoeffBuff[i] = refCoeffBuff[sizeof(refCoeffBuff) - i - 1];
	 refCoeffBuff[sizeof(refCoeffBuff) - i - 1] = temp;
	 }
	 */

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OCPMLength; i++) {
			controlCoeffBuff[j][i] = 0;
			powerOCPMWNSignal[j] = 1.0;

		}
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OCPMLength; i++) {
				powerIndirectErrorSignal[j][k] = 1.0;
				OCPMWNGain[j] = 1.0;

			}
		}
	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		for (int32_t i = 0; i < OCPMLength; i++) {
			powerUncorruptedErrorSignal[k] = 1.0;
#ifdef OCPMExtendedFilter
			OCPMExtendedCoeffBuff[k][i] = 0;
#endif

		}

	}
#ifdef OCPMExtendedFilter
	for (uint8_t k = 0; k < numErrorSignal; k++) {
		for (int32_t i = 0; i < OCPMInputSize; i++) {
			OCPMExtendedError[k][i] = 0.0000001;
		}

	}
#endif
	for (uint8_t j = 0; j < numControlSignal; j++) {
		stepSizeW = 0.001;	// 0.000005;
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			stepSizeSMin = 0.001;
			stepSizeS[j][k] = stepSizeSMin;
			stepSizeE = 0.0025;	// 0.000005;
			for (int32_t i = 0; i < OCPMInputSize; i++) {
				OCPMRefOutputBuff_pp[j][k][i] = 0.000001;
			}

			for (int32_t i = 0; i < OCPMLength; i++) {
				OCPMCoeffBuff[j][k][i] = 0;
			}
		}
#ifdef OFPMFilter
		for (int32_t i = 0; i < OCPMLength; i++) {
			OFPMCoeffBuff[j][i] =0;

		}
#endif

	}
	for (int32_t i = 0; i < OFPMLength; i++) {
		//stepSizeF[i] = stepSizeFMin;//0.00001;
#ifdef OFPMFilter
		OFPMErrorCoeffBuff[i] =0;
		powerOFPMErrorSignal = 1.0;
#endif
		//powerUncorruptedRefSignal = 1.0;
	}
#ifdef OFPMFilter
	stepSizeF = stepSizeFMin;
	stepSizeH = stepSizeHMin;
#endif
	//stepSizeH = 0.0001;
#ifdef RefFilter
	channelRef.nTapLength = refLength;
	channelRef.nWindowSize = refWindowSize;
	channelRef.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelRef.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelRef.nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
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
	channelControl[0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelControl[0].pInputBuffBase = (void *) controlInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelControl[0].pInputBuffIndex = (void *) controlInputBuff; /*!< Pointer to the current index of the input circular buffer */
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
		channelControl[j].nOutputBuffCount = controlWindowSize; /*!< Number of elements in the output circular buffer */
		channelControl[j].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

#ifdef OFPMFilter
	channelOFPM[0].nTapLength = OFPMLength;
	channelOFPM[0].nWindowSize = OFPMWindowSize;
	channelOFPM[0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOFPM[0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOFPM[0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOFPM[0].pInputBuffBase = (void *) OFPMInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOFPM[0].pInputBuffIndex = (void *) OFPMInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOFPM[0].nInputBuffCount = OFPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOFPM[0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOFPM[0].pCoefficientBase = (void *) OFPMCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */
	channelOFPM[0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOFPM[0].pCoefficientIndex = (void *) OFPMCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */

	channelOFPM[0].pOutputBuffBase = (void *) OFPMOutputBuff[0]; /*!< Pointer to the base of the output circular buffer */
	channelOFPM[0].pOutputBuffIndex = (void *) OFPMOutputBuff[0]; /*!< Pointer to the current index of the output circular buffer */
	channelOFPM[0].nOutputBuffCount = OFPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOFPM[0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t j = 1; j < numControlSignal; j++) {
		channelOFPM[j] = channelOFPM[0];

		channelOFPM[j].pCoefficientBase = (void *) OFPMCoeffBuff[j]; //!< Pointer to the start of the coefficient buffer
		channelOFPM[j].nCoefficientModify = 1;//!< Modify for the Coefficient Buffer
		channelOFPM[j].pCoefficientIndex = (void *) OFPMCoeffBuff[j];//!< Pointer to the start of the coefficient buffer

		channelOFPM[j].pOutputBuffBase = (void *) OFPMOutputBuff[j];//!< Pointer to the base of the output circular buffer
		channelOFPM[j].pOutputBuffIndex = (void *) OFPMOutputBuff[j];//!< Pointer to the current index of the output circular buffer
		channelOFPM[j].nOutputBuffCount = OFPMWindowSize;//!< Number of elements in the output circular buffer
		channelOFPM[j].nOutputBuffModify = 1;//!< Modifier to be used for the output circular buffer

	}

	channelOFPMError.nTapLength = OFPMLength;
	channelOFPMError.nWindowSize = OFPMWindowSize;
	channelOFPMError.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOFPMError.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOFPMError.nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOFPMError.pInputBuffBase = (void *) OFPMErrorInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOFPMError.pInputBuffIndex = (void *) OFPMErrorInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOFPMError.nInputBuffCount = OFPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOFPMError.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOFPMError.pCoefficientBase = (void *) OFPMErrorCoeffBuff; /*!< Pointer to the start of the coefficient buffer */
	channelOFPMError.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOFPMError.pCoefficientIndex = (void *) OFPMErrorCoeffBuff; /*!< Pointer to the start of the coefficient buffer */

	channelOFPMError.pOutputBuffBase = (void *) OFPMErrorOutputBuff; /*!< Pointer to the base of the output circular buffer */
	channelOFPMError.pOutputBuffIndex = (void *) OFPMErrorOutputBuff; /*!< Pointer to the current index of the output circular buffer */
	channelOFPMError.nOutputBuffCount = OFPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOFPMError.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */
	/*

	 for (uint8_t j = 1; j < numControlSignal; j++) {
	 channelOFPMError[j] = channelOFPMError[0];

	 channelOFPMError[j].pCoefficientBase = (void *) OFPMErrorCoeffBuff[j]; //!< Pointer to the start of the coefficient buffer
	 channelOFPMError[j].nCoefficientModify = 1; //!< Modify for the Coefficient Buffer
	 channelOFPMError[j].pCoefficientIndex = (void *) OFPMErrorCoeffBuff[j]; //!< Pointer to the start of the coefficient buffer

	 channelOFPMError[j].pOutputBuffBase = (void *) OFPMErrorOutputBuff[j]; //!< Pointer to the base of the output circular buffer
	 channelOFPMError[j].pOutputBuffIndex = (void *) OFPMErrorOutputBuff[j]; //!< Pointer to the current index of the output circular buffer
	 channelOFPMError[j].nOutputBuffCount = OFPMWindowSize; //!< Number of elements in the output circular buffer
	 channelOFPMError[j].nOutputBuffModify = 1; //!< Modifier to be used for the output circular buffer

	 }

	 */

#endif

	channelOCPMRef[0][0].nTapLength = OCPMLength;
	channelOCPMRef[0][0].nWindowSize = OCPMWindowSize;
	channelOCPMRef[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOCPMRef[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOCPMRef[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOCPMRef[0][0].pInputBuffBase = (void *) refInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOCPMRef[0][0].pInputBuffIndex = (void *) refInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOCPMRef[0][0].nInputBuffCount = OCPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOCPMRef[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOCPMRef[0][0].pCoefficientBase = (void *) OCPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */
	channelOCPMRef[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOCPMRef[0][0].pCoefficientIndex = (void *) OCPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */

	channelOCPMRef[0][0].pOutputBuffBase = (void *) OCPMRefOutputBuff[0][0]; /*!< Pointer to the base of the output circular buffer */
	channelOCPMRef[0][0].pOutputBuffIndex = (void *) OCPMRefOutputBuff[0][0]; /*!< Pointer to the current index of the output circular buffer */
	channelOCPMRef[0][0].nOutputBuffCount = OCPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOCPMRef[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOCPMRef[0][k] = channelOCPMRef[0][0];

		channelOCPMRef[0][k].pInputBuffBase = (void *) refInputBuff; /*!< Pointer to the base of the input circular buffer */
		channelOCPMRef[0][k].pInputBuffIndex = (void *) refInputBuff; /*!< Pointer to the current index of the input circular buffer */
		channelOCPMRef[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOCPMRef[0][k].pCoefficientBase = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOCPMRef[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOCPMRef[0][k].pCoefficientIndex = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOCPMRef[0][k].pOutputBuffBase = (void *) OCPMRefOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOCPMRef[0][k].pOutputBuffIndex =
				(void *) OCPMRefOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOCPMRef[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMRef[j][k] = channelOCPMRef[0][0];

			channelOCPMRef[j][k].pInputBuffBase = (void *) refInputBuff; /*!< Pointer to the base of the input circular buffer */
			channelOCPMRef[j][k].pInputBuffIndex = (void *) refInputBuff; /*!< Pointer to the current index of the input circular buffer */
			channelOCPMRef[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOCPMRef[j][k].pCoefficientBase =
					(void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOCPMRef[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOCPMRef[j][k].pCoefficientIndex =
					(void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOCPMRef[j][k].pOutputBuffBase =
					(void *) OCPMRefOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOCPMRef[j][k].pOutputBuffIndex =
					(void *) OCPMRefOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOCPMRef[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}

	channelOCPMAux[0][0].nTapLength = OCPMLength;
	channelOCPMAux[0][0].nWindowSize = OCPMWindowSize;
	channelOCPMAux[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOCPMAux[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOCPMAux[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOCPMAux[0][0].pInputBuffBase = (void *) OCPMAuxInputBuff[0]; /*!< Pointer to the base of the input circular buffer */
	channelOCPMAux[0][0].pInputBuffIndex = (void *) OCPMAuxInputBuff[0]; /*!< Pointer to the current index of the input circular buffer */
	channelOCPMAux[0][0].nInputBuffCount = OCPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOCPMAux[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOCPMAux[0][0].pCoefficientBase = (void *) OCPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */
	channelOCPMAux[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOCPMAux[0][0].pCoefficientIndex = (void *) OCPMCoeffBuff[0][0]; /*!< Pointer to the start of the coefficient buffer */

	channelOCPMAux[0][0].pOutputBuffBase = (void *) OCPMAuxOutputBuff[0][0]; /*!< Pointer to the base of the output circular buffer */
	channelOCPMAux[0][0].pOutputBuffIndex = (void *) OCPMAuxOutputBuff[0][0]; /*!< Pointer to the current index of the output circular buffer */
	channelOCPMAux[0][0].nOutputBuffCount = OCPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOCPMAux[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOCPMAux[0][k] = channelOCPMAux[0][0];

		channelOCPMAux[0][k].pInputBuffBase = (void *) OCPMAuxInputBuff[0]; /*!< Pointer to the base of the input circular buffer */
		channelOCPMAux[0][k].pInputBuffIndex = (void *) OCPMAuxInputBuff[0]; /*!< Pointer to the current index of the input circular buffer */
		channelOCPMAux[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOCPMAux[0][k].pCoefficientBase = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOCPMAux[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOCPMAux[0][k].pCoefficientIndex = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOCPMAux[0][k].pOutputBuffBase = (void *) OCPMAuxOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOCPMAux[0][k].pOutputBuffIndex =
				(void *) OCPMAuxOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOCPMAux[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMAux[j][k] = channelOCPMAux[0][0];

			channelOCPMAux[j][k].pInputBuffBase = (void *) OCPMAuxInputBuff[j]; /*!< Pointer to the base of the input circular buffer */
			channelOCPMAux[j][k].pInputBuffIndex = (void *) OCPMAuxInputBuff[j]; /*!< Pointer to the current index of the input circular buffer */
			channelOCPMAux[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOCPMAux[j][k].pCoefficientBase =
					(void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOCPMAux[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOCPMAux[j][k].pCoefficientIndex =
					(void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOCPMAux[j][k].pOutputBuffBase =
					(void *) OCPMAuxOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOCPMAux[j][k].pOutputBuffIndex =
					(void *) OCPMAuxOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOCPMAux[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}
#ifdef OCPMExtendedFilter
	channelOCPMExtended[0].nTapLength = OCPMLength;
	channelOCPMExtended[0].nWindowSize = OCPMWindowSize;
	channelOCPMExtended[0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOCPMExtended[0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOCPMExtended[0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOCPMExtended[0].pInputBuffBase = (void *) refInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOCPMExtended[0].pInputBuffIndex = (void *) refInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOCPMExtended[0].nInputBuffCount = OCPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOCPMExtended[0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOCPMExtended[0].pCoefficientBase = (void *) OCPMExtendedCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */
	channelOCPMExtended[0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOCPMExtended[0].pCoefficientIndex =
			(void *) OCPMExtendedCoeffBuff[0]; /*!< Pointer to the start of the coefficient buffer */

	channelOCPMExtended[0].pOutputBuffBase = (void *) OCPMExtendedOutputBuff[0]; /*!< Pointer to the base of the output circular buffer */
	channelOCPMExtended[0].pOutputBuffIndex =
			(void *) OCPMExtendedOutputBuff[0]; /*!< Pointer to the current index of the output circular buffer */
	channelOCPMExtended[0].nOutputBuffCount = OCPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOCPMExtended[0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOCPMExtended[k] = channelOCPMExtended[0];

		channelOCPMExtended[k].pCoefficientBase =
				(void *) OCPMExtendedCoeffBuff[k]; /*!< Pointer to the start of the coefficient buffer */
		channelOCPMExtended[k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOCPMExtended[k].pCoefficientIndex =
				(void *) OCPMExtendedCoeffBuff[k]; /*!< Pointer to the start of the coefficient buffer */

		channelOCPMExtended[k].pOutputBuffBase =
				(void *) OCPMExtendedOutputBuff[k]; /*!< Pointer to the base of the output circular buffer */
		channelOCPMExtended[k].pOutputBuffIndex =
				(void *) OCPMExtendedOutputBuff[k]; /*!< Pointer to the current index of the output circular buffer */
		channelOCPMExtended[k].nOutputBuffCount = OCPMWindowSize; /*!< Number of elements in the output circular buffer */
		channelOCPMExtended[k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}
#endif
	res = adi_fir_Open(0u, &hFir);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_Open failed\n");
		return -1;
	}
	// ------------------------------- Create Configurations --------------------------------------------
#ifdef RefFilter
	res = adi_fir_CreateConfig(hFir, ConfigMemoryRef,
			ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}

	res = adi_fir_RegisterCallback(hConfigRef, RefFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
#endif
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryControl,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigControl);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigControl, ControlFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
#ifdef OFPMFilter
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOFPM,
			ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOFPM);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOFPM, OFPMFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOFPMError,
			ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOFPMError);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOFPMError, OFPMErrorFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
#endif
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOCPMRef,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOCPMRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOCPMRef, OCPMRefFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOCPMAux,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOCPMAux);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOCPMAux, OCPMAuxFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
#ifdef OCPMExtendedFilter
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOCPMExtended,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOCPMExtended);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOCPMExtended, OCPMExtendedFIRCallback,
			NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
#endif
	// ----------------------------------  Add Channels ---------------------------------------------------
#ifdef RefFilter
	res = adi_fir_AddChannel(hConfigRef, ChannelMemoryRef,
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelRef,
			&hChannelRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel Ref failed\n");
		return -1;
	}
#endif

	for (uint8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl[j],
		ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl[j], &hChannelControl[j]);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_AddChannel Control failed\n");
			return -1;
		}
	}
#ifdef OCPMExtendedFilter

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		res = adi_fir_AddChannel(hConfigOCPMExtended,
				ChannelMemoryOCPMExtended[k],
				ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOCPMExtended[k],
				&hChannelOCPMExtended[k]);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_AddChannel OCPMExtended failed\n");
			return -1;
		}
	}

#endif

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOCPMRef, ChannelMemoryOCPMRef[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOCPMRef[j][k],
					&hChannelOCPMRef[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OCPM Ref %d%d failed\n");
				return -1;
			}
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOCPMAux, ChannelMemoryOCPMAux[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOCPMAux[j][k],
					&hChannelOCPMAux[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OCPM Aux %d%d failed\n");
				return -1;
			}
		}
	}
#ifdef OFPMFilter
	for (uint8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_AddChannel(hConfigOFPM, ChannelMemoryOFPM[j],
				ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOFPM[j],
				&hChannelOFPM[j]);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_AddChannel OFPM failed\n");
			return -1;
		}
	}

	res = adi_fir_AddChannel(hConfigOFPMError, ChannelMemoryOFPMError,
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOFPMError,
			&hChannelOFPMError);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel OFPMError failed\n");
		return -1;
	}
#endif

#ifdef RefFilter
	res = adi_fir_ChannelInterruptEnable(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
#endif
	res = adi_fir_ChannelInterruptEnable(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}

	res = adi_fir_ChannelInterruptEnable(hConfigOCPMRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	res = adi_fir_ChannelInterruptEnable(hConfigOCPMAux, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}

#ifdef OCPMExtendedFilter

	res = adi_fir_ChannelInterruptEnable(hConfigOCPMExtended, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}

#endif

#ifdef OFPMFilter
	res = adi_fir_ChannelInterruptEnable(hConfigOFPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}

	res = adi_fir_ChannelInterruptEnable(hConfigOFPMError, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
#endif
	return 0;
}

int main(void) {
	uint32_t DestAddress;

	int8_t firResult = 0;
	bool bExit;
	uint32_t Result = 0u;
	//uint32_t i;
	bExit = false;
#ifdef USE_ADAU1761
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_SPORT_INFO sportRxInfo1;
	ADI_ADAU1761_SPORT_INFO sportTxInfo1;

	ADI_ADAU1761_RESULT result2;
	ADI_ADAU1761_SPORT_INFO sportRxInfo2;
	ADI_ADAU1761_SPORT_INFO sportTxInfo2;

#endif


	adi_initComponents(); /* auto-generated code */
	randSeedInt = (unsigned int) rand();
	randSeed = &randSeedInt;

	//SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;

	/*
	 adi_sec_SetPriority(INTR_SOFT5, 62); // set the priority of SOFT5 interrupt (priority 60)
	 // Register and install a handler for the software interrupt f10000SOFT5 (priority 60)
	 adi_int_InstallHandler(INTR_SOFT5, ProcessBufferADC, 0, true);

	 */

	adi_int_InstallHandler(ADI_CID_FLTOI, aluHandler, NULL, true);
	//adi_int_InstallHandler (ADI_CID_FLTUI, aluHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FLTII, aluHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FIXI, aluHandler, NULL, true) ;

	if (Result == 0u) {
		Result = DMAInit();
	}

	configGpio();

	adi_gpio_Set(LED1_PORT, LED1_PIN);
	adi_gpio_Set(LED2_PORT, LED2_PIN);

#ifdef USE_ASRC
	PcgDacInit();
	AsrcDacInit();
#endif
	// Initialize ADAU1962a
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	firResult = FIR_init();
	if (firResult == 0) {
		printf("FIR init success\n");
	}

	adi_int_InstallHandler(INTR_PKIC0_IRQ, PKIC_ISR, 0, true);
	int iTemp;

	//Disable TRNG Module
	Disable_TRNG();

	//Initialize PKIC
	Set_PKIC_Polarity(0x2F);  //setting for high rising edge
	Set_PKIC_Level_type(0x2F);  // interrupt source is edge sensitive
	Disable_PKIC_Interrupt(0x2F);
	//Read RAW STAT for edge detected interrupt sources
	iTemp = Read_PKIC_Unmasked_Interrupt_Source();
	//Disable those edge triggered interrupts
	PKIC_Interrupt_ACK(iTemp);

	//Enable TRNG interrupt source in PKIC
	Enable_TRNG_Interrupt();

	//Read Enabled STAT
	iTemp = Read_PKIC_Masked_Interrupt_Source();

	//Start TRNG initialization
	Mask_Interrupt(0x1);  // all TRNG related interrupt sources are enabled

	Startup_Cycle_Number(256);  //setting minimum sampling rate supported
	Min_Refill_Cycle(64);		//setting minimum sampling rate supported

	Max_Refill_Cycle(8388608);   //setting a sample rate of 2^23
	Sample_division(0);			//can be any value from 0 to 15

	Set_Alarm_Threshold(1); //detects repeating pattern on each FRO and generates an interrupt.
	Set_Shutdown_Threshold(1); // even if 1 FRO shuts down generate an interrupt/

	Disbale_FRO(0xFFFFFF);
	Enable_FRO(0xFF);     //8 FRO in BD

	iTemp = Read_TRNG_Stat();
	Acknowledge_Interrupt(iTemp);
	iTemp = Read_TRNG_Stat();

	Enable_TRNG();
	if (Result == 1) {
		printf("Init Error");
		return 1;
	}

#ifdef USE_ADAU1761
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
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401C, 0x21);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401D, 0x00);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401E, 0x41);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x401F, 0x00);// MUTE
	CheckResult(result1);
	//result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4019, 0x33);
	//CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4015, 0x01);
	CheckResult(result1);
	//high pass 2 hz filter
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4019, 0x33);
	//64x
	//result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4017, 0x08);
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
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401C, 0x21);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401D, 0x00);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401E, 0x41);// MUTE
	CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x401F, 0x00);// MUTE
	CheckResult(result1);
	//result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4019, 0x33); // MUTE
	//CheckResult(result1);

	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4016, 0x00);
	CheckResult(result1);
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4015, 0x01);
	CheckResult(result1);
	//high pass 2 hz filter
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4019, 0x33);
	//64x
	//result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4017, 0x08);
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
#else

	// Initialize ADAU1979
	if (Result == 0u) {
		Result = Adau1979Init();
	}
#endif

#ifdef USE_ASRC
	PcgDacEnable();
	AsrcDacEnable();
#endif
	bEvent = true;
	eMode = START;

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case RECIEVE:
				if (bEnableOCPM) {
					//OCPMAuxFIR();
				}
				//ProcessBufferADC1();
				ProcessBufferADC();
				//ProcessBufferDAC();
				break;
			case START:
				printf("Started.\n");
#ifdef USE_ADAU1761
				MixerEnable(true);
				//result1 = adi_adau1761_EnableOutput(hADAU1761_1, false);
				//CheckResult(result1);
				result1 = adi_adau1761_EnableInput(hADAU1761_1, false);
				CheckResult(result1);
				//result2 = adi_adau1761_EnableOutput(hADAU1761_2, false);
				//CheckResult(result2);
				result2 = adi_adau1761_EnableInput(hADAU1761_2, false);
				CheckResult(result2);

				//submit ping pong buffer
				result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1,
						&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL *2 * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result1);
				result1 = adi_adau1761_SubmitRxBuffer(hADAU1761_1,
						&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL *2 * 1u],
						BUFFER_SIZE_1761);
				CheckResult(result1);

				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
						&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL *2 * 0u],
						BUFFER_SIZE_1761);
				CheckResult(result2);
				result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
						&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL *2 * 1u],
						BUFFER_SIZE_1761);
				CheckResult(result2);
#else

				//1979 ping pong buffers
				if (Result == 0u) {
					Result = Adau1979SubmitBuffers();
				}

#endif
				//1962a ping pong buffers
				if (Result == 0u) {
					Result = Adau1962aSubmitBuffers();
				}

#ifdef USE_ADAU1761
				//Start recording
				result1 = adi_adau1761_EnableInput(hADAU1761_1, true);
				CheckResult(result1);

				result2 = adi_adau1761_EnableInput(hADAU1761_2, true);
				CheckResult(result2);
#else
				// Enable data flow for the ADC
				if (Result == 0u) {
					Adau1979Enable();
				}

#endif

				// Enable data flow for the DAC
				if (Result == 0u) {
					Adau1962aEnable();
				}

				break;
			default:
				break;
			}

			bEvent = false;
		}
	}

	if (!bError) {
		printf("All done\n");
	} else {
		printf("Audio error\n");
	}

	return 0;
}

#ifdef RefFilter
void RefFIR() {
	res = adi_fir_SubmitInputCircBuffer(hChannelRef,
			refInputBuff, refInputBuff,
			refInputSize, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf(
				"adi_fir_SubmitInputCircBuffer hChannelRef failed\n");
	}

	res = adi_fir_EnableChannel(hChannelRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel hChannelRef failed\n");
	}

	bRefFIRInProgress=true;
	res = adi_fir_EnableConfig(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig hConfigRef failed\n");
	}
	while(bRefFIRInProgress);

	res = adi_fir_EnableChannel(hChannelRef, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel hChannelRef failed\n");
	}

}
#endif

//void ProcessBufferADC(uint32_t iid, void* handlerArg) {
void ProcessBufferADC() {

#ifdef USE_ADAU1761
	if (pADC1 != NULL && pADC2 != NULL) {
		/*
		 if(pADC1 == &AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
		 BUFFER_SIZE_1761);
		 pADC1 = (void *)&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1];
		 }

		 if(pADC1 == &AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
		 BUFFER_SIZE_1761);
		 pADC1 = (void *)&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 2];
		 }
		 else if(pADC1 == &AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
		 BUFFER_SIZE_1761);
		 pADC1 = (void *)&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0];
		 }
		 else if(pADC1 == &AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 2])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
		 BUFFER_SIZE_1761);
		 pADC1 = (void *)&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1];
		 }


		 if(pADC2 == &AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC2,
		 BUFFER_SIZE_1761);
		 pADC2 = (void *)&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1];
		 }

		 if(pADC2 == &AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC2,
		 BUFFER_SIZE_1761);
		 pADC2 = (void *)&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 2];
		 }
		 else if(pADC2 == &AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC2,
		 BUFFER_SIZE_1761);
		 pADC2 = (void *)&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 0];
		 }
		 else if(pADC2 == &AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 2])
		 {
		 adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC2,
		 BUFFER_SIZE_1761);
		 pADC2 = (void *)&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL * 2 * 1];
		 }

		 */
		pADC1Buffer = (int32_t *) pADC1;
		pADC2Buffer = (int32_t *) pADC2;

		adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
				BUFFER_SIZE_1761);
		adi_adau1761_SubmitRxBuffer(hADAU1761_2, (void *) pADC2,
				BUFFER_SIZE_1761);
		pADC1 = NULL;
		pADC2 = NULL;

#else

	if (pGetADC != NULL) {

		pADC = (void *) pGetADC;

		pADCBuffer = (int32_t *) pADC;

#endif

		if (!ANCInProgress) {
			ANCInProgress = true;
//N#pragma vector_for
			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
#ifdef USE_ADAU1761


//			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {


				//refSignal[i] = conv_float_by(((*(pADC1Buffer + 2 * i))<<8), -25);
				refInputBuff[i+controlLength-1] = conv_float_by(((*(pADC1Buffer + 2 * i))<<8), -28);
				//pADCBuffer1[i]=pADCBuffer[i];
				//errorSignal[0][i+controlLength-1] = conv_float_by(((*(pADC1Buffer + 2 * i + 1 ))<<8), -28);
				//errorSignal_temp[0][i] = conv_float_by(((*(pADC1Buffer + 2 * i + 1 ))<<8), -28);
				for(uint8_t k = 0; k < numErrorSignal; k++) {
					//errorSignal[k][i] = conv_float_by((pADCBuffer[4 * i + 1 + k]<<8), -5);
					errorSignal[k][i+controlLength-1] = conv_float_by(((*(pADC2Buffer + 2 * i + k))<<8), -28);
					//errorSignal_temp[k][i] = conv_float_by(((*(pADC2Buffer + 2 * i + k))<<8), -28);
				}
#else
				//refInputBuff[l] = conv_float_by((pADCBuffer[4 * i]<<8), -5);
				//refInputBuff[l] = conv_float_by(((*(pADCBuffer + 4 * i))<<8), -20);
				refSignal[i] = conv_float_by(((*(pADCBuffer +
#ifdef TDM_MODE
						8
#else
						4
#endif
						* i))),
						-23);
				for (uint8_t k = 0; k < numErrorSignal; k++) {

					errorSignal[k][i] = conv_float_by(
							((*(pADCBuffer +
#ifdef TDM_MODE
						8
#else
						4
#endif
									* i + 1 + k))), -23);
				}

#endif
			}
/*
			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &refInputBuff[controlLength - 1],
					(void *) &refSignal[0], 4, controlWindowSize);

			while (bMemCopyInProgress)
				;
				*/
#ifdef RefFilter
			RefFIR();
#endif
			ControlFIR();
			OCPMRefFIR();

			if(OCPMUpdate){
				while (!TRNGFlag)
					;

				TRNGFlag = false;

				WN_Gen();

				OCPMAuxFIR();

#ifdef OCPMExtendedFilter
				OCPMExtendedFIR();
#endif
				}
			while (bControlFIRInProgress)
				;

			for (uint32_t j = 0; j < numControlSignal; j++) {
				res = adi_fir_EnableChannel(hChannelControl[j], false);
				if (res != ADI_FIR_RESULT_SUCCESS) {
					printf(
							"adi_fir_EnableChannel hChannelControl disable failed\n");
				}

			}
			while (bOCPMRefFIRInProgress)
				;
			GenControlSignal();
#ifdef OFPMFilter
			bMemCopyInProgress=true;
			MemDma0Copy1D((void *)&OFPMErrorInputBuff[OFPMLength - 1], (void *)&controlOutputBuff[0], 4, OFPMWindowSize);
#pragma vector_for
			for (int32_t i = 0; i < OFPMOutputSize; i++) {

				float OFPMSumTemp = 0;
				for (uint8_t j = 0; j < numControlSignal; j++) {
					OFPMSumTemp += OFPMOutputBuff[j][i];
				}
				uncorruptedRefSignal[i+OFPMLength-1] = (refInputBuff[i+controlLength-1] - OFPMSumTemp);
			}

			while(bMemCopyInProgress);

			OFPMErrorFIR();

			OFPMFIR();
			while (bOFPMErrorFIRInProgress);
			ANCALG_1();

			while (bOFPMFIRInProgress);

			ANCALG_2();

#endif

			for(uint32_t j = 0; j < numControlSignal; j++){
			for(uint32_t k = 0; k < numErrorSignal; k++){


			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &OCPMRefOutputBuff_pp[j][k][NUM_AUDIO_SAMPLES_PER_CHANNEL-1],
					(void *)&OCPMRefOutputBuff[j][k][0], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL);

			while (bMemCopyInProgress)
				;
			}
			}
#ifdef OCPMExtendedFilter

			while (bOCPMExtendedFIRInProgress)
				;
#endif

			if(OCPMUpdate){
			ANCALG_3();



			}

			while (bOCPMAuxFIRInProgress)
				;
				while (bMemCopyInProgress);

			ANCALG_4();
#ifdef OCPMExtendedFilter
		    OCPMExtendedWeightUpdate();
#endif
			while (bMemCopyInProgress);

			ANCALG_5();
			PushControlSignal();
			if(OCPMUpdate){
			for (int32_t j = 0; j < numControlSignal; j++) {
				bMemCopyInProgress = true;
				MemDma0Copy1D((void *) &OCPMAuxInputBuff[j][0],
						(void *) &OCPMAuxInputBuff[j][OCPMLength],
						4, OCPMLength-1);
				while (bMemCopyInProgress)
					;
			}

			}
			TRNG_ACK();
			while (bMemCopyInProgress)
				;
			for(uint32_t k = 0; k < numErrorSignal; k++){
			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &errorSignal[k][0],
					(void *) &errorSignal[k][NUM_AUDIO_SAMPLES_PER_CHANNEL], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL-1);

			while (bMemCopyInProgress)
				;
		}

			for(uint32_t k = 0; k < numErrorSignal; k++){
			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &uncorruptedErrorSignal[k][0],
					(void *) &uncorruptedErrorSignal[k][NUM_AUDIO_SAMPLES_PER_CHANNEL], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL-1);

			while (bMemCopyInProgress)
				;
		}
			if(OCPMUpdate){

#ifdef OCPMExtendedFilter
			for(uint32_t j = 0; j < numControlSignal; j++){
			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &OCPMExtendedError[j][0],
					(void *) &OCPMExtendedError[j][NUM_AUDIO_SAMPLES_PER_CHANNEL], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL-1);

			while (bMemCopyInProgress)
				;
		}
#endif
			}
			for(uint32_t k = 0; k < numErrorSignal; k++){
			for(uint32_t j = 0; j < numControlSignal; j++){

			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &OCPMRefOutputBuff_pp[j][k][0],
					(void *)&OCPMRefOutputBuff[j][k][1], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL-1);

			while (bMemCopyInProgress)
				;
			}
			}

			for(uint32_t j = 0; j < numControlSignal; j++){
			bMemCopyInProgress = true;
			MemDma0Copy1D((void *) &controlOutputSignal[j][0],
					(void *) &controlOutputSignal[j][NUM_AUDIO_SAMPLES_PER_CHANNEL], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL-1);

			while (bMemCopyInProgress)
				;
		}
			ADC2Flag =false;
			ADC1Flag = false;
		} else {
			ANCERR = true;
			fprintf(stdout, "ERROR, ANC processing time too long");
		}

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

void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg) {
	switch (nEvent) {
	case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
		/* store pointer to the processed buffer that caused the callback */
		pGetADC = pArg;

		if (ANCInProgress) {
			ANCERR = true;
			//printf("ERR ANC");
		}
		bEvent = true;
		eMode = RECIEVE;

		//adi_sec_Raise(INTR_SOFT5);
		//ProcessBufferADC();
		break;
	default:
		bEventError = true;
		break;
	}
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
void MemDmaCallback(void *pCBParam, uint32_t Event, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_DMA_EVENT) Event) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_DMA_EVENT_BUFFER_PROCESSED):
		/* Update memory copy status flag */
		bMemCopyInProgress = false;
		break;

	default:
		break;
	}
}

#ifdef RefFilter
static void RefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
		/* CASE (Processed a one-shot/circular buffer) */
		case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bRefFIRInProgress = false;
		break;

		default:
		break;
	}
}

#endif
static void ControlFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bControlFIRInProgress = false;
		break;

	default:
		break;
	}
}

static void OCPMRefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOCPMRefFIRInProgress = false;
		break;

	default:
		break;
	}
}

static void OCPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOCPMAuxFIRInProgress = false;
		break;

	default:
		break;
	}
}

#ifdef OCPMExtendedFilter
static void OCPMExtendedFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOCPMExtendedFIRInProgress = false;
		break;

	default:
		break;
	}
}
#endif
#ifdef OFPMFilter
static void OFPMFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
		/* CASE (Processed a one-shot/circular buffer) */
		case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOFPMFIRInProgress = false;
		break;

		default:
		break;
	}
}

static void OFPMErrorFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
		/* CASE (Processed a one-shot/circular buffer) */
		case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOFPMErrorFIRInProgress = false;
		break;

		default:
		break;
	}
}
#endif

int32_t OCPMRefFIR(void) {
	ADI_FIR_RESULT res;

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMRef[j][k],
			//uncorruptedRefSignal, uncorruptedRefSignal
					(void*)&refInputBuff, (void*)&refInputBuff, OCPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hChannelOCPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hChannelOCPMRef[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hChannelOCPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}
	bOCPMRefFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOCPMRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	return 0;
}

int32_t WN_Gen(void) {

	*randSeed = (unsigned int) randBuff[0];
#pragma vector_for
	for (uint32_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0, l = OCPMLength - 1;
				i < OCPMLength, l < OCPMInputSize; i++, l++) {
			//OCPMAuxInputBuff[j][l]=     (((float) rand_r_imp(randSeed) / ((float) RAND_MAX))-0.5)*2*OCPMWNGain[j];
			OCPMAuxInputBuff[j][i+OCPMLength-1] = WNSignal[i + j * NUM_AUDIO_SAMPLES_PER_CHANNEL] *OCPMWNGain[j]; //*30
		}
	}
	return 0;
}

int32_t OCPMAuxFIR(void) {
	ADI_FIR_RESULT res;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMAux[j][k],
					(void*)&OCPMAuxInputBuff[j], (void*)&OCPMAuxInputBuff[j],
					OCPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hChannelOCPMAux[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hChannelOCPMAux[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel channelOCPMAux[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}
	bOCPMAuxFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOCPMAux, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	return 0;
}
#ifdef OCPMExtendedFilter
int32_t OCPMExtendedFIR(void) {
	ADI_FIR_RESULT res;

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		res = adi_fir_SubmitInputCircBuffer(hChannelOCPMExtended[k],
				(void*)&refInputBuff, (void*)&refInputBuff,
				OCPMInputSize, 1);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf(
					"adi_fir_SubmitInputCircBuffer hChannelOCPMExtended[%d] failed\n",
					k);
			return -1;
		}

	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		res = adi_fir_EnableChannel(hChannelOCPMExtended[k], true);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel channelOCPMExtended[%d] failed\n", k);
			return -1;
		}

	}

	bOCPMExtendedFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOCPMExtended, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	return 0;
}
#endif

#ifdef OFPMFilter
int32_t OFPMFIR(void) {
	ADI_FIR_RESULT res;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_SubmitInputCircBuffer(hChannelOFPM[j],
				controlOutputSignal[j], controlOutputSignal[j],
				OFPMInputSize, 1);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf(
					"adi_fir_SubmitInputCircBuffer hChannelOFPM[%d] failed\n",
					j);
			return -1;
		}

	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_EnableChannel(hChannelOFPM[j], true);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel channelOFPM[%d] failed\n", j);
			return -1;
		}
	}

	bOFPMFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOFPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	return 0;
}

int32_t OFPMErrorFIR(void) {
	ADI_FIR_RESULT res;

	res = adi_fir_SubmitInputCircBuffer(hChannelOFPMError,
			OFPMErrorInputBuff, OFPMErrorInputBuff,
			OFPMInputSize, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf(
				"adi_fir_SubmitInputCircBuffer hChannelOFPMError failed\n");
		return -1;
	}

	res = adi_fir_EnableChannel(hChannelOFPMError, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel hChannelOFPMError failed\n");
		return -1;
	}

	bOFPMErrorFIRInProgress = true;
	res = adi_fir_EnableConfig(hConfigOFPMError, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	}

	return 0;
}

int32_t ANCALG_1(void) {

	//OFPMError Coeff

#pragma vector_for
	for (int32_t i = 0; i < OFPMOutputSize; i++) {
		OFPMErrorSignal[i]= uncorruptedRefSignal[i+controlLength-1]-OFPMErrorOutputBuff[i];
	}
#pragma vector_for
	for (int32_t i = 0, l = OFPMOutputSize - 1;
			l > (-1), i < OFPMOutputSize; i++, l--) {
		float OFPMErrorCoeffSum =0;
#pragma vector_for
		for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++) {
			OFPMErrorCoeffSum+=OFPMErrorSignal[n]*controlOutputBuff[0][i]/NUM_AUDIO_SAMPLES_PER_CHANNEL;
		}

		OFPMErrorCoeffBuff[l] =
		//dconstrain((
		OFPMErrorCoeffBuff[l]*(OFPMErrorLeak)
		+ stepSizeH * OFPMErrorCoeffSum

		//), -0.0001, 0.0001 )
		;

	}

	float powerOFPMErrorSignalSum = 0;
#pragma vector_for
	for (int32_t i = 0;i < OFPMOutputSize; i++) {
		powerOFPMErrorSignalSum += OFPMErrorSignal[i]*OFPMErrorSignal[i]/OFPMOutputSize;
	}
	powerOFPMErrorSignal = //constrain(
	forgettingFactorOFPM*powerOFPMErrorSignal + (1 - forgettingFactorOFPM)*powerOFPMErrorSignalSum
	//, -1000, 1000)
	;

	res = adi_fir_EnableChannel(hChannelOFPMError, false);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableChannel hChannelOFPMError disable failed\n");
	}

	adi_fir_SetChannelCoefficientBuffer(hChannelOFPMError,
			OFPMErrorCoeffBuff, OFPMErrorCoeffBuff, 1);

	return 0;
}

int32_t ANCALG_2(void) {

	for (uint8_t j = 0; j < numControlSignal; j++) {
		res = adi_fir_EnableChannel(hChannelOFPM[j], false);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel hChannelOFPM %d disable failed\n", j);
		}
	}

	float powerUncorruptedRefSignalSum =0;
#pragma vector_for
	for (int32_t i = 0;i < OFPMOutputSize; i++) {
		powerUncorruptedRefSignalSum += uncorruptedRefSignal[i+OFPMWindowSize-1]*uncorruptedRefSignal[i+OFPMWindowSize-1]/NUM_AUDIO_SAMPLES_PER_CHANNEL;
	}

	powerUncorruptedRefSignal = //constrain(
	forgettingFactorOFPM*powerUncorruptedRefSignal + (1 - forgettingFactorOFPM)*powerUncorruptedRefSignalSum
	//,-1000,1000)
	;
	OFPMPowerRatio = powerOFPMErrorSignal/(0.00000001+ powerUncorruptedRefSignal);
	stepSizeF = constrain((OFPMPowerRatio*stepSizeFMin + (1-OFPMPowerRatio)*stepSizeFMax), stepSizeFMin, stepSizeFMax);

	//OFPM Coeff
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (int32_t i = 0, l = OFPMOutputSize - 1;
				l > (-1), i < OFPMOutputSize; i++, l--) {

			float OFPMCoeffSum =0;
#pragma vector_for
			for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++) {
				OFPMCoeffSum+=OFPMErrorSignal[n]*OCPMAuxInputBuff[j][i];
			}

			OFPMCoeffBuff[j][l] =
			//constrain((
			OFPMCoeffBuff[j][l]*(OFPMErrorLeak)
			+ stepSizeF * OFPMCoeffSum/NUM_AUDIO_SAMPLES_PER_CHANNEL;
			//), -10000, 10000 )*1;

		}

		adi_fir_SetChannelCoefficientBuffer(hChannelOFPM[j],
				OFPMCoeffBuff[j], OFPMCoeffBuff[j], 1);
	}

	return 0;
}

#endif
int32_t ANCALG_3(void) {

#ifdef RefFilter
	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&OCPMRefInputBuff[0], (void *)&refOutputBuff[0], 4, refOutputSize);
#endif



//N#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			powerOCPMWNSignal[j] =
					forgettingFactorOCPM
							* powerOCPMWNSignal[j]+ (1.0 - forgettingFactorOCPM)*
							vecdotf((void *)&OCPMAuxInputBuff[j][OCPMLength-1], (void *)&OCPMAuxInputBuff[j][OCPMLength-1] , OCPMLength)
							;
		}

#ifdef RefFilter
	while(bMemCopyInProgress);
#endif

	return 0;
}

int32_t ANCALG_4(void) {
	ADI_FIR_RESULT res;


#pragma vector_for
	for (int32_t i = 0; i < OCPMOutputSize; i++) {
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			float OCPMAuxSumTemp = 0;
			///float OCPMRefSumTemp = 0;
//N#pragma vector_for(numControlSignal)
			if(OCPMUpdate){
			for (uint8_t j = 0; j < numControlSignal; j++) {
				OCPMAuxSumTemp += OCPMAuxOutputBuff[j][k][i];
				//OCPMRefSumTemp+= OCPMRefOutputBuff[j][k][i];

			}

			uncorruptedErrorSignal[k][i+OCPMOutputSize-1] = errorSignal[k][i+OCPMOutputSize-1] - OCPMAuxSumTemp; //-OCPMRefSumTemp;
			}
			else{
				uncorruptedErrorSignal[k][i+OCPMOutputSize-1] = errorSignal[k][i+OCPMOutputSize-1]; //-OCPMRefSumTemp;
			}

		}
	}

	return 0;
}

int32_t ANCALG_5(void) {
	ADI_FIR_RESULT res;

	bMemCopyInProgress = true;
	MemDma0Copy1D((void *) &refInputBuff[0], (void *)  &refInputBuff[controlLength -1], 4,
			controlLength);
	if(OCPMUpdate){
		//INDIRECT ERROR SIGNAL

#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
				for (uint32_t i = 0; i < OCPMOutputSize; i++) {
					indirectErrorSignal[j][k][i] = uncorruptedErrorSignal[k][i+OCPMOutputSize-1]
						//	OCPMExtendedError[k][i+OCPMOutputSize-1]
							+ OCPMAuxOutputBuff[j][k][i];

				}
			}
		}

#ifdef OFPMFilter
		while(bMemCopyInProgress);
		bMemCopyInProgress=true;
		MemDma0Copy1D((void *)&OFPMErrorInputBuff[0], (void *)&controlOutputBuff[1], 4, OFPMLength-1);
#endif
		//	power of uncorruptedErrorSignal
//N#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			powerUncorruptedErrorSignal[k] =
					forgettingFactorOCPM
							* powerUncorruptedErrorSignal[k] + (1.0 - forgettingFactorOCPM) *
							vecdotf((void *)&uncorruptedErrorSignal[k][OCPMWindowSize-1], (void *)&uncorruptedErrorSignal[k][OCPMWindowSize-1], OCPMWindowSize)
							;
		}

		//power of indirectErrorSignal

#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
//N#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerIndirectErrorSignal[j][k] =
						forgettingFactorOCPM
								* powerIndirectErrorSignal[j][k] + (1.0 - forgettingFactorOCPM) *
								vecdotf((void *)&indirectErrorSignal[j][k][0], (void *)&indirectErrorSignal[j][k][0], OCPMWindowSize)
								;
			}
		}

		//stepSizeS
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				stepSizeS[j][k] = constrain(
						(stepSizeSMin * powerOCPMWNSignal[j]
								/ (powerIndirectErrorSignal[j][k]
										+ 0.000000000001)), 0.000000000000000001,
						1);
			}
		}

#ifdef OFPMFilter

	while(bMemCopyInProgress);

	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&uncorruptedRefSignal[0], (void *)&uncorruptedRefSignal[OFPMInputSize - OFPMLength], 4, OFPMLength - 1);
#endif

		//OCPMWNGain
		float powerUncorruptedErrorSignalSum = VecSumf((void *) &powerUncorruptedErrorSignal[0],numErrorSignal);
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			OCPMWNGain[j] = constrain(
					(
							powerUncorruptedErrorSignalSum
							/ (VecSumf((void *) &powerIndirectErrorSignal[j][0], numErrorSignal) + 0.000000000001)
							), 0,100);
		}
	}
		ControlWeightUpdate();

		OCPMWeightUpdate();
	while (bMemCopyInProgress)
		;
	/*
	 for (int32_t j=0; j< numControlSignal; j++){
	 bMemCopyInProgress=true;
	 MemDma0Copy1D((void *)&controlOutputSignal[j][0], (void *)&controlOutputSignal[j][controlInputSize- OFPMLength], 4, OFPMLength - 1);
	 while(bMemCopyInProgress);
	 }
	 */
	ANCInProgress = false;

	return 0;
}

int32_t ControlWeightUpdate(void) {
	//Control Coeff
	controlFirstIt = true;
	//OCPMUpdate =true;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		float controlCoeffNSum =0;

			for (uint8_t k = 0; k < numErrorSignal; k++) {
				controlCoeffNSum +=(vecdotf((void *) &OCPMRefOutputBuff_pp[j][k][0],
						(void *) &OCPMRefOutputBuff_pp[j][k][0], OCPMInputSize));
		}

		for (int32_t i = 0, l = controlOutputSize - 1;
				i < controlOutputSize || l > (-1); i++, l--) {

			float controlCoeffSum = 0;

				for (uint8_t k = 0; k < numErrorSignal; k++) {
					for (uint32_t x =0;x < OCPMOutputSize; x++) {
						controlCoeffSum += OCPMRefOutputBuff_pp[j][k][x+controlLength -1- l]*uncorruptedErrorSignal[k][x+controlLength-1];
				}

			}
				controlCoeffBuff_temp[j][i]=		controlCoeffBuff[j][i] * (1-stepSizeW*controlLeak) + stepSizeW
						* (controlCoeffSum
								// /controlLength
								)
								/controlCoeffNSum;



		//	if (controlCoeffBuff_temp[j][i] <0.0001 && controlCoeffBuff_temp[j][i]>-0.0001){
			if(controlFirstIt){
			if (controlCoeffBuff_temp[j][i] >=-0.7 && controlCoeffBuff_temp[j][i]<=0.7){
		//controlCoeffBuff[j][i] = controlCoeffBuff_temp[j][i];
			//	OCPMUpdate = false;

				//break;

			}
			else{

				//controlW=true;
				controlFirstIt = false;
				//controlCoeffBuff[j][i] = controlCoeffBuff_temp[j][i]/2;
			}

			//	OCPMUpdate = true;
			}
			else{
				//controlCoeffBuff[j][i] = controlCoeffBuff_temp[j][i]/2;
			}

				}
			}
	if(!controlFirstIt){


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (int32_t i = 0; i < controlOutputSize ; i++) {
	controlCoeffBuff[j][i]= controlCoeffBuff_temp[j][i]*0.9;
		}
	}

	}

	else{
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (int32_t i = 0; i < controlOutputSize ; i++) {
	controlCoeffBuff[j][i]= controlCoeffBuff_temp[j][i];
		}
	}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		adi_fir_SetChannelCoefficientBuffer(hChannelControl[j],
				(void*)&controlCoeffBuff[j], (void*)&controlCoeffBuff[j], 1);

	}

	//if(!controlFirstIt && OCPMCoeffBuff[numControlSignal-1][numErrorSignal-1][NUM_AUDIO_SAMPLES_PER_CHANNEL/2]!=0){
	//OCPMUpdate = false;
	//}
	return 0;
}

int32_t OCPMWeightUpdate(void) {

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hChannelOCPMRef[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_EnableChannel disable hChannelOCPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}
			if(OCPMUpdate){
			res = adi_fir_EnableChannel(hChannelOCPMAux[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel failed hChannelOCPMAux[%d][%d]\n",
						j, k);
				return -1;
			}
		}
		}
	}
	if(OCPMUpdate){
	//OCPM Coeff

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
float OCPMNsum=(vecdotf((void *) &OCPMAuxInputBuff[j][0],	(void *) &OCPMAuxInputBuff[j][0], OCPMInputSize));
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
			for (int32_t i = 0, l = OCPMOutputSize - 1;
					i < OCPMOutputSize || l > (-1); i++, l--) {



				float OCPMCoeffSum = 0;

					for (uint32_t x =0;x < OCPMOutputSize; x++) {
						OCPMCoeffSum += OCPMAuxInputBuff[j][x+controlLength -1 - l]*
#ifdef OCPMExtendedFilter
								//uncorruptedErrorSignal[k][x+controlLength-1]
								OCPMExtendedError[k][x+controlLength-1]
#else
								uncorruptedErrorSignal[k][x+controlLength-1]
#endif

													 ;//OCPMExtendedError[k][x+controlLength-1];
				}

				OCPMCoeffBuff[j][k][i] = constrain(
						(OCPMCoeffBuff[j][k][i] * (1-OCPMLeak*stepSizeS[j][k])
								+ stepSizeS[j][k]
								//stepSizeSMin
								*
					//			(vecdotf((void *) &OCPMAuxInputBuff[j][i],
											//(void *) &OCPMExtendedError[k][i], OCPMOutputSize)
								(OCPMCoeffSum
							//	 /OCPMOutputSize
								)

								/OCPMNsum
										)




						, -10000, 10000);
			}
			adi_fir_SetChannelCoefficientBuffer(hChannelOCPMRef[j][k],
					(void*)&OCPMCoeffBuff[j][k],(void*)&OCPMCoeffBuff[j][k], 1);
			adi_fir_SetChannelCoefficientBuffer(hChannelOCPMAux[j][k],
					(void*)&OCPMCoeffBuff[j][k], (void*)&OCPMCoeffBuff[j][k], 1);
		}
	}
	}
	return 0;
}

#ifdef OCPMExtendedFilter

int32_t OCPMExtendedWeightUpdate(void) {

	if(OCPMUpdate){
	ADI_FIR_RESULT res;

//#pragma vector_for
	for (uint8_t k = 0; k < numErrorSignal; k++) {

		for (uint32_t i = 0; i < OCPMLength; i++) {
			OCPMExtendedError[k][i+OCPMLength -1] = uncorruptedErrorSignal[k][i+OCPMLength-1]- OCPMExtendedOutputBuff[k][i];
		/*
		vecvsubf((void * )&uncorruptedErrorSignal[k][0],
				(void * )&OCPMExtendedOutputBuff[k][0],
				(void * )&OCPMExtendedError[k][0], OCPMLength);*/
		}
	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		res = adi_fir_EnableChannel(hChannelOCPMExtended[k], false);
		if (res != ADI_FIR_RESULT_SUCCESS) {
			printf("adi_fir_EnableChannel channelOCPMExtended[%d] failed\n", k);
			return -1;
		}

	}

	float OCPMExtendedNSum =  (vecdotf((void *) &refInputBuff[0],
			(void *) &refInputBuff[0], OCPMInputSize ));

	for (uint8_t k = 0; k < numErrorSignal; k++) {

		//float OCPMExtendedErrorSum = VecSumf((void *) &OCPMExtendedError[k], NUM_AUDIO_SAMPLES_PER_CHANNEL) / NUM_AUDIO_SAMPLES_PER_CHANNEL;
		for (int32_t i = 0, l = OCPMOutputSize - 1;
				i < OCPMOutputSize || l > (-1); i++, l--) {



			float OCPMExtendedCoeffSum = 0;




				for (uint32_t x =0;x < OCPMOutputSize; x++) {
					OCPMExtendedCoeffSum += refInputBuff[x+controlLength -1- l]*OCPMExtendedError[k][x+controlLength-1];
			}


			OCPMExtendedCoeffBuff[k][i] = constrain(
					(OCPMExtendedCoeffBuff[k][i] * (1-stepSizeE*OCPMExtendedLeak)
							+ stepSizeE * //OCPMExtendedError[k][i]*refSignal[i]
							/*(vecdotf((void *) &refInputBuff[i],
									(void *) &OCPMExtendedError[k][i], OCPMOutputSize)/OCPMOutputSize)*/

							(OCPMExtendedCoeffSum
								//	 /OCPMLength
									)
									/OCPMExtendedNSum
									), -10000, 10000);
		}
	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		adi_fir_SetChannelCoefficientBuffer(hChannelOCPMExtended[k],
				(void*)&OCPMExtendedCoeffBuff[k], (void*)&OCPMExtendedCoeffBuff[k], 1);

	}
	}
	return 0;
}

#endif
