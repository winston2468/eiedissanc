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
static volatile bool bOCPMRefFIRInProgress = false;
static volatile bool bOCPMAuxFIRInProgress = false;
#ifdef RefFilter
static volatile bool bRefFIRInProgress = false;
#endif
static volatile bool bControlFIRInProgress = false;
static volatile bool bOFPMFIRInProgress = false;
static volatile bool bOFPMErrorFIRInProgress = false;
//*****************************************************************************
uint8_t DacStarted = 0u;
/* ADC/DAC buffer pointer */
volatile void *pGetDAC = NULL;
volatile void *pDAC = NULL;


#pragma alignment_region (4)
float controlOutputBuff[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };

#pragma alignment_region_end


extern uint32_t PcgDacInit(void);
extern uint32_t AsrcDacInit(void);
extern uint32_t PcgDacEnable(void);
extern uint32_t AsrcDacEnable(void);

/* Initializes DAC */
extern uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
extern uint32_t Adau1962aSubmitBuffers(void);
extern uint32_t Adau1962aEnable(void);
extern uint32_t Adau1962aDoneWithBuffer(volatile void *pBuffer);
uint8_t ControlFIR(void);
uint8_t GenControlSignal(void);
uint8_t PushControlSignal(void);




int32_t randBuff[4] = {0};



int CMSE = 0;

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
#pragma align(4)
int32_t AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL* 2 * 2];
volatile void *pGetADC1 = NULL;
void *pADC1;
int32_t *pADC1Buffer;

volatile bool ADC1Flag =false;

volatile int32_t DacCount =0;

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















#pragma alignment_region (4)
int32_t * pADCBuffer;
float controlCoeffBuff[numControlSignal][controlLength] = { 0 };

/* ----------------------------   FIR Configuration ------------------------------------------- */

float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };


/* Channel Configurations parameters */
//Max FIR channels = 32
float refSignal[NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
float refInputBuff[refInputSize] = { 0 };

uint8_t ConfigMemoryRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryRef[ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryOCPMRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMRef[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
uint8_t ConfigMemoryOCPMAux[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMAux[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryControl[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryControl[numControlSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];




float OCPMRefInputBuff[OCPMInputSize] = { 0 };


float OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength] = { 0 };


float controlOutputSignal[numControlSignal][OFPMInputSize] = { 0 };

float OCPMAuxInputBuff[numControlSignal][OCPMInputSize] = { 0 };

float WNSignal[OCPMLength] = { 0 };
float controlInputBuff[controlInputSize] = { 0 };


float uncorruptedErrorSignal[numErrorSignal][OCPMLength] = { 0 };
float uncorruptedRefSignal[OFPMInputSize] = { 0 };

float OCPMWNGain[numControlSignal] = { 0 };
float powerOCPMWNSignal[numControlSignal] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal][OCPMLength];
float powerIndirectErrorSignal[numControlSignal][numErrorSignal] = {
		0 };
float powerUncorruptedErrorSignal[numErrorSignal] = { 0 };
float powerUncorruptedRefSignal = 0;



float forgettingFactorOCPM = 0.6;

float stepSizeW[numControlSignal] = { 0 };
float stepSizeSMin =  0.0005;// 0.0005;//0.00001;
float stepSizeS[numControlSignal][numErrorSignal] = { 0 };
#ifdef OFPMFilter
uint8_t ConfigMemoryOFPM[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOFPM[numControlSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

uint8_t ConfigMemoryOFPMError[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOFPMError[ADI_FIR_CHANNEL_MEMORY_SIZE];
float OFPMErrorSignal[OCPMLength] = { 0 };
float OFPMInputBuff[numControlSignal][OFPMInputSize] = { 0 };
float OFPMCoeffBuff[numControlSignal][OFPMLength] = { 0 };

float OFPMErrorInputBuff[OFPMInputSize] = { 0 };
float OFPMErrorCoeffBuff[OFPMLength] = { 0 };

float powerOFPMErrorSignal = 0;
float OFPMPowerRatio = 0;
float forgettingFactorOFPM = 0.9;
float stepSizeH = 0;
float stepSizeHMin =  0.00000001;//0.00001;
float stepSizeHMax =  0.00000000001;//0.0001;
float stepSizeFMin = 0.000001;//0.00001;
float stepSizeFMax = 0.0001;//0.0001;
float stepSizeF = 0;
float OFPMLeak = 0.999999;
float OFPMErrorLeak = 0.999999;
float OFPMOutputBuff[numControlSignal][OFPMOutputSize] = {0};
float OFPMErrorOutputBuff[OFPMOutputSize]= {0};
#endif
float controlLeak = 0.999;
float OCPMLeak = 0.999;

float OCPMRefOutputBuff[numControlSignal][numErrorSignal][OCPMOutputSize];

float OCPMAuxOutputBuff[numControlSignal][numErrorSignal][OCPMOutputSize];


#ifdef RefFilter
float refCoeffBuff[refLength] = {
#include "data/lowpass_filter_2000_2500.dat"
		};
float refOutputBuff[refOutputSize] = { 0 };
#endif

#pragma alignment_region_end


ADI_FIR_CHANNEL_PARAMS channelRef;
ADI_FIR_CHANNEL_PARAMS channelOCPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_PARAMS channelOCPMAux[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_PARAMS channelControl[numControlSignal];
#ifdef OFPMFilter
ADI_FIR_CHANNEL_PARAMS channelOFPM[numControlSignal];
ADI_FIR_CHANNEL_PARAMS channelOFPMError;
#endif

ADI_FIR_RESULT res;
ADI_FIR_HANDLE hFir;
ADI_FIR_CONFIG_HANDLE hConfigOCPMRef;
ADI_FIR_CONFIG_HANDLE hConfigOCPMAux;
ADI_FIR_CONFIG_HANDLE hConfigOFPM;
ADI_FIR_CONFIG_HANDLE hConfigOFPMError;
ADI_FIR_CONFIG_HANDLE hConfigControl;
ADI_FIR_CHANNEL_HANDLE hChannelOCPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelOCPMAux[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelControl[numControlSignal];
#ifdef OFPMFilter
ADI_FIR_CHANNEL_HANDLE hChannelOFPM[numControlSignal];
ADI_FIR_CHANNEL_HANDLE hChannelOFPMError;
#endif
#ifdef RefFilter
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
volatile uint32_t OCPMWNSignal_X =0;
//volatile uint32_t OCPMWNSignal_X =OCPMWindowSize-1;
//volatile uint32_t OCPMWNSignal_Y =0;

/* ADC buffer pointer */
volatile void *pGetADC = NULL;
volatile void *pADC = NULL;
volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;


extern int32_t MemDma0Copy1D(
	    void                            *pMemDest,
	    void                            *pMemSrc,
		uint8_t							nBytesInElement,
	    uint32_t                        ElementCount);


extern int32_t MemDma0Copy2D( void *pDestBuffer, void *pSrcBuffer, uint8_t nBytesInElement, uint32_t Xcount, uint32_t YCount );

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
int32_t OCPMRefFIR(void);
int32_t OCPMAuxFIR(void);
int32_t OFPMFIR(void);
int32_t OFPMErrorFIR(void);
int32_t ANCALG_1(void);
int32_t ANCALG_2(void);
int32_t ANCALG_3(void);
int32_t ANCALG_4(void);
int32_t ANCALG_5(void);
int8_t DisableAllOCPMChannels(void);
int32_t WN_Gen(void);

extern uint32_t DMAInit(void);
#ifdef RefFilter
static void RefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
#endif
static void OCPMRefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OCPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void ControlFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
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
extern void	Acknowledge_Interrupt(int iValue);
extern void Disable_TRNG(void);
extern void	Enable_TRNG(void);
extern void	Set_PKIC_Polarity(int iValue);
extern void	Set_PKIC_Level_type(int iValue);
extern void	Disable_PKIC_Interrupt(int iValue);

extern int Read_PKIC_Unmasked_Interrupt_Source(void);

extern void Read_TRNG_Output(int *iOutput);



void Read_TRNG_Output_Imp(float *iOutput);


static void PKIC_ISR(uint32_t iid, void* handlerArg);

static void TRNG_ISR(void);
static void TRNG_ACK(void);
unsigned int *randSeed;
unsigned int randSeedInt = 0;
int rand_r_imp(unsigned int *seed);
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
	if(ANCInProgress){
		printf("ANC ERR");
	}
		eMode = RECIEVE;
		ADC1Flag= true;
		bEvent = true;
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

		eMode = RECIEVE;
		bEvent = true;
		ADC2Flag= true;
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








uint8_t ControlFIR() {
#ifdef RefFilter
	bMemCopyInProgress=true;
	MemDma0Copy1D((void*) &OCPMRefInputBuff[NUM_AUDIO_SAMPLES_PER_CHANNEL - 1], (void *)&refOutputBuff[0], 4, NUM_AUDIO_SAMPLES_PER_CHANNEL);
	while(bMemCopyInProgress);



#endif






for(uint32_t j=0; j< numControlSignal; j++){

	res = adi_fir_SubmitInputCircBuffer(hChannelControl[j],
#ifdef RefFilter
			OCPMRefInputBuff, OCPMRefInputBuff,
#else
#ifdef OFPMFilter
			uncorruptedRefSignal, uncorruptedRefSignal,
#else
			refInputBuff, refInputBuff,
#endif
#endif
			controlInputSize, 1);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf(
				"adi_fir_SubmitInputCircBuffer hChannelControl %d failed\n", j);
	}

}

for(uint32_t j=0; j< numControlSignal; j++){
res = adi_fir_EnableChannel(hChannelControl[j], true);
if (res != ADI_FIR_RESULT_SUCCESS) {
	printf("adi_fir_EnableChannel hChannelControl failed\n");
}
}

	bControlFIRInProgress=true;
	res = adi_fir_EnableConfig(hConfigControl, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig hConfigControl failed\n");
	}




	return 0;
}

uint8_t GenControlSignal() {
#pragma SIMD_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
			controlOutputSignal[j][i+NUM_AUDIO_SAMPLES_PER_CHANNEL-1] = controlOutputBuff[j][i] + OCPMAuxInputBuff[j][i+ OCPMWindowSize -1];
		}
	}

	return 0;
}



uint8_t PushControlSignal() {
	if (pGetDAC != NULL) {
		pDAC = (void *) pGetDAC;

		pGetDAC = NULL;
		int32_t *pDst;
		pDst = (int32_t *) pDAC;
		for (uint32_t i = NUM_AUDIO_SAMPLES_PER_CHANNEL-1; i < OFPMInputSize; i++) {
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

			*pDst++ = (conv_fix_by(controlOutputSignal[0][i], 18));
			*pDst++ = (conv_fix_by(controlOutputSignal[1][i], 18));
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
		Adau1962aDoneWithBuffer(pGetDAC);
		//DacCount++;


		break;
	default:
		printf("err");
		break;
	}
}










//convert to float [-10,10]
void Read_TRNG_Output_Imp(float *iOutput)
{
	//int range [-2147483647,2147483647]
	float *iTemp;
	iTemp= iOutput;
	*iTemp=(((float)(*pREG_TRNG0_OUTPUT0))/(2147483647))-1;
	iTemp++;
	*iTemp=(((float)(*pREG_TRNG0_OUTPUT1))/(2147483647))-1;
	iTemp++;
	*iTemp=(((float)(*pREG_TRNG0_OUTPUT2))/(2147483647))-1;
	iTemp++;
	*iTemp=(((float)(*pREG_TRNG0_OUTPUT3))/(2147483647))-1;
}

void PKIC_ISR(uint32_t iid, void* handlerArg)
{
	int iTemp,iTemp2;
	iTemp=Read_PKIC_Masked_Interrupt_Source();
	iTemp2=iTemp && 0x8;
	if((iTemp && 0x8) ==1)
	{
		PKIC_Interrupt_ACK(0x8);
		TRNG_ISR();
	}
	else
		PKIC_Interrupt_ACK(iTemp);
}

void TRNG_ISR()
{
	int iTemp,iTemp2,iTemp3;
	iTemp= Read_TRNG_Stat();
	if((iTemp & 0x2)==1)
	{

		iTemp2= Read_Alarm_Mask();
		iTemp3=Read_Alarm_Stop();			//Fro will be disbaled automatically.
		if(iTemp2==iTemp3)
		{
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if((iTemp & 0x1)==1)
	{
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
		Read_TRNG_Output(randBuff);
		TRNGFlag = true;


		/*
			if(OCPMWNSignal_X < OCPMWindowSize ){
				//Read_TRNG_Output(randBuff);
				Read_TRNG_Output_Imp(&WNSignal[OCPMWNSignal_X]);
				OCPMWNSignal_X +=4;
				Acknowledge_Interrupt(iTemp);
			}
			else{
				TRNGFlag = true;

			}
			*/
	}
}

void TRNG_ACK()
{
	int iTemp,iTemp2,iTemp3;
	iTemp= Read_TRNG_Stat();
	if((iTemp & 0x2)==1)
	{

		iTemp2= Read_Alarm_Mask();
		iTemp3=Read_Alarm_Stop();			//Fro will be disbaled automatically.
		if(iTemp2==iTemp3)
		{
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if((iTemp & 0x1)==1)
	{
		/*
		OCPMWNSignal_Y =0;
		OCPMWNSignal_X=OCPMWindowSize - 1;
		*/
		//OCPMWNSignal_X=0;
		Acknowledge_Interrupt(iTemp);
	}




	asm("nop;");
}

float constrain(float input, float low, float high) {
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
			if(bEnableOutput){
			//LED on
			adi_gpio_Clear(LED1_PORT, LED1_PIN);
			bEnableOutput = false;
			}
			else{
			//LED off
			adi_gpio_Set(LED1_PORT, LED1_PIN);
			bEnableOutput = true;
			}

		}
	}
	if (ePinInt == PUSH_BUTTON2_PINT) {
		//push button 2
		if (PinIntData & PUSH_BUTTON2_PINT_PIN) {
			if(bEnableOCPM){
			//LED on bEnableOCPM
			adi_gpio_Clear(LED2_PORT, LED2_PIN);
			bEnableOCPM = false;
			}
			else{
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
#pragma SIMD_for
	for (uint32_t i = 0; i < (sizeof(refCoeffBuff) / 2); i++) {
		float temp = refCoeffBuff[i];
		refCoeffBuff[i] = refCoeffBuff[sizeof(refCoeffBuff) - i - 1];
		refCoeffBuff[sizeof(refCoeffBuff) - i - 1] = temp;
	}
*/

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OCPMLength; i++) {
			controlCoeffBuff[j][i] = 0;
			powerOCPMWNSignal[j] = 1;

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
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		stepSizeW[j] = 0.00001 ;// 0.000005;
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			stepSizeSMin =  0.0001;
			stepSizeS[j][k] = stepSizeSMin;
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
		powerUncorruptedRefSignal = 1.0;
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
		channelOFPM[j].nCoefficientModify = 1; //!< Modify for the Coefficient Buffer
		channelOFPM[j].pCoefficientIndex = (void *) OFPMCoeffBuff[j]; //!< Pointer to the start of the coefficient buffer

		channelOFPM[j].pOutputBuffBase = (void *) OFPMOutputBuff[j]; //!< Pointer to the base of the output circular buffer
		channelOFPM[j].pOutputBuffIndex = (void *) OFPMOutputBuff[j]; //!< Pointer to the current index of the output circular buffer
		channelOFPM[j].nOutputBuffCount = OFPMWindowSize; //!< Number of elements in the output circular buffer
		channelOFPM[j].nOutputBuffModify = 1; //!< Modifier to be used for the output circular buffer

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

	channelOCPMRef[0][0].pInputBuffBase = (void *) OCPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOCPMRef[0][0].pInputBuffIndex = (void *) OCPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
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

		channelOCPMRef[0][k].pInputBuffBase = (void *) OCPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
		channelOCPMRef[0][k].pInputBuffIndex = (void *) OCPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
		channelOCPMRef[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOCPMRef[0][k].pCoefficientBase = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOCPMRef[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOCPMRef[0][k].pCoefficientIndex = (void *) OCPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOCPMRef[0][k].pOutputBuffBase = (void *) OCPMRefOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOCPMRef[0][k].pOutputBuffIndex = (void *) OCPMRefOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOCPMRef[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMRef[j][k] = channelOCPMRef[0][0];

			channelOCPMRef[j][k].pInputBuffBase = (void *) OCPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
			channelOCPMRef[j][k].pInputBuffIndex = (void *) OCPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
			channelOCPMRef[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOCPMRef[j][k].pCoefficientBase = (void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOCPMRef[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOCPMRef[j][k].pCoefficientIndex = (void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOCPMRef[j][k].pOutputBuffBase = (void *) OCPMRefOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOCPMRef[j][k].pOutputBuffIndex = (void *) OCPMRefOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
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
		channelOCPMAux[0][k].pOutputBuffIndex = (void *) OCPMAuxOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOCPMAux[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMAux[j][k] = channelOCPMAux[0][0];

			channelOCPMAux[j][k].pInputBuffBase = (void *) OCPMAuxInputBuff[j]; /*!< Pointer to the base of the input circular buffer */
			channelOCPMAux[j][k].pInputBuffIndex = (void *) OCPMAuxInputBuff[j]; /*!< Pointer to the current index of the input circular buffer */
			channelOCPMAux[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOCPMAux[j][k].pCoefficientBase = (void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOCPMAux[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOCPMAux[j][k].pCoefficientIndex = (void *) OCPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOCPMAux[j][k].pOutputBuffBase = (void *) OCPMAuxOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOCPMAux[j][k].pOutputBuffIndex = (void *) OCPMAuxOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOCPMAux[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}



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

	for (int8_t j = 0; j < numControlSignal; j++) {
			res = adi_fir_AddChannel(hConfigControl, ChannelMemoryControl[j],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelControl[j],
					&hChannelControl[j]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel Control failed\n");
				return -1;
			}
	}
#ifdef OFPMFilter
	for (int8_t j = 0; j < numControlSignal; j++) {
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
	for (int8_t j = 0; j < numControlSignal; j++) {
		for (int8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOCPMRef, ChannelMemoryOCPMRef[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOCPMRef[j][k],
					&hChannelOCPMRef[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OCPM Ref %d%d failed\n");
				return -1;
			}
		}
	}

	for (int8_t j = 0; j < numControlSignal; j++) {
		for (int8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOCPMAux, ChannelMemoryOCPMAux[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOCPMAux[j][k],
					&hChannelOCPMAux[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OCPM Aux %d%d failed\n");
				return -1;
			}
		}
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
	ADI_ADAU1761_RESULT result1;
	ADI_ADAU1761_SPORT_INFO sportRxInfo1;
	ADI_ADAU1761_SPORT_INFO sportTxInfo1;

	ADI_ADAU1761_RESULT result2;
	ADI_ADAU1761_SPORT_INFO sportRxInfo2;
	ADI_ADAU1761_SPORT_INFO sportTxInfo2;




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





	if (Result == 0u) {
		Result = DMAInit();
	}

	configGpio();



	adi_gpio_Set(LED1_PORT, LED1_PIN);
	adi_gpio_Set(LED2_PORT, LED2_PIN);

	// Initialize ADAU1979
	if (Result == 0u) {
		Result = Adau1979Init();
	}
	PcgDacInit();
	AsrcDacInit();
	// Initialize ADAU1962a
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	firResult = FIR_init();
	if (firResult == 0) {
		printf("FIR init success\n");
	}


 	adi_int_InstallHandler(INTR_PKIC0_IRQ,PKIC_ISR,0,true);
 	int iTemp;

 	//Disable TRNG Module
 	Disable_TRNG();

 	//Initialize PKIC
 	Set_PKIC_Polarity(0x2F);  //setting for high rising edge
 	Set_PKIC_Level_type(0x2F);// interrupt source is edge sensitive
 	Disable_PKIC_Interrupt(0x2F);
 	//Read RAW STAT for edge detected interrupt sources
 	iTemp= Read_PKIC_Unmasked_Interrupt_Source();
 	//Disable those edge triggered interrupts
 	PKIC_Interrupt_ACK(iTemp);

 	//Enable TRNG interrupt source in PKIC
 	Enable_TRNG_Interrupt();

 	//Read Enabled STAT
 	iTemp=Read_PKIC_Masked_Interrupt_Source();

 	//Start TRNG initialization
 	Mask_Interrupt(0x1);  // all TRNG related interrupt sources are enabled

 	Startup_Cycle_Number(256);  //setting minimum sampling rate supported
 	Min_Refill_Cycle(64);		//setting minimum sampling rate supported

 	Max_Refill_Cycle(8388608);   //setting a sample rate of 2^23
 	Sample_division(0);			//can be any value from 0 to 15

 	Set_Alarm_Threshold(1);   //detects repeating pattern on each FRO and generates an interrupt.
 	Set_Shutdown_Threshold(1); // even if 1 FRO shuts down generate an interrupt/

 	Disbale_FRO(0xFFFFFF);
 	Enable_FRO(0xFF);     //8 FRO in BD

 	iTemp= Read_TRNG_Stat();
 	Acknowledge_Interrupt(iTemp);
 	iTemp= Read_TRNG_Stat();

 	Enable_TRNG();
	if(Result ==1){
		printf("Init Error");
		return 1;
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
	//result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4019, 0x33);
	//64x
	result1 = adi_adau1761_SetRegister(hADAU1761_1, 0x4017, 0x08);
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
	//result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4019, 0x33);
	//64x
	result1 = adi_adau1761_SetRegister(hADAU1761_2, 0x4017, 0x08);
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

	PcgDacEnable();
	AsrcDacEnable();

	bEvent = true;
	eMode = START;

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case RECIEVE:
				if(bEnableOCPM){
				//OCPMAuxFIR();
				}
				//ProcessBufferADC1();
				 ProcessBufferADC();
				//ProcessBufferDAC();
				break;
			case START:
				printf("Started.\n");

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
									&AdcBuf1[NUM_AUDIO_SAMPLES_PER_CHANNEL *2  * 1u],
									BUFFER_SIZE_1761);
							CheckResult(result1);


							result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
									&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL *2  * 0u],
									BUFFER_SIZE_1761);
							CheckResult(result2);
							result2 = adi_adau1761_SubmitRxBuffer(hADAU1761_2,
									&AdcBuf2[NUM_AUDIO_SAMPLES_PER_CHANNEL *2  * 1u],
									BUFFER_SIZE_1761);
							CheckResult(result2);

							//1962a ping pong buffers
							if (Result == 0u) {
								Result = Adau1962aSubmitBuffers();
							}




							/*
				//1979 ping pong buffers
				if (Result == 0u) {
					Result = Adau1979SubmitBuffers();
				}
*/


				//Start recording
				result1 = adi_adau1761_EnableInput(hADAU1761_1, true);
				CheckResult(result1);

				result2 = adi_adau1761_EnableInput(hADAU1761_2, true);
				CheckResult(result2);

				// Enable data flow for the ADC
				if (Result == 0u) {
					Adau1962aEnable();
				}
/*
				// Enable data flow for the ADC
				if (Result == 0u) {
					Adau1979Enable();
				}
*/

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
void RefFIR(){
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
	/*
	if (pGetADC != NULL) {

		pADC = (void *)pGetADC;
		Adau1979DoneWithBuffer(pGetADC);
				pGetADC = NULL;
		pADCBuffer = (int32_t *) pADC;
			*/
	if (pADC1 != NULL && pADC2 != NULL) {
		pADC1Buffer = (int32_t *) pADC1;
		pADC2Buffer = (int32_t *) pADC2;
		adi_adau1761_SubmitRxBuffer(hADAU1761_1, (void *) pADC1,
				BUFFER_SIZE_1761);
		adi_adau1761_SubmitRxBuffer(hADAU1761_2, (void *) pADC2,
				BUFFER_SIZE_1761);
		pADC1 = NULL;
		pADC2 = NULL;

		if (!ANCInProgress) {
			ANCInProgress=true;
			/*
			for (uint32_t i = 0, l = NUM_AUDIO_SAMPLES_PER_CHANNEL-1;
					i < NUM_AUDIO_SAMPLES_PER_CHANNEL, l < refInputSize; i++, l++) {
				//refInputBuff[l] = conv_float_by((pADCBuffer[4 * i]<<8), -5);
				//refInputBuff[l] = conv_float_by(((*(pADCBuffer + 4 * i))<<8), -20);
				refSignal[i] =  conv_float_by(((*(pADCBuffer + 4 * i))<<8), -20);
				//pADCBuffer1[i]=pADCBuffer[i];
				for(uint8_t k = 0; k < numErrorSignal; k++){
					//errorSignal[k][i] = conv_float_by((pADCBuffer[4 * i + 1 + k]<<8), -5);
					errorSignal[k][i] = conv_float_by(((*(pADCBuffer + 4 * i + 1 + k))<<8), -20);
				}
			}
*/

			for (uint32_t i = 0, l = NUM_AUDIO_SAMPLES_PER_CHANNEL-1; i < NUM_AUDIO_SAMPLES_PER_CHANNEL || l < refInputSize; i++, l++)
			{
				refSignal[i] =  conv_float_by(((*(pADC1Buffer + 2 * i))<<8), -23);

				//pADCBuffer1[i]=pADCBuffer[i];
				errorSignal[0][i] = conv_float_by(((*(pADC1Buffer + 2 * i + 1 ))<<8), -23);
				for(uint8_t k = 1; k < numErrorSignal; k++){
					//errorSignal[k][i] = conv_float_by((pADCBuffer[4 * i + 1 + k]<<8), -5);
					errorSignal[k][i] = conv_float_by(((*(pADC2Buffer + 2 * i + k-1))<<8), -23);
				}
			}

			bMemCopyInProgress=true;
			MemDma0Copy1D((void *)&refInputBuff[controlLength - 1], (void *)&refSignal[0], 4, controlWindowSize);

			while(bMemCopyInProgress);
#ifdef RefFilter
			RefFIR();
#endif
			ControlFIR();
			OCPMRefFIR();

			while(!TRNGFlag);

			TRNGFlag = false;
			WN_Gen();



			OCPMAuxFIR();
			while(bControlFIRInProgress);
			for(uint32_t j=0; j< numControlSignal; j++){
			res = adi_fir_EnableChannel(hChannelControl[j], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hChannelControl disable failed\n");
			}

			}

			GenControlSignal();
#ifdef OFPMFilter
			bMemCopyInProgress=true;
			MemDma0Copy1D((void *)&OFPMErrorInputBuff[OFPMLength - 1], (void *)&controlOutputBuff[0], 4, OFPMWindowSize);
#pragma SIMD_for
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
			while (bOCPMRefFIRInProgress);
			ANCALG_3();
			TRNG_ACK();

			while (bOCPMAuxFIRInProgress);




			ANCALG_4();



			ANCALG_5();
			PushControlSignal();

		}
		else{
			ANCERR = true;
			fprintf(stdout, "ERROR, ANC processing time too long");
		}

	}
}





void reverseArrayf(float arr[], uint32_t arrLength) {
#pragma SIMD_for
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

		if(ANCInProgress){
			ANCERR= true;
			printf("ERR ANC");
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

int8_t DisableAllOCPMChannels() {
	ADI_FIR_RESULT res;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hChannelOCPMRef[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_EnableChannel disable hChannelOCPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}
			res = adi_fir_EnableChannel(hChannelOCPMAux[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel failed hChannelOCPMAux[%d][%d]\n", j, k);
				return -1;
			}
		}
	}

	return 0;
}


int32_t OCPMRefFIR(void) {
	ADI_FIR_RESULT res;


#pragma SIMD_for
	for(uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for
		for(uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMRef[j][k],
					//uncorruptedRefSignal, uncorruptedRefSignal
					refInputBuff, refInputBuff
					, OCPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_SubmitInputCircBuffer hChannelOCPMRef[%d][%d] failed\n", j ,k);
				return -1;
			}
		}
	}



	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hChannelOCPMRef[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hChannelOCPMRef[%d][%d] failed\n", j,
						k);
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
#pragma SIMD_for
	for(uint32_t j =0; j < numControlSignal; j++){
	for(uint32_t i = 0, l = OCPMLength-1; i < OCPMLength, l < OCPMInputSize;i++,l++){
		OCPMAuxInputBuff[j][l]=     (((float) rand_r_imp(randSeed) / ((float) RAND_MAX))-0.5)*2*OCPMWNGain[j];
		//OCPMAuxInputBuff[j][l]=    WNSignal[i]*OCPMWNGain[j];
	}
	}
return 0;
}


int32_t OCPMAuxFIR(void) {
	ADI_FIR_RESULT res;


	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMAux[j][k],
					OCPMAuxInputBuff[j], OCPMAuxInputBuff[j],
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
				printf("adi_fir_EnableChannel channelOCPMAux[%d][%d] failed\n", j,
						k);
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

#pragma SIMD_for
	for (int32_t i = 0; i < OFPMOutputSize; i++) {
	OFPMErrorSignal[i]= uncorruptedRefSignal[i+controlLength-1]-OFPMErrorOutputBuff[i];
	}
#pragma SIMD_for
		for (int32_t i = 0, l = OFPMOutputSize - 1;
				l > (-1), i < OFPMOutputSize; i++, l--) {
			float OFPMErrorCoeffSum =0;
#pragma SIMD_for
			for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++){
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
#pragma SIMD_for
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
#pragma SIMD_for
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
#pragma SIMD_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for
		for (int32_t i = 0, l = OFPMOutputSize - 1;
				l > (-1), i < OFPMOutputSize; i++, l--) {

			float OFPMCoeffSum =0;
#pragma SIMD_for
			for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++){
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




#pragma SIMD_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
	float powerOCPMWNSignalSum =0;
#pragma SIMD_for
	for (int32_t i = OFPMWindowSize-1;i < OCPMInputSize; i++) {
		powerOCPMWNSignalSum += OCPMAuxInputBuff[j][i]* OCPMAuxInputBuff[j][i]/NUM_AUDIO_SAMPLES_PER_CHANNEL;
	}

	powerOCPMWNSignal[j] =
			forgettingFactorOCPM*powerOCPMWNSignal[j] + (1.0 - forgettingFactorOCPM)*powerOCPMWNSignalSum;

	}

#ifdef RefFilter
	while(bMemCopyInProgress);
#endif



	return 0;
}

int32_t ANCALG_4(void) {
	ADI_FIR_RESULT res;



	for (int32_t j=0; j< numControlSignal; j++){
		bMemCopyInProgress=true;
	MemDma0Copy1D( (void *)&OCPMAuxInputBuff[j][0], (void *)&OCPMAuxInputBuff[j][OCPMInputSize-1-OCPMLength], 4, OCPMLength);
	while(bMemCopyInProgress);
	}


#pragma SIMD_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma SIMD_for
		for (int32_t i = 0; i < OCPMOutputSize; i++) {

			float OCPMAuxSumTemp = 0;
			for (uint8_t j = 0; j < numControlSignal; j++) {
				OCPMAuxSumTemp += OCPMAuxOutputBuff[j][k][i];
			}

			uncorruptedErrorSignal[k][i] = errorSignal[k][i] - OCPMAuxSumTemp;
		}
	}

return 0;
}





int32_t ANCALG_5(void) {
	ADI_FIR_RESULT res;
	int8_t disableAllOCPMChannelsResult = 0;



	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&refInputBuff[0], (void *)&refSignal[1], 4, controlLength);



	//INDIRECT ERROR SIGNAL
#pragma SIMD_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma SIMD_for(OCPMOutputSize)
			for (uint32_t i = 0; i < OCPMOutputSize; i++) {
				indirectErrorSignal[j][k][i] = (uncorruptedErrorSignal[k][i]
						+ OCPMAuxOutputBuff[j][k][i]);
			}
		}
	}

#ifdef OFPMFilter
	while(bMemCopyInProgress);
	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&OFPMErrorInputBuff[0], (void *)&controlOutputBuff[1], 4, OFPMLength-1);
#endif
	//	power of uncorruptedErrorSignal
#pragma SIMD_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {


		float powerUncorruptedErrorSignalSum =0;
	#pragma SIMD_for
		for (int32_t i = 0;i < OCPMWindowSize; i++) {
			powerUncorruptedErrorSignalSum += uncorruptedErrorSignal[k][i]* uncorruptedErrorSignal[k][i]/OCPMWindowSize;
		}

		powerUncorruptedErrorSignal[k] = forgettingFactorOCPM
				* powerUncorruptedErrorSignal[k]
				+ (1.0 - forgettingFactorOCPM) * powerUncorruptedErrorSignalSum;

	}


	//power of indirectErrorSignal
#pragma SIMD_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {

		float powerIndirectErrorSignalSum =0;
	#pragma SIMD_for
		for (int32_t i = 0;i < OFPMWindowSize; i++) {
			powerIndirectErrorSignalSum += indirectErrorSignal[j][k][i]*indirectErrorSignal[j][k][i]/OCPMWindowSize;
		}

		powerIndirectErrorSignal[j][k] = forgettingFactorOCPM
				* powerIndirectErrorSignal[j][k]
				+ (1.0 - forgettingFactorOCPM) * powerIndirectErrorSignalSum;

	}

	}




	//stepSizeS
#pragma SIMD_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
				stepSizeS[j][k] = constrain((
						stepSizeSMin *powerOCPMWNSignal[j]
						/ (powerIndirectErrorSignal[j][k]+0.0000000001)
						),0.0000001, 0.01);
		}
	}

#ifdef OFPMFilter

	while(bMemCopyInProgress);

	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&uncorruptedRefSignal[0], (void *)&uncorruptedRefSignal[OFPMInputSize - OFPMLength], 4, OFPMLength - 1);
#endif

	//OCPMWNGain
#pragma SIMD_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {

			float powerUncorruptedErrorSignalSumTemp = 0;
			float powerIndirectErrorSignalTemp = 0;

			for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerUncorruptedErrorSignalSumTemp +=
						powerUncorruptedErrorSignal[k];
				powerIndirectErrorSignalTemp +=
						powerIndirectErrorSignal[j][k];
			}
			OCPMWNGain[j] =constrain((
					(powerUncorruptedErrorSignalSumTemp)
							/ (powerIndirectErrorSignalTemp+0.0000000001)
							),0,5)
							;

	}

	disableAllOCPMChannelsResult = DisableAllOCPMChannels();
	if (disableAllOCPMChannelsResult != 0) {
		printf("error disabling FIR channels");
	}

if(1){
	//Control Coeff
#pragma SIMD_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for
		for (int32_t i = 0, l = controlOutputSize - 1;
				i < controlOutputSize|| l > (-1); i++, l--) {


			float controlCoeffBuffSum = 0;
			float NSum = 0;
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				float uncorruptedErrorSignal_OCPMRef_SumTemp = 0;

				for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++){
				uncorruptedErrorSignal_OCPMRef_SumTemp += uncorruptedErrorSignal[k][n]
						* OCPMRefOutputBuff[j][k][i]/NUM_AUDIO_SAMPLES_PER_CHANNEL;

			}
				controlCoeffBuffSum+=uncorruptedErrorSignal_OCPMRef_SumTemp;
				NSum += OCPMRefOutputBuff[j][k][i]*OCPMRefOutputBuff[j][k][i];

			}
			controlCoeffBuff[j][l] =
					constrain((
							controlCoeffBuff[j][l]*(controlLeak)
					+ stepSizeW[j] * controlCoeffBuffSum/(NSum+0.000000001)

					), -2, 2 )
					;
		}

		adi_fir_SetChannelCoefficientBuffer(controlCoeffBuff[j],
				controlCoeffBuff[j], controlCoeffBuff[j], 1);

	}
}


	//OCPM Coeff

#pragma SIMD_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma SIMD_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma SIMD_for
			for (int32_t i = 0, l = OCPMOutputSize - 1;
					i < OCPMOutputSize|| l > (-1); i++, l--) {


				float OCPMCoeffBuffSum = 0;
				//float NSum = 0;
					for (int32_t n = 0; n < NUM_AUDIO_SAMPLES_PER_CHANNEL; n++){
						OCPMCoeffBuffSum += uncorruptedErrorSignal[k][n]/NUM_AUDIO_SAMPLES_PER_CHANNEL
							*  OCPMAuxInputBuff[j][i+controlLength-1];

						//NSum += OCPMAuxInputBuff[j][n]*OCPMAuxInputBuff[j][n];
				}


				OCPMCoeffBuff[j][k][l] =
						constrain((
						OCPMCoeffBuff[j][k][l]*OCPMLeak
								+ stepSizeS[j][k] * OCPMCoeffBuffSum/(OCPMAuxInputBuff[j][i+controlLength-1]*OCPMAuxInputBuff[j][i+controlLength-1])
						), -2, 2 );
			}
			adi_fir_SetChannelCoefficientBuffer(hChannelOCPMRef[j][k],
					OCPMCoeffBuff[j][k], OCPMCoeffBuff[j][k], 1);
		}
	}

	while(bMemCopyInProgress);
	for (int32_t j=0; j< numControlSignal; j++){
	bMemCopyInProgress=true;
	MemDma0Copy1D((void *)&controlOutputSignal[j][0], (void *)&controlOutputSignal[j][controlInputSize- OFPMLength], 4, OFPMLength - 1);
	while(bMemCopyInProgress);
	}

	ANCInProgress = false;

return 0;
}



