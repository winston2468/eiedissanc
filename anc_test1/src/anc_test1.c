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

#include <drivers/twi/adi_twi.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Audio_EI3/drivers/codec/adau1761/adi_adau1761.h"
#include "adi_initialize.h"
#include "Audio_EI3/drivers/codec/adau1761/export_IC_1.h"

#include <matrix.h>
#include "adi_initialize.h"
#include "SHARC_linkInterface.h"
#include <stdlib.h>
#include <adi_types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <drivers/fir/adi_fir.h>

#include <services/gpio/adi_gpio.h>
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


#include <filter.h>
#include "cycle_count.h"
#include "anc_test1.h"


/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE



static ADI_DMA_2D_MEM_TRANSFER Src_2DMemXfer;
static ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXfer;

volatile bool bEnableOutput = true;
volatile bool bEnableOSPM = true;
static volatile bool bMemCopyInProgress = false;
static volatile bool bOSPMRefFIRInProgress = false;
static volatile bool bOSPMAuxFIRInProgress = false;
static volatile bool bRefFIRInProgress = false;



float refCoeffBuff[refLength] = {
#include "data/lowpass_filter_2000_2500.dat"
		};



#pragma alignment_region (4)


/* ----------------------------   FIR Configuration ------------------------------------------- */

float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_ADC / 2] = { 0 };


/* Channel Configurations parameters */
//Max FIR channels = 32

float refInputBuff[refInputSize + 1] = { 0 };
ADI_FIR_CHANNEL_PARAMS channeRef;
uint8_t ConfigMemoryRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryRef[ADI_FIR_CHANNEL_MEMORY_SIZE];

float controlOutputBuff[numControlSignal][controlInputSize + 1] = { 0 };

uint8_t ConfigMemoryOSPMRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOSPMRef[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
uint8_t ConfigMemoryOSPMAux[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOSPMAux[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
ADI_FIR_CHANNEL_PARAMS channelOSPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_PARAMS channelOSPMAux[numControlSignal][numErrorSignal];

float OSPMRefInputBuff[OSPMInputSize] = { 0 };
float OSPMAuxInputBuff[OSPMInputSize] = { 0 };


float OSPMCoeffBuff[numControlSignal][numErrorSignal][OSPMLength] = { 0 };

float OSPMRef[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };
float OSPMAux[numControlSignal][numErrorSignal][OSPMOutputSize] = { 0 };
float outputSignal[numControlSignal][NUM_AUDIO_SAMPLES_DAC] = { 0 };

float OSPMWNSignal[numControlSignal][OSPMInputSize] = { 0 };
float OSPMWNSignalSend[numControlSignal][OSPMLength] = { 0 };

float filteredErrorSignal[numErrorSignal][OSPMLength] = { 0 };

float OSPMWNGain[numControlSignal][OSPMLength] = { 0 };
float powerOSPMWNSignal[numControlSignal][OSPMLength] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength];
float powerIndirectErrorSignal[numControlSignal][numErrorSignal][OSPMLength] = {
		0 };
float powerFilteredErrorSignal[numErrorSignal][OSPMLength] = { 0 };
float stepSizeS[numControlSignal][numErrorSignal][OSPMLength] = { 0 };
float stepSizeW[numControlSignal] = { 0 };


float OSPMRefOutputBuff[numControlSignal][numErrorSignal][OSPMOutputSize];

float OSPMAuxOutputBuff[numControlSignal][numErrorSignal][OSPMOutputSize];
#pragma alignment_region_end


float refOutputBuff[refInputSize + 1] = { 0 };


ADI_FIR_RESULT res;
ADI_FIR_HANDLE hFir;
ADI_FIR_CONFIG_HANDLE hConfigOSPMRef;
ADI_FIR_CONFIG_HANDLE hConfigOSPMAux;
ADI_FIR_CHANNEL_HANDLE hchannelOSPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hchannelOSPMAux[numControlSignal][numErrorSignal];


ADI_FIR_CONFIG_HANDLE hConfigRef;
ADI_FIR_CHANNEL_HANDLE hChannelRef;

volatile bool TRNGFlag = false;

float forgettingFactor = 0.6;
float stepSizeSMin = 0.00001;


/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;

int32_t *pADCBuffer;
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

uint32_t OSPMWNSignal_X =OSPMWindowSize-1;
uint32_t OSPMWNSignal_Y =0;

/* ADC buffer pointer */
 void *pGetADC = NULL;

volatile bool OSPMInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;




/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/
/* Initialize GPIO and reset peripherals */
extern uint32_t GpioInit(void);
/* Initializes ADC */
extern uint32_t Adau1979Init(void);

/* Submit buffers to ADC */
extern uint32_t Adau1979SubmitBuffers(void);

extern uint32_t Adau1979Enable(void);

extern uint32_t Adau1979DoneWithBuffer( void *pBuffer);
extern void configGpio(void);
float constrain(float input, float low, float high);

void ProcessBufferADC(uint32_t iid, void* handlerArg);
void ProcessBufferDAC(uint32_t iid, void* handlerArg);
int32_t FIR_init(void);
void reverseArrayf(float*, uint32_t);
int32_t ANCALG_2(void);
int8_t DisableAllOSPMChannels(void);
float WN_generator(void);
uint32_t SpuInit(void);
extern uint32_t DMAInit(void);

static void RefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OSPMRefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);
static void OSPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);

void PKIC_ISR(uint32_t iid, void* handlerArg);

void TRNG_ISR(void);
void TRNG_ACK(void);

void *DMASlaveDestinationAddress;

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";


extern void ConfigSoftSwitches(void);


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
		if(OSPMWNSignal_Y < numControlSignal ){
			if(OSPMWNSignal_X < OSPMInputSize ){

				Read_TRNG_Output(&OSPMWNSignal[OSPMWNSignal_Y][OSPMWNSignal_X]);
				Read_TRNG_Output(&OSPMWNSignalSend[OSPMWNSignal_Y][OSPMWNSignal_X-OSPMWindowSize-1]);
				OSPMWNSignal_X +=4;
				Acknowledge_Interrupt(iTemp);
			}
			OSPMWNSignal_X = OSPMWindowSize - 1;
			OSPMWNSignal_Y +=1;
		}
		TRNGFlag = true;
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
		OSPMWNSignal_Y =0;
		OSPMWNSignal_X=OSPMWindowSize - 1;
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
			if(bEnableOSPM){
			//LED on bEnableOSPM
			adi_gpio_Clear(LED2_PORT, LED2_PIN);
			bEnableOSPM = false;
			}
			else{
			//LED off
			adi_gpio_Set(LED2_PORT, LED2_PIN);
			bEnableOSPM = true;
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

#pragma vector_for
	for (uint32_t i = 0; i < (sizeof(refCoeffBuff) / 2); i++) {
		float temp = refCoeffBuff[i];
		refCoeffBuff[i] = refCoeffBuff[sizeof(refCoeffBuff) - i - 1];
		refCoeffBuff[sizeof(refCoeffBuff) - i - 1] = temp;
	}


	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			powerOSPMWNSignal[j][i] = 1;

		}
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OSPMLength; i++) {
				powerIndirectErrorSignal[j][k][i] = 1.0;
				OSPMWNGain[j][i] = 1;
			}
		}
	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		for (int32_t i = 0; i < OSPMLength; i++) {
			powerFilteredErrorSignal[k][i] = 1.0;
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		stepSizeW[j] = 0.0001;
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (int32_t i = 0; i < OSPMLength; i++) {
				OSPMCoeffBuff[j][k][i] = 1;
			}
		}
	}

	channelRef.nTapLength = RefLength;
	channelRef.nWindowSize = RefWindowSize;
	channelRef.eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelRef.nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelRef.nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelRef.pInputBuffBase = (void *) RefInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelRef.pInputBuffIndex = (void *) RefInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelRef.nInputBuffCount = RefInputSize; /*!< Number of elements in the input circular buffer */
	channelRef.nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelRef.pCoefficientBase = (void *) RefCoeffBuff; /*!< Pointer to the start of the coefficient buffer */
	channelRef.nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelRef.pCoefficientIndex = (void *) RefCoeffBuff; /*!< Pointer to the start of the coefficient buffer */

	channelRef.pOutputBuffBase = (void *) RefOutputBuff; /*!< Pointer to the base of the output circular buffer */
	channelRef.pOutputBuffIndex = (void *) RefOutputBuff; /*!< Pointer to the current index of the output circular buffer */
	channelRef.nOutputBuffCount = RefWindowSize; /*!< Number of elements in the output circular buffer */
	channelRef.nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */


	channelOSPMRef[0][0].nTapLength = OSPMLength;
	channelOSPMRef[0][0].nWindowSize = OSPMWindowSize;
	channelOSPMRef[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOSPMRef[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOSPMRef[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOSPMRef[0][0].pInputBuffBase = (void *) OSPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOSPMRef[0][0].pInputBuffIndex = (void *) OSPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOSPMRef[0][0].nInputBuffCount = OSPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOSPMRef[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPMRef[0][0].pCoefficientBase = (void *) OSPMCoeffBuff; /*!< Pointer to the start of the coefficient buffer */
	channelOSPMRef[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPMRef[0][0].pCoefficientIndex = (void *) OSPMCoeffBuff; /*!< Pointer to the start of the coefficient buffer */

	channelOSPMRef[0][0].pOutputBuffBase = (void *) OSPMRefOutputBuff; /*!< Pointer to the base of the output circular buffer */
	channelOSPMRef[0][0].pOutputBuffIndex = (void *) OSPMRefOutputBuff; /*!< Pointer to the current index of the output circular buffer */
	channelOSPMRef[0][0].nOutputBuffCount = OSPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOSPMRef[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOSPMRef[0][k] = channelOSPMRef[0][0];

		channelOSPMRef[0][k].pInputBuffBase = (void *) OSPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
		channelOSPMRef[0][k].pInputBuffIndex = (void *) OSPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
		channelOSPMRef[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOSPMRef[0][k].pCoefficientBase = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOSPMRef[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOSPMRef[0][k].pCoefficientIndex = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOSPMRef[0][k].pOutputBuffBase = (void *) OSPMRefOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOSPMRef[0][k].pOutputBuffIndex = (void *) OSPMRefOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOSPMRef[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOSPMRef[j][k] = channelOSPMRef[0][0];

			channelOSPMRef[j][k].pInputBuffBase = (void *) OSPMRefInputBuff; /*!< Pointer to the base of the input circular buffer */
			channelOSPMRef[j][k].pInputBuffIndex = (void *) OSPMRefInputBuff; /*!< Pointer to the current index of the input circular buffer */
			channelOSPMRef[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOSPMRef[j][k].pCoefficientBase = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOSPMRef[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOSPMRef[j][k].pCoefficientIndex = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOSPMRef[j][k].pOutputBuffBase = (void *) OSPMRefOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOSPMRef[j][k].pOutputBuffIndex = (void *) OSPMRefOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOSPMRef[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}


	channelOSPMAux[0][0].nTapLength = OSPMLength;
	channelOSPMAux[0][0].nWindowSize = OSPMWindowSize;
	channelOSPMAux[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOSPMAux[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOSPMAux[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOSPMAux[0][0].pInputBuffBase = (void *) OSPMAuxInputBuff; /*!< Pointer to the base of the input circular buffer */
	channelOSPMAux[0][0].pInputBuffIndex = (void *) OSPMAuxInputBuff; /*!< Pointer to the current index of the input circular buffer */
	channelOSPMAux[0][0].nInputBuffCount = OSPMInputSize; /*!< Number of elements in the input circular buffer */
	channelOSPMAux[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOSPMAux[0][0].pCoefficientBase = (void *) OSPMCoeffBuff; /*!< Pointer to the start of the coefficient buffer */
	channelOSPMAux[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
	channelOSPMAux[0][0].pCoefficientIndex = (void *) OSPMCoeffBuff; /*!< Pointer to the start of the coefficient buffer */

	channelOSPMAux[0][0].pOutputBuffBase = (void *) OSPMAuxOutputBuff; /*!< Pointer to the base of the output circular buffer */
	channelOSPMAux[0][0].pOutputBuffIndex = (void *) OSPMAuxOutputBuff; /*!< Pointer to the current index of the output circular buffer */
	channelOSPMAux[0][0].nOutputBuffCount = OSPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOSPMAux[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOSPMAux[0][k] = channelOSPMAux[0][0];

		channelOSPMAux[0][k].pInputBuffBase = (void *) OSPMAuxInputBuff; /*!< Pointer to the base of the input circular buffer */
		channelOSPMAux[0][k].pInputBuffIndex = (void *) OSPMAuxInputBuff; /*!< Pointer to the current index of the input circular buffer */
		channelOSPMAux[0][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

		channelOSPMAux[0][k].pCoefficientBase = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */
		channelOSPMAux[0][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
		channelOSPMAux[0][k].pCoefficientIndex = (void *) OSPMCoeffBuff[0][k]; /*!< Pointer to the start of the coefficient buffer */

		channelOSPMAux[0][k].pOutputBuffBase = (void *) OSPMAuxOutputBuff[0][k]; /*!< Pointer to the base of the output circular buffer */
		channelOSPMAux[0][k].pOutputBuffIndex = (void *) OSPMAuxOutputBuff[0][k]; /*!< Pointer to the current index of the output circular buffer */
		channelOSPMAux[0][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOSPMAux[j][k] = channelOSPMAux[0][0];

			channelOSPMAux[j][k].pInputBuffBase = (void *) OSPMAuxInputBuff; /*!< Pointer to the base of the input circular buffer */
			channelOSPMAux[j][k].pInputBuffIndex = (void *) OSPMAuxInputBuff; /*!< Pointer to the current index of the input circular buffer */
			channelOSPMAux[j][k].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

			channelOSPMAux[j][k].pCoefficientBase = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */
			channelOSPMAux[j][k].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */
			channelOSPMAux[j][k].pCoefficientIndex = (void *) OSPMCoeffBuff[j][k]; /*!< Pointer to the start of the coefficient buffer */

			channelOSPMAux[j][k].pOutputBuffBase = (void *) OSPMAuxOutputBuff[j][k]; /*!< Pointer to the base of the output circular buffer */
			channelOSPMAux[j][k].pOutputBuffIndex = (void *) OSPMAuxOutputBuff[j][k]; /*!< Pointer to the current index of the output circular buffer */
			channelOSPMAux[j][k].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

		}
	}



	res = adi_fir_Open(0u, &hFir);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_Open failed\n");
		return -1;
	}
	// ------------------------------- Create Configurations --------------------------------------------
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

	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOSPMRef,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOSPMRef);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOSPMRef, OSPMRefFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}
	// ------------------------------- Create Configurations --------------------------------------------
	res = adi_fir_CreateConfig(hFir, ConfigMemoryOSPMAux,
	ADI_FIR_CONFIG_MEMORY_SIZE, &hConfigOSPMAux);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_CreateConfig failed\n");
		return -1;
	}
	res = adi_fir_RegisterCallback(hConfigOSPMAux, OSPMAuxFIRCallback, NULL);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_RegisterCallback failed\n");
		return -1;
	}

	// ----------------------------------  Add Channels ---------------------------------------------------
	res = adi_fir_AddChannel(hConfigRef, ChannelMemoryRef[j][k],
	ADI_FIR_CHANNEL_MEMORY_SIZE, &channelRef[j][k],
			&hchannelRef[j][k]);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_AddChannel Ref %d%d failed\n");
		return -1;
	}

	for (int8_t j = 0; j < numControlSignal; j++) {
		for (int8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOSPMRef, ChannelMemoryOSPMRef[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPMRef[j][k],
					&hchannelOSPMRef[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OSPM Ref %d%d failed\n");
				return -1;
			}
		}
	}

	for (int8_t j = 0; j < numControlSignal; j++) {
		for (int8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_AddChannel(hConfigOSPMAux, ChannelMemoryOSPMAux[j][k],
			ADI_FIR_CHANNEL_MEMORY_SIZE, &channelOSPMAux[j][k],
					&hchannelOSPMAux[j][k]);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_AddChannel OSPM Aux %d%d failed\n");
				return -1;
			}
		}
	}

	res = adi_fir_ChannelInterruptEnable(hConfigOSPMRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	res = adi_fir_ChannelInterruptEnable(hConfigOSPMAux, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	res = adi_fir_ChannelInterruptEnable(hConfigRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	return 0;
}

int main(void) {

	int8_t firResult = 0;
	bool bExit;
	uint32_t Result = 0u;
	//uint32_t i;
	bExit = false;

	adi_initComponents(); /* auto-generated code */

	//SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;



	adi_sec_SetPriority(INTR_SOFT5, 62); // set the priority of SOFT5 interrupt (priority 60)
	// Register and install a handler for the software interrupt SOFT5 (priority 60)
	adi_int_InstallHandler(INTR_SOFT5, ProcessBufferADC, 0, true);




	/* Software Switch Configuration for the EZ-BOARD */
	ConfigSoftSwitches();
	if (Result == 0u) {
		Result = SpuInit();
	}
	/*
	if (Result == 0u) {
		Result = DMAInit();
	}
*/
	configGpio();



	adi_gpio_Set(LED1_PORT, LED1_PIN);
	adi_gpio_Set(LED2_PORT, LED2_PIN);

	// Initialize ADAU1979
	if (Result == 0u) {
		Result = Adau1979Init();
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

	bEvent = true;
	eMode = START;

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case RECIEVE:
				if(bEnableOSPM){
				ANCALG_2();
				}
				//ProcessBufferADC1();
				// ProcessBufferADC2();
				//ProcessBufferDAC();
				break;
			case START:
				printf("Started.\n");

				//1979 ping pong buffers
				if (Result == 0u) {
					Result = Adau1979SubmitBuffers();
				}

				//1962a ping pong buffers
				if (Result == 0u) {
					Result = Adau1962aSubmitBuffers();
				}
				// Enable data flow for the ADC
				if (Result == 0u) {
					Adau1979Enable();
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




void ProcessBufferADC(uint32_t iid, void* handlerArg) {

	if (pGetADC != NULL) {
		pADCBuffer = (int32_t *) pGetADC;
		if (!OSPMInProgress) {

			for (uint32_t i = 0, l = NUM_AUDIO_SAMPLES_PER_CHANNEL;
					i < NUM_AUDIO_SAMPLES_PER_CHANNEL-1, l < refInputSize; i++, l++) {
				refInputBuff[l] = conv_float_by((pADCBuffer[4 * i]<<8), -25);
				for(uint8_t k = 0; k < numErrorSignal; k++){
					errorSignal[k][i] = conv_float_by((pADCBuffer[4 * i + 1 + k]<<8), -25);
				}
			}

			res = adi_fir_SubmitInputCircBuffer(hchannelRef,
					refInputBuff, refInputBuff,
					refInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hchannelRef failed\n");
			}

			res = adi_fir_EnableChannel(hchannelRef, true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hchannelRef failed\n");
			}

			bRefFIRInProgress=true;
			res = adi_fir_EnableConfig(hConfigRef, true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableConfig hConfigRef failed\n");
			}
			while(bRefFIRInProgress);

			//*****************************************************************
			// Send RAW audio to slave SHARC
			//*****************************************************************
			if( SHARC_linkSend( MDMA_STREAM_ID_REF, (void *)refOutputBuff, DMASlaveDestinationAddress, 1, AUDIO_BUFFER_SIZE_ADC_1979 ) != 0 )
			{
				while(1){;} // If we get here, there is an error
			}



			*sharc_flag_in_L2 = *sharc_flag_in_L2 + 1;

			Adau1979DoneWithBuffer(pGetADC);

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

		adi_sec_Raise(INTR_SOFT5);

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

static void OSPMRefFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOSPMRefFIRInProgress = false;
		break;

	default:
		break;
	}
}

static void OSPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg) {
	/* CASEOF (Event) */
	switch ((ADI_FIR_EVENT) eEvent) {
	/* CASE (Processed a one-shot/circular buffer) */
	case (ADI_FIR_EVENT_ALL_CHANNEL_DONE):
		/* Update memory copy status flag */
		bOSPMAuxFIRInProgress = false;
		break;

	default:
		break;
	}
}

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

int8_t DisableAllOSPMChannels() {
	ADI_FIR_RESULT res;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hchannelOSPMRef[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel failed hchannelOSPMRef%d%d\n", j, k);
				return -1;
			}
		}
	}
	return 0;

}



int32_t ANCALG_2(void) {
	ADI_FIR_RESULT res;
	int8_t disableAllOSPMChannelsResult = 0;
	ADI_DMA_RESULT eResult = ADI_DMA_SUCCESS;


#pragma vector_for
	for(uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for(uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hchannelOSPMRef[j][k], refSignalPastFuture,
					refSignalPastFuture, OSPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_SubmitInputCircBuffer hchannelOSPMRef[%d][%d] failed\n", j ,k);
				return -1;
			}
		}
	}



	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_EnableChannel(hchannelOSPMRef[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel hchannelOSPMRef[%d][%d] failed\n", j,
						k);
				return -1;
			}

		}
	}

	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	} else {
		bOSPMFIRInProgress = true;
	}

	while (bOSPMFIRInProgress)
		;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hchannelOSPMRef[j][k], false);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_EnableChannel disable hchannelOSPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}
		}
	}



#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint32_t i = 0, l = OSPMLength;
				i < OSPMLength, l < (OSPMLength * 2 - 1); i++, l++) {
			OSPMWNSignal[j][i] = OSPMWNGain[j][i]
					* (float) WN_generator();
			OSPMWNSignalPastFuture[j][l] = OSPMWNSignal[j][i];
		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_SubmitInputCircBuffer(hchannelOSPMRef[j][k],
					OSPMWNSignalPastFuture[j], OSPMWNSignalPastFuture[j],
					OSPMInputSize, 1);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf(
						"adi_fir_SubmitInputCircBuffer hchannelOSPMRef[%d][%d] failed\n",
						j, k);
				return -1;
			}

		}
	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_EnableChannel(hchannelOSPMRef[j][k], true);
			if (res != ADI_FIR_RESULT_SUCCESS) {
				printf("adi_fir_EnableChannel channelOSPMRef[%d][%d] failed\n", j,
						k);
				return -1;
			}

		}
	}

	res = adi_fir_EnableConfig(hConfigOSPM, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_EnableConfig failed\n");
		return -1;
	} else {
		bOSPMFIRInProgress = true;
	}

#ifndef LowPassFilter
	// Initialize flag
	bMemCopyInProgress = true;
	eResult = adi_mdma_Copy1D (hMemDmaStream,
			(void *)&refSignalPastFuture[0],
			(void *)&refInputBuff[0],
			MEMCOPY_MSIZE,
			refLength);

	// IF (Failure)
	if (eResult != ADI_DMA_SUCCESS)
	{
		DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);

	}
	while (bMemCopyInProgress);
#endif

	while (bOSPMFIRInProgress)
		;
	//cycle_t start_count=0;

	// cycle_t final_count=0;
	//START_CYCLE_COUNT(start_count);


#pragma vector_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
		for (int32_t i = 0; i < OSPMOutputSize; i++) {

			float OSPMAuxSumTemp = 0;
			for (uint8_t j = 0; j < numControlSignal; j++) {
				OSPMAuxSumTemp += OSPMAuxOutputBuff[j][k][i];
			}
			filteredErrorSignal[k][i] = (errorSignal[k][i] - OSPMAuxSumTemp);
		}
	}

	//INDIRECT ERROR SIGNAL
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				indirectErrorSignal[j][k][i] = (filteredErrorSignal[k][i]
						+ OSPMOutputBuff[j][k][i]);
			}
		}
	}

	//	power of OSPMWNSignal
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(OSPMOutputSize)
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			powerOSPMWNSignal[j][i] = (forgettingFactor
					* powerOSPMWNSignal[j][i]
					+ (1.0 - forgettingFactor) * OSPMWNSignal[j][i]
							* OSPMWNSignal[j][i]);
		}
	}

	//	power of filteredErrorSignal
#pragma vector_for(numErrorSignal)
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
		for (uint32_t i = 0; i < OSPMOutputSize; i++) {
			powerFilteredErrorSignal[k][i] = (forgettingFactor
					* powerFilteredErrorSignal[k][i]
					+ (1.0 - forgettingFactor) * filteredErrorSignal[k][i]
							* filteredErrorSignal[k][i]);
		}
	}

	//power of indirectErrorSignal
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (uint32_t i = 0; i < OSPMOutputSize; i++) {
				powerIndirectErrorSignal[j][k][i] = (forgettingFactor
						* powerIndirectErrorSignal[j][k][i]
						+ (1.0 - forgettingFactor)
								* indirectErrorSignal[j][k][i]
								* indirectErrorSignal[j][k][i]);

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
				stepSizeS[j][k][i] = ((powerOSPMWNSignal[j][i] * stepSizeSMin)
						/ powerIndirectErrorSignal[j][k][i]);
			}
		}
	}

	//OSPMWNGain
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
			OSPMWNGain[j][i] = constrain(
					(powerFilteredErrorSignalSumTemp
							/ powerpowerIndirectErrorSignalTemp), -10000, 10000);

		}
	}

	disableAllOSPMChannelsResult = DisableAllOSPMChannels();
	if (disableAllOSPMChannelsResult != 0) {
		printf("error disabling FIR channels");
	}

	//OSPM Coeff

#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for(OSPMOutputSize)
			for (int32_t i = 0, l = OSPMOutputSize - 1;
					i < OSPMOutputSize, l > (-1); i++, l--) {
				OSPMCoeffBuff[j][k][l] =
				constrain(
						(OSPMCoeffBuff[j][k][l]
								+ (stepSizeS[j][k][i] * OSPMWNSignal[j][i]
										* filteredErrorSignal[k][i]))
						, -10000, 10000 )
						;
			}
			adi_fir_SetChannelCoefficientBuffer(hchannelOSPMRef[j][k],
					OSPMCoeffBuff[j][k], OSPMCoeffBuff[j][k], 1);
		}
	}

	//Control Coeff
#pragma vector_for(numControlSignal)
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (int32_t i = 0, l = controlOutputSize - 1;
				i < controlOutputSize, l > (-1); i++, l--) {
			float filteredErrorSignal_OSPMRef_SumTemp = 0;

			for (uint8_t k = 0; k < numErrorSignal; k++) {
				filteredErrorSignal_OSPMRef_SumTemp += filteredErrorSignal[k][i]
						* OSPMRef[j][k][i];
			}

			controlCoeffBuff[j][l] =
					constrain(
							(controlCoeffBuff[j][l]
					+ stepSizeW[j] * filteredErrorSignal_OSPMRef_SumTemp)
					, -10000, 10000 );

		}
	}


	/*
	 // Populate 2D Memory transfer instance for source channel
	 Src_2DMemXfer.pStartAddress    = &OSPMWNSignal[0][0];
	 Src_2DMemXfer.YCount           = numControlSignal;                    // Configure YCount for 2D transfer
	 Src_2DMemXfer.YModify          = 4;
	 Src_2DMemXfer.XCount           = OSPMLength;
	 Src_2DMemXfer.XModify          = 4;

	 // Populate 2D Memory transfer instance for destination channel
	 Dest_2DMemXfer.pStartAddress   = &OSPMWNSignalPastFuture[0][0];
	 Dest_2DMemXfer.YCount          = numControlSignal;                    // Configure YCount for 2D transfer
	 Dest_2DMemXfer.YModify         = 4;
	 Dest_2DMemXfer.XCount          = OSPMLength;
	 Dest_2DMemXfer.XModify         = 4;
	 // Initialize flag
	 bMemCopyInProgress = true;
	 eResult = adi_mdma_Copy2D (hMemDmaStream,
	 MEMCOPY_MSIZE,
	 &Dest_2DMemXfer,
	 &Src_2DMemXfer);

	 // IF (Failure)
	 if (eResult != ADI_DMA_SUCCESS)
	 {
	 DBG_MSG("Failed initialize MDMA 2D Copy \n", eResult);
	 }
	 while (bMemCopyInProgress);
	 */

	/*

	 #pragma vector_for
	 for (uint8_t j = 0; j < numControlSignal; j++) {



	 bMemCopyInProgress = true;
	 eResult = adi_mdma_Copy1D (hMemDmaStream,
	 (void *)&OSPMWNSignalPastFuture[j][0],
	 (void *)&OSPMWNSignal[j][0],
	 MEMCOPY_MSIZE,
	 (OSPMLength));

	 // IF (Failure)
	 if (eResult != ADI_DMA_SUCCESS)
	 {
	 DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
	 }

	 while (bMemCopyInProgress);

	 }


	 */
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < OSPMLength; i++) {
			OSPMWNSignalPastFuture[j][i] = OSPMWNSignal[j][i];
		}
	}

	OSPMInProgress = false;
	return 0;
}

