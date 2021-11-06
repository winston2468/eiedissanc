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
#include <filters.h>
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




volatile bool bEnableOutput = true;
volatile bool bEnableOCPMWeightUpdate = true;
static volatile bool bMemCopyInProgress = false;



//*****************************************************************************
uint8_t DacStarted = 0u;
/* ADC/DAC buffer pointer */
volatile void *pGetDAC = NULL;
volatile void *pDAC = NULL;
 int countaa =0;


//volatile int32_t DacCount = 0;
bool OCPMUpdate = true;
#pragma alignment_region (4)
int32_t * pADCBuffer;

/* ----------------------------   FIR Configuration ------------------------------------------- */
//int32_t test[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS]={0};
float errorSignal[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
//float errorSignal_temp[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
/* Channel Configurations parameters */
//Max FIR channels = 32

float controlOutputBuff[numControlSignal] = { 0 };
float controlOutputBuff_OFPMError[OFPMErrorLength] = { 0 };

float pm  controlCoeffBuff[numControlSignal][OCPMLength]={0};     /* coeffs array must be         */
                           /* initialized and in PM memory */
float controlState[numControlSignal][OCPMLength+1]={0};

float controlOutput[numControlSignal]={0};

float refInputBuff[refWindowSize] = {0};
float pm  refCoeffBuff[refLength]={
#include "data/lowpass_4000_8000.dat"
};

float refState[refLength+1]={0};
float refOutputBuff[refWindowSize] =0;




float controlOutputInputBuff = 0;
float pm  controlOutputCoeffBuff[controlOutputLength]={
#include "data/lowpass_4000_8000.dat"

};
float controlOutputState[controlOutputLength+1]={0};
float controlOutputOutputBuff =0;



float pm  OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength]={0};     /* coeffs array must be         */
                           /* initialized and in PM memory */
float OCPMRefState[numControlSignal][numErrorSignal][OCPMLength+1]={0};
float OCPMAuxState[numControlSignal][numErrorSignal][OCPMLength+1]={0};

float OCPMRefOutput[numControlSignal][numErrorSignal]={0};
float OCPMRefOutputBuff[numControlSignal][numErrorSignal][OCPMLength]={0};
float OCPMAuxOutputBuff[numControlSignal][numErrorSignal]={0};

#ifdef OCPMOnFIRA
ADI_FIR_CHANNEL_PARAMS channelOCPMAux[numControlSignal][numErrorSignal];
/*
ADI_FIR_CHANNEL_PARAMS channelOCPMRef[numControlSignal][numErrorSignal];
float OCPMRefInputBuff[OCPMLength - 1 + OCPMWindowSize] = { 0 };
uint8_t ConfigMemoryOCPMRef[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMRef[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];
float OCPMRefOutputBuff[numControlSignal][numErrorSignal][OCPMWindowSize] =
		{ 0 };
float OCPMRefOutputBuff_pp[numControlSignal][numErrorSignal][OCPMLength - 1 + OCPMWindowSize] =
		{ 0 };
*/
uint8_t ConfigMemoryOCPMAux[ADI_FIR_CONFIG_MEMORY_SIZE];
uint8_t ChannelMemoryOCPMAux[numControlSignal][numErrorSignal][ADI_FIR_CHANNEL_MEMORY_SIZE];

float OCPMAuxInputBuff[numControlSignal][OCPMLength - 1 + OCPMWindowSize] = { 0 };
float OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength] = { 0 };
float OCPMAuxOutputBuff[numControlSignal][numErrorSignal][OCPMWindowSize] =
		{ 0 };
float OCPMAuxOutputBuff_pp[numControlSignal][numErrorSignal][OCPMLength - 1 + OCPMWindowSize] =
		{ 0 };
#endif


float refSignal[NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };

float uncorruptedErrorSignal[numErrorSignal] = { 0 };

float OCPMWNGain[numControlSignal] = { 0 };
float powerOCPMWNSignal[numControlSignal] = { 0 };
float powerResidualErrorSignal[numErrorSignal] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal]={0};
float powerIndirectErrorSignal[numControlSignal][numErrorSignal] = { 0 };
float powerUncorruptedErrorSignal[numErrorSignal] = { 0 };
float powerUncorruptedRefSignal = 1.0;
float uncorruptedRefSignal = 0;
float OFPMErrorSignal=0;

float forgettingFactorOCPM = 0.6;

float stepSizeW = 0;
float stepSizeE = 0;
float stepSizeSMin = 1;
float stepSizeSMax = 10;
float stepSizeS[numControlSignal][numErrorSignal] = { 0 };
#ifdef OFPMFilter
float powerOFPMErrorSignal = 1.0;
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



float pm  OFPMCoeffBuff[numControlSignal][OFPMLength]= {0};
float pm  OFPMErrorCoeffBuff[OFPMErrorLength]= {0};

float OFPMState[numControlSignal][OFPMLength+1];
float OFPMErrorState[OFPMErrorLength+1];

float OFPMOutputBuff[numControlSignal]={0};
float OFPMErrorOutputBuff=0;



#pragma alignment_region_end


#ifdef OCPMOnFIRA
ADI_FIR_RESULT res;
ADI_FIR_HANDLE hFir;
//ADI_FIR_CONFIG_HANDLE hConfigOCPMRef;
ADI_FIR_CONFIG_HANDLE hConfigOCPMAux;
//ADI_FIR_CHANNEL_HANDLE hChannelOCPMRef[numControlSignal][numErrorSignal];
ADI_FIR_CHANNEL_HANDLE hChannelOCPMAux[numControlSignal][numErrorSignal];
//static volatile bool bOCPMRefFIRInProgress = false;
static volatile bool bOCPMAuxFIRInProgress = false;
#endif




//TRNG
volatile bool TRNGFlag = false;
#pragma align(4)
float randBuff[WNLength] = {0};
float WNSignal[numControlSignal] = {0};
float WNSignalBuff[numControlSignal][WNSignalBuffLength] = {0};

uint8_t WNCount = 0;




/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;
int32_t *pSrc;
int32_t *pDst;
volatile bool DACFlag = false;

static bool bError = false;
static uint32_t count;

volatile bool bEvent = false;

/* ADC buffer pointer */
volatile void *pGetADC = NULL;
volatile void *pADC = NULL;
volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;



/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";


 uint32_t iida = 0;




void aluFLTOIHandler(uint32_t iid, void* handlerArg) {
	printf("%d", iid);
	printf("ERR");
	iida = iid;
}



void TRNG_Init (){
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
}




uint8_t RefFIR() {
	 firf (refInputBuff,
	          refOutputBuff,
	            refCoeffBuff,
	            refState,
	            refWindowSize,
	            refLength);
	return 0;
}




uint8_t OFPMFIR() {
	uncorruptedRefSignal =  refOutputBuff[0];
for(uint8_t j =0; j < numControlSignal; j++){
	OFPMOutputBuff[j] = fir ((controlOutputBuff[j]+WNSignal[j]), OFPMCoeffBuff[j], OFPMState[j], OFPMLength);
	uncorruptedRefSignal -=  OFPMOutputBuff[j];
}
	return 0;
}

uint8_t OFPMErrorFIR() {

	OFPMErrorOutputBuff = fir (controlOutputBuff[0], OFPMErrorCoeffBuff, OFPMErrorState, OFPMErrorLength);
	//compiler circular buffer optimisation
    for(uint32_t i = 0; i < OFPMLength ; i++){
    	*(controlOutputBuff_OFPMError + i) = *(controlOutputBuff_OFPMError + i + 1);
    }
    controlOutputBuff_OFPMError[OFPMLength-1] = controlOutputBuff[0];
	return 0;
}


uint8_t ControlFIR() {

	for(uint8_t j =0; j < numControlSignal; j++){

		controlOutputBuff[j] = fir (uncorruptedRefSignal, controlCoeffBuff[j], controlState[j], controlLength);
	}

	return 0;
}

uint8_t GenControlSignal() {

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {

				controlOutputSignal[j][i + NUM_AUDIO_SAMPLES_PER_CHANNEL - 1] =
						controlOutputBuff[j][i]
								+ OCPMAuxInputBuff[j][i + OCPMWindowSize - 1];

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


		Adau1979DoneWithBuffer(pGetADC);
		pGetADC = NULL;

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
			*pDst++ = (conv_fix_by(controlOutputSignal[0][i], 10));
			*pDst++ = (conv_fix_by(controlOutputSignal[1][i], 10));
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;




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
if(WNCount < numControlSignal){
			Read_TRNG_Output_Imp(&randBuff[WNCount*4]);
			WNCount++;
	}
	else{
		WNCount =0;
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
			if (bEnableOCPMWeightUpdate) {
				//LED on bEnableOCPMWeightUpdate
				adi_gpio_Clear(LED2_PORT, LED2_PIN);
				bEnableOCPMWeightUpdate = false;
			} else {
				//LED off
				adi_gpio_Set(LED2_PORT, LED2_PIN);
				bEnableOCPMWeightUpdate = true;
			}
		}
	}

	/* reset the exit counter */
	count = 0u;

}

int32_t FIR_init() {

	//FIR stuff
	ADI_FIR_RESULT res;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (int32_t i = 0; i < OCPMLength; i++) {
			powerOCPMWNSignal[j] = 1.0;

		}
		for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerIndirectErrorSignal[j][k] = 1.0;
				OCPMWNGain[j] = 1.0;
		}
	}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
			powerUncorruptedErrorSignal[k] = 1.0;
			powerResidualErrorSignal[k]=1.0;

	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		stepSizeW = 0.001;	// 0.000005;
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			stepSizeSMin = 0.001;
			stepSizeS[j][k] = stepSizeSMin;
			stepSizeE = 0.001;	// 0.000005;


		}
	}
	stepSizeF = stepSizeFMin;
	stepSizeH = stepSizeHMin;

	//stepSizeH = 0.0001;



	for (int32_t i = 0; i < OCPMLength+1; i++) {
		 OFPMErrorState[i]=0;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		 controlState[j][i]=0;
		for (uint8_t k = 0; k < numErrorSignal; k++) {

				 refState[i]=0;

				 OCPMRefState[j][k][i]=0;
				 OCPMAuxState[j][k][i]=0;

			}
		}
	for (uint8_t k = 0; k < numErrorSignal; k++) {
		 OFPMState[k][i]=0;
	}
		}






#ifdef OCPMOnFIRA



	res = adi_fir_Open(0u, &hFir);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_Open failed\n");
		return -1;
	}

/*

	channelOCPMRef[0][0].nTapLength = OCPMLength;
	channelOCPMRef[0][0].nWindowSize = OCPMWindowSize;
	channelOCPMRef[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOCPMRef[0][0].nSamplingRatio = 1u; //!< Sampling Ratio
	channelOCPMRef[0][0].nGroupNum = 1u; //!< Group Number of the Channel - Channels in groups 0 will always be
	 //scheduled before group 1 and so on. Group number of the channel
	 //determines the order in which channels in a configuration will be linked

	channelOCPMRef[0][0].pInputBuffBase = (void *) &OCPMRefInputBuff[0]; //!< Pointer to the base of the input circular buffer
	channelOCPMRef[0][0].pInputBuffIndex = (void *) &OCPMRefInputBuff[0]; //!< Pointer to the current index of the input circular buffer
	channelOCPMRef[0][0].nInputBuffCount = OCPMLength - 1 + OCPMWindowSize; //!< Number of elements in the input circular buffer
	channelOCPMRef[0][0].nInputBuffModify = 1; //!< Modifier to be used for the input circular buffer

	channelOCPMRef[0][0].pCoefficientBase = (void *) &OCPMCoeffBuff[0][0][0]; //!< Pointer to the start of the coefficient buffer
	channelOCPMRef[0][0].nCoefficientModify = 1; //!< Modify for the Coefficient Buffer
	channelOCPMRef[0][0].pCoefficientIndex = (void *) &OCPMCoeffBuff[0][0][0]; //!< Pointer to the start of the coefficient buffer

	channelOCPMRef[0][0].pOutputBuffBase = (void *) &OCPMRefOutputBuff[0][0][0]; //!< Pointer to the base of the output circular buffer
	channelOCPMRef[0][0].pOutputBuffIndex = (void *) &OCPMRefOutputBuff[0][0][0]; //!< Pointer to the current index of the output circular buffer
	channelOCPMRef[0][0].nOutputBuffCount = OCPMWindowSize; //!< Number of elements in the output circular buffer
	channelOCPMRef[0][0].nOutputBuffModify = 1; //!< Modifier to be used for the output circular buffer

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOCPMRef[0][k] = channelOCPMRef[0][0];

		channelOCPMRef[0][k].pCoefficientBase = (void *) &OCPMCoeffBuff[0][k][0]; //!< Pointer to the start of the coefficient buffer
		channelOCPMRef[0][k].pCoefficientIndex = (void *) &OCPMCoeffBuff[0][k][0]; //!< Pointer to the start of the coefficient buffer

		channelOCPMRef[0][k].pOutputBuffBase = (void *) &OCPMRefOutputBuff[0][k][0]; //!< Pointer to the base of the output circular buffer
		channelOCPMRef[0][k].pOutputBuffIndex = (void *) &OCPMRefOutputBuff[0][k][0]; //!< Pointer to the current index of the output circular buffer
	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMRef[j][k] = channelOCPMRef[0][0];

			channelOCPMRef[j][k].pCoefficientBase = (void *) &OCPMCoeffBuff[j][k][0]; //!< Pointer to the start of the coefficient buffer
			channelOCPMRef[j][k].pCoefficientIndex = (void *) &OCPMCoeffBuff[j][k][0]; //!< Pointer to the start of the coefficient buffer

			channelOCPMRef[j][k].pOutputBuffBase = (void *) &OCPMRefOutputBuff[j][k][0]; //!< Pointer to the base of the output circular buffer
			channelOCPMRef[j][k].pOutputBuffIndex = (void *) &OCPMRefOutputBuff[j][k][0]; //!< Pointer to the current index of the output circular buffer

		}
	}
	*/

	channelOCPMAux[0][0].nTapLength = OCPMLength;
	channelOCPMAux[0][0].nWindowSize = OCPMWindowSize;
	channelOCPMAux[0][0].eSampling = ADI_FIR_SAMPLING_SINGLE_RATE;
	channelOCPMAux[0][0].nSamplingRatio = 1u; /*!< Sampling Ratio */
	channelOCPMAux[0][0].nGroupNum = 1u; /*!< Group Number of the Channel - Channels in groups 0 will always be
	 scheduled before group 1 and so on. Group number of the channel
	 determines the order in which channels in a configuration will be linked */

	channelOCPMAux[0][0].pInputBuffBase = (void *) &OCPMAuxInputBuff[0][0]; /*!< Pointer to the base of the input circular buffer */
	channelOCPMAux[0][0].pInputBuffIndex = (void *) &OCPMAuxInputBuff[0][0]; /*!< Pointer to the current index of the input circular buffer */
	channelOCPMAux[0][0].nInputBuffCount = OCPMLength - 1 + OCPMWindowSize; /*!< Number of elements in the input circular buffer */
	channelOCPMAux[0][0].nInputBuffModify = 1; /*!< Modifier to be used for the input circular buffer */

	channelOCPMAux[0][0].pCoefficientBase = (void *) &OCPMCoeffBuff[0][0][0]; /*!< Pointer to the start of the coefficient buffer */
	channelOCPMAux[0][0].pCoefficientIndex = (void *) &OCPMCoeffBuff[0][0][0]; /*!< Pointer to the start of the coefficient buffer */
	channelOCPMAux[0][0].nCoefficientModify = 1; /*!< Modify for the Coefficient Buffer */

	channelOCPMAux[0][0].pOutputBuffBase = (void *) &OCPMAuxOutputBuff[0][0][0]; /*!< Pointer to the base of the output circular buffer */
	channelOCPMAux[0][0].pOutputBuffIndex = (void *) &OCPMAuxOutputBuff[0][0][0]; /*!< Pointer to the current index of the output circular buffer */
	channelOCPMAux[0][0].nOutputBuffCount = OCPMWindowSize; /*!< Number of elements in the output circular buffer */
	channelOCPMAux[0][0].nOutputBuffModify = 1; /*!< Modifier to be used for the output circular buffer */

	for (uint8_t k = 1; k < numErrorSignal; k++) {
		channelOCPMAux[0][k] = channelOCPMAux[0][0];

		channelOCPMAux[0][k].pInputBuffBase = (void *) &OCPMAuxInputBuff[0][0]; /*!< Pointer to the base of the input circular buffer */
		channelOCPMAux[0][k].pInputBuffIndex = (void *) &OCPMAuxInputBuff[0][0]; /*!< Pointer to the current index of the input circular buffer */

		channelOCPMAux[0][k].pCoefficientBase = (void *) &OCPMCoeffBuff[0][k][0]; /*!< Pointer to the start of the coefficient buffer */
		channelOCPMAux[0][k].pCoefficientIndex = (void *) &OCPMCoeffBuff[0][k][0]; /*!< Pointer to the start of the coefficient buffer */

		channelOCPMAux[0][k].pOutputBuffBase = (void *) &OCPMAuxOutputBuff[0][k][0]; /*!< Pointer to the base of the output circular buffer */
		channelOCPMAux[0][k].pOutputBuffIndex = (void *) &OCPMAuxOutputBuff[0][k][0]; /*!< Pointer to the current index of the output circular buffer */

	}

	for (uint8_t j = 1; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			channelOCPMAux[j][k] = channelOCPMAux[0][0];

			channelOCPMAux[j][k].pInputBuffBase = (void *) &OCPMAuxInputBuff[j][0]; /*!< Pointer to the base of the input circular buffer */
			channelOCPMAux[j][k].pInputBuffIndex = (void *) &OCPMAuxInputBuff[j][0]; /*!< Pointer to the current index of the input circular buffer */

			channelOCPMAux[j][k].pCoefficientBase = (void *) &OCPMCoeffBuff[j][k][0]; /*!< Pointer to the start of the coefficient buffer */
			channelOCPMAux[j][k].pCoefficientIndex = (void *) &OCPMCoeffBuff[j][k][0]; /*!< Pointer to the start of the coefficient buffer */

			channelOCPMAux[j][k].pOutputBuffBase = (void *) &OCPMAuxOutputBuff[j][k][0]; /*!< Pointer to the base of the output circular buffer */
			channelOCPMAux[j][k].pOutputBuffIndex = (void *) &OCPMAuxOutputBuff[j][k][0]; /*!< Pointer to the current index of the output circular buffer */

		}
	}

/*
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
	*/


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
	/*
	// ----------------------------------  Add Channels ---------------------------------------------------

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
	*/

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
	/*
	res = adi_fir_ChannelInterruptEnable(hConfigOCPMRef, true);
	if (res != ADI_FIR_RESULT_SUCCESS) {
		printf("adi_fir_ChannelInterruptEnable failed\n");
		return -1;
	}
	*/
	res = adi_fir_ChannelInterruptEnable(hConfigOCPMAux, true);
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


	adi_initComponents(); /* auto-generated code */

	//SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;

	adi_int_InstallHandler(ADI_CID_FLTOI, aluFLTOIHandler, NULL, true);
	//adi_int_InstallHandler (ADI_CID_FLTUI, aluHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FLTII, aluHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FIXI, aluHandler, NULL, true) ;

	if (Result == 0u) {
		Result = DMAInit();
	}

	configGpio();

	adi_gpio_Set(LED1_PORT, LED1_PIN);
	adi_gpio_Set(LED2_PORT, LED2_PIN);


	// Initialize ADAU1962a
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	firResult = FIR_init();
	if (firResult == 0) {
		printf("FIR init success\n");
	}

	TRNG_Init();
	if (Result == 1) {
		printf("Init Error");
		return 1;
	}


	// Initialize ADAU1979
	if (Result == 0u) {
		Result = Adau1979Init();
	}






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

	// Enable data flow for the DAC
	if (Result == 0u) {
		Adau1962aEnable();
	}


	while (!bExit) {
		if (bEvent) {


				if (bEnableOCPMWeightUpdate) {
				}
				ProcessBuffers();
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



void ProcessBuffers() {



	if (pGetADC != NULL) {

		pADC = (void *) pGetADC;
		adi_adau1979_SubmitBuffer(phAdau1979, (void *) pADC, AUDIO_BUFFER_SIZE_ADC_1979);
		pGetADC=NULL;



		if (!ANCInProgress) {
			ANCInProgress = true;
			pADCBuffer = (int32_t *) pADC;
//N#pragma vector_for
			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {

				refSignal[i] = conv_float_by(((*(pADCBuffer +
#ifdef TDM_MODE
						8
#else
						4
#endif
						* i))
						//<<8
				),-23);

				for (uint8_t k = 0; k < numErrorSignal; k++) {
					errorSignal[k][i+controlLength-1] = conv_float_by(
							((*(pADCBuffer +
#ifdef TDM_MODE
						8
#else
						4
#endif
									* i + 1 + k))
									//<<8
							), -23);
				}
			}


			RefFIR();



			ControlFIR();
			while (!TRNGFlag)
				;

			TRNGFlag = false;
			WN_Gen();
			OFPMFIR();
			OFPMErrorFIR();
			OCPMRefFIR();
			OCPMAuxFIR();
			ANCALG_1();
			ANCALG_2();
			ANCALG_3();
			ANCALG_4();
			ANCALG_5();
			OFPMErrorWeightUpdate();
			OFPMWeightUpdate();
			ControlWeightUpdate();
			OCPMWeightUpdate();

			GenControlSignal();

			PushControlSignal();

			TRNG_ACK();
			ANCInProgress = false;
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
			printf("ERR ANC");
		}
		bEvent = true;
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





int32_t OCPMRefFIR(void) {
#ifdef OCPMOnFIRA
	ADI_FIR_RESULT res;


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {

			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMRef[j][k],
					(void*)&OCPMRefInputBuff, (void*)&OCPMRefInputBuff, (OCPMLength - 1 + OCPMWindowSize), 1);
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

#else
	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){
		OCPMRefOutput[j][k] = fir (uncorruptedRefSignal, OCPMCoeffBuff[j][k], OCPMRefState[j][k], OCPMLength);
		}
	}



#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
	for (uint32_t i = 0; i < OCPMLength; i++) {
		OCPMRefOutputBuff[j][k][i] = OCPMRefOutputBuff[j][k][i+1];
	}

	OCPMRefOutputBuff[j][k][OCPMLength-1] = OCPMRefOutput[j][k];
		}
	}

#endif

	return 0;
}



int32_t OCPMAuxFIR(void) {
#ifdef OCPMOnFIRA
	ADI_FIR_RESULT res;

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			res = adi_fir_SubmitInputCircBuffer(hChannelOCPMAux[j][k],
					(void*)&OCPMAuxInputBuff[j], (void*)&OCPMAuxInputBuff[j],
					(OCPMLength - 1 + OCPMWindowSize), 1);
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

#else


	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){
		OCPMAuxOutputBuff[j][k] = fir (WNSignal[j], OCPMCoeffBuff[j][k], OCPMAuxState[j][k], OCPMLength);
		}
	}


#endif
	return 0;
}






int32_t WN_Gen(void) {


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		WNSignal[j]= randBuff[j] *OCPMWNGain[j]; //*30

	}

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint32_t i = 0; i < WNSignalBuffLength; i++) {
		WNSignalBuff[j][i] = WNSignalBuff[j][i+1];
	}

	WNSignalBuff[j][WNSignalBuffLength-1] = WNSignal[j]; //*30
	}


	return 0;
}




#ifdef OFPMFilter
int32_t ANCALG_1(void) {

	OFPMErrorSignal= uncorruptedRefSignal-OFPMErrorOutputBuff;
	powerOFPMErrorSignal = forgettingFactorOFPM*powerOFPMErrorSignal + (1 - forgettingFactorOFPM)*OFPMErrorSignal*OFPMErrorSignal;

	return 0;
}

int32_t ANCALG_2(void) {




	powerUncorruptedRefSignal = forgettingFactorOFPM*powerUncorruptedRefSignal + (1 - forgettingFactorOFPM)*uncorruptedRefSignal*uncorruptedRefSignal;

	OFPMPowerRatio = powerOFPMErrorSignal/(0.000000000000000001+ powerUncorruptedRefSignal);

	stepSizeF = constrain((OFPMPowerRatio*stepSizeFMin + (1-OFPMPowerRatio)*stepSizeFMax), stepSizeFMin, stepSizeFMax);



	return 0;
}

#endif

int32_t ANCALG_3(void) {


#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			powerOCPMWNSignal[j] =
					forgettingFactorOCPM
							* powerOCPMWNSignal[j]+ (1.0 - forgettingFactorOCPM)* WNSignal[j]* WNSignal[j];
		}

	return 0;
}


int32_t ANCALG_4(void) {
	ADI_FIR_RESULT res;
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			float OCPMAuxSumTemp = 0;

			for (uint8_t j = 0; j < numControlSignal; j++) {
				OCPMAuxSumTemp += OCPMAuxOutputBuff[j][k];
			}

			uncorruptedErrorSignal[k] = errorSignal[k] - OCPMAuxSumTemp; //-OCPMRefSumTemp;

		}

	return 0;
}


int32_t ANCALG_5(void) {

		//INDIRECT ERROR SIGNAL

#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
					indirectErrorSignal[j][k] = uncorruptedErrorSignal[k] + OCPMAuxOutputBuff[j][k];
			}
		}


		//	power of uncorruptedErrorSignal
#pragma vector_for(numErrorSignal)
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			powerUncorruptedErrorSignal[k] =
					forgettingFactorOCPM
							* powerUncorruptedErrorSignal[k] + (1.0 - forgettingFactorOCPM) * uncorruptedErrorSignal[k] * uncorruptedErrorSignal[k];
		}

		//power of indirectErrorSignal
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerIndirectErrorSignal[j][k] =
						forgettingFactorOCPM
								* powerIndirectErrorSignal[j][k] + (1.0 - forgettingFactorOCPM) *indirectErrorSignal[j][k] * indirectErrorSignal[j][k];
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
										+ 0.000000000000000001)), 0.000000000000000001,
				stepSizeSMax);
			}
		}



		//OCPMWNGain
		float powerUncorruptedErrorSignalSum = 0;


#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			powerUncorruptedErrorSignalSum += powerUncorruptedErrorSignal[k];
		}
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			float powerIndirectErrorSignalSum = 0;
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			powerIndirectErrorSignalSum += powerIndirectErrorSignal[j][k];
		}

			OCPMWNGain[j] = constrain(
					(powerUncorruptedErrorSignalSum / (powerIndirectErrorSignalSum + 0.000000000001)), 0,100);
		}



	return 0;
}




#ifdef OFPMFilter

int32_t OFPMWeightUpdate(void) {
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		float OFPMCoeffNSum =0;
#pragma vector_for
		for (uint32_t i = 0; i < OFPMLength; i++) {
				OFPMCoeffNSum += WNSignalBuff[j][i]*WNSignalBuff[j][i];
		}
#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMCoeffBuff[j][i] = OFPMCoeffBuff[j][i] * (1-stepSizeF*OFPMLeak) + stepSizeF * OFPMErrorSignal * WNSignalBuff[j][i]/OFPMCoeffNSum;
		}

	}

	return 0;
}




int32_t OFPMErrorWeightUpdate(void) {
		float OFPMErrorCoeffNSum =0;
#pragma vector_for
		for (uint32_t i = 0; i < OFPMErrorLength; i++) {
				OFPMErrorCoeffNSum += controlOutputBuff_OFPMError[i]*controlOutputBuff_OFPMError[i] ;
		}

#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMErrorCoeffBuff[i] = OFPMErrorCoeffBuff[i] * (1-stepSizeH*OFPMErrorLeak) + stepSizeH * OFPMErrorSignal * controlOutputBuff_OFPMError[i]/OFPMErrorCoeffNSum;
		}
	return 0;

}
#endif




int32_t ControlWeightUpdate(void) {
	//Control Coeff
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {

		float controlCoeffNSum =0;
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
				for (uint32_t i = 0; i < OCPMLength; i++) {
				controlCoeffNSum += OCPMRefOutputBuff[j][k][i]*OCPMRefOutputBuff[j][k][i];
				}
			}

#pragma vector_for
		for (int32_t i = 0; i < OCPMLength ; i++) {
			float controlCoeffSum =0;
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				controlCoeffSum += OCPMRefOutputBuff[j][k][i]*uncorruptedErrorSignal[k];
			}
			controlCoeffBuff[j][i] =	controlCoeffBuff[j][i] * (1-stepSizeW*controlLeak) - stepSizeW *controlCoeffSum/controlCoeffNSum;
		}
	}
	return 0;
}

int32_t OCPMWeightUpdate(void) {
#ifdef OCPMOnFIRA
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
#endif
	//OCPM Coeff

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {

float OCPMNsum=0;
#pragma vector_for
				for (uint32_t i = 0; i < OCPMLength; i++) {
					OCPMNsum += WNSignalBuff[j][i]* WNSignalBuff[j][i];
				}

#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
			for (int32_t i = 0; i < OCPMLength ; i++) {


				OCPMCoeffBuff[j][k][i] = OCPMCoeffBuff[j][k][i] * (1-OCPMLeak*stepSizeS[j][k]) + stepSizeS[j][k]*WNSignalBuff[j][i]*uncorruptedErrorSignal[k]/OCPMNsum;

			}

			}

	}

	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {

#ifdef OCPMOnFIRA
			adi_fir_SetChannelCoefficientBuffer(hChannelOCPMRef[j][k],
					(void*)&OCPMCoeffBuff[j][k],(void*)&OCPMCoeffBuff[j][k], 1);
			adi_fir_SetChannelCoefficientBuffer(hChannelOCPMAux[j][k],
					(void*)&OCPMCoeffBuff[j][k], (void*)&OCPMCoeffBuff[j][k], 1);
#endif
		}
	}
	return 0;
}
