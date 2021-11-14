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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "adi_initialize.h"
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
int32_t *pDst;
int32_t *pADCBuffer;
/* ADC buffer pointer */
volatile void *pGetADC = NULL;
 void *pADC = NULL;

/* ADC/DAC buffer pointer */
volatile void *pGetDAC = NULL;
 void *pDAC = NULL;



volatile bool bEnableOutput = true;
volatile bool bEnableOCPMWeightUpdate = true;
static volatile bool bMemCopyInProgress = false;



//*****************************************************************************
uint8_t DacStarted = 0u;

volatile int32_t DacCount = 0;
volatile int32_t AdcCount = 0;
bool OCPMUpdate = true;
#pragma alignment_region (4)


/* ----------------------------   FIR Configuration ------------------------------------------- */
//int32_t test[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS]={0};

//float errorSignal_temp[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
/* Channel Configurations parameters */
//Max FIR channels = 32

float controlOutputBuff[numControlSignal][1] = { 0 };
//float controlOutputBuff_OFPMError[OFPMErrorLength] = { 0 };

float pm  controlCoeffBuff[numControlSignal][controlLength]={0};     // coeffs array must be initialized and in PM memory
float pm  controlCoeffBuff_temp[numControlSignal][controlLength]={0};
bool controlCoeffLimit[numControlSignal] = {false, false};
float controlState[numControlSignal][controlLength+1]={0};
//float controlOutput[numControlSignal]={0};



 float refInputBuff[refWindowSize] = {0};
 float refOutputBuff[refWindowSize] = {0};
 float refInputBuffB[refWindowSizeB] = {0};
 float refOutputBuffB[refWindowSizeB] = {0};
 float refInputBuffC[refWindowSizeC] = {0};
 float refOutputBuffC[refWindowSizeC] = {0};
float pm  refCoeffBuff[refLength]={
#include "data/lowpass_4000_8000_reversed.dat"
};
float pm  refCoeffBuffB[refLengthB]={
#include "data/lowpass_4000_8000B_reversed.dat"
};
float pm  refCoeffBuffC[refLengthC]={
#include "data/lowpass_4000_8000C_reversed.dat"
};

float refState[refLength+1]={0};
float refStateB[refLengthB+1]={0};

float refStateC[refLengthC+1]={0};

float refOutputBuffD[refWindowSizeD] ={0};
float refOutputBuffT[124] ={0};

float errorSignal[NUM_ADAU1979_CHANNELS-1][refWindowSize] = { 0 };
float errorSignalO[numErrorSignal][refWindowSize] = { 0 };
float errorSignalB[numErrorSignal][refWindowSizeB] = { 0 };
float errorSignalOB[numErrorSignal][refWindowSizeB] = { 0 };
float errorSignalC[numErrorSignal][refWindowSizeC] = { 0 };
float errorSignalOC[numErrorSignal][refWindowSizeC] = { 0 };
float errorSignalOD[numErrorSignal][refWindowSizeD] = { 0 };


float dm errorState[numErrorSignal][refLength+1]={0};
float dm errorStateB[numErrorSignal][refLengthC+1]={0};
float dm errorStateC[numErrorSignal][refLengthC+1]={0};


float pm  OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength]={0};
float dm OCPMRefState[numControlSignal][numErrorSignal][OCPMLength+1]={0};
float dm OCPMAuxState[numControlSignal][numErrorSignal][OCPMLength+1]={0};
float dm OCPMRefOutput[numControlSignal][numErrorSignal][1]={0};
float dm  OCPMRefOutputBuff[numControlSignal][numErrorSignal][controlLength]={0};
float dm OCPMAuxOutputBuff[numControlSignal][numErrorSignal][1]={0};


float dm uncorruptedErrorSignal[numErrorSignal] = { 0 };
float OCPMWNGain[numControlSignal] = { 0 };
float powerOCPMWNSignal[numControlSignal] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal]={0};
float powerIndirectErrorSignal[numControlSignal][numErrorSignal] = { 0 };
float powerUncorruptedErrorSignal[numErrorSignal] = { 0 };
float forgettingFactorOCPM = 0.6;

float stepSizeW = 0;
float stepSizeSMin = 0;
float stepSizeSMax = 0;
float stepSizeS[numControlSignal][numErrorSignal] = { 0 };
#ifdef OFPMFilter
float powerOFPMErrorSignal = 1.0;
float OFPMPowerRatio = 1;
float forgettingFactorOFPM = 0.6;
float stepSizeH = 0;
float stepSizeHMin = 0.000001; //0.00001;
float stepSizeHMax = 0.1;//0.0001;
float stepSizeFMin = 0.000001;//0.00001;
float stepSizeFMax = 0.1;//0.0001;
float stepSizeF = 0;




float  OFPMInputBuffJ0[1]= {0};
float OFPMState0[OFPMLength+1]={0};
float pm  OFPMCoeffBuffJ0[OFPMLength]= {0};
float OFPMOutputBuffJ0[1]={0};


float  OFPMInputBuffJ1[1]= {0};
float OFPMState1[OFPMLength+1]={0};
float pm  OFPMCoeffBuffJ1[OFPMLength]= {0};
float OFPMOutputBuffJ1[1]={0};

float pm  OFPMErrorCoeffBuff[OFPMErrorLength]= {0};
float OFPMErrorState[OFPMErrorLength+1]={0};
float OFPMErrorOutputBuff[1]={0};


float powerUncorruptedRefSignal = 1.0;
float uncorruptedRefSignal[1] = {0};
float OFPMErrorSignal=0;
#endif
uint32_t WNCount = 0;
float randBuff[WNLength] = {0};
float WNSignal[numControlSignal][1] = {0};
float WNSignalBuff[numControlSignal][WNSignalBuffLength] = {0};

float  OutputSignalCoeffBuffC_temp[OutputSignalLengthC]={
#include "data/lowpass_4000_8000C.dat"
};
float pm  OutputSignalCoeffBuffC[OutputSignalLengthC]={0};
float dm OutputSignalInputBuffC[numControlSignal][refWindowSizeD] = {0};
float dm OutputSignalStateC[numControlSignal][OutputSignalLengthC+1]={0};



float  OutputSignalCoeffBuffB_temp[OutputSignalLengthB]={
#include "data/lowpass_4000_8000B.dat"
};
float pm  OutputSignalCoeffBuffB[OutputSignalLengthB]={0};
float dm OutputSignalInputBuffB[numControlSignal][refWindowSizeC] = {0};
float dm OutputSignalStateB[numControlSignal][OutputSignalLengthB+1]={0};

float  OutputSignalCoeffBuff_temp[OutputSignalLength]={
#include "data/lowpass_4000_8000.dat"
};
float pm OutputSignalCoeffBuff[OutputSignalLength]={0};
float dm OutputSignalInputBuff[numControlSignal][refWindowSizeB] = {0};
float dm OutputSignalState[numControlSignal][OutputSignalLength+1]={0};
float dm OutputSignalOutputBuff[numControlSignal][refWindowSize] ={0};

int32_t dm OutputSignalInt32[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] ={0};
#pragma alignment_region_end







/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;

volatile bool DACFlag = false;

static bool bError = false;
static uint32_t count=0;

volatile bool bEvent = false;


volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;


//TRNG
volatile bool TRNGFlag = false;

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";


 //uint32_t iida = 0;




void aluFLTOIHandler(uint32_t iid, void* handlerArg) {
	printf("%d\n", iid);
	//printf("ERR");
	//iida = iid;
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
	//bMemCopyInProgress=true;
	//MemDma0Copy1D((void *) &refInputBuffB[0], (void *) &refInputBuff[0], 4, refLength);
	//while(bMemCopyInProgress);
	//memcpy(&refInputBuffB[0] , &refInputBuff[0], refLength*4);
	//refInputBuffB[0] = refInputBuff[0];
	/*
	 firf (refInputBuff,
	          refOutputBuff,
	            refCoeffBuff,
	            refState,
				refWindowSize,
	            refLength);

*/
/*
	 fir_decima (refInputBuff,
	          refInputBuffB,
	            refCoeffBuff,
	            refState,
				refWindowSizeB,
	            refLength, DECIMATION_FACTOR_A);
*/
	 firf (refInputBuff,
	          refOutputBuff,
	            refCoeffBuff,
	            refState,
				refWindowSize,
	            refLength);

	for(uint32_t i =0; i< refWindowSizeB; i ++){

		refInputBuffB[i]= refOutputBuff[i*DECIMATION_FACTOR_A];
	}


	 firf (refInputBuffB,
	          refOutputBuffB,
	            refCoeffBuffB,
	            refStateB,
				refWindowSizeB,
	            refLengthB);


	for(uint32_t i =0; i< refWindowSizeC; i ++){

		refInputBuffC[i]= refOutputBuffB[i*DECIMATION_FACTOR_B];
	}

	fir (refInputBuffC,
	          refOutputBuffC,
	            refCoeffBuffC,
	            refStateC,
				refWindowSizeC,
	            refLengthC);
	refOutputBuffD[0]= refOutputBuffC[0];
/*

	 for(uint8_t k =0; k < numErrorSignal; k++){
		 fir_decima (errorSignal[k],
	          errorSignalB[k],
	            refCoeffBuff,
	            errorState[k],
	            refWindowSizeB,
	            refLength, DECIMATION_FACTOR_A);

	 fir_decima (errorSignalB[k],
	          errorSignalC[k],
	            refCoeffBuffB,
	            errorStateB[k],
	            refWindowSizeC,
	            refLengthB, DECIMATION_FACTOR_B);

	 fir_decima (errorSignalC[k],
	          errorOutputBuffC[k],
	            refCoeffBuffC,
	            errorStateC[k],
	            refWindowSizeD,
	            refLengthC, DECIMATION_FACTOR_C);

	 }

*/
	 for(uint8_t k =0; k < numErrorSignal; k++){
	firf (errorSignal[k],
			 errorSignalO[k],
            refCoeffBuff,
            errorState[k],
            refWindowSize,
            refLength);


	for(uint32_t i =0; i< refWindowSizeB; i ++){

		errorSignalB[k][i]= errorSignalO[k][i*DECIMATION_FACTOR_A];
	}


 firf (errorSignalB[k],
		 errorSignalOB[k],
            refCoeffBuffB,
            errorStateB[k],
            refWindowSizeB,
            refLengthB);


	for(uint32_t i =0; i< refWindowSizeC; i ++){

		errorSignalC[k][i]= errorSignalOB[k][i*DECIMATION_FACTOR_B];
	}


 fir (errorSignalC[k],
		 errorSignalOC[k],
            refCoeffBuffC,
            errorStateC[k],
            refWindowSizeC,
            refLengthC);


 errorSignalOD[k][0]= errorSignalOC[k][0];
 uncorruptedErrorSignal[k] = errorSignalOD[k][0];
	 }



return 0;
}


uint8_t OFPMFIR() {
	/*
	uncorruptedRefSignal[0] =   refOutputBuffC[0];
for(uint8_t j =0; j < numControlSignal; j++){
	OFPMInputBuff[j][0]=controlOutputBuff[j][0]+WNSignal[j][0];
	 fir (OFPMInputBuff[j], OFPMOutputBuff[j], OFPMCoeffBuff[j], OFPMState[j], OFPMWindowSize, OFPMLength);
	uncorruptedRefSignal[0] -=  OFPMOutputBuff[j][0];
}

//

	uncorruptedRefSignal[0] =   refOutputBuffD[0];
	OFPMInputBuffJ0[0]=controlOutputBuffJ0[0]+WNSignalJ0[0];
	fir (OFPMInputBuffJ0, OFPMOutputBuffJ0, OFPMCoeffBuffJ0, OFPMState0, OFPMWindowSize, OFPMLength);
	fir (OFPMInputBuffJ1, OFPMOutputBuffJ1, OFPMCoeffBuffJ1, OFPMState1, OFPMWindowSize, OFPMLength);
	uncorruptedRefSignal[0] -=  OFPMOutputBuffJ0[0];
	uncorruptedRefSignal[0] -=  OFPMOutputBuffJ1[0];
*/
	return 0;
}

uint8_t OFPMErrorFIR() {
/*
	fir (controlOutputBuffJ0, OFPMErrorOutputBuff, OFPMErrorCoeffBuff, OFPMErrorState, OFPMWindowSize, OFPMErrorLength);
	//compiler circular buffer optimisation
    for(uint32_t i = 0; i < OFPMLength -1; i++){
    	controlOutputBuff_OFPMError[i] = controlOutputBuff_OFPMError[i + 1];
    }
    controlOutputBuff_OFPMError[OFPMLength-1] = controlOutputBuffJ0[0]  ;//uncorruptedRefSignal[0] - OFPMErrorOutputBuff[0];

	*/
	return 0;
}


uint8_t ControlFIR() {

	for(uint8_t j =0; j < numControlSignal; j++){
		//fir (uncorruptedRefSignal, controlOutputBuff[j], controlCoeffBuff[j], controlState[j], controlWindowSize, controlLength);
		fir (refOutputBuffD, controlOutputBuff[j], controlCoeffBuff[j], controlState[j], controlWindowSize, controlLength);
	}

	return 0;
}




int32_t OCPMRefFIR(void) {

	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){

		 fir (refOutputBuffD,OCPMRefOutput[j][k], OCPMCoeffBuff[j][k], OCPMRefState[j][k], OCPMWindowSize, OCPMLength);
		}
	}
	#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
	for (uint32_t i = 0; i < controlLength-1; i++) {
		OCPMRefOutputBuff[j][k][i] = OCPMRefOutputBuff[j][k][i+1];
	}

	OCPMRefOutputBuff[j][k][controlLength-1] = OCPMRefOutput[j][k][0];
		}
	}


	return 0;
}



int32_t OCPMAuxFIR(void) {

	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){
		fir (WNSignal[j],OCPMAuxOutputBuff[j][k], OCPMCoeffBuff[j][k], OCPMAuxState[j][k], OCPMWindowSize, OCPMLength);
		}
	}

	return 0;
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
	//AdcCount++;
		break;
	default:
		bEventError = true;
		printf("errA");
		break;
	}
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

		//DACFlag = true;

		if (ANCInProgress){
		 ANCERR = true;
		 printf("anc err\n");
		 }


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
			Acknowledge_Interrupt(iTemp);
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


int32_t FIR_init() {
/*
	//FIR stuff
	ADI_FIR_RESULT res;
	for (int32_t i = 0; i < OCPMLength; i++) {
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
				 OCPMCoeffBuff[j][k][i]=0.1;
			}
faccount	}

	for (int32_t i = 0; i < OFPMLength; i++) {
		OFPMErrorCoeffBuff[i] =0.1;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		OFPMCoeffBuff[j][i]=0.1;
			}
		}


	for (int32_t i = 0; i < controlLength; i++) {
	for (uint8_t j = 0; j < numControlSignal; j++) {
		controlCoeffBuff[j][i]=0.1;
			}
		}





	for (int32_t i = 0; i < OCPMLength+1; i++) {
		 OFPMErrorState[i]=0;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		for (uint8_t k = 0; k < numErrorSignal; k++) {
				 OCPMRefState[j][k][i]=0;
				 OCPMAuxState[j][k][i]=0;
			}
		}

	for (uint8_t k = 0; k < numErrorSignal; k++) {
		 OFPMState[k][i]=0;
	}
		}


	for (int32_t i = 0; i < refLength+1; i++) {
				 refState[i]=0;


		}
	for (uint8_t k = 0; k < numErrorSignal; k++) {
	for (int32_t i = 0; i < refLength+1; i++) {
				 errorState[k][i]=0;

		}

	}
	for (uint8_t j = 0; j < numControlSignal; j++) {
	for (int32_t i = 0; i < controlLength+1; i++) {
		 controlState[j][i]=0;
	}
	for (int32_t i = 0; i < refLength+1; i++) {
	 OutputSignalState[j][i]=0;
	}
	}
*/
	/*
	for (int32_t i = 0; i < OCPMLength; i++) {
				 OCPMCoeffBuff00[i]=0.1;
				 OCPMCoeffBuff01[i]=0.1;
				 OCPMCoeffBuff10[i]=0.1;
				 OCPMCoeffBuff11[i]=0.1;
			}


	for (int32_t i = 0; i < OFPMLength; i++) {
		OFPMErrorCoeffBuff[i] =0.1;
		OFPMCoeffBuffJ0[i]=0.1;
		OFPMCoeffBuffJ1[i]=0.1;
		}


	for (int32_t i = 0; i < controlLength; i++) {
		controlCoeffBuffJ0[i]=0.1;
		controlCoeffBuffJ1[i]=0.1;
		}




	for (int32_t i = 0; i < OCPMLength+1; i++) {

		 OCPMRefState00[i]=0;
		 OCPMAuxState00[i]=0;
		 OCPMRefState01[i]=0;
		 OCPMAuxState01[i]=0;
		 OCPMRefState10[i]=0;
		 OCPMAuxState10[i]=0;
		 OCPMRefState11[i]=0;
		 OCPMAuxState11[i]=0;
	}

	for (int32_t i = 0; i < OFPMLength+1; i++) {
		 OFPMErrorState[i]=0;
		 OFPMState0[i]=0;
		 OFPMState1[i]=0;
	}
*/


	for (int32_t i = 0; i < refLength+1; i++) {
				 refState[i]=0;
				 refStateB[i]=0;
				 refStateC[i]=0;
		}

for (uint8_t k = 0; k < numErrorSignal; k++) {
	for (int32_t i = 0; i < refLength+1; i++) {
				 errorState[k][i]=0;
				 errorStateB[k][i]=0;
				 errorStateC[k][i]=0;
		}

}
for (uint8_t j = 0; j < numControlSignal; j++) {
	for (int32_t i = 0; i < controlLength+1; i++) {
		 controlState[j][i]=0;
	}
	for (int32_t i = 0; i < refLength+1; i++) {
	 OutputSignalState[j][i]=0;
	}
}


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
	}

	stepSizeSMax= 0.01;
	//stepSizeFMax= 0.00001;
	//stepSizeHMax= 0.00001;

	stepSizeW   = 0.0000000001;	// 0.000005;
	stepSizeSMin= 0.000000001;
	//stepSizeFMin= 0.000001;
	//stepSizeHMin= 0.000001;
	//stepSizeF = stepSizeFMin;
	//stepSizeH = stepSizeHMin;

	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (uint8_t k = 0; k < numErrorSignal; k++) {

			stepSizeS[j][k] = stepSizeSMin;
		}
	}
	//stepSizeH = 0.0001;

	/* Initialize the delay line */

	/* Transform the normal order coefficients from a filter design
	   tool into coefficients for the fir_interp function */

	int32_t x = OutputSignalLength;
	for (int32_t np = 1; np <= OutputSignalPolyPhases; np++){
	    for (int32_t nc = 1; nc <= (OutputSignalCoeffsPerPoly); nc++){
	    	OutputSignalCoeffBuff[--x] = OutputSignalCoeffBuff_temp[(nc * OutputSignalPolyPhases) - np];
	    }

	}


	int32_t x1 = OutputSignalLengthB;
	for (int32_t np = 1; np <= OutputSignalPolyPhasesB; np++){
	    for (int32_t nc = 1; nc <= (OutputSignalCoeffsPerPolyB); nc++){
	    	OutputSignalCoeffBuffB[--x1] = OutputSignalCoeffBuffB_temp[(nc * OutputSignalPolyPhasesB) - np];
	    }

	}


	int32_t x2 = OutputSignalLengthC;
	for (int32_t np = 1; np <= OutputSignalPolyPhasesC; np++){
	    for (int32_t nc = 1; nc <= (OutputSignalCoeffsPerPolyC); nc++){
	    	OutputSignalCoeffBuffC[--x2] = OutputSignalCoeffBuffC_temp[(nc * OutputSignalPolyPhasesC) - np];
	    }

	}
	return 0;

}

static uint32_t core_work = 0u;
static void do_core_work(uint32_t cycles);
#ifndef __ADSPARM__
/* Optimize this function for speed so that each iteration of the loop
** takes 1 cycle.
*/
#pragma optimize_for_speed
#endif

static void do_core_work(uint32_t cycles)
{
#ifdef __ADSPARM__
  asm volatile("  movs r3, #0\n"
               ".do_core_work_loop%=:\n"
               "  nop\n"
               "  adds r3, r3, #1\n"
               "  cmp r3, %0\n"
               "  bne .do_core_work_loop%=\n" : : "r"(cycles/4u) : "r3" );
#else
  uint32_t i;
  for (i = 0u; i < cycles; i++)
  {
    NOP();
  }
#endif
  core_work += cycles;
}
volatile bool allz = false;


int main(void) {
	uint32_t DestAddress;

	int8_t firResult = 0;
	bool bExit;
	uint32_t Result = 0u;
	//uint32_t i;
	bExit = false;


	adi_initComponents(); /* auto-generated code */
	//ALUSAT
	//sysreg_bit_clr(sysreg_MMASK,0x2000);

	//sysreg_bit_set(sysreg_MODE1,0x2000);
	//SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;

	adi_int_InstallHandler(ADI_CID_FLTOI, aluFLTOIHandler, NULL, true);
	//adi_int_InstallHandler (ADI_CID_FLTUI, aluFLTOIHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FIXI, aluHandler, NULL, true) ;

	if (Result == 0u) {
		//Result = DMAInit();
	}



	// Initialize ADAU1962a
	if (Result == 0u) {
		Result = Adau1962aInit();
	}
	// Initialize ADAU1979
	if (Result == 0u) {
		Result = Adau1979Init();
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


	ProcessBuffers();



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

		if (pGetADC!=NULL) {
			pADC = (void *) pGetADC;
			Adau1979DoneWithBuffer((void *) pADC);
			pGetADC = NULL;
		}

		if (pGetDAC!=NULL) {
			pDAC = (void *) pGetDAC;
			Adau1962aDoneWithBuffer((void *) pDAC);
			pGetDAC = NULL;
		}


				if ((pDAC!=NULL) && (pADC != NULL)) {

					ProcessBuffers();

					}




	}

	if (!bError) {
		printf("All done\n");
	} else {
		printf("Audio error\n");
	}

	return 0;
}




void reverseArrayf(float arr[], uint32_t arrLength) {
#pragma vector_for
	for (uint32_t i = 0; i < (arrLength / 2); i++) {
		float temp = arr[i];
		arr[i] = arr[arrLength - i - 1];
		arr[arrLength - i - 1] = temp;
	}
}





int32_t WN_Gen(void) {




	 #pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		WNSignal[j][0]= randBuff[j] *OCPMWNGain[j];

	}

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint32_t i = 0; i < WNSignalBuffLength-1; i++) {
		WNSignalBuff[j][i] = WNSignalBuff[j][i+1];
	}

	WNSignalBuff[j][WNSignalBuffLength-1] = WNSignal[j][0];
	}



	return 0;
}




#ifdef OFPMFilter
int32_t ANCALG_1(void) {

	OFPMErrorSignal= uncorruptedRefSignal[0]-OFPMErrorOutputBuff[0];
	powerOFPMErrorSignal = forgettingFactorOFPM*powerOFPMErrorSignal + (1 - forgettingFactorOFPM)*OFPMErrorSignal*OFPMErrorSignal;
	if(isnanf(powerOFPMErrorSignal)){
		powerOFPMErrorSignal = 1.0;
	}
	return 0;
}

int32_t ANCALG_2(void) {




	powerUncorruptedRefSignal = forgettingFactorOFPM*powerUncorruptedRefSignal + (1 - forgettingFactorOFPM)*uncorruptedRefSignal[0]*uncorruptedRefSignal[0];
	if(isnanf(powerUncorruptedRefSignal)){
		powerUncorruptedRefSignal = 1.0;
	}
	OFPMPowerRatio = powerOFPMErrorSignal/(0.000000000000000001+ powerUncorruptedRefSignal);



	stepSizeF = constrain((OFPMPowerRatio*stepSizeFMin + (1-OFPMPowerRatio)*stepSizeFMax), stepSizeFMin, stepSizeFMax);

	if(isnanf(stepSizeF)){
		stepSizeF = stepSizeFMin;
	}

	return 0;
}

#endif

int32_t ANCALG_3(void) {


#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			powerOCPMWNSignal[j] =
					forgettingFactorOCPM
							* powerOCPMWNSignal[j]+ (1.0 - forgettingFactorOCPM)* WNSignal[j][0]* WNSignal[j][0];
		}

	return 0;
}


int32_t ANCALG_4(void) {

#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			for (uint8_t j = 0; j < numControlSignal; j++) {
				uncorruptedErrorSignal[k] -= OCPMAuxOutputBuff[j][k][0];
			}
		}

	return 0;
}


int32_t ANCALG_5(void) {

		//INDIRECT ERROR SIGNAL

#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
					indirectErrorSignal[j][k] = uncorruptedErrorSignal[k] + OCPMAuxOutputBuff[j][k][0];
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
				stepSizeS[j][k] =// fclipf(
						(stepSizeSMin * powerOCPMWNSignal[j]
								/ (powerIndirectErrorSignal[j][k]
										+ 0.000000000000000001))
										// stepSizeSMax)
		;
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

			OCPMWNGain[j] =
				powerUncorruptedErrorSignalSum / (powerIndirectErrorSignalSum + 0.000000000000000001);
		}


	return 0;
}




#ifdef OFPMFilter

int32_t OFPMWeightUpdate(void) {
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		/*
		float OFPMCoeffNSum =0;
#pragma vector_for
		for (uint32_t i = 0; i < OFPMLength; i++) {
				OFPMCoeffNSum += WNSignalBuff[j][i]*WNSignalBuff[j][i];
		}
		if (OFPMCoeffNSum ==0.0){
			OFPMCoeffNSum =1.0;
		}

#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMCoeffBuff[j][i] = OFPMCoeffBuff[j][i] * (1.0-stepSizeF*OFPMLeak) + stepSizeF * constrain(OFPMErrorSignal * WNSignalBuff[j][i],-1000,1000);
		}

	}
		*/

		/*
		float OFPMCoeffNSumJ0 =0;
		float OFPMCoeffNSumJ1 =0;
#pragma vector_for
		for (uint32_t i = 0; i < OFPMLength; i++) {
				OFPMCoeffNSumJ0 += WNSignalBuffJ0[i]*WNSignalBuffJ0[i];
				OFPMCoeffNSumJ1 += WNSignalBuffJ1[i]*WNSignalBuffJ1[i];
		}
		if (OFPMCoeffNSumJ0 ==0.0){
			OFPMCoeffNSumJ0 =1.0;
		}
		if (OFPMCoeffNSumJ1 ==0.0){
			OFPMCoeffNSumJ1=1.0;
		}
*/

#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMCoeffBuffJ0[i] = OFPMCoeffBuffJ0[i] * (1.0-stepSizeF*OFPMLeak) + stepSizeF * constrain(OFPMErrorSignal * WNSignalBuffJ0[i],-1000,1000);
				OFPMCoeffBuffJ1[i] = OFPMCoeffBuffJ1[i] * (1.0-stepSizeF*OFPMLeak) + stepSizeF * constrain(OFPMErrorSignal * WNSignalBuffJ1[i],-1000,1000);
		}

	}
	return 0;
}




int32_t OFPMErrorWeightUpdate(void) {
	/*
		float OFPMErrorCoeffNSum =0;
#pragma vector_for
		for (uint32_t i = 0; i < OFPMErrorLength; i++) {
				OFPMErrorCoeffNSum += controlOutputBuff_OFPMError[i]*controlOutputBuff_OFPMError[i] ;
		}
		if (OFPMErrorCoeffNSum ==0.0){
			OFPMErrorCoeffNSum =1.0;
		}
*/
#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMErrorCoeffBuff[i] = OFPMErrorCoeffBuff[i] * (1.0-stepSizeH*OFPMErrorLeak) + stepSizeH * constrain(OFPMErrorSignal * controlOutputBuff_OFPMError[i], -1000,1000);
		}


	return 0;

}
#endif




int32_t ControlWeightUpdate(void) {
if(count >=32000){
	//Control Coeff

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {

		//controlCoeffLimit[j] = false;
		float controlCoeffNSum =0;
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
				for (uint32_t i = 0; i < controlLength; i++) {
				controlCoeffNSum += OCPMRefOutputBuff[j][k][i]*OCPMRefOutputBuff[j][k][i];
				}
			}



#pragma vector_for
		for (int32_t i = 0; i < controlLength ; i++) {
			float controlCoeffSum =0;

//N#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				controlCoeffSum += OCPMRefOutputBuff[j][k][i]*uncorruptedErrorSignal[k];
			}
			controlCoeffBuff[j][i] =	controlCoeffBuff[j][i] * (1-stepSizeW*controlLeak) - stepSizeW *controlCoeffSum /(controlCoeffNSum+0.0000000000000001);
		//	if (controlCoeffBuff_temp[j][i]>=0.03 || controlCoeffBuff_temp[j][i]<= -0.04){
		//		controlCoeffLimit[j] = true;
		//	}
		//	else {}
		}
	}
/*
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
if(!controlCoeffLimit[j]){

#pragma vector_for
		for (int32_t i = 0; i < controlLength ; i++) {

		controlCoeffBuff[j][i] = controlCoeffBuff_temp[j][i];

			}
			}
else{
#pragma vector_for
		for (int32_t i = 0; i < controlLength ; i++) {

		controlCoeffBuff[j][i] = controlCoeffBuff_temp[j][i]*0.9;

			}
}
}
*/
}
	return 0;
}

int32_t OCPMWeightUpdate(void) {
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


				OCPMCoeffBuff[j][k][i] = OCPMCoeffBuff[j][k][i] * (1.0-OCPMLeak*stepSizeS[j][k]) + stepSizeS[j][k]*WNSignalBuff[j][i]*uncorruptedErrorSignal[k]/(OCPMNsum+0.0000000000000001);
				//OCPMCoeffBuff[j][k][i] = OCPMCoeffBuff[j][k][i] + stepSizeS[j][k]*WNSignalBuff[j][i]*uncorruptedErrorSignal[k];
			}

			}

	}



	return 0;
}


void ProcessBuffers(){
	ANCInProgress = true;
		/*
		pADC = (void *) pGetADC;
		Adau1979DoneWithBuffer((void *) pADC);
		pDAC = (void *) pGetDAC;
		Adau1962aDoneWithBuffer((void *) pDAC);
		pGetDAC = NULL;

*/

			pADCBuffer = (int32_t *) pADC;

			pDst = (int32_t *) pDAC;

			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
				refInputBuff[i] = conv_float_by(*pADCBuffer++,-18);
				for(uint8_t k =0; k < NUM_ADAU1979_CHANNELS - 1; k ++){
				errorSignal[k][i] = conv_float_by(*pADCBuffer++, -18);
				}

			}


			RefFIR();
			while (!TRNGFlag)
				;

			TRNGFlag = false;
			WN_Gen();
			/*
			OFPMFIR();
			OFPMErrorFIR();
*/

			ControlFIR();

			OCPMRefFIR();
			OCPMAuxFIR();
/*
			ANCALG_1();
			ANCALG_2();
*/
			ANCALG_3();

			ANCALG_4();

			ANCALG_5();

/*
			OFPMErrorWeightUpdate();
			OFPMWeightUpdate();
			*/
			ControlWeightUpdate();
			OCPMWeightUpdate();
			limitSig();
			GenControlSignal();


			TRNG_ACK();


#pragma vector_for

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++){
	OutputSignalInt32[j][i] = conv_fix_by(OutputSignalOutputBuff[j][i],10);
	}
}



for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
	//TDM8 SHIFT <<8

	for (uint8_t j = 0; j < numControlSignal; j++) {
	*pDst++ = OutputSignalInt32[j][i] ;
}
	for (uint8_t j = numControlSignal; j < NUM_DAC_CHANNELS; j++) {
	*pDst++ =  0 ;
}


}


pDAC = NULL;
pADC = NULL;

if(count < 32000){
count +=1;
}
ANCInProgress = false;
}




uint8_t limitSig(){
	for (uint8_t j = 0; j < numControlSignal; j++) {
	if(fabsf(controlOutputBuff[j][0]> 400.0) ){
		//count+=1;
		controlOutputBuff[j][0]= (controlOutputBuff[j][0]*Amax/fabsf(controlOutputBuff[j][0]));
		for (int32_t i = 0; i < controlLength ; i++) {
		controlCoeffBuff[j][i]= (controlCoeffBuff[j][i]*Amax/fabsf(controlOutputBuff[j][0]))*0.99 ;
		}
	}
	}
		return 0 ;
}
uint8_t GenControlSignal() {

//N#pragma vector_for

	for (uint8_t j = 0; j < numControlSignal; j++) {
		OutputSignalInputBuffC[j][0]= controlOutputBuff[j][0]+WNSignal[j][0];
		fir_interp(OutputSignalInputBuffC[j],
				OutputSignalInputBuffB[j],
				OutputSignalCoeffBuffC,
				OutputSignalStateC[j],
				OutputSignalInputSizeC,
				OutputSignalCoeffsPerPolyC,
				OutputSignalInterpC);

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < OutputSignalInputSizeC; i++){
			OutputSignalInputBuffB[j][i] = OutputSignalInputBuffB[j][i] / OutputSignalInterpC;
		}
	}
		fir_interp(OutputSignalInputBuffB[j],
				OutputSignalInputBuff[j],
				OutputSignalCoeffBuffB,
				OutputSignalStateB[j],
				OutputSignalInputSizeB,
				OutputSignalCoeffsPerPolyB,
				OutputSignalInterpB);

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < OutputSignalInputSize; i++){
			OutputSignalInputBuff[j][i] = OutputSignalInputBuff[j][i] / OutputSignalInterpB;
		}
	}
		fir_interp(OutputSignalInputBuff[j],
				OutputSignalOutputBuff[j],
				OutputSignalCoeffBuff,
				OutputSignalState[j],
				OutputSignalInputSize,
				OutputSignalCoeffsPerPoly,
				OutputSignalInterp);


	}


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++){
			OutputSignalOutputBuff[j][i] = OutputSignalOutputBuff[j][i] / OutputSignalInterp;
		}
	}










	return 0;
}
