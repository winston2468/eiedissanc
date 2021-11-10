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

//volatile int32_t DacCount = 0;
bool OCPMUpdate = true;
#pragma alignment_region (4)


/* ----------------------------   FIR Configuration ------------------------------------------- */
//int32_t test[NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS]={0};

//float errorSignal_temp[numErrorSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
/* Channel Configurations parameters */
//Max FIR channels = 32

float controlOutputBuff[numControlSignal][1] = { 0 };
float controlOutputBuff_OFPMError[OFPMErrorLength] = { 0 };

float pm  controlCoeffBuff[numControlSignal][controlLength]={0};     /* coeffs array must be         */
                           /* initialized and in PM memory */
float controlState[numControlSignal][OCPMLength+1]={0};

float controlOutput[numControlSignal]={0};


 float dm errorSignal[NUM_ADAU1979_CHANNELS][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
 float  dm refInputBuff[refWindowSize] = {0};
float pm  refCoeffBuff[refLength]={
#include "data/lowpass_4000_8000_reversed.dat"
};
float dm refState[refLength+1]={0};
float dm refOutputBuff[refWindowSize] ={0};





float dm errorState[numErrorSignal][refLength+1]={0};
float dm errorOutputBuff[numErrorSignal][refWindowSize] ={0};


float dm controlOutputSignalInputBuff[numControlSignal][24] = {0};
float dm controlOutputSignalCoeffBuff_temp[controlOutputSignalLength]={
#include "data/lowpass_4000_8000.dat"
};

float pm  controlOutputSignalCoeffBuff[controlOutputSignalLength]={
0

};

float dm controlOutputSignalState[numControlSignal][controlOutputSignalLength+1]={0};
float dm controlOutputSignalOutputBuff[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] ={0};
float dm controlOutputSignal[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] ={0};
int32_t dm controlOutputSignalB[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] ={0};
float pm  OCPMCoeffBuff[numControlSignal][numErrorSignal][OCPMLength]={0};     /* coeffs array must be         */
                           /* initialized and in PM memory */
float dm OCPMRefState[numControlSignal][numErrorSignal][OCPMLength+1]={0};
float dm OCPMAuxState[numControlSignal][numErrorSignal][OCPMLength+1]={0};

float dm OCPMRefOutput[numControlSignal][numErrorSignal][1]={0};
float dm  OCPMRefOutputBuff[numControlSignal][numErrorSignal][controlLength]={0};
float dm OCPMAuxOutputBuff[numControlSignal][numErrorSignal][1]={0};




float dm uncorruptedErrorSignal[numErrorSignal] = { 0 };

float OCPMWNGain[numControlSignal] = { 0 };
float powerOCPMWNSignal[numControlSignal] = { 0 };
float powerResidualErrorSignal[numErrorSignal] = { 0 };
float indirectErrorSignal[numControlSignal][numErrorSignal]={0};
float powerIndirectErrorSignal[numControlSignal][numErrorSignal] = { 0 };
float powerUncorruptedErrorSignal[numErrorSignal] = { 0 };
float powerUncorruptedRefSignal = 1.0;
float uncorruptedRefSignal[1] = {0};
float OFPMErrorSignal=0;

float forgettingFactorOCPM = 0.6;

float stepSizeW = 0;
float stepSizeSMin = 0.000001;
float stepSizeSMax = 0.1;
float stepSizeS[numControlSignal][numErrorSignal] = { 0 };
#ifdef OFPMFilter
float powerOFPMErrorSignal = 1.0;
float OFPMPowerRatio = 1;
float forgettingFactorOFPM = 0.9;
float stepSizeH = 0;
float stepSizeHMin = 0.000001; //0.00001;
float stepSizeHMax = 0.1;//0.0001;
float stepSizeFMin = 0.000001;//0.00001;
float stepSizeFMax = 0.1;//0.0001;
float stepSizeF = 0;



float  OFPMInputBuff[numControlSignal][1]= {0};

float pm  OFPMCoeffBuff[numControlSignal][OFPMLength]= {0};
float pm  OFPMErrorCoeffBuff[OFPMErrorLength]= {0};

float OFPMState[numControlSignal][OFPMLength+1]={0};
float OFPMErrorState[OFPMErrorLength+1]={0};

float OFPMOutputBuff[numControlSignal][1]={0};
float OFPMErrorOutputBuff[1]={0};
#endif



#pragma alignment_region_end


//TRNG
volatile bool TRNGFlag = false;
#pragma align(4)
float randBuff[WNLength] = {0};
#pragma align(4)
float WNSignal[numControlSignal][1] = {0};
#pragma align(4)
float WNSignalBuff[numControlSignal][WNSignalBuffLength] = {0};
#pragma align(4)
uint32_t WNCount = 0;




/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;

volatile bool DACFlag = false;

static bool bError = false;
static uint32_t count;

volatile bool bEvent = false;


volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;



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
	 fir (refInputBuff,
	          refOutputBuff,
	            refCoeffBuff,
	            refState,
	            refWindowSize,
	            refLength);
	 /*
	 refInputBuffB[0] = refInputBuff[0];
	 fir (refInputBuffB,
	          refOutputBuffB,
	            refCoeffBuffB,
	            refStateB,
	            refWindowSizeB,
	            refLengthB);
	            */
	 for(uint8_t k =0; k < numErrorSignal; k++){
	 firf (errorSignal[k],
	          errorOutputBuff[k],
	            refCoeffBuff,
	            errorState[k],
	            refWindowSize,
	            refLength);
	 }

	return 0;
}




uint8_t OFPMFIR() {
	uncorruptedRefSignal[0] =   constrain(refOutputBuff[0], constrain_min, constrain_max);
for(uint8_t j =0; j < numControlSignal; j++){
	OFPMInputBuff[j][0]=controlOutputBuff[j][0]+WNSignal[j][0];
	 fir (OFPMInputBuff[j], OFPMOutputBuff[j], OFPMCoeffBuff[j], OFPMState[j], OFPMWindowSize, OFPMLength);
	uncorruptedRefSignal[0] -= constrain( OFPMOutputBuff[j][0], constrain_min, constrain_max);
}
	return 0;
}

uint8_t OFPMErrorFIR() {

	fir (controlOutputBuff[0], OFPMErrorOutputBuff, OFPMErrorCoeffBuff, OFPMErrorState, OFPMWindowSize, OFPMErrorLength);
	OFPMErrorOutputBuff[0] = constrain(OFPMErrorOutputBuff[0] , constrain_min, constrain_max);
	//compiler circular buffer optimisation
    for(uint32_t i = 0; i < OFPMLength ; i++){
    	*(controlOutputBuff_OFPMError + i) = *(controlOutputBuff_OFPMError + i + 1);
    }
    controlOutputBuff_OFPMError[OFPMLength-1] =controlOutputBuff[0][0] - OFPMErrorOutputBuff[0];
	return 0;
}


uint8_t ControlFIR() {

	for(uint8_t j =0; j < numControlSignal; j++){
		fir (uncorruptedRefSignal, controlOutputBuff[j], controlCoeffBuff[j], controlState[j], controlWindowSize, controlLength);
	}

	return 0;
}




int32_t OCPMRefFIR(void) {

	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){
		 fir (uncorruptedRefSignal,OCPMRefOutput[j][k], OCPMCoeffBuff[j][k], OCPMRefState[j][k], OCPMWindowSize, OCPMLength);
		 OCPMRefOutput[j][k][0]=constrain(OCPMRefOutput[j][k][0], constrain_min, constrain_max);
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

	OCPMRefOutputBuff[j][k][controlLength-1] = constrain(OCPMRefOutput[j][k][0], constrain_min, constrain_max);
		}
	}



	return 0;
}



int32_t OCPMAuxFIR(void) {
	for(uint8_t j =0; j < numControlSignal; j++){
		for(uint8_t k =0; k < numErrorSignal; k++){
		fir (WNSignal[j],OCPMAuxOutputBuff[j][k], OCPMCoeffBuff[j][k], OCPMAuxState[j][k], OCPMWindowSize, OCPMLength);
		OCPMAuxOutputBuff[j][k][0] = constrain(OCPMAuxOutputBuff[j][k][0], constrain_min,constrain_max);
		}
	}

	return 0;
}






uint8_t GenControlSignal() {

//N#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		controlOutputSignalInputBuff[j][0]=constrain(controlOutputBuff[j][0]+WNSignal[j][0], constrain_min, constrain_max);
		//fir_interp(controlOutputSignalInputBuff[j], controlOutputSignalOutputBuff[j],controlOutputSignalCoeffBuff, controlOutputSignalState[j], controlOutputSignalInputSize, controlOutputSignalCoeffsPerPoly, controlOutputSignalWindowSize);
		fir(controlOutputSignalInputBuff[j], controlOutputSignalOutputBuff[j],refCoeffBuff, controlOutputSignalState[j], 24, 256);
		//fir(WNSignal[j], WNOutputSignalOutputBuff[j],refCoeffBuff, WNOutputSignalState[j], 24, 256);
	}
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++){
			controlOutputSignal[j][i] = constrain(controlOutputSignalOutputBuff[j][i], constrain_min, constrain_max);/// controlOutputSignalInterp;
		}
		//controlOutputSignal[j][0]+= WNSignal[j][0];
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

		//DACFlag = true;

		 if (ANCInProgress){
		 ANCERR = true;
		 printf("anc err\n");
		 }

		DACFlag = true;
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

	stepSizeSMax= 0.0001;
	stepSizeFMax= 0.0001;
	stepSizeHMax= 0.0001;

	stepSizeW   = 0.00001;	// 0.000005;
	stepSizeSMin= 0.00001;
	stepSizeFMin= 0.00001;
	stepSizeHMin= 0.00001;
	stepSizeF = stepSizeFMin;
	stepSizeH = stepSizeHMin;

	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (uint8_t k = 0; k < numErrorSignal; k++) {

			stepSizeS[j][k] = stepSizeSMin;
		}
	}
	//stepSizeH = 0.0001;

	/* Initialize the delay line */

	for (int32_t i = 0; i < OCPMLength+1; i++) {
		 OFPMErrorState[i]=0;
	for (uint8_t j = 0; j < numControlSignal; j++) {
		 controlState[j][i]=0;
		 controlOutputSignalState[j][i]=0;
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


	/* Transform the normal order coefficients from a filter design
	   tool into coefficients for the fir_interp function */

	int32_t x = controlOutputSignalLength;
	for (int32_t np = 1; np <= controlOutputSignalPolyPhases; np++){
	    for (int32_t nc = 1; nc <= (controlOutputSignalCoeffsPerPoly); nc++){
	    	controlOutputSignalCoeffBuff[--x] = controlOutputSignalCoeffBuff_temp[(nc * controlOutputSignalPolyPhases) - np];
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
	//adi_int_InstallHandler (ADI_CID_FLTUI, aluFLTOIHandler, NULL, true) ;
	//adi_int_InstallHandler (ADI_CID_FIXI, aluHandler, NULL, true) ;

	if (Result == 0u) {
//		Result = DMAInit();
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


	printf("Started.\n");
	while (!bExit) {


				if (bEnableOCPMWeightUpdate) {
				}
				ProcessBuffers();




	}

	if (!bError) {
		printf("All done\n");
	} else {
		printf("Audio error\n");
	}

	return 0;
}

volatile bool allz = false;

void ProcessBuffers() {

	if (ADCFlag && DACFlag) {

		ANCInProgress = true;

	    pDAC = NULL;
	    pADC = NULL;


		pADC = (void *) pGetADC;
		pDAC = (void *) pGetDAC;
		Adau1979DoneWithBuffer((void *) pADC);
		Adau1962aDoneWithBuffer((void *) pDAC);





		pGetADC = NULL;
		pGetDAC = NULL;
			pADCBuffer = (int32_t *) pADC;

			pDst = (int32_t *) pDAC;

			for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {

				refInputBuff[i] = conv_float_by(pADCBuffer[4*i],-7);
				if(refInputBuff[i]!=0){
					allz = true;
				}
				for (uint8_t k = 0; k < NUM_ADAU1979_CHANNELS; k++) {
					errorSignal[k][i] = conv_float_by(pADCBuffer[4*i], -7);
					if(errorSignal[k][i]!=0){
						allz = true;
					}
				}
			}


if(!allz){

			RefFIR();
			while (!TRNGFlag)
				;

			TRNGFlag = false;
			WN_Gen();
			OFPMFIR();
			OFPMErrorFIR();


			ControlFIR();

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


			TRNG_ACK();


#pragma vector_for
for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++){
	controlOutputSignalB[j][i] = conv_fix_by(controlOutputSignal[j][i],7);
}
}




for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
	//TDM8 SHIFT <<8
	 *pDst++ = controlOutputSignalB[0][i];
 *pDst++ = controlOutputSignalB[1][i];
//	*pDst++ = (conv_fix_by(refInputBuff[i], 16));
//	*pDst++ = (conv_fix_by(errorSignal[1][i], 16));
	 *pDst++ = 0;
	 *pDst++ = 0;

}

}
allz = false;
			ADCFlag = false;
			DACFlag = false;
			ANCInProgress = false;
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

		ADCFlag = true;

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






int32_t WN_Gen(void) {


#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
		WNSignal[j][0]= randBuff[j] *OCPMWNGain[j]; //*30

	}

#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
	for (uint32_t i = 0; i < WNSignalBuffLength-1; i++) {
		WNSignalBuff[j][i] = WNSignalBuff[j][i+1];
	}

	WNSignalBuff[j][WNSignalBuffLength-1] = WNSignal[j][0]; //*30
	}


	return 0;
}




#ifdef OFPMFilter
int32_t ANCALG_1(void) {

	OFPMErrorSignal= uncorruptedRefSignal[0]-OFPMErrorOutputBuff[0];
	powerOFPMErrorSignal = constrain(forgettingFactorOFPM*powerOFPMErrorSignal + (1 - forgettingFactorOFPM)*OFPMErrorSignal*OFPMErrorSignal, 0.000000000001,100);

	return 0;
}

int32_t ANCALG_2(void) {




	powerUncorruptedRefSignal = constrain(forgettingFactorOFPM*powerUncorruptedRefSignal + (1 - forgettingFactorOFPM)*uncorruptedRefSignal[0]*uncorruptedRefSignal[0], 0.000000000001,100);

	OFPMPowerRatio = powerOFPMErrorSignal/(0.000000000000000001+ powerUncorruptedRefSignal);

	stepSizeF = constrain((OFPMPowerRatio*stepSizeFMin + (1-OFPMPowerRatio)*stepSizeFMax), stepSizeFMin, stepSizeFMax);



	return 0;
}

#endif

int32_t ANCALG_3(void) {


#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			powerOCPMWNSignal[j] =constrain(
					forgettingFactorOCPM
							* powerOCPMWNSignal[j]+ (1.0 - forgettingFactorOCPM)* WNSignal[j][0]* WNSignal[j][0], 0.000000000001,100);
		}

	return 0;
}


int32_t ANCALG_4(void) {
	ADI_FIR_RESULT res;
#pragma vector_for
		for (uint8_t k = 0; k < numErrorSignal; k++) {
			float OCPMAuxSumTemp = 0;

			for (uint8_t j = 0; j < numControlSignal; j++) {
				OCPMAuxSumTemp += OCPMAuxOutputBuff[j][k][0];
			}

			uncorruptedErrorSignal[k] = constrain( errorOutputBuff[k][0], constrain_min, constrain_max) - OCPMAuxSumTemp; //-OCPMRefSumTemp;

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
			powerUncorruptedErrorSignal[k] =constrain(
					forgettingFactorOCPM
							* powerUncorruptedErrorSignal[k] + (1.0 - forgettingFactorOCPM) * uncorruptedErrorSignal[k] * uncorruptedErrorSignal[k], 0.000000000001,100);
		}

		//power of indirectErrorSignal
#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				powerIndirectErrorSignal[j][k] =constrain(
						forgettingFactorOCPM
								* powerIndirectErrorSignal[j][k] + (1.0 - forgettingFactorOCPM) *indirectErrorSignal[j][k] * indirectErrorSignal[j][k], 0.000000000001,100);
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
					(powerUncorruptedErrorSignalSum / (powerIndirectErrorSignalSum + 0.000000000001)), 0.000000000001,100);
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
		*/
#pragma vector_for
		for (int32_t i = 0 ; i < OFPMErrorLength ; i++) {
				OFPMCoeffBuff[j][i] = OFPMCoeffBuff[j][i] * (1.0-stepSizeF*OFPMLeak) + stepSizeF * OFPMErrorSignal * WNSignalBuff[j][i];
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
				OFPMErrorCoeffBuff[i] = OFPMErrorCoeffBuff[i] * (1.0-stepSizeH*OFPMErrorLeak) + stepSizeH * OFPMErrorSignal * controlOutputBuff_OFPMError[i];
		}
	return 0;

}
#endif




int32_t ControlWeightUpdate(void) {
	//Control Coeff
#pragma vector_for
	for (uint8_t j = 0; j < numControlSignal; j++) {
/*
		float controlCoeffNSum =0;
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
				for (uint32_t i = 0; i < OCPMLength; i++) {
				controlCoeffNSum += OCPMRefOutputBuff[j][k][i]*OCPMRefOutputBuff[j][k][i];
				}
			}
			if (controlCoeffNSum ==0.0){
				controlCoeffNSum =1.0;
			}
*/
#pragma vector_for
		for (int32_t i = 0; i < controlLength ; i++) {
			float controlCoeffSum =0;
//N#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
				controlCoeffSum += OCPMRefOutputBuff[j][k][i]*uncorruptedErrorSignal[k];
			}
			controlCoeffBuff[j][i] =	controlCoeffBuff[j][i] * (1-stepSizeW*controlLeak) - stepSizeW *controlCoeffSum;
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
/*
float OCPMNsum=0;
#pragma vector_for
				for (uint32_t i = 0; i < OCPMLength; i++) {
					OCPMNsum += WNSignalBuff[j][i]* WNSignalBuff[j][i];
				}
				if (OCPMNsum ==0.0){
					OCPMNsum =1.0;
				}
				*/
#pragma vector_for
			for (uint8_t k = 0; k < numErrorSignal; k++) {
#pragma vector_for
			for (int32_t i = 0; i < OCPMLength ; i++) {


				OCPMCoeffBuff[j][k][i] = OCPMCoeffBuff[j][k][i] * (1-OCPMLeak*stepSizeS[j][k]) + stepSizeS[j][k]*WNSignalBuff[j][i]*uncorruptedErrorSignal[k];

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
