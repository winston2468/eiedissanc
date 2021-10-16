/*****************************************************************************
 * anc_Core2.c
 *****************************************************************************/
#include <sys/platform.h>
#include <sys/adi_core.h>
#include <SRU.h>


#include <services/int/adi_sec.h>
#include <drivers/dac/adau1962a/adi_adau1962a.h>
#include <drivers/twi/adi_twi.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <adi_types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <filter.h>
#include "adi_initialize.h"
#include "SHARC_linkInterface.h"

#include "anc_test2.h"


#define TIMESTAMP	if(CycleCountBufferIndex < 1024) {CycleCountBuffer[CycleCountBufferIndex][0] =__LINE__; CycleCountBuffer[CycleCountBufferIndex++][1] = __builtin_emuclk();}

//*****************************************************************************
// Local Variables
//*****************************************************************************
uint8_t DacStarted = 0u;
/* ADC/DAC buffer pointer */
void *pGetDAC = NULL;
uint32_t DMASlaveDestinationAddress = 0u;

// These variables are used for instrumenting the code
volatile int CycleCountBufferIndex = 0;
volatile unsigned long CycleCountBuffer[4096][2];

#pragma alignment_region (4)
float controlOutputBuff[numControlSignal][controlInputSize + 1] = { 0 };
float outputSignal[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
float OSPMWNSignal[numControlSignal][NUM_AUDIO_SAMPLES_PER_CHANNEL] = { 0 };
#pragma alignment_region_end

volatile bool bControlFIRFlag=false;
volatile bool bOSPMFlag=false;
volatile bool bUpdateFlag=false;
/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";
extern uint32_t PcgDacInit(void);
extern uint32_t AsrcDacInit(void);
extern uint32_t PcgDacEnable(void);
extern uint32_t AsrcDacEnable(void);

/* Initializes DAC */
extern uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
extern uint32_t Adau1962aSubmitBuffers(void);
extern uint32_t Adau1962aEnable(void);
extern uint32_t Adau1962aDoneWithBuffer(void *pBuffer);
uint8_t ControlFIR(void);
uint8_t GenOutputSignal(void);
uint8_t PushOutputSignal(void);
uint8_t UpdateControlCoeff(void);

typedef enum {
	NONE, START, RECIEVE_REF, RECIEVE_OSPMWNSIGNAL, RECIEVE_CONTROL_COEFF
} MODE;

static volatile MODE eMode = NONE;
volatile int queue = 0;

static bool bError;
static uint32_t count;

volatile bool bEvent;
float pm controlCoeffBuff[numControlSignal][controlLength] = { 0 };

float controlState[numControlSignal][controlLength + 1] = { 0 };

//*************************************************************************
// Interrupt handler for MDMA RAW data transfer complete
//*************************************************************************
static void RefDataTransferFromMasterComplete(uint32_t SID, void *pCBParam) {
	static uint32_t PreviousFrameCounter = 0xFFFFu;
	uint32_t CounterFromMasterSHARC = *sharc_flag_in_L2;

	if (CounterFromMasterSHARC != PreviousFrameCounter)	// Fix for stutter
			{
		PreviousFrameCounter = CounterFromMasterSHARC;
		TIMESTAMP
		if(bUpdateFlag){
			printf("Sync error 1");
		}
		bControlFIRFlag = true;
		ControlFIR();
		bControlFIRFlag = false;
	}
}

//*************************************************************************
// Interrupt handler for MDMA OSPMWNSignal data transfer complete
//*************************************************************************
static void OSPMWNSignalTransferFromMasterComplete(uint32_t SID, void *pCBParam) {
	static uint32_t PreviousFrameCounter = 0xFFFFu;
	uint32_t CounterFromMasterSHARC = *sharc_flag_in_L2;

	if (CounterFromMasterSHARC != PreviousFrameCounter)	// Fix for stutter
			{
		PreviousFrameCounter = CounterFromMasterSHARC;
		TIMESTAMP
		if(bControlFIRFlag){
			printf("Sync error 2");
		}
		bOSPMFlag = true;
		GenOutputSignal();
		bOSPMFlag = false;

	}
}

//*************************************************************************
// Interrupt handler for MDMA ControlCoeff data transfer complete
//*************************************************************************
static void ControlCoeffTransferFromMasterComplete(uint32_t SID, void *pCBParam) {
	static uint32_t PreviousFrameCounter = 0xFFFFu;
	uint32_t CounterFromMasterSHARC = *sharc_flag_in_L2;

	if (CounterFromMasterSHARC != PreviousFrameCounter)	// Fix for stutter
			{
		PreviousFrameCounter = CounterFromMasterSHARC;
		TIMESTAMP
		if(bOSPMFlag){
			printf("Sync error 3");
		}
		bUpdateFlag = true;
		UpdateControlCoeff();
		PushOutputSignal();
		bUpdateFlag = true;

	}
}

int32_t FIR_init() {

	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (int32_t i = 0; i < controlLength; i++) {
			controlState[j][i] = 0;
			controlCoeffBuff[j][i] = 0.0001;		//0.0000001;
		}
	}
	return 0;
}

uint8_t ControlFIR() {
	for (uint8_t j = 0; j < numControlSignal; j++) {
		firf((float *) MDMA_LOCAL_ADDR, controlOutputBuff[j],
				controlCoeffBuff[j], controlState[j], controlInputSize + 1,
				controlLength);
	}
	return 0;
}

uint8_t GenOutputSignal() {
	//SAFE
	memcpy(&OSPMWNSignal[0][0], (void *)(MDMA_LOCAL_ADDR + refOutputBufferSize),
			control_BufferSize);
	//float **OSPMWNSignal = (float **) MDMA_LOCAL_ADDR + refOutputBufferSize;
	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (uint32_t i = 0, l = (((controlInputSize + 1) / 4) - 1);
				i < NUM_AUDIO_SAMPLES_PER_CHANNEL, l < controlInputSize; i++) {
			outputSignal[j][i] = controlOutputBuff[j][l] + OSPMWNSignal[j][i];
		}
	}

	return 0;
}

uint8_t PushOutputSignal() {
	if (pGetDAC != NULL) {

		int32_t *pDst = (int32_t *) pGetDAC;
		for (uint32_t i = 0; i < NUM_AUDIO_SAMPLES_PER_CHANNEL; i++) {
			//TDM8 SHIFT <<8
			/*
			 for(int32_t j =0; j< numControlSignal;j++){
			 *pDst++ = (conv_fix_by(outputSignal[j][i], 10)) ;
			 }

			 for(int32_t m =0; m< NUM_DAC_CHANNELS - numControlSignal;m++){
			 *pDst++ = 0;
			 }
			 */

			/*
			 *pDst++ = (conv_fix_by(outputSignal[0][i], 10)) ;
			 *pDst++ = (conv_fix_by(outputSignal[1][i], 10)) ;
			 *pDst++ = (conv_fix_by(outputSignal[2][i], 10)) ;
			 *pDst++ = (conv_fix_by(outputSignal[3][i], 10)) ;
			 *pDst++ = (conv_fix_by(outputSignal[4][i], 10)) ;
			 *pDst++ = (conv_fix_by(outputSignal[5][i], 10)) ;
			 *pDst++ = 0 ;
			 *pDst++ = (conv_fix_by(outputSignal[6][i], 10)) ;
			 *pDst++ = 0 ;
			 */
			*pDst++ = (conv_fix_by(outputSignal[0][i], 10));
			*pDst++ = (conv_fix_by(outputSignal[1][i], 10));
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;
			*pDst++ = 0;

		}
	}

	return 0;
}

uint8_t UpdateControlCoeff() {
	float * recieveControlCoeff = (float *) MDMA_LOCAL_ADDR
			+ refOutputBufferSize + OSPMWNSignal_BufferSize;
/*
	 for (uint8_t j = 0; j < numControlSignal; j++) {

	 for (int32_t i = 0; i < controlLength; i++) {
	 controlCoeffBuff[j][i]=recieveControlCoeff[j][i];
	 }
	 }*/


	//SAFE
	memcpy(&controlCoeffBuff[0][0], recieveControlCoeff,
			control_BufferSize);

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
		TIMESTAMP
		pGetDAC = pArg;
		Adau1962aDoneWithBuffer(pArg);
		break;
	default:
		break;
	}
}

int main(int argc, char *argv[]) {
	uint32_t Result = 0u;
	bool bExit;
	bExit = false;
	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */


	//Enable the interrupts globally
	*pREG_SEC0_GCTL=ENUM_SEC_GCTL_EN;
	*pREG_SEC0_CCTL2=ENUM_SEC_CCTL2_EN;
	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;
	adi_initComponents();

	/* Begin adding your custom code here */


	// Initialize ADAU1962A
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	bEvent = true;
	eMode = START;

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case RECIEVE_REF:
				//ControlFIR();

				break;

			case RECIEVE_OSPMWNSIGNAL:

				//GenOutputSignal();


				//PushOutputSignal();

				break;
			case RECIEVE_CONTROL_COEFF:
				//UpdateControlCoeff();
				eMode = NONE;
				break;

			case START:
				printf("Started.\n");

				//1962a ping pong buffers
				if (Result == 0u) {
					Result = Adau1962aSubmitBuffers();
				}
				eMode = NONE;
				// Enable data flow for the DAC
				if (Result == 0u && DacStarted == 0) {

					Adau1962aEnable();
					DacStarted = 1;
					fprintf(stdout, "Core2: DAC Started\n");
				}
				//*************************************************************************
				// Initialize MDMA - Code blocks until both cores complete init
				//*************************************************************************
				Result = SHARC_linkSlaveInit(RefDataTransferFromMasterComplete,
						OSPMWNSignalTransferFromMasterComplete,
						ControlCoeffTransferFromMasterComplete,
						(void *) DMASlaveDestinationAddress);
				if (Result != 0u) {
					DBG_MSG("Core2: SHARClinkSlaveInit() failed\n" );
					while (1) {
						;
					}
				}

				break;
			default:
				break;
			}

			bEvent = false;
		}
	}
	return 0;
}

