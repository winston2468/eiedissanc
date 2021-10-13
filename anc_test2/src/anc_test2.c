/*****************************************************************************
 * anc_Core2.c
 *****************************************************************************/
#include <stdio.h>
#include <sys/platform.h>
#include <sys/adi_core.h>
#include "adi_initialize.h"
#include "anc_test2.h"
#include "SHARC_linkInterface.h"
#include <services/int/adi_int.h>
#define TIMESTAMP	if(CycleCountBufferIndex < 1024) {CycleCountBuffer[CycleCountBufferIndex][0] =__LINE__; CycleCountBuffer[CycleCountBufferIndex++][1] = __builtin_emuclk();}


//*****************************************************************************
// Local Variables
//*****************************************************************************
uint32_t DacCount = 0u;
uint32_t RAWBuffersReceived = 0u;
uint32_t FILTEREDBuffersReceived = 0u;
uint8_t  DacStarted = 0u;
/* ADC/DAC buffer pointer */
 void *pGetDAC = NULL;
uint32_t DMASlaveDestinationAddress = 0u;

// These variables are used for instrumenting the code
volatile int CycleCountBufferIndex = 0;
volatile unsigned long CycleCountBuffer[4096][2];

//*************************************************************************
// Interrupt handler for MDMA RAW data transfer complete
//*************************************************************************
static void RawDataTransferFromMasterComplete(uint32_t SID, void *pCBParam)
{
	static uint32_t PreviousFrameCounter = 0xFFFFu;
	uint32_t CounterFromMasterSHARC = *sharc_flag_in_L2;

	if( CounterFromMasterSHARC != PreviousFrameCounter )	// Fix for stutter
	{
		PreviousFrameCounter = CounterFromMasterSHARC;
		TIMESTAMP
		SHARC1Filter( (int8_t *)MDMA_LOCAL_ADDR, AncCoeff, AUDIO_BUFFER_SIZE );
		++RAWBuffersReceived;
	}
}

//*************************************************************************
// Interrupt handler for MDMA FILTERED data transfer complete
//*************************************************************************
static void AncCoeffTransferFromMasterComplete(uint32_t SID, void *pCBParam)
{
	static uint32_t PreviousFrameCounter = 0xFFFFu;
	uint32_t CounterFromMasterSHARC = *sharc_flag_in_L2;

	if( CounterFromMasterSHARC != PreviousFrameCounter )	// Fix for stutter
	{
		PreviousFrameCounter = CounterFromMasterSHARC;
		TIMESTAMP
		++FILTEREDBuffersReceived;

		// Merge the AncCoeff buffer with the audio frame just received from the Master SHARC.
		if( pSportOutputBuffer != 0 )
			MergeAudioChannels( (void *)(MDMA_LOCAL_ADDR+AUDIO_BUFFER_SIZE), AncCoeff, pSportOutputBuffer, AUDIO_BUFFER_SIZE );
	}
}

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";
extern uint32_t PcgDacInit(void);
extern uint32_t AsrcDacInit(void);
extern uint32_t PcgDacEnable(void);
extern uint32_t AsrcDacEnable(void);
extern uint32_t SpuInit(void);

/* Initializes DAC */
extern uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
extern uint32_t Adau1962aSubmitBuffers(void);
extern uint32_t Adau1962aEnable(void);
extern uint32_t Adau1962aDoneWithBuffer( void *pBuffer);

/** 
 * If you want to use command program arguments, then place them in the following string. 
 */
char __argv_string[] = "";

typedef enum {
	NONE, START, RECIEVE
} MODE;

static MODE eMode = NONE;


static bool bError;
static uint32_t count;

volatile bool bEvent;
float pm controlCoeffBuff[numControlSignal][controlLength] = { 0 };

float controlState[numControlSignal][controlLength + 1] = { 0 };


int32_t FIR_init() {

	//FIR stuff
	ADI_FIR_RESULT res;

	//reverseArrayf(refCoeffBuff, sizeof(refCoeffBuff));

	/*
	 for (int32_t i = 0; i < controlLength; i++) {
	 controlCoeffBuff1[i] = 1;		//0.0000001;
	 controlCoeffBuff2[i] = 1;		//0.0000001;
	 }
	 */


	for (uint8_t j = 0; j < numControlSignal; j++) {

		for (int32_t i = 0; i < controlLength; i++) {
			controlState[j][i] = 0;
			controlCoeffBuff[j][i] = 0.0001;		//0.0000001;
			//controlState1[i] = 0;
			//controlState2[i] = 0;
		}
	}
	return 0;
}

void ProcessBufferDAC(uint32_t iid, void* handlerArg) {

	ADI_DMA_RESULT eResult = ADI_DMA_SUCCESS;
	ADI_FIR_RESULT res;


	// process ADC to DAC buffer

	if (pGetDAC != NULL) {

		pDst = (int32_t *) pGetDAC;

#ifdef LowPassFilter
		// ---------------------------------------   Enable Channels -----------------------------------------------

		//memcpy(&refInputBuff[0],refSignal, 4*NUM_AUDIO_SAMPLES_ADC_SINGLE);
		firf(refInputBuff, refOutputBuff, refCoeffBuff, refState,
		refInputSize + 1, refLength);
		if (!OSPMInProgress) {
			/*
			bMemCopyInProgress = true;
			eResult = adi_mdma_Copy1D(hMemDmaStream, (void *) &OSPMInputBuff[0],
					(void *) &refOutputBuff[0],
					MEMCOPY_MSIZE,
					OSPMInputSize);
			// IF (Failure)
			if (eResult != ADI_DMA_SUCCESS) {
				DBG_MSG("Failed initialize MDMA 1D Copy \n", eResult);
			}
			while (bMemCopyInProgress)
				;
				*/
		}

		for (uint8_t j = 0; j < numControlSignal; j++) {
			firf(refOutputBuff, controlOutputBuff[j], controlCoeffBuff[j],
					controlState[j], controlInputSize + 1, controlLength);
		}

#else

		for(uint8_t k =0; k<numControlSignal; k++) {
			firf(refInputBuff, controlOutputBuff[k], controlCoeffBuff[k], controlState[k], controlInputSize+1, controlLength);
		}

#endif

#pragma vector_for
		for (uint8_t j = 0; j < numControlSignal; j++) {
			for (uint32_t i = 0, l = NUM_AUDIO_SAMPLES_DAC - 1;
					i < NUM_AUDIO_SAMPLES_DAC, l < NUM_AUDIO_SAMPLES_DAC * 2 - 2;
					i++, l++) {
				outputSignal[j][i] = controlOutputBuff[l][i]
						+ OSPMWNSignal[j][i];
			}
		}

		if (bEnableOutput) {

			 for (int32_t i=0,j=NUM_AUDIO_SAMPLES_DAC -1; i < NUM_AUDIO_SAMPLES_DAC, j>-1;i++, j--) {
			 //TDM8 SHIFT <<8

			 *pDst++ = (conv_fix_by(refInputBuff[i], 15)) ;
			 *pDst++ = (conv_fix_by(errorSignal[0][i], 15)) ;
			 *pDst++ = (conv_fix_by(errorSignal[1][i], 15)) ;
			 *pDst++ = (conv_fix_by(refInputBuff[i], 15));
			 *pDst++ = (conv_fix_by(errorSignal[0][i], 15));
			 *pDst++ = (conv_fix_by(errorSignal[1][i], 15));
			 *pDst++ = (conv_fix_by(refInputBuff[i], 15)) ;
			 *pDst++ = (conv_fix_by(errorSignal[0][i], 15));

			 }
			 /*
			for (int32_t i = 0, j = NUM_AUDIO_SAMPLES_DAC - 1;
					i < NUM_AUDIO_SAMPLES_DAC, j > -1; i++, j--) {
				//TDM8 SHIFT <<8

				*pDst++ = (conv_fix_by(outputSignal[0][i], 1));
				*pDst++ = (conv_fix_by(outputSignal[1][i], 1));
				*pDst++ = 0;
				*pDst++ = 0;
				*pDst++ = 0;
				*pDst++ = 0;
				*pDst++ = 0;
				*pDst++ = 0;

			}
		} */


	}

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
		Adau1962aDoneWithBuffer(pArg);
		break;
	default:
		break;
	}
}

int main(int argc, char *argv[])
{
	uint32_t Result = 0u;
	bool bExit;
	bExit = false;
	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();
	adi_sec_SetPriority(INTR_SOFT7, 61);
	adi_int_InstallHandler(INTR_SOFT7, ProcessBufferDAC, 0, true);

	/* Begin adding your custom code here */
	if (Result == 0u) {
		Result = SpuInit();
	}

	// Initialize ADAU1962A
	if (Result == 0u) {
		Result = Adau1962aInit();
	}

	bEvent = true;
	eMode = START;

	while (!bExit) {
		if (bEvent) {
			switch (eMode) {
			case RECIEVE:
				ANCALG_2();
				//ProcessBufferADC1();
				// ProcessBufferADC2();
				//ProcessBufferDAC();
				break;
			case START:
				printf("Started.\n");

				//1962a ping pong buffers
				if (Result == 0u) {
					Result = Adau1962aSubmitBuffers();
				}
				//*************************************************************************
				// Initialize MDMA - Code blocks until both cores complete init
				//*************************************************************************
				Result = SHARC_linkSlaveInit( RawDataTransferFromMasterComplete, AncCoeffTransferFromMasterComplete, (void *)DMASlaveDestinationAddress );
				if(Result != 0u)
				{
					DBG_MSG("Core2: SHARClinkSlaveInit() failed\n" );
					while(1){;}
				}

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
	return 0;
}

