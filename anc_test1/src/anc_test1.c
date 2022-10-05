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

/* Standard includes. */
#include <stdlib.h>
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
#include <services/gpio/adi_gpio.h>
#include <services/int/adi_int.h>
#include <stdio.h>
#include "Button_LED_GPIO.h"
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

//***************************1**************************************************
uint8_t DacStarted = 0u;

volatile int32_t DacCount = 0;
volatile int32_t AdcCount = 0;
bool OCPMUpdate = true;
#pragma alignment_region(4)

extern int button_led_gpio_init(void);
/* ----------------------------   FIR Configuration ------------------------------------------- */

float controlOutput[1] = {0};
float controlOutputBuff[controlLength] = {0};
float pm controlCoeffBuff[controlLength] = {0}; // coeffs array must be initialized and in PM memory

bool controlCoeffLimit = false;
float controlState[controlLength + 1] = {0};

float refInput[1] = {0};
float refInputBuff[controlLength] = {0};

float errorSignal[1] = {0};
float errorSignalBuff[OCPMLength] = {0};

float pm OCPMCoeffBuff[OCPMLength] = {0};
float dm OCPMRefState[OCPMLength + 1] = {0};
float dm OCPMAuxState[OCPMLength + 1] = {0};
float dm OCPMRefOutput[1] = {0};
float dm OCPMRefOutputBuff[controlLength] = {0};
float dm OCPMAuxOutput[1] = {0};
float dm OCPMAuxOutputBuff[OCPMLength] = {0};
float WNSignalBuff[OCPMLength] = {0};

float secondayPathErrorTerm[1] = {0};

float stepSizeS = 0.0001;
float stepSizeW = 0.0001;
uint32_t WNCount = 0;
float randBuff[WNLength] = {0};
float WNSignal[1] = {0};

float outputSignal[1] = {0};
float loudest = 0;
int32_t dm outputSignalInt32[1] = {0};
int32_t outputSignalInt32Buff[1024] = {0};
#pragma alignment_region_end

/* used for exit timeout */
//#define MAXCOUNT (50000000000u)
//#define MAXCOUNT (50000000u)
/*=============  D A T A  =============*/

volatile bool ADCFlag = false;

volatile bool DACFlag = false;
volatile bool enableOCPM = true;
volatile bool enableControl = true;
static bool bError = false;
static uint32_t count = 0;

volatile bool bEvent = false;

volatile bool ANCERR = false;
volatile bool ANCInProgress = false;
/* Flag to register callback error */
volatile bool bEventError = false;

// TRNG
volatile bool TRNGFlag = false;

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";

// uint32_t iida = 0;

void gpioCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t Data, void *pCBParam)
{
	if (ePinInt == PUSH_BUTTON1_PINT)
	{
		if (Data & PUSH_BUTTON1_PINT_PIN)
		{
			/* push button 1 - toggle LED */
			adi_gpio_Toggle(LED1_PORT, LED1_PIN);
			if(enableOCPM){
				enableOCPM = false;
			}
			else{
				enableOCPM = true;
			}
		}
	}

	if (ePinInt == PUSH_BUTTON2_PINT)
	{
		if (Data & PUSH_BUTTON2_PINT_PIN)
		{
			/* push button 2 - toggle LED */
			adi_gpio_Toggle(LED2_PORT, LED2_PIN);
			if(enableControl){
				enableControl = false;
			}
			else{
				enableControl = true;
			}
		}
	}

	/* reset the exit counter */
	//    count = 0u;
}

void aluFLTOIHandler(uint32_t iid, void *handlerArg)
{
	printf("%d\n", iid);
	// printf("ERR");
	// iida = iid;
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
void MemDmaCallback(void *pCBParam, uint32_t Event, void *pArg)
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

void TRNG_Init()
{
	adi_int_InstallHandler(INTR_PKIC0_IRQ, PKIC_ISR, 0, true);
	int iTemp;

	// Disable TRNG Module
	Disable_TRNG();

	// Initialize PKIC
	Set_PKIC_Polarity(0x2F);   // setting for high rising edge
	Set_PKIC_Level_type(0x2F); // interrupt source is edge sensitive
	Disable_PKIC_Interrupt(0x2F);
	// Read RAW STAT for edge detected interrupt sources
	iTemp = Read_PKIC_Unmasked_Interrupt_Source();
	// Disable those edge triggered interrupts
	PKIC_Interrupt_ACK(iTemp);

	// Enable TRNG interrupt source in PKIC
	Enable_TRNG_Interrupt();

	// Read Enabled STAT
	iTemp = Read_PKIC_Masked_Interrupt_Source();

	// Start TRNG initialization
	Mask_Interrupt(0x1); // all TRNG related interrupt sources are enabled

	Startup_Cycle_Number(256); // setting minimum sampling rate supported
	Min_Refill_Cycle(64);	   // setting minimum sampling rate supported

	Max_Refill_Cycle(8388608); // setting a sample rate of 2^23
	Sample_division(0);		   // can be any value from 0 to 15

	Set_Alarm_Threshold(1);	   // detects repeating pattern on each FRO and generates an interrupt.
	Set_Shutdown_Threshold(1); // even if 1 FRO shuts down generate an interrupt/

	Disbale_FRO(0xFFFFFF);
	Enable_FRO(0xFF); // 8 FRO in BD

	iTemp = Read_TRNG_Stat();
	Acknowledge_Interrupt(iTemp);
	iTemp = Read_TRNG_Stat();

	Enable_TRNG();
}






void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
	switch (nEvent)
	{
	case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
		/* store pointer to the processed buffer that caused the callback */
		pGetADC = pArg;

		if (ANCInProgress)
		{
			ANCERR = true;
			printf("ERR ANC");
		}
		// AdcCount++;
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
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{

	switch (nEvent)
	{
	case ADI_SPORT_EVENT_TX_BUFFER_PROCESSED:
		// store pointer to the processed buffer that caused the callback
		// We can still copy to the buffer after it is submitted to the driver
		pGetDAC = pArg;

		// DACFlag = true;

		if (ANCInProgress)
		{
			ANCERR = true;
			printf("anc err, too much processing load\n");
		}

		// DacCount++;

		break;
	default:
		printf("err");
		break;
	}
}

// convert to float [-10,10]
void Read_TRNG_Output_Imp(float *iOutput)
{
	// int range [-2147483647,2147483647]
	float *iTemp;
	iTemp = iOutput;
	*iTemp = (((float)(*pREG_TRNG0_OUTPUT0)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float)(*pREG_TRNG0_OUTPUT1)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float)(*pREG_TRNG0_OUTPUT2)) / (2147483647)) - 1;
	iTemp++;
	*iTemp = (((float)(*pREG_TRNG0_OUTPUT3)) / (2147483647)) - 1;
}

void PKIC_ISR(uint32_t iid, void *handlerArg)
{
	int iTemp, iTemp2;
	iTemp = Read_PKIC_Masked_Interrupt_Source();
	iTemp2 = iTemp && 0x8;
	if ((iTemp && 0x8) == 1)
	{
		PKIC_Interrupt_ACK(0x8);
		TRNG_ISR();
	}
	else
		PKIC_Interrupt_ACK(iTemp);
}

void TRNG_ISR()
{
	int iTemp, iTemp2, iTemp3;
	iTemp = Read_TRNG_Stat();
	if ((iTemp & 0x2) == 1)
	{

		iTemp2 = Read_Alarm_Mask();
		iTemp3 = Read_Alarm_Stop(); // Fro will be disbaled automatically.
		if (iTemp2 == iTemp3)
		{
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if ((iTemp & 0x1) == 1)
	{
		if (WNCount < numControlSignal)
		{
			Read_TRNG_Output_Imp(&randBuff[WNCount * 4]);
			WNCount++;
			Acknowledge_Interrupt(iTemp);
		}
		else
		{
			WNCount = 0;
			TRNGFlag = true;
		}
	}
}

void TRNG_ACK()
{
	int iTemp, iTemp2, iTemp3;
	iTemp = Read_TRNG_Stat();
	if ((iTemp & 0x2) == 1)
	{

		iTemp2 = Read_Alarm_Mask();
		iTemp3 = Read_Alarm_Stop(); // Fro will be disbaled automatically.
		if (iTemp2 == iTemp3)
		{
			printf("\n FRO shutdown \n");
			Disbale_FRO(iTemp3);
			Detune_FRO(iTemp3);
		}
		Acknowledge_Interrupt(iTemp);
	}
	if ((iTemp & 0x1) == 1)
	{
		Acknowledge_Interrupt(iTemp);
	}

	asm("nop;");
}

int32_t FIR_init()
{

	for (int32_t i = 0; i < OCPMLength; i++)
	{
		OCPMCoeffBuff[i] = 0.01;
	}
	for (int32_t i = 0; i < controlLength; i++)
	{
		controlCoeffBuff[i] = 0.01;
	}

	return 0;
}

int main(void)
{
	uint32_t DestAddress;

	int8_t firResult = 0;
	bool bExit;
	uint32_t Result = 0u;
	// uint32_t i;
	bExit = false;

	adi_initComponents(); /* auto-generated code */
	// ALUSAT
	// sysreg_bit_clr(sysreg_MMASK,0x2000);

	// sysreg_bit_set(sysreg_MODE1,0x2000);
	// SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int)0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int)0x001FFFFE;

	// FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;

	adi_int_InstallHandler(ADI_CID_FLTOI, aluFLTOIHandler, NULL, true);
	// adi_int_InstallHandler (ADI_CID_FLTUI, aluFLTOIHandler, NULL, true) ;
	// adi_int_InstallHandler (ADI_CID_FIXI, aluHandler, NULL, true) ;

	if (Result == 0u)
	{
		// Result = DMAInit();
	}

	button_led_gpio_init();

	// Initialize ADAU1962a
	if (Result == 0u)
	{
		Result = Adau1962aInit();
	}
	// Initialize ADAU1979
	if (Result == 0u)
	{
		Result = Adau1979Init();
	}

	if (FIR_init() == 0)
	{
		printf("FIR init success\n");
	}

	TRNG_Init();
	if (Result == 1)
	{
		printf("Init Error");
		return 1;
	}

	ProcessBuffers();

	printf("Started.\n");

	// 1979 ping pong buffers
	if (Result == 0u)
	{
		Result = Adau1979SubmitBuffers();
	}

	// 1962a ping pong buffers
	if (Result == 0u)
	{
		Result = Adau1962aSubmitBuffers();
	}

	// Enable data flow for the ADC
	if (Result == 0u)
	{
		Adau1979Enable();
	}

	// Enable data flow for the DAC
	if (Result == 0u)
	{
		Adau1962aEnable();
	}

	while (!bExit)
	{

		if (pGetADC != NULL)
		{
			pADC = (void *)pGetADC;
			Adau1979DoneWithBuffer((void *)pADC);
			pGetADC = NULL;
		}

		if (pGetDAC != NULL)
		{
			pDAC = (void *)pGetDAC;
			Adau1962aDoneWithBuffer((void *)pDAC);
			pGetDAC = NULL;
		}

		if ((pDAC != NULL) && (pADC != NULL))
		{

			ProcessBuffers();
		}
	}

	if (!bError)
	{
		printf("All done\n");
	}
	else
	{
		printf("Audio error\n");
	}

	return 0;
}

int temp = 0;

void ProcessBuffers()
{

	ANCInProgress = true;

	pADCBuffer = (int32_t *)pADC;
	pDst = (int32_t *)pDAC;
	refInput[0] = conv_float_by(*pADCBuffer++, refInputBuff_conv_float_by_exp);
	errorSignal[0] = conv_float_by(*pADCBuffer++, errorSignal_conv_float_by_exp);
	temp = *pADCBuffer++;
	temp = *pADCBuffer++;


	while (!TRNGFlag)
		;

	TRNGFlag = false;

	TRNG_ACK();

	// OCPM
	fir(refInput, OCPMRefOutput, OCPMCoeffBuff, OCPMRefState, OCPMWindowSize, OCPMLength);
	//OCPM
	fir(WNSignal, OCPMAuxOutput, OCPMCoeffBuff, OCPMAuxState, OCPMWindowSize, OCPMLength);

	// Control
	fir(OCPMRefOutput, controlOutput, controlCoeffBuff, controlState, controlWindowSize, controlLength);

// Circular buffer
#pragma vector_for
	for (uint32_t i = 0; i < controlLength - 1; i++)
	{
		OCPMRefOutputBuff[i] = OCPMRefOutputBuff[i + 1];
	}

	OCPMRefOutputBuff[controlLength - 1] = OCPMRefOutput[0];

	// Circular buffer
	#pragma vector_for
		for (uint32_t i = 0; i < controlLength - 1; i++)
		{
			controlOutputBuff[i] = controlOutputBuff[i + 1];
		}

		controlOutputBuff[controlLength - 1] = controlOutput[0];
// Circular buffer
#pragma vector_for
	for (uint32_t i = 0; i < OCPMLength - 1; i++)
	{
		WNSignalBuff[i] = WNSignalBuff[i + 1];
	}

	WNSignalBuff[OCPMLength - 1] = WNSignal[0];

	secondayPathErrorTerm[0] = errorSignal[0] - OCPMAuxOutput[0];


	// Circular buffer
	#pragma vector_for
		for (uint32_t i = 0; i < OCPMLength - 1; i++)
		{
			errorSignalBuff[i] = errorSignalBuff[i + 1];
		}

		errorSignalBuff[OCPMLength - 1] = errorSignal[0];



		if (enableControl){
	// Control weight update

	float controlCoeffDotProduct = 0;
#pragma vector_for
	for (uint32_t i = 0; i < controlLength; i++)
	{
		controlCoeffDotProduct += OCPMRefOutputBuff[i] * OCPMRefOutputBuff[i];
	}

	if (controlCoeffDotProduct == 0)
	{
		controlCoeffDotProduct = 1;
	}

#pragma vector_for
	for (int32_t i = 0; i < controlLength; i++)
	{

		controlCoeffBuff[i] = controlCoeffBuff[i] * (1 - stepSizeW * controlLeak) - stepSizeW * errorSignal[0] * OCPMRefOutputBuff[i] / (controlCoeffDotProduct);
	}

		}







if (enableOCPM){

	// OCPM Weight Update

	float OCPMDotProduct = 0;
#pragma vector_for
	for (uint32_t i = 0; i < OCPMLength; i++)
	{
		OCPMDotProduct += WNSignalBuff[i] * WNSignalBuff[i];
	}

	if (OCPMDotProduct == 0)
	{
		OCPMDotProduct = 1;
	}

#pragma vector_for
	for (int32_t i = 0; i < OCPMLength; i++)
	{

		OCPMCoeffBuff[i] = OCPMCoeffBuff[i] * (1.0 - OCPMLeak * stepSizeS) + stepSizeS * secondayPathErrorTerm[0] * WNSignalBuff[i] / (OCPMDotProduct);
	}
}
	limitSig();

	WNSignal[0] = randBuff[0] * OCPMWNGainCompensation;

	outputSignal[0] = controlOutput[0] + WNSignal[0];

	if (outputSignal[0] > loudest)
	{
		loudest = outputSignal[0];
	}

	outputSignalInt32[0] = conv_fix_by(outputSignal[0], outputSignalInt32_conv_fix_by_exp);
#pragma vector_for
	for (uint32_t i = 0; i < 1024 - 1; i++)
	{
		outputSignalInt32Buff[i] = outputSignalInt32Buff[i + 1];
	}

	outputSignalInt32Buff[1024 - 1] = outputSignalInt32[0];
	*pDst++ = outputSignalInt32[0];

	for (uint8_t j = numControlSignal; j < NUM_DAC_CHANNELS; j++)
	{
		*pDst++ = 0;
	}

	pDAC = NULL;
	pADC = NULL;

	ANCInProgress = false;
}

uint8_t limitSig()
{

	if (fabsf(controlOutput[0] > Amax))
	{

		controlOutput[0] = (controlOutput[0] * Amax / fabsf(controlOutput[0]));

		for (int32_t i = 0; i < controlLength; i++)
		{
			controlCoeffBuff[i] = (controlCoeffBuff[i] * Amax / fabsf(controlOutput[0])) * 0.999;
		}
	}
	return 0;
}
