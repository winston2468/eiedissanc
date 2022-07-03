/*****************************************************************************
 * anc_test1.h
 *****************************************************************************/

#ifndef __ANC_TEST1_H__
#define __ANC_TEST1_H__
#include "PKIC.h"
#include "TRNG.h"


////////////////// Customizable options ////////////////////////////////////////////////////////////////////

// Converts val to a floating-point value, then scales that value by adding amount to the value's exponent
// Reference input integer * exponent then convert to float
#define refInputBuff_conv_float_by_exp -16
// errorSignal input integer * exponent then convert to float
#define errorSignal_conv_float_by_exp -16


// Scales val by adding amount to val exponent, then converts the result to an integer
// Output Signal float * exponent convert to integer
#define OutputSignalInt32_conv_fix_by_exp 11


//Multi rate filtering configuration
//Modfying this requires also modifying src/data/* filters' values
#define NUM_AUDIO_SAMPLES_PER_CHANNEL     24
#define DECIMATION_FACTOR 24
#define DECIMATION_FACTOR_A 6
#define DECIMATION_FACTOR_B 2
#define DECIMATION_FACTOR_C 2




//Reference decimation filter A Settings
#define refLength 6
#define refWindowSize 24

//Reference decimation filter B Settings
#define refLengthB 6
#define refWindowSizeB 24/DECIMATION_FACTOR_A

//Reference decimation filter C Settings
#define refLengthC 6
#define refWindowSizeC refWindowSizeB/DECIMATION_FACTOR_B
#define refWindowSizeD refWindowSizeC/DECIMATION_FACTOR_C

//Output interpolation filter C Settings
#define OutputSignalInputSizeC refWindowSizeD
#define OutputSignalLengthC 6
#define OutputSignalInterpC  DECIMATION_FACTOR_C
#define OutputSignalPolyPhasesC      OutputSignalInterpC
#define OutputSignalWindowSizeC (OutputSignalInputSizeC*OutputSignalInterpC)
#define OutputSignalCoeffsPerPolyC     (OutputSignalLengthC / OutputSignalPolyPhasesC)

//Output interpolation filter B Settings
#define OutputSignalInputSizeB refWindowSizeC
#define OutputSignalLengthB 6
#define OutputSignalInterpB  DECIMATION_FACTOR_B
#define OutputSignalPolyPhasesB      OutputSignalInterpB
#define OutputSignalWindowSizeB (OutputSignalInputSizeB*OutputSignalInterpB)
#define OutputSignalCoeffsPerPolyB     (OutputSignalLengthB / OutputSignalPolyPhasesB)

//Output interpolation filter A Settings
#define OutputSignalInputSize refWindowSizeB
#define OutputSignalLength 6
#define OutputSignalInterp  DECIMATION_FACTOR_A
#define OutputSignalPolyPhases      OutputSignalInterp
#define OutputSignalWindowSize (OutputSignalInputSize*OutputSignalInterp)
#define OutputSignalCoeffsPerPoly     (OutputSignalLength / OutputSignalPolyPhases)

//Leakage settings

//ANC Control filter Tap Leakage
#define controlLeak 0.0f
//ANC Secondary path identification filter Tap Leakage
#define OCPMLeak  0.0f
//ANC Secondary path identification filter forgetting gactor
#define forgettingFactorOCPM_set (0.6f)

//ANC Control filter length  (length affects processing load)
#define controlLength 32
//ANC Control window size
#define controlWindowSize 1
//Control Filter LMS step size
#define stepSizeW_set (0.00000005f)

//Max amplitude for output ANC control filter signal
#define Amax 400.0f

//secondary path identification filter LMS step size minimum value
#define stepSizeSMin_set (0.0000001f)
//Secondary path identification filter LMS step size maximum value
#define stepSizeSMax_set (0.01f)
//White noise signal of secondary path identification filter gain
#define OCPMWNGainCompensation (20.0f)
//Secondary path identification filter length (length affects processing load)
#define OCPMLength 16
//Secondary path identification window size
#define OCPMWindowSize 1

//Number of input error signal (values 1 to 3)
#define numErrorSignal 2
//Number of input error signal (values 1 to 4)
#define numControlSignal 2
///////////////////////////////end of customizable settings/////////////////////////////////////////////////////








//#define USE_ADAU1761
#define USE_ADAU1962a
//#define TDM_MODE
//#define OCPMExtendedFilter
//#define USE_ASRC
void SPE1_ISR();
#define OCPMExtendedLeak  1.0f
#define constrain_min -1000
#define constrain_max 1000
//#define OFPMFilter
//#define ControlFIRA
//#define OCPMOnFIRA
/*
#define NUM_AUDIO_SAMPLES_ADC_SINGLE      (NUM_AUDIO_SAMPLES_ADC/2)
#define NUM_AUDIO_SAMPLES_ADC_1979     NUM_AUDIO_SAMPLES_ADC_SINGLE
#define NUM_AUDIO_SAMPLES_DAC       2048
*/
//#define TAP_LENGTH 128u
//#define WINDOW_SIZE 128u

//#define OFPMLength 32
//#define OFPMWindowSize 1
//#define OFPMErrorLength 32
//#define OFPMErrorWindowSize 1
#define  OFPMLeak 0.1f
#define  OFPMErrorLeak  0.1f
#define WNSignalBuffLength (OCPMLength+0u)


#define NUM_DAC_CHANNELS (4u)
#define NUM_ADAU1979_CHANNELS (4u)
#define NUM_ADAU1761_CHANNELS (2u)
#define AUDIO_BUFFER_SIZE_DAC 	        (NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_DAC_CHANNELS*4)
#define AUDIO_BUFFER_SIZE_ADC_1979	        (NUM_AUDIO_SAMPLES_PER_CHANNEL*NUM_ADAU1979_CHANNELS*4)


#define OCPMWNSignal_BufferSize (numControlSignal*OCPMLength*sizeof(float))
#define control_BufferSize (numControlSignal*controlLength*sizeof(float))
#define WNDelay NUM_AUDIO_SAMPLES_PER_CHANNEL*numErrorSignal
#define WNLength (4*(numControlSignal+1)*2) //extra

    /* Clock C 24.576 MHz /(numASRC * 64 * Fs) */
#define pcgCLKDIV 8u
  /* FS 24.576 MHz /Fs kHz */
#define pcgFSDIV 2048u

#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)


/* select input source */
#define USE_LINE_IN
//#define USE_MICROPHONE
#define USE_LINE_OUT
/* the ADAU1761 Rec Mixer Left 0 register */
#define REC_MIX_LEFT_REG    (0x400A)
/* the ADAU1761 Rec Mixer Right 0 register */
#define REC_MIX_RIGHT_REG   (0x400C)

/* codec device instance to be tested */
#define ADAU1761_DEV_NUM          0



#define MEMCOPY_STREAM_ID           (ADI_DMA_MEMDMA_S0)       // Stream 0
/* select sample rate */
#define SAMPLE_RATE  (ADI_ADAU1761_SAMPLE_RATE_8KHZ)


/* The SPORT device selection depends on which BF609 EZ-Board connector
 * the Audio EZ-Extender is attached.
 * P1A - Sport TX 0, Sport RX 0
 * P1B - Can't be used due to SoftSwitch conflict with Push Button 2
 * P2A - Can't be used due to SoftSwitch conflict with Push Button 2
 * P3A - Sport TX 2, Sport RX 2
 */

#define SPORT_RX_DEVICE1  2
#define SPORT_TX_DEVICE1  2
#define SPORT_2A_SPU_PID  19
#define SPORT_2A_DMA4_SPU_PID 70
#define SPORT_2B_SPU_PID  20
#define SPORT_2B_DMA5_SPU_PID 71

#define SPORT_RX_DEVICE2  0
#define SPORT_TX_DEVICE2  0
#define SPORT_0A_SPU_PID  15
#define SPORT_0A_DMA0_SPU_PID 66
#define SPORT_0B_SPU_PID  16
#define SPORT_0B_DMA1_SPU_PID 67

/* Macro to specify delay count for DAC reset */
#define DELAY_COUNT             (1000000u)

/* TWI device instance used for communicating with the codec device */
#define TWI_DEV_NUM            0


#define PUSH_BUTTON1_PORT (ADI_GPIO_PORT_F)
/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON1_PIN            (ADI_GPIO_PIN_1)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT_PIN       (ADI_GPIO_PIN_1)
#define PUSH_BUTTON1_PINT           (ADI_GPIO_PIN_INTERRUPT_4)

#define PUSH_BUTTON2_PORT (ADI_GPIO_PORT_F)
/* GPIO pin within the port to which push button 2 is connected to */
#define PUSH_BUTTON2_PIN            (ADI_GPIO_PIN_0)

/* pin within the pint to which push button 2 is connected to */
#define PUSH_BUTTON2_PINT_PIN       (ADI_GPIO_PIN_0)
#define PUSH_BUTTON2_PINT           (ADI_GPIO_PIN_INTERRUPT_4)


/*
 * LED GPIO settings
 */

/* GPIO port to which LED 1 is connected to */
#define LED1_PORT                   (ADI_GPIO_PORT_E)
/* GPIO pin within the port to which LED 1 is connected to */
#define LED1_PIN                    (ADI_GPIO_PIN_14)



/* GPIO port to which LED 2 is connected to */
#define LED2_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 2 is connected to */
#define LED2_PIN                    (ADI_GPIO_PIN_13)

/*
 * ADC settings
 */
/* ADAU1979 SPORT config parameters */
#define LR_B_CLK_MASTER_1979    (true)
#define BCLK_RISING_1979 	    (true)
#ifdef TDM_MODE
#define LRCLK_HI_LO_1979 	    (false)
#else
#define LRCLK_HI_LO_1979 	    (true)
#endif

/*
 * DAC settings
 */
/* DAC Master clock frequency */
#define ADAU1962A_MCLK_IN       (24576000u)
/* DAC sample rate */
#define SAMPLE_RATE_A   			(192000u)

/* ADAU1962A SPORT config parameters */
#define LR_B_CLK_MASTER_1962    (true)
#define BCLK_RISING_1962 	    (true)
#define LRCLK_HI_LO_1962        (true)


/* SPU Peripheral ID */
#define	SPORT_4A_SPU_PID		    (23u)
#define	SPORT_4B_SPU_PID		    (24u)
#define	SPORT_4A_DMA10_SPU_PID		(74u)
#define	SPORT_4B_DMA11_SPU_PID		(75u)


#define PIN_MASK (0x0000007Fu)
#define PIN_LEN  (7u)

#define DATA_MASK (0x0000003Fu)
#define DATA_LEN  (6u)

#define CLK_MASK (0x0000001Fu)
#define CLK_LEN  (5u)

#define FS_MASK (0x0000001Fu)
#define FS_LEN  (5u)

#define PE_MASK (0x0000003Fu)
#define PE_LEN  (6u)







/* IF (Debug info enabled) */
#if defined(ENABLE_DEBUG_INFO)
#define DBG_MSG                     printf
#else
#define DBG_MSG(...)
#endif













/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/

/* Initializes ADC */
extern uint32_t Adau1979Init(void);

/* Submit buffers to ADC */
extern uint32_t Adau1979SubmitBuffers(void);

extern uint32_t Adau1979Enable(void);

extern uint32_t Adau1979DoneWithBuffer(void *pBuffer);
extern void configGpio(void);
float constrain(float input, float low, float high);


void ProcessBuffers(void);



int32_t FIR_init(void);
void reverseArrayf(float*, uint32_t);
int32_t ControlWeightUpdate(void);
int32_t OCPMRefFIR(void);
int32_t OCPMAuxFIR(void);
int32_t OCPMWeightUpdate(void);
uint8_t RefFIR(void);
uint8_t ControlFIR(void);
uint8_t limitSig(void);
uint8_t GenControlSignal(void);
uint8_t PushControlSignal(void);
#ifdef OFPMFilter
uint8_t OFPMFIR(void);
uint8_t OFPMErrorFIR(void);
int32_t OFPMWeightUpdate(void);
int32_t OFPMErrorWeightUpdate(void);
#endif
int32_t ANCALG_1(void);
int32_t ANCALG_2(void);
int32_t ANCALG_3(void);
int32_t ANCALG_4(void);
int32_t ANCALG_5(void);

int32_t WN_Gen(void);

extern uint32_t DMAInit(void);



static void OCPMAuxFIRCallback(void *pCBParam, uint32_t eEvent, void *pArg);


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
void TRNG_Init (void);



/*
 int rand_r_imp(unsigned int *seed);
 */
void aluFLTOIHandler(uint32_t iid, void* handlerArg);



extern int32_t MemDma0Copy1D(void *pMemDest, void *pMemSrc,
		uint8_t nBytesInElement, uint32_t ElementCount);

extern int32_t MemDma0Copy2D(void *pDestBuffer, void *pSrcBuffer,
		uint8_t nBytesInElement, uint32_t Xcount, uint32_t YCount);



void Read_TRNG_Output_Imp(float *iOutput);

static void PKIC_ISR(uint32_t iid, void* handlerArg);

static void TRNG_ISR(void);
static void TRNG_ACK(void);
float VecSumf(const void* x, uint32_t length);

/* Initializes DAC */
extern uint32_t Adau1962aInit(void);
/* Submit buffers to DAC */
extern uint32_t Adau1962aSubmitBuffers(void);
extern uint32_t Adau1962aEnable(void);
extern uint32_t Adau1962aDoneWithBuffer(void *pBuffer);



#endif /* __ANC_CORE1_H__ */
