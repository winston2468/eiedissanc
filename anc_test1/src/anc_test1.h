/*****************************************************************************
 * anc_test1.h
 *****************************************************************************/

#ifndef __ANC_TEST1_H__
#define __ANC_TEST1_H__



/* The SPORT device selection depends on which BF609 EZ-Board connector
 * the Audio EZ-Extender is attached.
 * P1A - Sport TX 0, Sport RX 0
 * P1B - Can't be used due to SoftSwitch conflict with Push Button 2
 * P2A - Can't be used due to SoftSwitch conflict with Push Button 2
 * P3A - Sport TX 2, Sport RX 2
 */
#define SPORT_TX_DEVICE  2
#define SPORT_RX_DEVICE  2

/* TWI device instance used for communicating with the codec device */
#define TWI_DEV_NUM             0

#define PUSH_BUTTON1_PORT (ADI_GPIO_PORT_F)
/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON1_PIN            (ADI_GPIO_PIN_0)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT_PIN       (ADI_GPIO_PIN_0)
#define PUSH_BUTTON1_PINT           (ADI_GPIO_PIN_INTERRUPT_4)

#define PUSH_BUTTON2_PORT (ADI_GPIO_PORT_F)
/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON2_PIN            (ADI_GPIO_PIN_1)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT_PIN       (ADI_GPIO_PIN_1)
#define PUSH_BUTTON2_PINT           (ADI_GPIO_PIN_INTERRUPT_4)

/* GPIO port to which LED 1 is connected to */
#define LED1_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 1 is connected to */
#define LED1_PIN                    (ADI_GPIO_PIN_13)

/*
 * LED 2 GPIO settings
 */

/* GPIO port to which LED 2 is connected to */
#define LED2_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 2 is connected to */
#define LED2_PIN                    (ADI_GPIO_PIN_14)

/*
 * DAC settings
 */
/* DAC Master clock frequency */
#define ADAU1962A_MCLK_IN       (24576000u)
/* DAC sample rate */
#define SAMPLE_RATE_A   			(48000u)

/* ADAU1962A SPORT config parameters */
#define LR_B_CLK_MASTER_1962    (true)
#define BCLK_RISING_1962 	    (true)
#define LRCLK_HI_LO_1962        (false)


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




/* Sine wave parameters */
#define REFERENCE_FREQ 				(3000u)
#define SAMPLES_PER_PERIOD 			(SAMPLE_RATE) / (REFERENCE_FREQ)
#define AMPLITUDE					((float)3388607)
#define PI							((float)3.141592765309)
#define SAMPLE_SIZE 				(4u)

#define NUM_CHANNELS				(4u)

/* Macro to set buffer size */
//#define AUDIO_BUFFER_SIZE 	        (SAMPLES_PER_PERIOD * NUM_CHANNELS * SAMPLE_SIZE * 4u)
#define AUDIO_BUFFER_SIZE 	        ((NUM_AUDIO_SAMPLES/2)*8*sizeof(uint32_t))

/* IF (Debug info enabled) */
#if defined(ENABLE_DEBUG_INFO)
#define DBG_MSG                     printf
#else
#define DBG_MSG(...)
#endif

#endif /* __ANC_CORE1_H__ */
