/*****************************************************************************
 * anc_test1.h
 *****************************************************************************/

#ifndef __ANC_TEST1_H__
#define __ANC_TEST1_H__

#define LowPassFilter
#define NUM_AUDIO_SAMPLES       1024u

//#define TAP_LENGTH 128u
//#define WINDOW_SIZE 128u
#define refLength 64u
#define refWindowSize 64u
#define controlLength 64u
#define controlWindowSize 64u
#define OSPMLength 64u
#define OSPMWindowSize 64u
#define numErrorSignal 3
#define numControlSignal 6
#define NUM_DAC_CHANNELS				(8u)
#define BUFFER_SIZE_1761      (NUM_AUDIO_SAMPLES*sizeof(uint32_t))
#define AUDIO_BUFFER_SIZE 	        ((NUM_AUDIO_SAMPLES/2)*NUM_DAC_CHANNELS*sizeof(uint32_t))

/* select sample rate */
#define SAMPLE_RATE  (ADI_ADAU1761_SAMPLE_RATE_32KHZ)


/* The SPORT device selection depends on which BF609 EZ-Board connector
 * the Audio EZ-Extender is attached.
 * P1A - Sport TX 0, Sport RX 0
 * P1B - Can't be used due to SoftSwitch conflict with Push Button 2
 * P2A - Can't be used due to SoftSwitch conflict with Push Button 2
 * P3A - Sport TX 2, Sport RX 2
 */

#define SPORT_RX_DEVICE1  2
#define SPORT_2B_SPU_PID  20
#define SPORT_2B_DMA5_SPU_PID 71

#define SPORT_RX_DEVICE2  0
#define SPORT_0B_SPU_PID  16
#define SPORT_0B_DMA1_SPU_PID 67

/* Macro to specify delay count for DAC reset */
#define DELAY_COUNT             (100000u)

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
 * DAC settings
 */
/* DAC Master clock frequency */
#define ADAU1962A_MCLK_IN       (24576000u)
/* DAC sample rate */
#define SAMPLE_RATE_A   			(32000u)

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







/* IF (Debug info enabled) */
#if defined(ENABLE_DEBUG_INFO)
#define DBG_MSG                     printf
#else
#define DBG_MSG(...)
#endif

#endif /* __ANC_CORE1_H__ */
