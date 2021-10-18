/*****************************************************************************
 * anc_Core0.h
 *****************************************************************************/

#ifndef __ANC_TEST0_H__
#define __ANC_TEST0_H__
/*********************************************************************************

Copyright(c) 2015 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*
 * Project		SHARC L1-L1 using MDMA example - Dual SHARC Talk through.
 * Description  Four channels in, SHARC0 filters two channels while SHARC1 filters
 *              the other two channels in parallel, resulting in four filtered channels.
 * 	            The purpose of this project is to demonstrate using
 * 				two MDMA channels to move raw and filtered data from SHARC0 to SHARC1.
 * 				The project takes four channels of audio from the ADAU1979 CODEC into
 * 				SHARC0 L1 SRAM and sends the raw data to SHARC1 L1 SRAM using MDAM channel A0.
 * 				SHARC0 processes two of the four channels at the time that SHARC1
 * 				is processing the other two channels (parallel processing).
 * 				After SHARC0 completes its processing it uses MDMA channel A1 to
 * 				move the filtered audio channels to SHARC1 L1 SRAM. SHARC1 combines its filtered audio channels
 * 				with the filtered audio channels from SHARC0 resulting in four channels of filtered audio.
 * 				The resulting four channels of filtered audio is sent from SHARC1 L1 SRAM to
 * 				the ADAU1962 DAC driver running on SHARC1. The ARM is used during initialization
 * 				but is not used during runtime.
 *
 *              SHARC0							| SHARC1
 *              ---------------------------------------------------------------
 *			T0	Init ADAU1979 ADC				| Init ADAU1962 DAC
 *			T1	Initialize MDMA channels		| Wait for MDMA init complete
 *			T2	Wait for MDMA interrupt install | Install MDMA interrupt handlers
 *			T3  Enable ADAU1979					| Wait for first audio frame
 *
 *			T4	Send raw audio frame over MDMAA0| MDMA A0 complete callback
 *			T5	Filter audio channels 0,2		| Filter audio channels 1,3  <<<<<<<<<<<<<<<<<<<
 *			T6	Send filtered audio over MDMA A1| MDMA A1 complete callback
 *			T7	Wait for another audio frame	| Merge audio channels, send to ADAU1962 DAC
 *
 * 				--------------   -------------------------------------
 * 				|            |   |     SHARC0 filters channels 0,2   |
 * 				|  ADAU1979  |-->| 4 channels in                     |
 * 				|            |   |                     filtered audio|
 * 				--------------   -------------------------------------
 * 				                      | MDMA A0             | MDMA A1
 * 				                      | raw audio           | 2 channels filtered
 * 				                 ----------------------------------------   ------------
 * 				                 |SHARC1 filters channels 1,3 and merges|   |          |
 *                               |                        4 channels out|-->| ADAU1962 |
 *                               |  SHARC0 chan 0,2 + SHARC1 chan 1,3   |   |          |
 *                               ----------------------------------------   ------------
 *
 *				Griffin's MDMA hardware provides the unique ability to perform data transfer and
 *				signaling in the same package. Data can be transfered directly from SHARC0 L1 to
 *				SHARC1 L1 with both SHARC's getting signaled when the transfer completes. Multiple
 *				MDMA channels are supported and can run concurrently. Each channel provides a unique
 *				signal when the transfer completes.
 *
 *
 * File
 * Description	This is the first user code run out of reset main().
 * 				The ARM runs first out of reset, initializes the SPU, GPIO, and power.
 * 				The board softswitches are also configured by the ARM.
 * 				The Sharcs are held in reset until the ARM completes its initialization.
 * 				After the ARM lets the Sharcs run, it sits in a loop.
 *
 */


/* Macro to specify delay count for ADC/DAC reset */
#define DELAY_COUNT             (100000u)


/* Core clock is generated from CGU0 */
#define CGU_DEV                 	(0)
#define MHZ                     	(1000000u)
#define CLKIN                   	(25u * MHZ)
#define CORE_MAX                	(450u * MHZ)
#define SYSCLK_MAX              	(225u * MHZ)
#define SCLK_MAX                	(112u * MHZ)

/* SPU Peripheral ID */
#define	SPORT_4A_SPU_PID		    (23u)
#define	SPORT_4B_SPU_PID		    (24u)
#define	SPORT_4A_DMA10_SPU_PID		(74u)
#define	SPORT_4B_DMA11_SPU_PID		(75u)

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


/* ADSP-SC589 Processor family */
#if defined(__ADSPSC589_FAMILY__)

/* SPU PID for MDMA0 Source */
#define MDMA0_SRC_DMA8_SPU_PID      (88u)
/* SPU PID for MDMA0 Destination */
#define MDMA0_DST_DMA9_SPU_PID      (89u)



// SPU PID for MDMA1 Source
#define MDMA1_SRC_DMA18_SPU_PID      (90u)
// SPU PID for MDMA1 Destination
#define MDMA1_DST_DMA19_SPU_PID      (91u)

// SPU PID for MDMA2
#define MDMA2_Medium_BW_MDMA_SPU_PID      (140u)

// SPU PID for MDMA3
#define MDMA3_Maximum_BW_MDMA_SPU_PID      (141u)


#endif // __ADSPSC589_FAMILY__

#define PIN_MASK 					(0x0000007Fu)
#define PIN_LEN  					(7u)

#define DATA_MASK 					(0x0000003Fu)
#define DATA_LEN  					(6u)

#define CLK_MASK 					(0x0000001Fu)
#define CLK_LEN  					(5u)

#define FS_MASK 					(0x0000001Fu)
#define FS_LEN  					(5u)

#define PE_MASK 					(0x0000003Fu)
#define PE_LEN  					(6u)

/* IF (Debug info enabled) */
#if defined(ENABLE_DEBUG_INFO)
#define DBG_MSG                     printf
#else
#define DBG_MSG(...)
#endif


#endif /* __ANC_CORE0_H__ */

