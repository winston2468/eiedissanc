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
 * File
 * Description	In order to use MDMA to transfer data between cores, the driver needs
 * 				to be setup in a particular sequence. SHARC1 is the source or master
 * 				core. SHARC2 is the destination or slave core.
 * 				SHARC1 Opens and configures the MDMA channel. Using L2 it sends a signal to SHARC2.
 * 				SHARC2 installs a MDMA complete interrupt handler AFTER receiving the signal from SHARC1.
 * 				SHARC2 sends a signal to SHARC1 telling it the interrupt handler is installed.
 * 				After SHARC1 receives the signal, it starts sending audio frames to SHARC2 via MDMA.
 *
 * 				L1 Master and Slave Ports
 *					Each SHARC+ core has two master and two slave ports to/from
 *					the system fabric. One master port fetches instructions; the second
 *					master port drives data to the system world. Both slave
 *					ports allow conflict free core/DMA streams to the individual
 *					memory blocks. For slave port addresses, refer to the L1 memory
 *					address map.
 *
 *				The MDMA SID and destination address are hard coded and shared between cores in a header file.
 *				For MDMA to write to SHARC L1 it must use the MULTIPROCESSOR SPACE.
 *
 *				SHARC2 L1 local address | MDMA destination address SHARC2 Slave port 1,Slave port 2
 *				-----------------------------------------------------------------
 *				L1 Block 0 = 0x00240000 | 0x28A40000, 0x28E40000
 *				L1 Block 1 = 0x002C0000 | 0x28AC0000, 0x28EC0000
 *				L1 Block 2 = 0x00300000 | 0x28B00000, 0x28F00000
 *				L1 Block 3 = 0x00380000 | 0x28B80000, 0x28F80000
 *
 */
#include <services/dma/adi_dma.h>
#include <services/mcapi/mcapi.h>
#include "SHARC_linkInterface.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <services/int/adi_sec.h>
//*****************************************************************************
// Called from Master Sharc (sending Sharc) to initialize the SHARC link.
// Uses MCAPI to establish SHARC link. SHARC link does not use MCAPI during runtime.
//
//               MASTER                     |                 SLAVE
// 1) Master opens/configures MDMA channel  |  Slave waits for signal from Master
// 2) Master sends signal to slave via L2   |
// 3)                                       |  Slave gets signal from master via L2
// 4) Master waits for signal from slave    |  Slave installs MDMA complete interrupt handler
// 5)                                       |  Slave sends signal to master via L2
// 6) Master starts sending audio frames to slave
//*****************************************************************************
int SHARC_linkSlaveInit( ADI_INT_HANDLER_PTR pfHandler_Ref, ADI_INT_HANDLER_PTR pfHandler_OSPMWNSignal, ADI_INT_HANDLER_PTR pfHandler_ControlCoeff, void *DMASlaveDestinationAddress )
{
	// Wait for the master SHARC to open the MDMA channel
    while( *sharc_flag_in_L2 != 1 )
    {
   	 unsigned long CycleDelay = __builtin_emuclk()+450000; // 1 ms delay
   	 while( __builtin_emuclk() < CycleDelay )
   		 asm("nop;");
    }

	//*************************************************************************
	// Insert interrupt handler based on Sid
	//*************************************************************************
    adi_int_InstallHandler (MDMA_SID_REF, pfHandler_Ref, NULL, true);
	DEBUGMSG( stdout, "Core2: MDMA REF interrupt handler installed\n" );

	adi_int_InstallHandler (MDMA_SID_OSPMWNSIGNAL, pfHandler_OSPMWNSignal, NULL, true);
	DEBUGMSG( stdout, "Core2: MDMA OSPMWNSignal interrupt handler installed\n" );

	adi_int_InstallHandler (MDMA_SID_CONTROL_COEFF, pfHandler_ControlCoeff, NULL, true);
	DEBUGMSG( stdout, "Core2: MDMA ControlCoeff interrupt handler installed\n" );

	*sharc_flag_in_L2 = 2;														// Tell master SHARC we have installed the handler

	//*************************************************************************
	// Set MDMA destination address
	//*************************************************************************
	*(uint32_t *)DMASlaveDestinationAddress = MDMA_LOCAL_ADDR;
    DEBUGMSG( stdout, "Core2: SHARC link connection established\n" );

	return (SHARC_LINK_SUCCESS);
}
