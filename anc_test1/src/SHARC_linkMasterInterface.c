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
#include <services/int/adi_sec.h>
#include "SHARC_linkInterface.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//*****************************************************************************
// MDMA handles
//*****************************************************************************
//
// MDMA used to stream the reference signal/frame to the other SHARC
//
static ADI_DMA_STREAM_HANDLE   hMemDmaStream_Ref;								// DMA Stream Handle
static ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel_Ref;								// Source DMA Handle
static ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel_Ref;								// Destination DMA Handle
static uint8_t MemDmaStreamMem_Ref[ADI_DMA_STREAM_REQ_MEMORY];					// Memory to handle DMA Stream

//
// MDMA used to stream the OSPMWNSignal frame to the other SHARC
//
static ADI_DMA_STREAM_HANDLE   hMemDmaStream_OSPMWNSignal;							// DMA Stream Handle
static ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel_OSPMWNSignal;							// Source DMA Handle
static ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel_OSPMWNSignal;						// Destination DMA Handle
static uint8_t MemDmaStreamMem_OSPMWNSignal[ADI_DMA_STREAM_REQ_MEMORY];				// Memory to handle DMA Stream

//
// MDMA used to stream the control coefficients  to the other SHARC
//
static ADI_DMA_STREAM_HANDLE   hMemDmaStream_ControlCoeff;								// DMA Stream Handle
static ADI_DMA_CHANNEL_HANDLE  hSrcDmaChannel_ControlCoeff;								// Source DMA Handle
static ADI_DMA_CHANNEL_HANDLE  hDestDmaChannel_ControlCoeff;								// Destination DMA Handle
static uint8_t MemDmaStreamMem_ControlCoeff[ADI_DMA_STREAM_REQ_MEMORY];					// Memory to handle DMA Stream

/***** Instances to handle One-shot 2D Memory transfers *****/
static ADI_DMA_2D_MEM_TRANSFER Src_2DMemXferLink;
static ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXferLink;

//*****************************************************************************
// MDMA transfer complete interrupt - Ref data transfer
//*****************************************************************************
static void RefMemDmaCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    /* CASEOF (Event) */
    switch (Event)
    {
    	/* CASE (Processed a one-shot/circular buffer) */
    	case (ADI_DMA_EVENT_BUFFER_PROCESSED):
    	    /* Update memory copy status flag */
    		break;
    }
}

//*****************************************************************************
// MDMA transfer complete interrupt - OSPMWNSignal data transfer
//*****************************************************************************
static void OSPMWNSignalMemDmaCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    /* CASEOF (Event) */
    switch (Event)
    {
    	/* CASE (Processed a one-shot/circular buffer) */
    	case (ADI_DMA_EVENT_BUFFER_PROCESSED):
    	    /* Update memory copy status flag */
    		break;
    }
}

//*****************************************************************************
// MDMA transfer complete interrupt - ControlCoeff data transfer
//*****************************************************************************
static void ControlCoeffMemDmaCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    /* CASEOF (Event) */
    switch (Event)
    {
    	/* CASE (Processed a one-shot/circular buffer) */
    	case (ADI_DMA_EVENT_BUFFER_PROCESSED):
    	    /* Update memory copy status flag */
    		break;
    }
}

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
int SHARC_linkMasterInit( uint32_t *DMASlaveDestinationAddress )
{
	uint32_t			nSid;
	uint32_t			DestAddress;
    ADI_DMA_RESULT      eResult = ADI_DMA_SUCCESS;			// DMA return code

	//*************************************************************************
	// Open MDMA streams
	//*************************************************************************
    //
    // Ref stream
    //
    DEBUGMSG(stdout, "Core1: Opening MDMA REF stream\n" );
    eResult = adi_mdma_Open (MDMA_STREAM_ID_REF,
                             &MemDmaStreamMem_Ref[0],
                             &hMemDmaStream_Ref,
                             &hSrcDmaChannel_Ref,
                             &hDestDmaChannel_Ref,
                             NULL,
                             NULL);

    if (eResult != ADI_DMA_SUCCESS)
    {
    	DEBUGMSG(stdout,"Failed to open MDMA REF stream, Error Code: 0x%08X\n", eResult);
    	return SHARC_LINK_ERROR;
    }

    //
    // OSPMWNGSignal stream
    //
    DEBUGMSG(stdout, "Core1: Opening MDMA OSPMWNGSignal stream\n" );
    eResult = adi_mdma_Open (MDMA_STREAM_ID_OSPMWNSIGNAL,
                             &MemDmaStreamMem_OSPMWNSignal[0],
                             &hMemDmaStream_OSPMWNSignal,
                             &hSrcDmaChannel_OSPMWNSignal,
                             &hDestDmaChannel_OSPMWNSignal,
                             NULL,
                             NULL);

    if (eResult != ADI_DMA_SUCCESS)
    {
    	DEBUGMSG(stdout,"Failed to open MDMA FILTERED stream, Error Code: 0x%08X\n", eResult);
    	return SHARC_LINK_ERROR;
    }

    //
    // ControlCoeff stream
    //
    DEBUGMSG(stdout, "Core1: Opening MDMA FILTERED stream\n" );
    eResult = adi_mdma_Open (MDMA_STREAM_ID_CONTROL_COEFF,
                             &MemDmaStreamMem_ControlCoeff[0],
                             &hMemDmaStream_ControlCoeff,
                             &hSrcDmaChannel_ControlCoeff,
                             &hDestDmaChannel_ControlCoeff,
                             NULL,
                             NULL);

    if (eResult != ADI_DMA_SUCCESS)
    {
    	DEBUGMSG(stdout,"Failed to open MDMA FILTERED stream, Error Code: 0x%08X\n", eResult);
    	return SHARC_LINK_ERROR;
    }

	//*************************************************************************
	// Configure MDMA streams
	//*************************************************************************
    //
    // RAW stream
    //
    adi_mdma_EnableChannelInterrupt(hDestDmaChannel_Ref,false,false);			// Disable the MDMA destination transfer complete interrupt
    adi_mdma_GetChannelSID (hDestDmaChannel_Ref,&nSid);							// Get the channel SID for the MDMA destination complete interrupt
    adi_sec_SetCoreID(nSid, ADI_SEC_CORE_1);									// Set interrupt to occur on Core 2 (unfortunate enumeration name in driver)
    adi_mdma_EnableChannelInterrupt(hSrcDmaChannel_Ref,true,true);				// Enable the MDMA source transfer complete interrupt
    eResult = adi_dma_UpdateCallback (hSrcDmaChannel_Ref, RefMemDmaCallback,
    													hMemDmaStream_Ref); 	// Register source transfer complete interrupt
     /* IF (Failure) */
     if (eResult != ADI_DMA_SUCCESS)
     {
         DEBUGMSG("Failed to set DMA REF stream callback, Error Code: 0x%08X\n", eResult);
         return SHARC_LINK_ERROR;
     }


      //
      // OSPMWNSignal stream
      //
      adi_mdma_EnableChannelInterrupt(hDestDmaChannel_OSPMWNSignal,false,false);		// Disable the MDMA destination transfer complete interrupt
      adi_mdma_GetChannelSID (hDestDmaChannel_OSPMWNSignal,&nSid);					// Get the channel SID for the MDMA destination complete interrupt
      adi_sec_SetCoreID(nSid, ADI_SEC_CORE_1);									// Set interrupt to occur on Core 2 (unfortunate enumeration name in driver)
      adi_mdma_EnableChannelInterrupt(hSrcDmaChannel_OSPMWNSignal,true,true);		// Enable the MDMA source transfer complete interrupt
      eResult = adi_dma_UpdateCallback (hSrcDmaChannel_OSPMWNSignal, OSPMWNSignalMemDmaCallback,
      													hMemDmaStream_OSPMWNSignal);// Register source transfer complete interrupt
       /* IF (Failure) */
       if (eResult != ADI_DMA_SUCCESS)
       {
           DEBUGMSG("Failed to set DMA OSPMWNSignal stream callback, Error Code: 0x%08X\n", eResult);
           return SHARC_LINK_ERROR;
       }

       //
       // ControlCoeff stream
       //
       adi_mdma_EnableChannelInterrupt(hDestDmaChannel_ControlCoeff,false,false);		// Disable the MDMA destination transfer complete interrupt
       adi_mdma_GetChannelSID (hDestDmaChannel_ControlCoeff,&nSid);					// Get the channel SID for the MDMA destination complete interrupt
       adi_sec_SetCoreID(nSid, ADI_SEC_CORE_1);									// Set interrupt to occur on Core 2 (unfortunate enumeration name in driver)
       adi_mdma_EnableChannelInterrupt(hSrcDmaChannel_ControlCoeff,true,true);		// Enable the MDMA source transfer complete interrupt
       eResult = adi_dma_UpdateCallback (hSrcDmaChannel_ControlCoeff, ControlCoeffMemDmaCallback,
       													hMemDmaStream_ControlCoeff);// Register source transfer complete interrupt
        /* IF (Failure) */
        if (eResult != ADI_DMA_SUCCESS)
        {
            DEBUGMSG("Failed to set DMA ControlCoeff stream callback, Error Code: 0x%08X\n", eResult);
            return SHARC_LINK_ERROR;
        }


     *sharc_flag_in_L2 = 1;														// Tell master the MDMA driver is installed

     //*************************************************************************
     // Wait for slave to install interrupt handlers
     //*************************************************************************
     DEBUGMSG(stdout, "Core1: Waiting for slave to install interrupt handlers\n");
     while( *sharc_flag_in_L2 < 2 )
     {
    	 unsigned long CycleDelay = __builtin_emuclk()+450000; // 1 ms delay
    	 while( __builtin_emuclk() < CycleDelay )
    		 asm("nop;");
     }

	//*************************************************************************
	// SHARC_link connection established
	//*************************************************************************
	*DMASlaveDestinationAddress = MDMA_DESTINATION;								// Return DMA destination address
	DEBUGMSG(stdout, "Core1: SHARK link connection established\n");

    return SHARC_LINK_SUCCESS;
}


//*****************************************************************************
// Send a buffer to the slave core using MDMA. Upon transfer completion, an
// interrupt is generated on the slave core.
//*****************************************************************************
int SHARC_linkSend( uint32_t stream, void *pSrcBuffer, void *pDestBuffer, uint8_t nBytesInElement, uint32_t nElementsInBuffer )
{
	ADI_DMA_STREAM_HANDLE	StreamHandle;
	ADI_DMA_MSIZE			ElementSize;
    ADI_DMA_RESULT      	eResult = ADI_DMA_SUCCESS;

    switch( nBytesInElement )													// Set ElementSize to 1,2, or 4 bytes
    {
    	case 1: ElementSize = ADI_DMA_MSIZE_1BYTE;  break;
    	case 2: ElementSize = ADI_DMA_MSIZE_2BYTES; break;
    	case 4: ElementSize = ADI_DMA_MSIZE_4BYTES; break;
    	default: ElementSize = 0; return -1;
    }


    StreamHandle = hMemDmaStream_Ref;


    //*************************************************************************
    // Submit One-shot 1D buffers to Source and destination channel
    //*************************************************************************
    eResult = adi_mdma_Copy1D (StreamHandle,									// initiate MDMA transfer
                              (void *)pDestBuffer,
                              (void *)pSrcBuffer,
                              ElementSize,
                              (nElementsInBuffer / nBytesInElement));

    /* IF (Failure) */
    if (eResult != ADI_DMA_SUCCESS)
    {
        DEBUGMSG(stdout,"Failed to initiate One-shot 1D memory copy, Error Code: 0x%08X\n", eResult);
        return SHARC_LINK_ERROR;
    }

#if 0
    eResult = adi_mdma_IsCopyInProgress (hMemDmaStream, (bool*)&bMemCopyInProgress);
    while( bMemCopyInProgress == true ) // Wait for previous MDMA to complete
    {
    	eResult = adi_mdma_IsCopyInProgress (hMemDmaStream, (bool*)&bMemCopyInProgress);
    }
#endif


    return SHARC_LINK_SUCCESS;
}


//*****************************************************************************
// Send a buffer to the slave core using MDMA. Upon transfer completion, an
// interrupt is generated on the slave core.
//*****************************************************************************
int SHARC_linkSend2D( uint32_t stream, void *pSrcBuffer, void *pDestBuffer, uint8_t nBytesInElement, uint32_t Xcount, uint32_t YCount )
{
	ADI_DMA_STREAM_HANDLE	StreamHandle;
	ADI_DMA_MSIZE			ElementSize;
    ADI_DMA_RESULT      	eResult = ADI_DMA_SUCCESS;

    switch( nBytesInElement )													// Set ElementSize to 1,2, or 4 bytes
    {
    	case 1: ElementSize = ADI_DMA_MSIZE_1BYTE;  break;
    	case 2: ElementSize = ADI_DMA_MSIZE_2BYTES; break;
    	case 4: ElementSize = ADI_DMA_MSIZE_4BYTES; break;
    	default: ElementSize = 0; return -1;
    }
    StreamHandle = hMemDmaStream_OSPMWNSignal;
    if( stream == MDMA_STREAM_ID_CONTROL_COEFF )
    {
    	StreamHandle = hMemDmaStream_ControlCoeff;
    }

    // Prepare 2D memory transfer instances

    /* Populate 2D Memory transfer instance for source channel */
    Src_2DMemXferLink.pStartAddress    = pSrcBuffer;
    Src_2DMemXferLink.YCount           = YCount;                    /* Configure YCount for 2D transfer */
    Src_2DMemXferLink.YModify          = 4;
    Src_2DMemXferLink.XCount           = Xcount;
    Src_2DMemXferLink.XModify          = 4;

    /* Populate 2D Memory transfer instance for destination channel */
    Dest_2DMemXferLink.pStartAddress   = pDestBuffer;
    Dest_2DMemXferLink.YCount          = YCount;                    /* Configure YCount for 2D transfer */
    Dest_2DMemXferLink.YModify         = 4;
    Dest_2DMemXferLink.XCount          = Xcount;
    Dest_2DMemXferLink.XModify         = 4;

    /* IF (Failure) */
    if (eResult != ADI_DMA_SUCCESS)
    {
        DEBUGMSG(stdout,"Failed to initiate One-shot 2D memory copy, Error Code: 0x%08X\n", eResult);
        return SHARC_LINK_ERROR;
    }

#if 0
    eResult = adi_mdma_IsCopyInProgress (hMemDmaStream, (bool*)&bMemCopyInProgress);
    while( bMemCopyInProgress == true ) // Wait for previous MDMA to complete
    {
    	eResult = adi_mdma_IsCopyInProgress (hMemDmaStream, (bool*)&bMemCopyInProgress);
    }
#endif


    return SHARC_LINK_SUCCESS;
}


