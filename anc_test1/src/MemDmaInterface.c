/*
 * MDMAInterface.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include <stdio.h>
#include <services/int/adi_int.h>
#include <services/dma/adi_dma.h>
#include "anc_test1.h"

/* DMA Stream Handle */
static ADI_DMA_STREAM_HANDLE hMemDmaStream;
/* Source DMA Handle */
static ADI_DMA_CHANNEL_HANDLE hSrcDmaChannel;
/* Destination DMA Handle */
static ADI_DMA_CHANNEL_HANDLE hDestDmaChannel;

/* Memory to handle DMA Stream */
static uint8_t MemDmaStreamMem[ADI_DMA_STREAM_REQ_MEMORY];


extern void MemDmaCallback(void);

uint32_t DMAInit(void) {
	ADI_DMA_RESULT eResult = ADI_DMA_SUCCESS;
	eResult = adi_mdma_Open(MEMCOPY_STREAM_ID, &MemDmaStreamMem[0],
			&hMemDmaStream, &hSrcDmaChannel, &hDestDmaChannel,
			NULL,
			NULL);

	/* IF (Failure) */
	if (eResult != ADI_DMA_SUCCESS) {
		DBG_MSG("Failed to open MDMA stream, Error Code: 0x%08X\n", eResult);
	}

	/* IF (Success) */
	if (eResult == ADI_DMA_SUCCESS) {
		/* Set DMA Callback Function. No need to set for source channel since callback is not supported for it */
		eResult = adi_dma_UpdateCallback(hDestDmaChannel, MemDmaCallback,
				hMemDmaStream);

		/* IF (Failure) */
		if (eResult != ADI_DMA_SUCCESS) {
			DBG_MSG("Failed to set DMA callback, Error Code: 0x%08X\n", eResult);
		}
	}

	return 0;
}

uint32_t DmaTransfer1D(
	    void                            *pMemDest,
	    void                            *pMemSrc,
	    ADI_DMA_MSIZE                   eElementWidth,
	    uint32_t                        ElementCount){
	ADI_DMA_RESULT  eResult;
	 eResult = adi_mdma_Copy1D (hMemDmaStream,
			 pMemDest,
			 pMemSrc,
			 eElementWidth,
			 ElementCount);

	 // IF (Failure)
	 if (eResult != ADI_DMA_SUCCESS)
	 {
	 DBG_MSG("Failed initialize MDMA 1D Copy CODE: %d \n", eResult);
	 }


return 0u;
}


uint32_t DmaTransfer2D(ADI_DMA_2D_MEM_TRANSFER Src_2DMemXfer,
		ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXfer){
	ADI_DMA_RESULT  eResult;
	/*
	// Populate 2D Memory transfer instance for source channel
	Src_2DMemXfer.pStartAddress    = &array[0];
	Src_2DMemXfer.YCount           = numErrorSignal;                    // Configure YCount for 2D transfer
	Src_2DMemXfer.YModify          = 4;
	Src_2DMemXfer.XCount           = OSPMWindowSize;
	Src_2DMemXfer.XModify          = 4;

	// Populate 2D Memory transfer instance for destination channel
	Dest_2DMemXfer.pStartAddress   = &array1[0];
	Dest_2DMemXfer.YCount          = numErrorSignal;                    // Configure YCount for 2D transfer
	Dest_2DMemXfer.YModify         = 4;
	Dest_2DMemXfer.XCount          = OSPMWindowSize;
	Dest_2DMemXfer.XModify         = 4;
*/
	eResult = adi_mdma_Copy2D (hMemDmaStream,
	MEMCOPY_MSIZE,
	&Dest_2DMemXfer,
	&Src_2DMemXfer);

	// IF (Failure)
	if (eResult != ADI_DMA_SUCCESS)
	{
	DBG_MSG("Failed initialize MDMA 2D Copy \n", eResult);
	}

	 // IF (Failure)
	 if (eResult != ADI_DMA_SUCCESS)
	 {
	 DBG_MSG("Failed initialize MDMA 1D Copy CODE: %d \n", eResult);
	 }


return 0u;
}



