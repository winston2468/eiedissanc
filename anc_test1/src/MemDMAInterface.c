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
static ADI_DMA_STREAM_HANDLE hMemDmaStream0;
/* Source DMA Handle */
static ADI_DMA_CHANNEL_HANDLE hSrcDmaChannel;
/* Destination DMA Handle */
static ADI_DMA_CHANNEL_HANDLE hDestDmaChannel;

/* Memory to handle DMA Stream */
static uint8_t MemDmaStreamMem[ADI_DMA_STREAM_REQ_MEMORY];
/***** Instances to handle One-shot 2D Memory transfers *****/
static ADI_DMA_2D_MEM_TRANSFER Src_2DMemXfer;
static ADI_DMA_2D_MEM_TRANSFER Dest_2DMemXfer;

extern void MemDmaCallback(void *pCBParam, uint32_t Event, void *pArg);

uint32_t DMAInit(void) {
	ADI_DMA_RESULT eResult = ADI_DMA_SUCCESS;
	eResult = adi_mdma_Open(MEMCOPY_STREAM_ID, &MemDmaStreamMem[0],
			&hMemDmaStream0, &hSrcDmaChannel, &hDestDmaChannel,
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
				hMemDmaStream0);

		/* IF (Failure) */
		if (eResult != ADI_DMA_SUCCESS) {
			DBG_MSG("Failed to set DMA callback, Error Code: 0x%08X\n", eResult);
		}
	}

	return 0;
}

int32_t MemDma0Copy1D(
	    void                            *pMemDest,
	    void                            *pMemSrc,
		uint8_t							nBytesInElement,
	    uint32_t                        ElementCount){
	ADI_DMA_RESULT  eResult;
	ADI_DMA_MSIZE			ElementSize;
    switch( nBytesInElement )													// Set ElementSize to 1,2, or 4 bytes
    {
    	case 1: ElementSize = ADI_DMA_MSIZE_1BYTE;  break;
    	case 2: ElementSize = ADI_DMA_MSIZE_2BYTES; break;
    	case 4: ElementSize = ADI_DMA_MSIZE_4BYTES; break;
    	default: ElementSize = 0; return -1;
    }
	 eResult = adi_mdma_Copy1D (hMemDmaStream0,
			 pMemDest,
			 pMemSrc,
			 ElementSize,
			 ElementCount);

	 // IF (Failure)
	 if (eResult != ADI_DMA_SUCCESS)
	 {
	 DBG_MSG("Failed initialize MDMA 1D Copy CODE: %d \n", eResult);
	 }


return 0u;
}


int32_t MemDma0Copy2D( void *pDestBuffer, void *pSrcBuffer, uint8_t nBytesInElement, uint32_t Xcount, uint32_t YCount ){
	ADI_DMA_RESULT  eResult = ADI_DMA_SUCCESS;
	ADI_DMA_MSIZE			ElementSize;

    // Prepare 2D memory transfer instances
    switch( nBytesInElement )													// Set ElementSize to 1,2, or 4 bytes
    {
    	case 1: ElementSize = ADI_DMA_MSIZE_1BYTE;  break;
    	case 2: ElementSize = ADI_DMA_MSIZE_2BYTES; break;
    	case 4: ElementSize = ADI_DMA_MSIZE_4BYTES; break;
    	default: ElementSize = 0; return -1;
    }


    /* Populate 2D Memory transfer instance for source channel */
    Src_2DMemXfer.pStartAddress    = pSrcBuffer;
    Src_2DMemXfer.YCount           = YCount;                    /* Configure YCount for 2D transfer */
    Src_2DMemXfer.YModify          = 4;
    Src_2DMemXfer.XCount           = Xcount;
    Src_2DMemXfer.XModify          = 4;

    /* Populate 2D Memory transfer instance for destination channel */
    Dest_2DMemXfer.pStartAddress   = pDestBuffer;
    Dest_2DMemXfer.YCount          = YCount;                    /* Configure YCount for 2D transfer */
    Dest_2DMemXfer.YModify         = 4;
    Dest_2DMemXfer.XCount          = Xcount;
    Dest_2DMemXfer.XModify         = 4;

	eResult = adi_mdma_Copy2D (hMemDmaStream0,
	ElementSize,
	&Dest_2DMemXfer,
	&Src_2DMemXfer);

	// IF (Failure)
	if (eResult != ADI_DMA_SUCCESS)
	{
	DBG_MSG("Failed initialize MDMA 2D Copy \n", eResult);
	}

return 0u;
}
