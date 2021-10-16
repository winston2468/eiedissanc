/*
 * SPUInterface.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include <stdio.h>
#include <services/spu/adi_spu.h>
#include "anc_test2.h"

/* Memory required for the SPU operation */
static uint32_t SpuMemory[ADI_SPU_MEMORY_SIZE];
/* SPU handle */
static ADI_SPU_HANDLE hSpu;


uint32_t SpuInit(void) {

	uint32_t Result = 0u;

	//Initialize SPU Service
	if (adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to initialize SPU service\n");
		return (ADI_SPU_FAILURE);
	}


	/* Make SPORT 4B to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
		return (ADI_SPU_FAILURE);
	}


	/* Make SPORT 4B DMA to generate secure transactions */
	if (adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
		return (ADI_SPU_FAILURE);
	}

	return (Result);
}
