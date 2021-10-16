/*
 * SPUInterface.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include <stdio.h>
#include <services/spu/adi_spu.h>
#include "anc_test1.h"

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

	 // Make SPORT 4A to generate secure transactions
	 if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true)
	 != ADI_SPU_SUCCESS) {
	 DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
	 return (ADI_SPU_FAILURE);
	 }



	 // Make SPORT 4A DMA to generate secure transactions
	 if (adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true)
	 != ADI_SPU_SUCCESS) {
	 DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
	 return (ADI_SPU_FAILURE);
	 }



	// Make MDMA0 Source to generate secure transactions
	if (adi_spu_EnableMasterSecure(hSpu, MDMA0_SRC_DMA8_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for MDMA 0 Source\n");
		return (ADI_SPU_FAILURE);
	}

	// Make MDMA0 Destination to generate secure transactions
	if (adi_spu_EnableMasterSecure(hSpu, MDMA0_DST_DMA9_SPU_PID, true)
			!= ADI_SPU_SUCCESS) {
		DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
		return (ADI_SPU_FAILURE);
	}

	return (Result);
}
