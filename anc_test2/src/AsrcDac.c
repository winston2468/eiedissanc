/*
 * ASRCInit.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include "anc_test2.h"
#include <drivers/asrc/adi_asrc.h>

uint8_t AsrcMemory4[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc4;
uint8_t AsrcMemory5[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc5;
uint8_t AsrcMemory6[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc6;
uint8_t AsrcMemory7[ADI_ASRC_MEMORY_SIZE];
static ADI_ASRC_HANDLE phAsrc7;




/*
 * Opens and initializes ASRC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t AsrcDacInit(void) {
	uint32_t Result = 0u;
	ADI_ASRC_SPORT_CONFIG SportConfig;

	uint32_t nBlockNum;
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	nBlockNum = 1u;
#elif defined(__ADSPSC573_FAMILY__)
	nBlockNum = 0u;
#else
#error "processor not supported"
#endif
	/* For ADSP-SC573 family processors, open ASRC 0.  Otherwise open ASRC 4 */
	if (adi_asrc_Open(nBlockNum, 0u, &AsrcMemory4[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc4) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	/* For ADSP-SC573 family processors, open ASRC 1.  Otherwise open ASRC 5 */
	if (adi_asrc_Open(nBlockNum, 1u, &AsrcMemory5[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc5) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	/* For ADSP-SC573 family processors, open ASRC 2.  Otherwise open ASRC 6 */
	if (adi_asrc_Open(nBlockNum, 2u, &AsrcMemory6[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc6) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}

	/* For ADSP-SC573 family processors, open ASRC 3.  Otherwise open ASRC 7 */
	if (adi_asrc_Open(nBlockNum, 3u, &AsrcMemory7[0],
	ADI_ASRC_MEMORY_SIZE, &phAsrc7) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}

	//adi_asrc_SetMatchPhaseMode(phAsrc6, true);
	if (adi_asrc_SetSerialFormat(phAsrc4, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc5, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc6, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if (adi_asrc_SetSerialFormat(phAsrc7, ADI_ASRC_INPUT_TDM,
			ADI_ASRC_OUTPUT_TDM, ADI_ASRC_WORD_LENGTH_24) != ADI_ASRC_SUCCESS) {
		Result = 1u;
	}
	if(Result ==1u){
		printf("ASRC Initialization Failure");
	}
	return Result;
}



uint32_t AsrcDacEnable(void){

	 // Enable the ASRCs
	 if( (uint32_t) adi_asrc_Enable(phAsrc4, true) == 1u){
		 printf("ASRC 4 Enable failure");
		 return 1u;
	 }
	 if( (uint32_t) adi_asrc_Enable(phAsrc5, true) == 1u){
		 printf("ASRC 5 Enable failure");
		 return 1u;
	 }
	 if( (uint32_t) adi_asrc_Enable(phAsrc6, true) == 1u){
		 printf("ASRC 6 Enable failure");
		 return 1u;
	 }
	 if( (uint32_t) adi_asrc_Enable(phAsrc7, true) == 1u){
		 printf("ASRC 7 Enable failure");
		 return 1u;
	 }

	 return 0u;


}

