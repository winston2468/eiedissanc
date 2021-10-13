/*
 * PCGInit.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include "anc_test2.h"

#include <services/pcg/adi_pcg.h>
/* PCG - C */
uint8_t PcgMemory[ADI_PCG_MEMORY_SIZE];
static ADI_PCG_HANDLE phPcg;

/*
 * Opens and initializes PCG Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */

uint32_t PcgDacInit(void) {
	uint32_t Result = 0u;

	/* Init PCG
	 *
	 * Using ext clk 24.576MHz
	 *
	 * CLK = numASRC * 64 * Fs Hz
	 * FS = 24 kHz
	 */

	uint32_t DeviceNum;
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	DeviceNum = 2u; /* PCG C */
#elif defined(__ADSPSC573_FAMILY__)
	DeviceNum = 0u; /* PCG A */
#else
#error "processor not supported"
#endif

	/* Open PCG */
	if (adi_pcg_Open(DeviceNum, &PcgMemory[0],
	ADI_PCG_MEMORY_SIZE, &phPcg) != ADI_PCG_SUCCESS) {
		Result = 1u;
	}

	/* Select FS and clock outputs for PCG */
	if ((uint32_t) adi_pcg_SelectOutput(phPcg, ADI_PCG_OUT_CLK_FS) != 0u) {
		/* return error */
		return 1u;
	}

	/* Set the PCG input clock source to external*/
	if ((uint32_t) adi_pcg_ClockSource(phPcg, ADI_PCG_CLK_EXT) != 0u) {
		/* return error */
		return 1u;
	}

	/* Set the PCG input fs to external */
	if ((uint32_t) adi_pcg_FrameSyncSource(phPcg, ADI_PCG_FS_EXT) != 0u) {
		/* return error */
		return 1u;
	}

	/* Clock C numASRC * 64 * Fs Hz */
	if ((uint32_t) adi_pcg_ClockDivide(phPcg, pcgCLKDIV) != 0u) {
		/* return error */
		return 1u;
	}
	/* FS C kHz */
	if ((uint32_t) adi_pcg_FrameSyncDivide(phPcg, pcgFSDIV) != 0u) {
		/* return error */
		return 1u;
	}

	 if( Result == 1u){
		 printf("PCG Initialization failure");
		 return 1u;
	 }

	return Result;
}


uint32_t PcgDacEnable(void){

	 // Enable the PCG

	 if( (uint32_t) adi_pcg_Enable(phPcg, true) == 1u){
		 printf("PCG Enable failure");
		 return 1u;
	 }
	 return 0u;


}
