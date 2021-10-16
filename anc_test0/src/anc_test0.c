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
 * 				The SHARCs are held in reset until the ARM completes its initialization.
 * 				After the ARM lets the SHARCs run, it sits in a loop.
 *
 */


#include <sys/platform.h>
#include <sys/adi_core.h>
#include "adi_initialize.h"
#include <services/spu/adi_spu.h>
#include <services/pwr/adi_pwr.h>
#include <services/gpio/adi_gpio.h>
#include <stdio.h>
#include "anc_test0.h"


//*****************************************************************************
// External Functions
//*****************************************************************************

/* Configures soft switches */
extern void ConfigSoftSwitches(void);


//*****************************************************************************
// Local Variables
//*****************************************************************************
/* Gpio */
static uint32_t GpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE];
static uint32_t gpioMaxCallbacks;
/* Memory required for the SPU operation */
static uint8_t  SpuMemory[ADI_SPU_MEMORY_SIZE];
/* SPU handle */
static ADI_SPU_HANDLE hSpu;


//*****************************************************************************
// Local Functions
//*****************************************************************************

/* Initialize power service */
uint32_t    PowerServiceInit(void);
/* Initialize GPIO and reset peripherals */
uint32_t    GpioInit(void);

/**
 * If you want to use command program arguments, then place them in the following string.
 */
char __argv_string[] = "";

int main(int argc, char *argv[])
{
	uint32_t Result = 0u;

	/**
	 * Initialize managed drivers and/or services that have been added to 
	 * the project.
	 * @return zero on success 
	 */
	adi_initComponents();


	/* enable all DAI pads */
	*pREG_PADS0_DAI0_IE=0xffffffff;
	*pREG_PADS0_DAI1_IE=0xffffffff;
	//SC589 adau1979
	/* PADS0 DAI0 Port Input Enable Control Register */
	*pREG_PADS0_DAI0_IE = (unsigned int) 0x001FFFFE;

	/* PADS0 DAI1 Port Input Enable Control Register */
	*pREG_PADS0_DAI1_IE = (unsigned int) 0x001FFFFE;

	//FIR driver SPU
	*pREG_SPU0_SECUREP155 = 2u;

	/* Software Switch Configuration for the Griffin EZ-Board */
	ConfigSoftSwitches();

	/* Initialise power service */
	if(Result == 0u)
	{
	Result = PowerServiceInit();
	}

	/* Initialise GPIO for ADC/DAC reset control */
	if(Result == 0u)
	{
		Result = GpioInit();
	}

    /* Initialize SPU Service */
    if(adi_spu_Init(0, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to initialize SPU service\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA0 Source to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA0_SRC_DMA8_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Source\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA0 Destination to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA0_DST_DMA9_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA1 Source to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA1_SRC_DMA18_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Source\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA1 Destination to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA1_DST_DMA19_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA2  to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA2_Medium_BW_MDMA_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make MDMA3  to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, MDMA3_Maximum_BW_MDMA_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make FIR0  to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, 155, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for MDMA 0 Destination\n");
        return (ADI_SPU_FAILURE);
    }

	/**
	 * The default startup code does not include any functionality to allow
	 * core 0 to enable core 1 and core 2. A convenient way to enable
	 * core 1 and core 2 is to use the adi_core_enable function. 
	 */
	adi_core_enable(ADI_CORE_SHARC0);
	adi_core_enable(ADI_CORE_SHARC1);

	/* Begin adding your custom code here */
	while(1) {;}

	return 0;
}


/*
 * Initializes power service
 */
uint32_t PowerServiceInit(void)
{
	uint32_t Result = 0u;

	if((uint32_t)adi_pwr_Init(CGU_DEV, CLKIN) != 0u)
	{
		/* return error */
		return 1u;
	}
	if((uint32_t)adi_pwr_SetPowerMode(CGU_DEV, ADI_PWR_MODE_FULL_ON) != 0u)
	{
		/* return error */
		return 1u;
	}
	if((uint32_t)adi_pwr_SetFreq(CGU_DEV, CORE_MAX, SYSCLK_MAX) != 0u)
	{
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Initializes GPIO service
 * A GPIO line is used to control reset of the ADC and DAC devices
 */
uint32_t GpioInit(void)
{
	uint32_t Result = 0u;
	/* Loop variable */
	volatile uint32_t i;

	if((uint32_t)adi_gpio_Init((void*)GpioMemory, ADI_GPIO_CALLBACK_MEM_SIZE, &gpioMaxCallbacks) != 0u)
	{
		/* return error */
		return 1u;
	}
	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}
	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	return Result;
}

