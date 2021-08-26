/******************************************************************************

Copyright (c) 2012 Analog Devices.  All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.
 *******************************************************************************
 *
 * @file:    adi_adau1761.c
 * @brief:   ADAU1761 low power stereo audio codec device driver implementation
 * @version: $Revision: 17758 $
 * @date:    $Date: 2014-05-22 13:13:39 -0400 (Thu, 22 May 2014) $
 *
 ******************************************************************************/

/* disable misra diagnostics as necessary */
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_8_7:"Objects shall be defined at block scope if they are only accessed from within a single function.")
#pragma diag(suppress:misra_rule_11_1:"Conversions shall not be performed between a pointer to function and any type other than an integral type.")
#pragma diag(suppress:misra_rule_13_7:"Boolean operations whose results are invariant shall not be permitted.")
#pragma diag(suppress:misra_rule_14_7:"A function shall have a single point of exit at the end of the function.")
#pragma diag(suppress:misra_rule_17_4:"Array indexing shall be the only allowed form of pointer arithmetic.")
#endif /* _MISRA_RULES */

/** @addtogroup ADAU1761_Driver ADAU1761 Low power stereo audio codec driver
 *  @{
 *
 *  The ADAU1761 is a low power, stereo audio codec with integrated digital
 *  audio processing that supports stereo 48 kHz record and playback at 14 mW
 *  from a 1.8 V analog supply. The stereo audio ADCs and DACs support sample
 *  rates from 8 kHz to 96 kHz as well as a digital volume control.
 *
 */

#include "adi_adau1761.h"
#include <stddef.h>
#include <string.h>
#include <adi_osal.h>
#include <assert.h>

#if defined(ADI_DEBUG)
#include <stdio.h>
#endif

/* Number of ADAU1761 devices on the board */
#define ADAU1761_NUM_INSTANCES       (sizeof(adau1761Dev)/sizeof(ADI_ADAU1761_DEVICE))

/* SPI Clock rate */
#define ADAU1761_SPI_CLOCK_RATE      (1000u)

/* TWI buffer parameters */
#define TWI_ADDR_SIZE                (2u)
#define TWI_BUFF_SIZE                (32u)
#define PARAM_RAM_ADDR               (0x0000u)
#define PROG_RAM_ADDR                (0x0800u)

#define TWI_PRESCALE                 (120u/10u)
#define TWI_BITRATE                  100u
#define TWI_DUTYCYCLE                50u

/* Forward declaration of the structure pointer to avoid compilation error
 * due to circular dependency.
 */
typedef struct adi_adau1761_info * ADI_ADAU1761_INFO_PTR;

/* Opens the serial communication driver and sets it up */
typedef ADI_ADAU1761_RESULT (* ADI_ADAU1761_OPEN) (
    ADI_ADAU1761_INFO_PTR    pDevInfo
);

/* Closes the serial communication driver */
typedef ADI_ADAU1761_RESULT (* ADI_ADAU1761_CLOSE) (
    ADI_ADAU1761_INFO_PTR    pDevInfo
);

/* Function to read the data from the given register to the given buffer.
 * Supports blocking only */
typedef ADI_ADAU1761_RESULT (* ADI_ADAU1761_READ) (
    ADI_ADAU1761_INFO_PTR    pDevInfo,
    uint16_t                 RegAddr,
    uint16_t                 Length,
    uint8_t                 *pData
);

/* Function to write register data */
typedef ADI_ADAU1761_RESULT (* ADI_ADAU1761_WRITE) (
    ADI_ADAU1761_INFO_PTR     pDevInfo,
    uint16_t                  RegAddr,
    uint16_t                  Length,
    uint8_t                  *pData
);

/* Structure to hold the information regarding the TWI device configuration */
typedef struct adi_adau1761_TWI_info
{
    /* TWI Device number to be used for communicating with ADAU1761 */
    uint32_t                nDeviceNum;

    /* TWI Address for ADAU1761 device. */
    ADI_ADAU1761_TWI_ADDR   eTWIAddr;

    /* TWI serial clock rate in kHz */
    uint16_t                nBitRate;

    /* TWI pre-scale value */
	uint16_t                nPreScale;

    /* TWI duty cycle */
	uint16_t                nDutyCycle;

    /* TWI device handle */
    ADI_TWI_HANDLE          hTWIDevice;

} ADI_ADAU1761_TWI_INFO;

/* Structure to hold the information regarding the SPI device configuration */
typedef struct adi_adau1761_SPI_info
{
    /* SPI Device number to be used for communicating with ADAU1761 */
    uint32_t                nDeviceNum;

    /* SPI Slave select to be used for communicating with ADAU1761 */
    ADI_SPI_SSENABLE        eSlaveSelect;

    /* SPI device handle */
    ADI_SPI_HANDLE          hSPIDevice;

} ADI_ADAU1761_SPI_INFO;

/* Structure to hold the ADAU1761 device related instance data. This structure
 * is defined using the memory passed by the application. */
typedef struct adi_adau1761_info
{
    /* Serial device to be used to communicate with ADAU1761. */
	ADI_ADAU1761_COMM_DEV  eCommDev;

    /* Callback function pointer */
    ADI_CALLBACK            pfCallback;

    /* Callback parameter */
    void *                  pCBParam;

    /* Communication device information */
    void *                  pCommDevInfo;

    /* Handle to the Mutex created with static memory passed from application. */
    ADI_OSAL_MUTEX_HANDLE   hMutex;

    /* Function to open the communication device */
    ADI_ADAU1761_OPEN       pfDevOpen;

    /* Function to close the communication device */
    ADI_ADAU1761_CLOSE      pfDevClose;

    /* Function to read the device registers */
    ADI_ADAU1761_READ       pfDevRead;

    /* Function to write the device registers */
    ADI_ADAU1761_WRITE      pfDevWrite;

    /* Pointer to the memory required for opening the communication device */
    void *                  pCommDevMem;

    /* SPORT device information */
    ADI_ADAU1761_SPORT_INFO  *pSportRxInfo;
    ADI_ADAU1761_SPORT_INFO  *pSportTxInfo;
} ADI_ADAU1761_INFO;

/* ADAU1761 device instance data */
typedef struct adi_adau1761_device
{
	/* driver in use flag */
	bool bInUse;

    /* Pointer to the device instance information */
    ADI_ADAU1761_INFO    *pDevInfo;

} ADI_ADAU1761_DEVICE;


/* ADAU1761 instance information */
static ADI_ADAU1761_DEVICE    adau1761Dev[] =
{
    {
    	false,
    	NULL
    }

    /* Add more instances here */
};

/******************************************************
 Internal functions:
 *******************************************************/

static ADI_ADAU1761_RESULT ReadRegister (
	ADI_ADAU1761_DEVICE        *pDevice,
    uint16_t                    RegAddr,
    uint16_t                    Length,   /* bytes */
    uint8_t                    *pRegData);

static ADI_ADAU1761_RESULT WriteRegister (
	ADI_ADAU1761_DEVICE        *pDevice,
    uint16_t                    RegAddr,
    uint16_t                    Length,   /* bytes */
    uint8_t                    *pRegData);


static ADI_ADAU1761_RESULT TWI_Open(ADI_ADAU1761_INFO const *pDevInfo);

static ADI_ADAU1761_RESULT TWI_Close(ADI_ADAU1761_INFO const *pDevInfo);

static ADI_ADAU1761_RESULT TWI_Read(
    ADI_ADAU1761_INFO    *pDevInfo,
    uint16_t              RegAddr,
    uint16_t              Length,   /* bytes */
    uint8_t              *pData);

static ADI_ADAU1761_RESULT TWI_Write(
    ADI_ADAU1761_INFO     *pDevInfo,
    uint16_t               RegAddr,
    uint16_t               Length,   /* bytes */
    uint8_t               *pData);

static ADI_ADAU1761_RESULT SPI_Open(ADI_ADAU1761_INFO const *pDevInfo);

static ADI_ADAU1761_RESULT SPI_Close(ADI_ADAU1761_INFO const *pDevInfo);

static ADI_ADAU1761_RESULT SPI_Read(
    ADI_ADAU1761_INFO     *pDevInfo,
    uint16_t               RegAddr,
    uint16_t               Length,
    uint8_t               *pData);

static ADI_ADAU1761_RESULT SPI_Write(
    ADI_ADAU1761_INFO     *pDevInfo,
    uint16_t               RegAddr,
    uint16_t               Length,   /* bytes */
    uint8_t               *pData);

#if defined(ADI_DEBUG)
/*
 * Verifies a pointer to a driver points to one of the driver
 * struct's internal to this file.
 */
static bool IsDeviceHandle(ADI_ADAU1761_DEVICE const *pDevice);

static bool IsDeviceHandle(ADI_ADAU1761_DEVICE const *pDevice)
{
	uint32_t i = 0u;
	bool bFound = false;
	for (i=0u; i<ADAU1761_NUM_INSTANCES; i++)
	{
		if (pDevice == &adau1761Dev[i])
		{
			bFound = true;
			break;
		}
	}
	return bFound;
}
#endif

/*
 * Device register access functions.
 */
static ADI_ADAU1761_RESULT ReadRegister (
    ADI_ADAU1761_DEVICE        *pDevice,
    uint16_t                    RegAddr,
    uint16_t                    Length,   /* bytes */
    uint8_t                    *pRegData)
{
    ADI_ADAU1761_INFO        *pDevInfo = pDevice->pDevInfo;
    ADI_ADAU1761_RESULT      eResult = ADI_ADAU1761_SUCCESS;

    /* Open the communication device */
    if(pDevInfo->pfDevOpen(pDevInfo) != ADI_ADAU1761_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

	/* Perform device read */
	if(pDevInfo->pfDevRead (pDevInfo,
							RegAddr,
							Length,
							pRegData) != ADI_ADAU1761_SUCCESS)
	{
		eResult = ADI_ADAU1761_COMM_DEV_FAILED;
	}

    /* Close communication device */
    if(pDevInfo->pfDevClose(pDevInfo) != ADI_ADAU1761_SUCCESS)
    {
    	eResult = ADI_ADAU1761_COMM_DEV_FAILED;
    }

    return eResult;
}

static ADI_ADAU1761_RESULT WriteRegister (
    ADI_ADAU1761_DEVICE        *pDevice,
    uint16_t                    RegAddr,
    uint16_t                    Length,   /* bytes */
    uint8_t                    *pRegData)
{
    ADI_ADAU1761_INFO        *pDevInfo = pDevice->pDevInfo;
    ADI_ADAU1761_RESULT      eResult = ADI_ADAU1761_SUCCESS;
    uint16_t data;

    /* Open the communication device */
    if(pDevInfo->pfDevOpen(pDevInfo) != ADI_ADAU1761_SUCCESS)
    {
    	eResult = ADI_ADAU1761_COMM_DEV_FAILED;
    }

	/* Perform device write */
	if(pDevInfo->pfDevWrite (pDevInfo,
							 RegAddr,
							 Length,
							 pRegData ) != ADI_ADAU1761_SUCCESS)
	{
		eResult = ADI_ADAU1761_COMM_DEV_FAILED;
	}

    /* Close communication device */
    if(pDevInfo->pfDevClose(pDevInfo) != ADI_ADAU1761_SUCCESS)
    {
    	eResult = ADI_ADAU1761_COMM_DEV_FAILED;
    }

    return eResult;
}

/*
 * TWI communication
 */

/* Opens  and sets up the TWI driver */
static ADI_ADAU1761_RESULT TWI_Open (
    ADI_ADAU1761_INFO      const *pDevInfo
)
{
    /* Pointer to TWI device information */
    ADI_ADAU1761_TWI_INFO    *pTWIInfo   = (ADI_ADAU1761_TWI_INFO *) pDevInfo->pCommDevInfo;

    /* Check if driver is already opened, if opened simply return success */
    if(pTWIInfo->hTWIDevice != NULL)
    {
        return ADI_ADAU1761_SUCCESS;
    }

    /* Open TWI device */
    if(adi_twi_Open(pTWIInfo->nDeviceNum,
    		ADI_TWI_MASTER,
    		pDevInfo->pCommDevMem,
    		(uint32_t)ADAU1761_COMM_DEV_MEM_SIZE,
    		&pTWIInfo->hTWIDevice) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the prescale value */
    if(adi_twi_SetPrescale(pTWIInfo->hTWIDevice, pTWIInfo->nPreScale) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the bit rate to Fast frequency (100 kHz) */
    if(adi_twi_SetBitRate(pTWIInfo->hTWIDevice, pTWIInfo->nBitRate) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the duty cycle */
    if(adi_twi_SetDutyCycle(pTWIInfo->hTWIDevice, pTWIInfo->nDutyCycle) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the hardware slave address */
    if(adi_twi_SetHardwareAddress(pTWIInfo->hTWIDevice,
    		(uint16_t)pTWIInfo->eTWIAddr) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    return ADI_ADAU1761_SUCCESS;
}

/* Closes the TWI driver. */
static ADI_ADAU1761_RESULT TWI_Close (
   ADI_ADAU1761_INFO    const  *pDevInfo
)
{
    /* Pointer to TWI device information */
    ADI_ADAU1761_TWI_INFO    *pTWIInfo   = (ADI_ADAU1761_TWI_INFO *) pDevInfo->pCommDevInfo;

    /* Close the TWI device */
    if(adi_twi_Close(pTWIInfo->hTWIDevice) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Clear the TWI handle */
    pTWIInfo->hTWIDevice = NULL;

    return ADI_ADAU1761_SUCCESS;
}

/* Function to read the data using TWI interface. */
static ADI_ADAU1761_RESULT TWI_Read(
    ADI_ADAU1761_INFO           *pDevInfo,
    uint16_t                     RegAddr,
    uint16_t                     Length,
    uint8_t                     *pData)
{
	uint8_t LocalBuf[2];
	ADI_ADAU1761_TWI_INFO *pTWIInfo;

    LocalBuf[0] = (uint8_t)((RegAddr & 0xFF00u) >> (uint8_t)8);
    LocalBuf[1] = (uint8_t)(RegAddr & 0x00FFu);

    /* Pointer to TWI device information */
    pTWIInfo = (ADI_ADAU1761_TWI_INFO *) pDevInfo->pCommDevInfo;

    /*
     * Prepare slave for a register read
     */
    if(adi_twi_Write(pTWIInfo->hTWIDevice,
                     &LocalBuf,
                     2u,
                     true) != ADI_TWI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

	/* Do blocking read */
	if(adi_twi_Read(pTWIInfo->hTWIDevice,
					pData,
					(uint32_t)Length,
					false) != ADI_TWI_SUCCESS)
	{
		return ADI_ADAU1761_COMM_DEV_FAILED;
	}

    return ADI_ADAU1761_SUCCESS;
}

/* write the data using TWI interface */
static ADI_ADAU1761_RESULT TWI_Write(
    ADI_ADAU1761_INFO     *pDevInfo,
    uint16_t               RegAddr,
    uint16_t               Length,
    uint8_t               *pData
)
{
	uint16_t i, dataSize;
	uint32_t size;
    uint32_t addr;
	uint32_t count;
    uint8_t  *ptr;
	uint8_t twiBuf[TWI_BUFF_SIZE];
	ADI_ADAU1761_TWI_INFO *pTWIInfo;

    /* Pointer to TWI device information */
    pTWIInfo = (ADI_ADAU1761_TWI_INFO *) pDevInfo->pCommDevInfo;

    /* break up data into blocks */
    if (RegAddr == PARAM_RAM_ADDR)
    {
    	/* parameter data - set data size to 4 bytes (per data sheet) */
    	dataSize = 4u;
    }
    else if (RegAddr == PROG_RAM_ADDR)
    {
    	/* program data - set data size to 5 bytes (per data sheet) */
    	dataSize = 5u;
    }
    else
    {
    	dataSize = TWI_BUFF_SIZE - TWI_ADDR_SIZE;
    }

    addr = RegAddr;
    count = Length;
    ptr = pData;

    while (count > 0u)
    {
    	/* register start address */
        twiBuf[0] = (uint8_t)((addr & 0xFF00u) >> (uint8_t)8);
        twiBuf[1] = (uint8_t)(addr & 0x00FFu);

        if (count > dataSize)
        {
        	size = dataSize;
        }
        else
        {
        	size = count;
        }

    	/* copy data into transfer buffer after register address */
    	memcpy(&twiBuf[2], ptr, size);

        /* write the address and data */
        if(adi_twi_Write(pTWIInfo->hTWIDevice,
                         twiBuf,
                         size+TWI_ADDR_SIZE,
                         false) != ADI_TWI_SUCCESS)
        {
            return ADI_ADAU1761_COMM_DEV_FAILED;
        }

        if (size == count)
        {
        	/* transfer done */
        	count = 0u;

        }
        else
        {
        	count -= size;
        	addr += size;
        	ptr += size;
        }
    }

    return ADI_ADAU1761_SUCCESS;
}

/*
 * SPI communication
 */

/* Opens and sets up the SPI driver */
static ADI_ADAU1761_RESULT SPI_Open (
    ADI_ADAU1761_INFO      const *pDevInfo
)
{
    /* Pointer to SPI device information */
    ADI_ADAU1761_SPI_INFO *pSPIInfo   = (ADI_ADAU1761_SPI_INFO *) pDevInfo->pCommDevInfo;

    /* Check if driver is already opened, if opened simply return success */
    if(pSPIInfo->hSPIDevice != NULL)
    {
        return ADI_ADAU1761_SUCCESS;
    }

    /* Open SPI device */
    if(adi_spi_Open(pSPIInfo->nDeviceNum,
    		pDevInfo->pCommDevMem,
    		(uint32_t)ADAU1761_COMM_DEV_MEM_SIZE,
    		&pSPIInfo->hSPIDevice) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set master mode */
    if(adi_spi_SetMaster(pSPIInfo->hSPIDevice, true) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set HW slave select */
    if(adi_spi_SetHwSlaveSelect(pSPIInfo->hSPIDevice, false) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Enable underflow errors */
    if(adi_spi_SetTransmitUnderflow(pSPIInfo->hSPIDevice, true) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the SPI clock rate */
    if(adi_spi_SetClock(pSPIInfo->hSPIDevice, (uint16_t)ADAU1761_SPI_CLOCK_RATE) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the chip select */
    if(adi_spi_SetSlaveSelect(pSPIInfo->hSPIDevice, pSPIInfo->eSlaveSelect) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Set the word size */
    if(adi_spi_SetWordSize(pSPIInfo->hSPIDevice, ADI_SPI_TRANSFER_8BIT) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

#if defined(__ADSPBF609_FAMILY__) || defined(__ADSPBF707_FAMILY__)
    /* Set the SPI clock phase */
    if(adi_spi_SetClockPhase(pSPIInfo->hSPIDevice, false) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

	if(adi_spi_SetTxWatermark(pSPIInfo->hSPIDevice,  ADI_SPI_WATERMARK_50,
			ADI_SPI_WATERMARK_DISABLE, ADI_SPI_WATERMARK_DISABLE) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

	if(adi_spi_SetRxWatermark(pSPIInfo->hSPIDevice,  ADI_SPI_WATERMARK_50,
			ADI_SPI_WATERMARK_DISABLE, ADI_SPI_WATERMARK_DISABLE) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }
#endif

    return ADI_ADAU1761_SUCCESS;
}

/* Closes the SPI driver. */
static ADI_ADAU1761_RESULT SPI_Close (
   ADI_ADAU1761_INFO     const  *pDevInfo
)
{
    /* Pointer to SPI device information */
    ADI_ADAU1761_SPI_INFO *pSPIInfo = (ADI_ADAU1761_SPI_INFO *) pDevInfo->pCommDevInfo;

    /* Close the SPI device */
    if(adi_spi_Close(pSPIInfo->hSPIDevice) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    /* Clear the SPI handle */
    pSPIInfo->hSPIDevice = NULL;

    return ADI_ADAU1761_SUCCESS;
}

/* Function to read the data using SPI interface. */
static ADI_ADAU1761_RESULT SPI_Read(
    ADI_ADAU1761_INFO           *pDevInfo,
    uint16_t                     RegAddr,
    uint16_t                     Length,
    uint8_t                     *pData)
{
    /* SPI transceiver instance */
    ADI_SPI_TRANSCEIVER Transceiver;

    /* Pointer to SPI device information */
    ADI_ADAU1761_SPI_INFO *pSPIInfo;

    uint8_t commandWord[3];

    /* command word */
	commandWord[0] = 0x01u;
	commandWord[1] = (uint8_t)((RegAddr & 0xFF00u) >> (uint8_t)8);
	commandWord[2] = (uint8_t)(RegAddr & 0x00FFu);

    pSPIInfo = (ADI_ADAU1761_SPI_INFO *) pDevInfo->pCommDevInfo;

    /* Initialize the transceiver */
    Transceiver.pPrologue           =   &commandWord[0];
    Transceiver.PrologueBytes       =   3u;
    Transceiver.pReceiver           =   pData;
    Transceiver.ReceiverBytes       =   Length;
    Transceiver.pTransmitter        =   NULL;
    Transceiver.TransmitterBytes    =   0u;

    /* Transmit the first sequence */
    if(adi_spi_ReadWrite(pSPIInfo->hSPIDevice, &Transceiver) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    return ADI_ADAU1761_SUCCESS;
}

/* Function to write the data using SPI interface. Assumes single register write */
static ADI_ADAU1761_RESULT SPI_Write(
    ADI_ADAU1761_INFO       *pDevInfo,
    uint16_t                 RegAddr,
    uint16_t                 Length,
    uint8_t                 *pData
)
{
    /* SPI transceiver instance */
    ADI_SPI_TRANSCEIVER Transceiver;

    /* Pointer to SPI device information */
    ADI_ADAU1761_SPI_INFO *pSPIInfo;

    uint8_t LocalBuf[3];

    /* command word */
    LocalBuf[0] = 0x00u;
    LocalBuf[1] = (uint8_t)((RegAddr & 0xFF00u) >> (uint8_t)8);
    LocalBuf[2] = (uint8_t)(RegAddr & 0x00FFu);

    pSPIInfo = (ADI_ADAU1761_SPI_INFO *) pDevInfo->pCommDevInfo;

    /* Initialize the transceiver */
    Transceiver.pPrologue           =   &LocalBuf[0];
    Transceiver.PrologueBytes       =   3u;
    Transceiver.pReceiver           =   NULL;
    Transceiver.ReceiverBytes       =   0u;
    Transceiver.pTransmitter        =   (uint8_t*)pData;
    Transceiver.TransmitterBytes    =   Length;

    /* Transmit the first sequence */
    if(adi_spi_ReadWrite(pSPIInfo->hSPIDevice, &Transceiver) != ADI_SPI_SUCCESS)
    {
        return ADI_ADAU1761_COMM_DEV_FAILED;
    }

    return ADI_ADAU1761_SUCCESS;
}

/*
 * ADAU1761 API's
 */

/**
 * @brief       Open the ADAU1761 driver.
 *
 * @details     This function opens the ADAU1761 driver and upon success returns the handle
 *              to the driver. All the other APIs accept this handle as the parameter
 *              which determines the instance of the driver.
 *
 *              The ADAU1761 supports TWI and SPI for communicating with the host
 *              processor. The driver supports both modes of communication and
 *              the mode can be chosen when opening the driver. Although the
 *              driver supports both the modes, the board design may support only
 *              one of the modes. This has to be set based on the board design.
 *
 * @param [in]  nDeviceNum          ADAU1761 device instance number. There can be
 *                                  more than one ADAU1761 devices on a board. This
 *                                  determines the instance that is to be opened.
 *
 * @param [in]  pDeviceMemory       Memory required for ADAU1761 device operation.
 *                                  This is defined by ADI_ADAU1761_MEMORY_SIZE.
 *                                  The given memory should be word aligned.
 *
 * @param [in]  nMemSize            The size of the memory passed in bytes.
 *
 * @param [in]  eCommDev            Device type to be used for communication. It
 *                                  can be either SPI or TWI.
 *
 * @param [out] phDevice            Pointer to a location where the device handle
 *                                  is written.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                      If successfully opened the device.
 *  - #ADI_ADAU1761_INVALID_DEVICE_NUM       [D] If the given device number is outside
 *                                              the number of instances supported.
 *  - #ADI_ADAU1761_NULL_POINTER             [D] If one of the given pointers are
 *                                              invalid.
 *  - #ADI_ADAU1761_INSUFFICIENT_MEM         [D] If the given memory is not sufficient
 *                                              to operate the device.
 *  - #ADI_ADAU1761_NULL_POINTER             [D] If one of the pointer parameter
 *                                              points to NULL.
 *  - #ADI_ADAU1761_DEVICE_IN_USE            [D] If the given device is already opened.
 *  - #ADI_ADAU1761_MUTEX_FAILED             [D] If failed to create the mutex.
 *
 * @sa          adi_adau1761_Close()
 */
ADI_ADAU1761_RESULT adi_adau1761_Open (
    uint32_t              const     nDeviceNum,
    void *                const     pDeviceMemory,
    uint32_t              const     nMemSize,
    ADI_ADAU1761_COMM_DEV const     eCommDev,
    ADI_ADAU1761_HANDLE * const     phDevice)
{
    /* Pointer to the ADAU1761 device */
    ADI_ADAU1761_DEVICE      *pDevice;
    /* Pointer to the ADAU1761 instance information */
    ADI_ADAU1761_INFO        *pDevInfo;
    ADI_ADAU1761_SPORT_INFO  *pSPORTInfo;
    /* Pointer for keeping track of application supplied memory usage */
    uint8_t                 *pAvailableMem = (uint8_t *) pDeviceMemory;

#ifdef ADI_DEBUG
    /* Check if the given device number is valid */
    if(nDeviceNum > ADAU1761_NUM_INSTANCES)
    {
        return ADI_ADAU1761_INVALID_DEVICE_NUM;
    }

    /* Check if the given memory is sufficient to operate at least in interrupt mode */
    if(nMemSize < ADI_ADAU1761_MEMORY_SIZE)
    {
        return ADI_ADAU1761_INSUFFICIENT_MEM;
    }

    /* Check if the given pointer parameters are valid. */
    if((pDeviceMemory == NULL) || (phDevice == NULL))
    {
        return ADI_ADAU1761_NULL_POINTER;
    }

    /* Asserts to validate the memory size macros */
    assert(ADI_ADAU1761_MEMORY_SIZE >= sizeof(ADI_ADAU1761_INFO) + sizeof(ADI_ADAU1761_TWI_INFO)
    		+ 2u*sizeof(ADI_ADAU1761_SPORT_INFO) + ADI_OSAL_MAX_MUTEX_SIZE_CHAR
    		+ ADAU1761_COMM_DEV_MEM_SIZE);

    assert(ADI_ADAU1761_MEMORY_SIZE >= sizeof(ADI_ADAU1761_INFO) + sizeof(ADI_ADAU1761_SPI_INFO)
    		+ 2u*sizeof(ADI_ADAU1761_SPORT_INFO) + ADI_OSAL_MAX_MUTEX_SIZE_CHAR
    		+ ADAU1761_COMM_DEV_MEM_SIZE);

    /* Check if the given device instance is already opened */
    if(adau1761Dev[nDeviceNum].bInUse)
    {
        return ADI_ADAU1761_DEVICE_IN_USE;
    }
#endif /* ADI_DEBUG */

    pDevice = &adau1761Dev[nDeviceNum];

    /*
     * Fragment the given memory for various needs
     */

    pDevice->pDevInfo = (ADI_ADAU1761_INFO * )pAvailableMem;
    pDevInfo = pDevice->pDevInfo;

    /* Clear the Device Information structure */
    memset((void *)pDevice->pDevInfo, 0, sizeof(ADI_ADAU1761_INFO));

    /* Increment the available memory pointer with amount of memory used */
    pAvailableMem += (sizeof (ADI_ADAU1761_INFO));

    /* Create the bus mutex */
    if(adi_osal_MutexCreateStatic((void *) pAvailableMem,
    		ADI_OSAL_MAX_MUTEX_SIZE_CHAR,
    		&pDevInfo->hMutex) != ADI_OSAL_SUCCESS)
    {
        return ADI_ADAU1761_MUTEX_FAILED;
    }

    /* Increment the available memory pointer with amount of memory used */
    pAvailableMem += ADI_OSAL_MAX_MUTEX_SIZE_CHAR;

    /* Allocate memory for communication device information structure */
    pDevInfo->pCommDevInfo = (void *) pAvailableMem;

    if (eCommDev == ADI_ADAU1761_COMM_DEV_TWI)
    {
    	pAvailableMem += (sizeof (ADI_ADAU1761_TWI_INFO));

    	/* Clear the communication device information structure */
    	memset(pDevInfo->pCommDevInfo, 0, sizeof(ADI_ADAU1761_TWI_INFO));
    }
    else
    {
    	pAvailableMem += (sizeof (ADI_ADAU1761_SPI_INFO));

    	/* Clear the communication device information structure */
    	memset(pDevInfo->pCommDevInfo, 0, sizeof(ADI_ADAU1761_SPI_INFO));
    }

    /* Save the pointer to the memory required to open the communication device */
    pDevInfo->pCommDevMem = (void *) pAvailableMem;
    pAvailableMem += ADAU1761_COMM_DEV_MEM_SIZE;

    /* Save the given parameters */
    pDevInfo->eCommDev = eCommDev;

    /* Initialize the device access function pointers based upon chosen
     * communication device.*/
    if(eCommDev == ADI_ADAU1761_COMM_DEV_TWI)
    {
        pDevInfo->pfDevOpen = (ADI_ADAU1761_OPEN) &TWI_Open;
        pDevInfo->pfDevClose = (ADI_ADAU1761_CLOSE) &TWI_Close;
        pDevInfo->pfDevRead = (ADI_ADAU1761_READ) &TWI_Read;
        pDevInfo->pfDevWrite = (ADI_ADAU1761_WRITE) &TWI_Write;
    }
    else
    {
        pDevInfo->pfDevOpen = (ADI_ADAU1761_OPEN) &SPI_Open;
        pDevInfo->pfDevClose = (ADI_ADAU1761_CLOSE) &SPI_Close;
        pDevInfo->pfDevRead = (ADI_ADAU1761_READ) &SPI_Read;
        pDevInfo->pfDevWrite = (ADI_ADAU1761_WRITE) &SPI_Write;
    }

	/* SPORT Rx device information structure */
	pAvailableMem += (sizeof (ADI_ADAU1761_SPORT_INFO));

	/* SPORT Tx device information structure */
	pAvailableMem += (sizeof (ADI_ADAU1761_SPORT_INFO));

    /* Mark the device as opened */
    pDevice->bInUse = true;

    /* return the handle to the opened device*/
    *phDevice = pDevice;

    return ADI_ADAU1761_SUCCESS;

}

/**
 * @brief       Close the ADAU1761 driver.
 *
 * @details     This function puts the ADAU1761 into shutdown mode and
 *              closes the device.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device to be closed.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully closed the ADAU1761
 *                                         device.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE [D]    If the given ADAU1761 device handle is
 *                                          invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN   [D]    If the given device is not yet opened.
 *  - #ADI_ADAU1761_MUTEX_FAILED             Failed to destroy the mutex.
 *
 * @sa          adi_adau1761_Open()
 */
ADI_ADAU1761_RESULT adi_adau1761_Close (
    ADI_ADAU1761_HANDLE  const       hDevice)
{
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;
    ADI_ADAU1761_INFO *pDevInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

    /* Get the pointer to device information */
    pDevInfo = pDevice->pDevInfo;

    /* disable SPORT */
    adi_adau1761_EnableInput(hDevice, false);
    adi_adau1761_EnableOutput(hDevice, false);

    if ((pDevice->pDevInfo->pSportRxInfo != NULL) &&
    		(pDevice->pDevInfo->pSportRxInfo->hDevice != NULL))
    {
		adi_sport_Close(pDevice->pDevInfo->pSportRxInfo->hDevice);
		pDevice->pDevInfo->pSportRxInfo->hDevice = NULL;
    }

    if ((pDevice->pDevInfo->pSportTxInfo != NULL) &&
    		(pDevice->pDevInfo->pSportTxInfo->hDevice != NULL))
    {
		adi_sport_Close(pDevice->pDevInfo->pSportTxInfo->hDevice);
		pDevice->pDevInfo->pSportTxInfo->hDevice = NULL;
    }

    /* Mark the device as closed */
    pDevice->bInUse = false;

    /* Destroy the mutex */
    if(adi_osal_MutexDestroyStatic(pDevInfo->hMutex) != ADI_OSAL_SUCCESS)
    {
        return ADI_ADAU1761_MUTEX_FAILED;
    }

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Configure the SPI device used for communication.
 *
 * @details     This API can be used to configure the SPI used for communication
 *              with the host processor. These parameters will be based on the
 *              board design. This API should be called only when using SPI
 *              as the communication channel between host processor and ADAU1761.
 *
 * @note        This API should be called before calling any other API so that
 *              accessing the device registers will be successful.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device for which SPI to
 *                                  be configured.
 * @param [in]  nSPIDevNum          SPI device number that is connected
 *                                  to the ADAU1761.
 * @param [in]  eSlaveSelect        Chip select number to be used for addressing
 *                                  the ADAU1761 part.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                   If successfully configured SPI.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE     [D] If the given ADAU1761 device handle is
 *                                          invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN       [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_INVALID_OPERATION     [D] If TWI was chosen as communication
 *                                          channel and trying to configure SPI
 *                                          device.
 *  - #ADI_ADAU1761_MUTEX_FAILED              If mutex related failure occurred.
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_ConfigSPI (
    ADI_ADAU1761_HANDLE  const            hDevice,
    uint32_t             const            nSPIDevNum,
    ADI_SPI_SSENABLE     const            eSlaveSelect
)
{
	ADI_ADAU1761_INFO *pDevInfo;
	ADI_ADAU1761_SPI_INFO *pSPIInfo;
	ADI_ADAU1761_RESULT result;
	ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	/* Get the pointer to device information */
	pDevInfo = pDevice->pDevInfo;

    /* Pointer to SPI device information */
    pSPIInfo = (ADI_ADAU1761_SPI_INFO *) pDevInfo->pCommDevInfo;

    /* Save the given SPI device number and chip select for addressing LCD */
    pSPIInfo->nDeviceNum = nSPIDevNum;
    pSPIInfo->eSlaveSelect = eSlaveSelect;

    /* put into SPI control mode by pulling CLATCH low three times.
     * This is done by performing three dummy writes to the SPI port
     */

    /* Perform three device writes */
	result = WriteRegister (pDevice, 0u, 0u, NULL);
	result = WriteRegister (pDevice, 0u, 0u, NULL);
	result = WriteRegister (pDevice, 0u, 0u, NULL);

    return result;
}

/**
 * @brief       Configure the TWI device used for communication.
 *
 * @details     This API can be used to configure the TWI used for communication
 *              with the host processor. These parameters will be based on the
 *              board design. This API should be called only when using TWI
 *              as the communication channel between host processor and ADAU1761.
 *
 * @note        This API should be called before calling any other API so that
 *              accessing the device registers will be successful.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device whose TWI device
 *                                  be configured.
 *
 * @param [in]  nTWIDevNum          TWI device number that is connected
 *                                  to the ADAU1761.
 *
 * @param [in]  eTWIAddr            TWI Address to be used for communicating with
 *                                  ADAU1761. It can be one of the values from
 *                                  the enumeration #ADI_ADAU1761_TWI_ADDR.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully configured TWI.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is
 *                                          invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_INVALID_OPERATION    [D] If SPI was chosen as communication
 *                                          channel and trying to configure TWI
 *                                          device.
 *  - #ADI_ADAU1761_MUTEX_FAILED             If mutex related failure occurred.
 */
ADI_ADAU1761_RESULT adi_adau1761_ConfigTWI (
    ADI_ADAU1761_HANDLE     const  hDevice,
    uint32_t                const  nTWIDevNum,
    ADI_ADAU1761_TWI_ADDR   const  eTWIAddr)
{
    ADI_ADAU1761_DEVICE      *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;
    ADI_ADAU1761_TWI_INFO    *pTWIInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}

    /* Check if TWI was chosen as communication channel */
    if(pDevice->pDevInfo->eCommDev != ADI_ADAU1761_COMM_DEV_TWI)
    {
        return ADI_ADAU1761_INVALID_OPERATION;
    }
#endif /* ADI_DEBUG */

    /* Store the given TWI related parameters */
    pTWIInfo = (ADI_ADAU1761_TWI_INFO *) pDevice->pDevInfo->pCommDevInfo;

    pTWIInfo->nDeviceNum = nTWIDevNum;
    pTWIInfo->eTWIAddr = eTWIAddr;
    pTWIInfo->nBitRate = TWI_BITRATE;
    pTWIInfo->nPreScale = TWI_PRESCALE;
    pTWIInfo->nDutyCycle = TWI_DUTYCYCLE;

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Configure the SPORT device used for transferring audio data between
 *              the ADAU1761 and host processor.
 *
 * @details     This API is used to configure the SPORT device used for transferring
 *              audio data with the host processor. These parameters will be based on the
 *              board design and transfer mode.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  eDir                Input or Output SPORT configuration.
 *
 * @param [in]  pInfoStruct         Information structure used to configure the SPORT.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully configured SPORT.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_NULL_POINTER         [D] A NULL pointer is invalid for this API.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *
 * @sa adi_adau1761_EnableInput()
 * @sa adi_adau1761_EnableOutput()
 */
ADI_ADAU1761_RESULT adi_adau1761_ConfigSPORT (
    ADI_ADAU1761_HANDLE       const  hDevice,
    ADI_ADAU1761_SPORT_DIR    const  eDir,
    ADI_ADAU1761_SPORT_INFO          *pInfoStruct)
{
    uint32_t nSportMemSize;
    ADI_ADAU1761_SPORT_INFO  *pSPORTInfo;
    ADI_SPORT_DIRECTION       eDirection;
    ADI_SPORT_HANDLE          hSPORTDevice;
    ADI_SPORT_MODE            eSPORTMode = ADI_SPORT_I2S_MODE;
    ADI_ADAU1761_DEVICE      *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif /* ADI_DEBUG */

	if ((pInfoStruct->eDataLen < ADI_ADAU1761_SPORT_DATA_16) || (pInfoStruct->eDataLen > ADI_ADAU1761_SPORT_DATA_32))
	{
		pInfoStruct->eDataLen = ADI_ADAU1761_SPORT_DATA_24; /* default */
	}

	if (eDir == ADI_ADAU1761_SPORT_INPUT)
	{
		pDevice->pDevInfo->pSportRxInfo = pInfoStruct;
		pSPORTInfo = pDevice->pDevInfo->pSportRxInfo;
		eDirection = ADI_SPORT_DIR_RX;
	}
	else
	{
		pDevice->pDevInfo->pSportTxInfo = pInfoStruct;
		pSPORTInfo = pDevice->pDevInfo->pSportTxInfo;
		eDirection = ADI_SPORT_DIR_TX;
	}

	/* open and configure SPORT */
	if (pSPORTInfo->bEnableDMA)
	{
		nSportMemSize = (uint32_t)(ADI_SPORT_DMA_MEMORY_SIZE);
	}
	else
	{
		nSportMemSize = (uint32_t)(ADI_SPORT_INT_MEMORY_SIZE);
	}

	if (pSPORTInfo->eMode != ADI_ADAU1761_SPORT_I2S)
	{
		eSPORTMode = ADI_SPORT_MC_MODE;
	}

    if(adi_sport_Open(pSPORTInfo->nDeviceNum,
    		pSPORTInfo->eChannel,
    		eDirection,
    		eSPORTMode,
    		pSPORTInfo->pMemory,
    		nSportMemSize,
    		&hSPORTDevice) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

    pSPORTInfo->hDevice = hSPORTDevice;

	if (pSPORTInfo->eMode == ADI_ADAU1761_SPORT_I2S)
	{
		/* configure the sport for data length, LSB first etc */
		if(adi_sport_ConfigData(pSPORTInfo->hDevice,
				ADI_SPORT_DTYPE_SIGN_FILL,
				pSPORTInfo->eDataLen,
				false,
				false,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
		/*configure the clock for the SPORT. This API set the whether use the internal clock, SPORT clock etc */
		if(adi_sport_ConfigClock(pSPORTInfo->hDevice,
				1u,
				false,
				true,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
		/* Configure the frame sync. This API configure the SPORT whether to use frame sync or not , external or internal framesync etc */
		if(adi_sport_ConfigFrameSync(pSPORTInfo->hDevice,
				100u,
				true,
				false,
				false,
				false,
				false,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
	}
	else
	{
		/* configure multi-channel mode */

		/* configure the sport for data length, LSB first etc */
		if(adi_sport_ConfigData(pSPORTInfo->hDevice,
				ADI_SPORT_DTYPE_SIGN_FILL,
				pSPORTInfo->eDataLen,
				false,
				false,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
		/*configure the clock for the SPORT. This API set the whether use the internal clock, SPORT clock etc */
		if(adi_sport_ConfigClock(pSPORTInfo->hDevice,
				1u,
				false,
				true,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
		/* Configure the frame sync. This API configure the SPORT whether to use frame sync or not , external or internal framesync etc */
		if(adi_sport_ConfigFrameSync(pSPORTInfo->hDevice,
				100u,
				true,
				false,
				true,
				false,
				false,
				false) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}

		/* configure multi-channel mode for TDM4 */
		if (pSPORTInfo->eMode == ADI_ADAU1761_SPORT_TDM4)
		{
			if (adi_sport_ConfigMC(pSPORTInfo->hDevice,
					1u,
					3u,
					0u,
					false) != ADI_SPORT_SUCCESS)
			{
				return ADI_ADAU1761_SPORT_ERROR;
			}
		}

		/* configure multi-channel mode for TDM8 */
		if (pSPORTInfo->eMode == ADI_ADAU1761_SPORT_TDM8)
		{
			if (adi_sport_ConfigMC(pSPORTInfo->hDevice,
					1u,
					7u,
					0u,
					false) != ADI_SPORT_SUCCESS)
			{
				return ADI_ADAU1761_SPORT_ERROR;
			}
		}
	}

	if (pSPORTInfo->bEnableDMA)
	{
		/* Enable/Disable SPORT DMA */
		if(adi_sport_EnableDMAMode(pSPORTInfo->hDevice, true)
				!= ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}

		/* Enable/Disable streaming mode */
		if(adi_sport_StreamingEnable(pSPORTInfo->hDevice,
				pSPORTInfo->bEnableStreaming) != ADI_SPORT_SUCCESS)
		{
			return ADI_ADAU1761_SPORT_ERROR;
		}
	}

	return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Enable/Disable transfer of audio input data between the ADAU1761 and
 *              host processor.
 *
 * @details     When transfer is enabled the SPORT receive device is enabled which
 *              transfers audio data from the ADAU1761.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  bEnable             True to enable data transfer, false to disable.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully enabled input.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 * @sa adi_adau1761_ConfigSPORT()
 */
ADI_ADAU1761_RESULT adi_adau1761_EnableInput (
		ADI_ADAU1761_HANDLE       const hDevice,
		bool                      const bEnable)
{
    uint32_t nSportError;
    uint32_t Arg;
    ADI_ADAU1761_SPORT_INFO  *pSPORTInfo;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportRxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

	if (bEnable)
	{
		/* clear error status */
		adi_sport_GetHWErrorStatus (pSPORTInfo->hDevice, &nSportError, (void*)&Arg);
	}

	/* Enable/Disable SPORT transfer  */
	if(adi_sport_Enable(pSPORTInfo->hDevice, bEnable) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Enable/Disable transfer of audio output data between the ADAU1761 and
 *              host processor.
 *
 * @details     When transfer is enabled the SPORT transmit device is enabled which
 *              transfers audio data to the ADAU1761.  Two transmit buffers must be
 *              submitted before calling this API.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  bEnable             True to enable data transfer, false to disable.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully enabled output.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *  - #ADI_ADAU1761_NO_BUFFER                A transmit buffer is not available.
 *
 * @sa adi_adau1761_ConfigSPORT()
 * @sa adi_adau1761_SubmitTxBuffer()
 */
ADI_ADAU1761_RESULT adi_adau1761_EnableOutput (
		ADI_ADAU1761_HANDLE       const hDevice,
		bool                      const bEnable)
{
    uint32_t nSportError;
    uint32_t Arg;
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportTxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

	if (bEnable)
	{
		/* clear error status */
		adi_sport_GetHWErrorStatus (pSPORTInfo->hDevice, &nSportError, (void*)&Arg);
	}

	/* Enable/Disable SPORT transfer  */
	if(adi_sport_Enable(pSPORTInfo->hDevice, bEnable) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Registers a transmit callback for reporting the events.
 *
 * @details     The driver can operate with or without a callback. By default the driver is
 *              opened without the callback. It is not recommended to register a callback
 *              unless the application requires the events to be notified immediately.
 *
 *              When callback is registered the adi_adau1761_GetBuffer cannot be used.
 *              The pointer to the buffer provided back to the application as
 *              an argument to the ADI_SPORT_EVENT_BUFFER_PROCESSED event.
 *
 *              When callback is not registered application can use adi_adau1761_GetTxBuffer
 *              or adi_ADAU1761_GetRxBuffer to get back the pointer to buffer.
 *
 *              The callback can only be registered after Sport is configured and
 *              before the output is enabled.
 *
 * @param [in]  hDevice                 Handle to the driver instance with which the callback
 *                                      to be registered.
 *
 * @param [in]  pfCallback              Callback function to be registered.
 *
 * @param [in]  pCBParam                Callback parameter which will be passed back to the
 *                                      application when callback function is called.
 *
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully register the callback.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given adau1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 *  @sa adi_adau1761_ConfigSPORT()
 */
ADI_ADAU1761_RESULT adi_adau1761_RegisterTxCallback (
    ADI_ADAU1761_HANDLE  const      hDevice,
    ADI_CALLBACK                    pfCallback,
    void *const                     pCBParam
)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportTxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

    if(adi_sport_RegisterCallback (pSPORTInfo->hDevice,
        pfCallback, pCBParam) != ADI_SPORT_SUCCESS)
    {
        return ADI_ADAU1761_SPORT_ERROR;
    }

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Registers a receive callback for reporting the events.
 *
 * @details     The driver can operate with or without a callback. By default the driver is
 *              opened without the callback. It is not recommended to register a callback
 *              unless the application requires the events to be notified immediately.
 *
 *              When callback is registered the adi_adau1761_GetBuffer cannot be used.
 *              The pointer to the buffer provided back to the application as
 *              an argument to the ADI_SPORT_EVENT_BUFFER_PROCESSED event.
 *
 *              When callback is not registered application can use adi_adau1761_GetTxBuffer
 *              or adi_ADAU1761_GetRxBuffer to get back the pointer to buffer.
 *
 *              The callback can only be registered after Sport is configured and
 *              before the output is enabled.
 *
 * @param [in]  hDevice                 Handle to the driver instance with which the callback
 *                                      to be registered.
 *
 * @param [in]  pfCallback              Callback function to be registered.
 *
 * @param [in]  pCBParam                Callback parameter which will be passed back to the
 *                                      application when callback function is called.
 *
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully register the callback.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given adau1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 *  @sa adi_adau1761_ConfigSPORT()
 */
ADI_ADAU1761_RESULT adi_adau1761_RegisterRxCallback (
    ADI_ADAU1761_HANDLE  const      hDevice,
    ADI_CALLBACK                    pfCallback,
    void *const                     pCBParam
)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportRxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

    if(adi_sport_RegisterCallback (pSPORTInfo->hDevice,
        pfCallback, pCBParam) != ADI_SPORT_SUCCESS)
    {
        return ADI_ADAU1761_SPORT_ERROR;
    }

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Submit a transmit buffer to the ADAU1761 device (non-blocking).
 *
 * This function returns immediately (does not wait for the write to complete).
 * An interrupt is generated when the transfer is complete or an error occurs.
 * The supplied buffer is owned by the driver during the data transfer process.
 * This buffer should not be modified or deleted by the application until the
 * data transfer is complete.  It's recommended that the buffer not be a
 * local variable.
 *
 * @param [in]  hDevice             Handle to the ADAU1761 device.
 * @param [in]  *pBuffer            The buffer that contains the data to transmit.
 * @param [in]  nBufSize;           The size of the data transfer buffer (bytes).
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully submitted the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 *  @sa adi_adau1761_GetTxBuffer()
 *  @sa adi_adau1761_IsTxBufferAvailable()
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_SubmitTxBuffer(
		ADI_ADAU1761_HANDLE    const hDevice,
		void                        *pBuffer,
		uint32_t               const nBufSize)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportTxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

	/* Submit Tx buffer */
	if(adi_sport_SubmitBuffer(pSPORTInfo->hDevice,
			pBuffer, nBufSize) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

	return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Submit a receive buffer to the ADAU1761 device (non-blocking).
 *
 * This function returns immediately (does not wait for the write to complete).
 * An interrupt is generated when the transfer is complete or an error occurs.
 * The supplied buffer is owned by the driver during the data transfer process.
 * This buffer should not be modified or deleted by the application until the
 * data transfer is complete.  It's recommended that the buffer not be a
 * local variable.
 *
 * @param [in]  hDevice             Handle to the ADAU1761 device.
 * @param [in]  *pBuffer            The buffer that contains the data to receive.
 * @param [in]  nBufSize;           The size of the data transfer buffer (bytes).
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully submitted the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 *  @sa adi_adau1761_GetRxBuffer()
 *  @sa adi_adau1761_IsRxBufferAvailable()
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_SubmitRxBuffer(
		ADI_ADAU1761_HANDLE    const hDevice,
		void                        *pBuffer,
		uint32_t               const nBufSize)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportRxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

	/* Submit Tx buffer */
	if(adi_sport_SubmitBuffer(pSPORTInfo->hDevice,
			pBuffer, nBufSize) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

	return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief Return the transmit buffer if a filled buffer is available, otherwise
 *        wait until a buffer is filled.
 *
 * @note This API is used for non-blocking mode only.
 *
 * @param [in]  hDevice       Handle to the ADAU1761 device.
 * @param [out] ppBuffer      The available Tx buffer.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully got the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 * @sa adi_ADAU1761_SubmitTxBuffer()
 * @sa adi_adau1761_IsTxBufferAvailable()
 */
ADI_ADAU1761_RESULT adi_adau1761_GetTxBuffer (
		ADI_ADAU1761_HANDLE   const hDevice,
		void                 **ppBuffer)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportTxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

	if(adi_sport_GetBuffer(pSPORTInfo->hDevice, ppBuffer) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

	return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief Return the receive buffer if a filled buffer is available, otherwise
 *        wait until a buffer is filled.
 *
 * @note This API is used for non-blocking mode only.
 *
 * @param [in]  hDevice       Handle to the ADAU1761 device.
 * @param [out] ppBuffer      The available Tx buffer.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully got the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 * @sa adi_ADAU1761_SubmitRxBuffer()
 * @sa adi_adau1761_IsRxBufferAvailable()
 */
ADI_ADAU1761_RESULT adi_adau1761_GetRxBuffer (
		ADI_ADAU1761_HANDLE   const hDevice,
		void                 **ppBuffer)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportRxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

	if(adi_sport_GetBuffer(pSPORTInfo->hDevice, ppBuffer) != ADI_SPORT_SUCCESS)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}

	return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Checks if a transmit buffer is available.
 *
 * @details     This is a non-blocking API which checks if a buffer is available for
 *              reuse.
 *
 * @param [in]  hDevice                 Handle to the driver instance from which the
 *                                      buffer status to be queried.
 *
 * @param [out]  pbAvailable            Pointer to a location where the buffer
 *                                      availability status is written.
 *
 *                                      true if a buffer is available, otherwise false.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully submitted the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 * @sa  adi_adau1761_GetTxBuffer()
 * @sa  adi_adau1761_SubmitTxBuffer()
 */
ADI_ADAU1761_RESULT adi_adau1761_IsTxBuffAvailable (

	ADI_ADAU1761_HANDLE            const hDevice,
    bool *const                    pbAvailable)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportTxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

    if(adi_sport_IsBufferAvailable(pSPORTInfo->hDevice, pbAvailable) != ADI_SPORT_SUCCESS)
    {
        return ADI_ADAU1761_SPORT_ERROR;
    }

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Checks if a receive buffer is available.
 *
 * @details     This is a non-blocking API which checks if a buffer is available for
 *              reuse.
 *
 * @param [in]  hDevice                 Handle to the driver instance from which the
 *                                      buffer status to be queried.
 *
 * @param [out]  pbAvailable            Pointer to a location where the buffer
 *                                      availability status is written.
 *
 *                                      true if a buffer is available, otherwise false.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully submitted the buffer.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_SPORT_ERROR              A SPORT device error occurred.
 *
 * @sa  adi_adau1761_GetRxBuffer()
 * @sa  adi_adau1761_SubmitRxBuffer()
 */
ADI_ADAU1761_RESULT adi_adau1761_IsRxBuffAvailable (

	ADI_ADAU1761_HANDLE            const hDevice,
    bool *const                    pbAvailable)
{
    ADI_ADAU1761_SPORT_INFO   *pSPORTInfo;

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

	/* SPORT input related parameters */
	pSPORTInfo = (ADI_ADAU1761_SPORT_INFO *) pDevice->pDevInfo->pSportRxInfo;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pSPORTInfo == NULL)
	{
		return ADI_ADAU1761_SPORT_ERROR;
	}
#endif

    if(adi_sport_IsBufferAvailable(pSPORTInfo->hDevice, pbAvailable) != ADI_SPORT_SUCCESS)
    {
        return ADI_ADAU1761_SPORT_ERROR;
    }

    return ADI_ADAU1761_SUCCESS;
}

/**
 * @brief       Select ADC or Digital Microphone as the audio input source.
 *
 * @details     Sets the required registers to switch the audio input
 *              to either ADC or digital microphone.
 *
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  eSource             The ADAU1761 input source selection.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully selected the audio input source.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *  - #ADI_ADAU1761_INVALID_OPERATION        The input source is invalid.
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_SelectInputSource (
	ADI_ADAU1761_HANDLE            const hDevice,
	ADI_ADAU1761_INPUT_SOURCE      const eSource)
{
	uint8_t regMicCtrl;
	uint8_t regMicBias;
	uint8_t regAdcCtrl;
	ADI_ADAU1761_RESULT result = ADI_ADAU1761_SUCCESS;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	result = adi_adau1761_GetRegister(hDevice, REG_MIC_CTRL_ADDR, &regMicCtrl);
	regMicCtrl = regMicCtrl & (uint8_t)~BITM_MIC_JDFUNC;

	result = adi_adau1761_GetRegister(hDevice, REG_MIC_BIAS_ADDR, &regMicBias);
	regMicBias = regMicBias & (uint8_t)~BITM_MIC_MBIEN;

	result = adi_adau1761_GetRegister(hDevice, REG_ADC_CTRL_ADDR, &regAdcCtrl);
	regAdcCtrl = regAdcCtrl & (uint8_t)~BITM_ADC_INSEL;

	if (eSource == ADI_ADAU1761_INPUT_ADC)
	{
		/* disable MIC input */
		regMicCtrl |= (uint8_t)(ENUM_MIC_DISABLE << BITP_MIC_JDFUNC);

		/* disable MIC bias */
		regMicBias |= (uint8_t)(ENUM_MIC_BIAS_DISABLE << BITP_MIC_MBIEN);

		/* ADC select */
		regAdcCtrl |= (uint8_t)(ENUM_ADC_MIC_IN_DISABLE << BITP_ADC_INSEL);

	}
	else if (eSource == ADI_ADAU1761_INPUT_MIC)
	{
		/* enable MIC input */
		regMicCtrl |= (uint8_t)(ENUM_MIC_ENABLE << BITP_MIC_JDFUNC);

		/* enable MIC bias */
		regMicBias |= (uint8_t)(ENUM_MIC_BIAS_ENABLE << BITP_MIC_MBIEN);

		/* enable MIC */
		regAdcCtrl |= (uint8_t)(ENUM_ADC_MIC_IN_ENABLE << BITP_ADC_INSEL);
	}
	else
	{
		result = ADI_ADAU1761_INVALID_OPERATION;
	}

	result = adi_adau1761_SetRegister (hDevice, REG_MIC_CTRL_ADDR, regMicCtrl);
	result = adi_adau1761_SetRegister (hDevice, REG_MIC_BIAS_ADDR, regMicBias);
	result = adi_adau1761_SetRegister (hDevice, REG_ADC_CTRL_ADDR, regAdcCtrl);

	return result;
}

/**
 * @brief       Set a volume control level.
 *
 * @details     This API is used to set volume levels and enable/disable
 *              volume controls.  Refer to the #ADI_ADAU1761_VOLUME_CONTROL
 *              enums for the valid volume controls.  Left, right or both
 *              channels can be set for each volume control.
 *
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  eVolumeCtrl         The volume control to set.
 *
 * @param [in]  eChannel            The channel to set.
 *
 * @param [in]  bEnable             True to enable the volume control, false to disable.
 *
 * @param [in]  value               The volume setting.  Refer to the following tables:
 *
 * Headphone volume - Each 1-bit step corresponds to a 1 dB increase in volume.
 *   Value        Volume
 *   0            -57 dB (default)
 *   ...
 *   57            0 dB
 *   ...
 *   63            6 dB
 *
 * Line out volume - Each 1-bit step corresponds to a 1 dB increase in volume.
 *   Value        Volume
 *   0            -57 dB (default)
 *   ...
 *   57            0 dB
 *   ...
 *   63            6 dB
 *
 * ADC/MIC input volume - Each bit corresponds to a 0.375 dB step with slewing between settings.
 *   Value        Volume
 *   0             0 dB (default)
 *   1            -0.375 dB
 *   ...
 *   254          -95.25 dB
 *   255          -95.625 dB
 *
 * DAC volume - Each bit corresponds to a 0.375 dB step with slewing between settings.
 *   Value        Volume
 *   0             0 dB (default)
 *   1            -0.375 dB
 *   ...
 *   254          -95.25 dB
 *   255          -95.625 dB
 *
 * Differential input volume - Each step corresponds to a 0.75 dB increase in gain.
 *   Value        Volume
 *   0            -12 dB (default)
 *   1            -11.25 dB
 *   ...
 *   16            0 dB
 *   ...
 *   62            34.5 dB
 *   63            35.25 dB
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully set a volume control level.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *  - #ADI_ADAU1761_INVALID_OPERATION        The input volume control is invalid.
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_SetVolume (
	ADI_ADAU1761_HANDLE            const hDevice,
	ADI_ADAU1761_VOLUME_CONTROL    const eVolumeCtrl,
	ADI_ADAU1761_VOLUME_CHANNEL    const eChannel,
	bool                           const bEnable,
	uint8_t                        const value)
{
	uint16_t regLeftAddr;
	uint16_t regRightAddr;
	uint8_t regValue;
	ADI_ADAU1761_RESULT result = ADI_ADAU1761_SUCCESS;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	switch(eVolumeCtrl)
	{
	case ADI_ADAU1761_VOL_HEADPHONE:
		regLeftAddr = REG_HEADPHONE_LEFT_ADDR;
		regRightAddr = REG_HEADPHONE_RIGHT_ADDR;
		regValue = 0u;
		if (bEnable)
		{
			regValue = (uint8_t)(value << 2u) | 3u;
		}
		break;
	case ADI_ADAU1761_VOL_LINE_OUT:
		regLeftAddr = REG_LINE_OUT_LEFT_ADDR;
		regRightAddr = REG_LINE_OUT_RIGHT_ADDR;
		regValue = 1u;
		if (bEnable)
		{
			regValue = (uint8_t)(value << 2u) | 2u;
		}
		break;
	case ADI_ADAU1761_VOL_ADC_MIC:
		regLeftAddr = REG_ADC_MIC_LEFT_ADDR;
		regRightAddr = REG_ADC_MIC_RIGHT_ADDR;
		regValue = 255u;
		if (bEnable)
		{
			regValue = value;
		}
		break;
	case ADI_ADAU1761_VOL_DAC:
		regLeftAddr = REG_DAC_LEFT_ADDR;
		regRightAddr = REG_DAC_RIGHT_ADDR;
		regValue = 255u;
		if (bEnable)
		{
			regValue = value;
		}
		break;
	case ADI_ADAU1761_VOL_LINE_IN_DIFF:
		regLeftAddr = REG_DIFF_IN_LEFT_ADDR;
		regRightAddr = REG_DIFF_IN_RIGHT_ADDR;
		regValue = 0u;
		if (bEnable)
		{
			regValue = (uint8_t)(value << 2u) | 3u;
		}
		break;
	default:
		result = ADI_ADAU1761_INVALID_OPERATION;
		break;
	}

	if (result == ADI_ADAU1761_SUCCESS)
	{
		if ((eChannel == ADI_ADAU1761_VOL_CHAN_LEFT) ||
			(eChannel == ADI_ADAU1761_VOL_CHAN_BOTH))
		{
			result = adi_adau1761_SetRegister (hDevice, regLeftAddr, regValue);
		}

		if ((eChannel == ADI_ADAU1761_VOL_CHAN_RIGHT) ||
			(eChannel == ADI_ADAU1761_VOL_CHAN_BOTH))
		{
			result = adi_adau1761_SetRegister (hDevice, regRightAddr, regValue);
		}
	}

	return result;
}

/**
 * @brief       Set the ADC, DAC, serial port and DSP sample rate.
 *
 * @details     The sampling rate selected is a ratio of the base sampling
 *              rate, fS. The base sampling rate is determined by the operating
 *              frequency of the core clock. Note: the 44.1 KHz sample rate
 *              requires a PLL fractional setting.
 *
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  eSampleRate         The ADAU1761 sample rate setting.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully selected the audio input source.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *  - #ADI_ADAU1761_INVALID_OPERATION        The sample rate is invalid.
 *	- #ADI_ADAU1761_MUTEX_FAILED             A mutex related failure occurred.
 *	- #ADI_ADAU1761_PLL_FRAC_MODE_REQ    [D] This sample rate requires PLL fractional mode.
 *
 */
ADI_ADAU1761_RESULT adi_adau1761_SetSampleRate (
	ADI_ADAU1761_HANDLE            const hDevice,
	ADI_ADAU1761_SAMPLE_RATE       const eSampleRate)
{
	uint8_t regValue;
	uint8_t conValue;
	uint8_t serValue;
	uint8_t dspValue;
	uint8_t regData[6u];

	ADI_ADAU1761_RESULT result = ADI_ADAU1761_SUCCESS;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}

	/* the 44.1 KHz sample rate requires PLL fractional setting */
	if (eSampleRate == ADI_ADAU1761_SAMPLE_RATE_44_1KHZ)
	{
		/* read the PLL register to check for fractional setting */
		result = adi_adau1761_GetRegisterBlock (hDevice,
				REG_PLL_CRL_ADDR, 6u, &regData[0]);
		if (result != ADI_ADAU1761_SUCCESS)
		{
			return result;
		}

		if ((regData[4] & 0x01u) != 0x1u)
		{
			return ADI_ADAU1761_PLL_FRAC_MODE_REQ;
		}
	}
#endif

	switch(eSampleRate)
	{
		case ADI_ADAU1761_SAMPLE_RATE_8KHZ:
			conValue = 1u;
			serValue = 1u;
			dspValue = 6u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_12KHZ:
			conValue = 2u;
			serValue = 2u;
			dspValue = 5u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_16KHZ:
			conValue = 3u;
			serValue = 3u;
			dspValue = 4u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_24KHZ:
			conValue = 4u;
			serValue = 4u;
			dspValue = 3u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_32KHZ:
			conValue = 5u;
			serValue = 5u;
			dspValue = 2u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_44_1KHZ:
		case ADI_ADAU1761_SAMPLE_RATE_48KHZ:
			conValue = 0u;
			serValue = 0u;
			dspValue = 1u;
			break;
		case ADI_ADAU1761_SAMPLE_RATE_96KHZ:
			conValue = 6u;
			serValue = 6u;
			dspValue = 0u;
			break;
		default:
			result = ADI_ADAU1761_INVALID_OPERATION;
			break;
	}

	if (result != ADI_ADAU1761_INVALID_OPERATION)
	{
	    /* Acquire the mutex */
	    if(adi_osal_MutexPend(pDevice->pDevInfo->hMutex,
	                          ADI_OSAL_TIMEOUT_FOREVER) != ADI_OSAL_SUCCESS)
	    {
	        return ADI_ADAU1761_MUTEX_FAILED;
	    }

		/* set the converter sample rate */
		adi_adau1761_GetRegister (hDevice, REG_CONV_CTRL_0_ADDR, &regValue );
		regValue &= ~0x7u;
		regValue |= conValue;
		adi_adau1761_SetRegister (hDevice, REG_CONV_CTRL_0_ADDR, regValue );

		/* set the serial port sample rate */
		adi_adau1761_GetRegister (hDevice, REG_SERIAL_PORT_SAMP_ADDR, &regValue );
		regValue &= ~0x7u;
		regValue |= serValue;
		adi_adau1761_SetRegister (hDevice, REG_SERIAL_PORT_SAMP_ADDR, regValue );

		/* set the DSP sample rate */
		adi_adau1761_GetRegister (hDevice, REG_DSP_SAMP_RATE_ADDR, &regValue );
		regValue &= ~0xFu;
		regValue |= dspValue;
		adi_adau1761_SetRegister (hDevice, REG_DSP_SAMP_RATE_ADDR, regValue );

	    /* Release the mutex */
	    if(adi_osal_MutexPost(pDevice->pDevInfo->hMutex) != ADI_OSAL_SUCCESS)
	    {
	        return ADI_ADAU1761_MUTEX_FAILED;
	    }

	}

	return result;
}

/**
 * @brief       Write values to one or more ADAU1761 device registers.
 *
 * @details     Write register values using the communications device specified in
 *              adi_adau1761_Open().
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  regAddr             The ADAU1761 starting register address.
 *
 * @param [in]  length              The number of bytes to write.
 *
 * @param [in]  pData               The data buffer which contains the register
 *                                  values to write.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully wrote register values.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 */
ADI_ADAU1761_RESULT adi_adau1761_SetRegisterBlock (
	ADI_ADAU1761_HANDLE       const hDevice,
    uint16_t                  const regAddr,
	uint16_t                  const length,
	uint8_t                         *pData)
{
	ADI_ADAU1761_RESULT result;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	/* Perform device write */
	result = WriteRegister (pDevice, regAddr, length, pData);

	return result;
}

/**
 * @brief       Write a value to a single ADAU1761 device register.
 *
 * @details     Write register values using the communications device specified in
 *              adi_adau1761_Open().
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  regAddr             The ADAU1761 starting register address.
 *
 * @param [in]  value               The register value to write.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully wrote register values.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 */
ADI_ADAU1761_RESULT adi_adau1761_SetRegister (
	ADI_ADAU1761_HANDLE       const hDevice,
    uint16_t                  const regAddr,
	uint8_t                         value)
{
	ADI_ADAU1761_RESULT result;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	/* Perform device write */
	result = WriteRegister (pDevice, regAddr, 1u, &value);

	return result;
}

/**
 * @brief       Read values from one or more ADAU1761 device registers.
 *
 * @details     Read register values using the communications device specified in
 *              adi_adau1761_Open().
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  regAddr             The ADAU1761 starting register address.
 *
 * @param [in]  length              The number of bytes to read.
 *
 * @param [out]  pData              The data buffer which contains the register
 *                                  values read.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully read register values.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 */
ADI_ADAU1761_RESULT adi_adau1761_GetRegisterBlock (
	ADI_ADAU1761_HANDLE       const hDevice,
    uint16_t                  const regAddr,
	uint16_t                  const length,    /* bytes */
	uint8_t                         *pData)
{
	ADI_ADAU1761_RESULT result;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	/* Perform device read */
	result = ReadRegister (pDevice, regAddr, length, pData);

	return result;
}

/**
 * @brief       Read a value from a single ADAU1761 device register.
 *
 * @details     Read register values using the communications device specified in
 *              adi_adau1761_Open().
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  regAddr             The ADAU1761 starting register address.
 *
 * @param [out]  pValue             The register value.

 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully read register values.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_COMM_DEV_FAILED          The selected communication device failed.
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 */
ADI_ADAU1761_RESULT adi_adau1761_GetRegister (
	ADI_ADAU1761_HANDLE            const hDevice,
    uint16_t                       const regAddr,
    uint8_t                              *pValue)
{
	ADI_ADAU1761_RESULT result;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
#endif

	/* Perform device read */
	result = ReadRegister (pDevice, regAddr, 1u, pValue);

	return result;
}

/*
 * Sigma Studio functions
 */
static ADI_ADAU1761_HANDLE SIGMA_DEVICE_HANDLE = (void *) 0u;

/**
 * @brief       Loads SigmaStudio program data, parameter data and register settings.
 *
 * @details     This API is used to call the the SigmaStudio default_download_IC_x() function.
 *              Use the SigmaStudio export system files function to create the *_IC_1.h
 *              file.  The exported *_IC_1.h file contains program data, parameter data
 *              and register settings required for ADAU1761 operation.
 *
 * @param [in]  hDevice             Handle to ADAU1761 device instance.
 *
 * @param [in]  pfSSLoad            The  SigmaStudio download function.
 *
 * @return      Status
 *  - #ADI_ADAU1761_SUCCESS                  If successfully loaded SigmaStudio settings.
 *  - #ADI_ADAU1761_BAD_DEVICE_HANDLE    [D] If the given ADAU1761 device handle is invalid.
 *  - #ADI_ADAU1761_DEVICE_NOT_OPEN      [D] If the given device is not yet opened.
 *  - #ADI_ADAU1761_NULL_POINTER         [D] A NULL pointer is invalid for this API.
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 * @sa SIGMA_WRITE_REGISTER_BLOCK()
 * @sa SIGMA_WRITE_DELAY()
 */
ADI_ADAU1761_RESULT adi_adau1761_SigmaStudioLoad (
	    ADI_ADAU1761_HANDLE     const  hDevice,
	    ADI_SIGMA_STUDIO_LOAD   const  pfSSLoad)
{
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

#if defined(ADI_DEBUG)
	if (!IsDeviceHandle(pDevice))
	{
		return ADI_ADAU1761_BAD_DEVICE_HANDLE;
	}
	if (!pDevice->bInUse)
	{
		return ADI_ADAU1761_DEVICE_NOT_OPEN;
	}
	if (pfSSLoad == NULL)
	{
        return ADI_ADAU1761_NULL_POINTER;
	}
#endif

	SIGMA_DEVICE_HANDLE = hDevice;

	/* from Sigma Studio export *_IC_1.h */
	pfSSLoad();

    SIGMA_DEVICE_HANDLE = (void*) 0u;

    return ADI_ADAU1761_SUCCESS;
}

#if defined(ADI_DEBUG)
void adi_adau1761_DumpRam(ADI_ADAU1761_HANDLE const hDevice,
		uint16_t regAddr, uint16_t length);
/*
 * A debug function which dumps program and parameter RAM.
 */
void adi_adau1761_DumpRam(ADI_ADAU1761_HANDLE const hDevice,
		uint16_t regAddr, uint16_t length)
{
	uint16_t i, size, dataSize;
    uint16_t addr;
	uint16_t count;
	uint8_t data[16];

    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) hDevice;

    /* break up data into blocks */
    if (regAddr == PARAM_RAM_ADDR)
    {
    	/* parameter data - set data size to 4 bytes (per data sheet) */
    	dataSize = 4u;
    }
    else if (regAddr == PROG_RAM_ADDR)
    {
    	/* program data - set data size to 5 bytes (per data sheet) */
    	dataSize = 5u;
    }
    else
    {
    	dataSize = TWI_BUFF_SIZE - TWI_ADDR_SIZE;
    }

    addr = regAddr;
    count = length;

    while (count > 0u)
    {
        if (count > dataSize)
        {
        	size = dataSize;
        }
        else
        {
        	size = count;
        }

		/* verify the register data */
		adi_adau1761_GetRegisterBlock (pDevice, addr, size, &data[0]);

	    if (regAddr == PARAM_RAM_ADDR)
	    {
			printf("0x%02X, 0x%02X, 0x%02X, 0x%02X\n", data[0], data[1], data[2], data[3]);
	    }
	    else if (regAddr == PROG_RAM_ADDR)
	    {
			printf("0x%02X, 0x%02x, 0x%02X, 0x%02X, 0x%02X\n", data[0], data[1], data[2], data[3], data[4]);
	    }
	    else {
			printf("Address: 0x%04X\n", addr);
			for (i=0u; i<size; i++)
			{
				printf("0x%02X, ", data[i]);
			}
			printf("\n");
	    }

	    if (size == count)
        {
        	/* transfer done */
        	count = 0u;

        }
        else
        {
        	count -= size;
        	addr += size;
        }
    }
}
#endif

/**
 * @brief       Write values to one or more ADAU1761 device registers.
 *
 * @details     This API is called from the SigmaStudio default_download_IC_1() function.
 *              Use the SigmaStudio export system files function to create the *_IC_1.h
 *              file.  The exported *_IC_1.h file contains program data, parameter data
 *              and register settings required for ADAU1761 operation.
 *
 * @param [in]  devAddress          Not used.
 *
 * @param [in]  regAddr             The ADAU1761 starting register address.
 *
 * @param [in]  length              The number of bytes to write.
 *
 * @param [in]  pRegData            The data buffer which contains the register
 *                                  values to write.
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 * @sa adi_adau1761_SigmaStudioLoad()
 */
void SIGMA_WRITE_REGISTER_BLOCK(
		uint16_t  devAddress,
		uint16_t  regAddr,
		uint16_t  length,    /* bytes */
		uint8_t   *pRegData)
{
	ADI_ADAU1761_RESULT result;
    ADI_ADAU1761_DEVICE *pDevice = (ADI_ADAU1761_DEVICE *) SIGMA_DEVICE_HANDLE;

	if (SIGMA_DEVICE_HANDLE != (void*)0u)
	{
		/* Perform device write */
		result = WriteRegister (pDevice, regAddr, length, pRegData);

#if defined(ADI_DEBUG)
		if (result != ADI_ADAU1761_SUCCESS)
		{
			printf("SIGMA_WRITE_REGISTER_BLOCK failure\n");
		}

		/* read back the register block
		adi_adau1761_DumpRam(SIGMA_DEVICE_HANDLE, regAddr, length);
		*/

#endif
	}
}

/**
 * @brief       A simple delay loop required by SigmaStudio load.
 *
 * @details     This API is called from the SigmaStudio default_download_IC_1() function.
 *              Use the SigmaStudio export system files function to create the *_IC_1.h
 *              file.  The exported *_IC_1.h file contains program data, parameter data
 *              and register settings required for ADAU1761 operation.
 *
 * @param [in]  devAddress          The communications device address.
 *
 * @param [in]  length              The size of the data buffer.
 *
 * @param [in]  pData               A buffer containing the delay times (msec).
 *
 * @sa adi_adau1761_Open()
 * @sa adi_adau1761_ConfigTWI()
 * @sa adi_adau1761_ConfigSPI()
 * @sa adi_adau1761_SigmaStudioLoad()
 */
void SIGMA_WRITE_DELAY(
		uint16_t  devAddress,
		uint16_t  length,
		uint8_t   *pData)
{
	uint16_t i;
	uint32_t d;

	for (i=0u; i<length; i++)
	{
		/* crude delay loop */
		for (d=0u; d < (40000u * (uint32_t)pData[i]); d++)
		{
			asm("nop;");
		}
	}
}


/**@}*/

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */
