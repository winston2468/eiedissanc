/*******************************************************************************
Copyright(c) 2008-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

$Revision: 62861 $
$Date: 2019-11-27 09:36:45 -0500 (Wed, 27 Nov 2019) $

*******************************************************************************/
/**
 * @file    adi_dev_class.h
 *
 * @brief   Header file containing data type definitions common to all device
 *          class drivers.
 *
 */

/** @addtogroup Device_Class Device Class
 *  @{
 */

#ifndef __ADI_DEV_CLASS_H__
#define __ADI_DEV_CLASS_H__

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */


/*=============  I N C L U D E S   =============*/
/* Generic typedefs */
#include <stdint.h>
#include <stdbool.h>

/*==============  D E F I N E S  ===============*/

/*********************************************************************
Device Class Enumeration Start Values
-------------------------------------

Values between 0x50000000 and 0x5FFFFFFF are reserved for Device Class

 Bits 31 to 28  is always '5' and represents that the value corresponds to a
                Device Class
 Bits 27 to 16  Used to Identify a specific class of device
 Bits 15 to 0   Used to define Command IDs and Return codes of a specific
                Device Class

Each Device class can extend the existing Command IDs and Return codes.
In order to avoid conflicts, these enumerations must be unique so that no
two device classes share the same enumeration values.

The macros below define the enumeration start values for each device class.
Within primary header file of each device class implementation, the enumeration
values for new command IDs, event IDs and return codes should begin with the
appropriate value from the list below. Each device class can add up to 65535
new types before any conflicts occur.

NOTE:
All device class drivers should use the return code of value 0,for all successful
function calls

When a new device class is added to the distribution, begin the enumeration
for the new device class with the value indicated below, then replace that value
on the line below with it's current value + 0x10000.

*********************************************************************/

/* ADI Device Class Enumeration start common to all device classes */
#define E_ADI_DEV_CLASS_COMMON_START                (0x50000000U)

/* ADI Device Class Enumeration start for Audio Class Devices */
#define E_ADI_DEV_CLASS_AUDIO_START                 (0x50010000U)

/* ADI Device Class Enumeration start for Video Class Devices */
#define E_ADI_DEV_CLASS_VIDEO_START                 (0x50020000U)

/* ADI Device Class Enumeration start for Input Class Devices */
#define E_ADI_DEV_CLASS_INPUT_START                 (0x50030000U)

#if defined(_LANGUAGE_C)
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */



/*=============  D A T A    T Y P E S   =============*/

/*! Enumerations of Device Class Return Codes */
typedef enum __AdiDevClassResult
{
    /*! Generic Success */
    ADI_DEV_CLASS_RESULT_SUCCESS                   =  0U,
    /*! Generic Failure */
    ADI_DEV_CLASS_RESULT_FAILED                    =  1U,

    /*! Pointer supplied by the client is invalid (NULL pointer) */
    ADI_DEV_CLASS_RESULT_INVALID_POINTER           =  (E_ADI_DEV_CLASS_COMMON_START + 0U),
    /*! The required mutex creation failed  */
    ADI_DEV_CLASS_RESULT_MUTEX_CREATE_FAILED       =  (E_ADI_DEV_CLASS_COMMON_START + 1U),
    /*! Failed to destroy the created mutex */
    ADI_DEV_CLASS_RESULT_MUTEX_DESTROY_FAILED      =  (E_ADI_DEV_CLASS_COMMON_START + 2U),
    /*! Failed to acquire the required mutex   */
    ADI_DEV_CLASS_RESULT_MUTEX_ACQUIRE_FAILED      =  (E_ADI_DEV_CLASS_COMMON_START + 3U),
    /*! Failed to release the required mutex   */
    ADI_DEV_CLASS_RESULT_MUTEX_RELEASE_FAILED      =  (E_ADI_DEV_CLASS_COMMON_START + 4U),
    /*! The required semaphore creation failed  */
    ADI_DEV_CLASS_RESULT_SEM_CREATE_FAILED         =  (E_ADI_DEV_CLASS_COMMON_START + 5U),
    /*! Failed to destroy the created semaphore */
    ADI_DEV_CLASS_RESULT_SEM_DESTROY_FAILED        =  (E_ADI_DEV_CLASS_COMMON_START + 6U),
    /*! Failed to Pend on the semaphore */
    ADI_DEV_CLASS_RESULT_SEM_PEND_FAILED           =  (E_ADI_DEV_CLASS_COMMON_START + 7U),
    /*! Memory allocation failed */
    ADI_DEV_CLASS_RESULT_MEM_ALLOC_FAILED          =  (E_ADI_DEV_CLASS_COMMON_START + 8U)

} ADI_DEV_CLASS_RESULT;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#else /* not _LANGUAGE_C */

/* Assembler-specific */

#endif /* not _LANGUAGE_C */

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#endif /* __ADI_DEV_CLASS_H__ */

/* @} */

