/*******************************************************************************
Copyright(c) 2008-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

$Revision: 62861 $
$Date: 2019-11-27 09:36:45 -0500 (Wed, 27 Nov 2019) $

*******************************************************************************/

/*******************************************************************************
 * @file       adi_dev_video_entrypoint.h
 *
 * @brief      This is the internal include file that contains entry point
 *             structures.
 *
 ******************************************************************************/

#ifndef __ADI_DEV_VIDEO_ENTRYPOINT_H__
#define __ADI_DEV_VIDEO_ENTRYPOINT_H__


#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_18_1:"Allows phantom structures in the include file")
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */


/*=============  I N C L U D E S   =============*/
/* Interrupt Manager include file */
#include <services/int/adi_int.h>

#if defined(_LANGUAGE_C)
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*=============  D A T A    T Y P E S   =============*/

/* Handle to a Physical Video class device */
typedef void*    ADI_DEV_VIDEO_PDD_HANDLE;

/*
**  Data structures
*/

/* Forward declaration for video frame properties */
struct __AdiDevVideoFrameProperties;

/* Forward declaration for video frame properties pointer */
typedef struct __AdiDevVideoFrameProperties*  ADI_DEV_VIDEO_FRAME_PROPERTIES_PTR;

/*
** Structure to get the video device specific parameters required for
** Video Manager
*/
typedef struct __AdiDevVideoDeviceParams
{
    /* Default PPI Number to be Used */
    uint32_t                              nDefaultPPI;

    /* Video output Frame properties */
    ADI_DEV_VIDEO_FRAME_PROPERTIES_PTR    poFrameProperties;

} ADI_DEV_VIDEO_DEVICE_PARAMS;


/* Function prototype which gets the Video Class Device Information */
typedef uint32_t  (*ADI_DEV_VIDEO_MGR_GETPARAMS_FN) (
    ADI_DEV_VIDEO_PDD_HANDLE       hVideoOutPDD,
    ADI_DEV_VIDEO_DEVICE_PARAMS   *poDeviceParams
);

/* Physical Video class driver entry point structure */
typedef struct __AdiDevVideoClassEntryPoint
{
    /* Open Physical Video device */
    uint32_t (*adi_pdd_Video_Open)
    (
        /* Physical device number of the Video device */
        uint32_t                        nDeviceNumber,
        /* Pointer to application callback function */
        ADI_CALLBACK                    pfCallback,
        /* Client Parameter to be passed with application callback function */
        void                           *pCallbackParam,
        /* Location to store handle to Physical Video Class Device */
        ADI_DEV_VIDEO_PDD_HANDLE       *phPhysicalDevice,
        /* Pointer to a location to store the function pointer which
           is called by video manager to get the device specific
           parameters */
        ADI_DEV_VIDEO_MGR_GETPARAMS_FN *pfGetParams
    );

    /* Close Physical Video device */
    uint32_t (*adi_pdd_Video_Close)
    (
        /* Handle to Physical Video Device to close */
        ADI_DEV_VIDEO_PDD_HANDLE        hPhysicalDevice
    );

    /* Configure the Device */
    uint32_t (*adi_pdd_Video_ConfigDevice)
    (
    	/* Handle to Physical Video Device */
    	ADI_DEV_VIDEO_PDD_HANDLE        hPhysicalDevice,

    	/* PPI Number to be used */
    	uint32_t                        nPPINum,

    	/* Flag to indicate whether PPI configuration is necessary */
    	bool                            bConfigPPI
    );


    /* Enable/Disable Video Device */
    uint32_t (*adi_pdd_Video_Enable)
    (
        /* Handle to Physical Video Device */
        ADI_DEV_VIDEO_PDD_HANDLE        hPhysicalDevice,

        /* 'true' to enable display, 'false' to disable */
        bool                            bEnable
    );

    /* Submit Frame to Video Device */
    uint32_t (*adi_pdd_Video_SubmitFrame)
    (
        /* Handle to Physical Video Device */
        ADI_DEV_VIDEO_PDD_HANDLE        hPhysicalDevice,
        /* Pointer to Frame Memory */
        uint8_t *const                  pFrame
    );

    /* Set/Get Configuration parameters */
    uint32_t (*adi_pdd_Video_Control)
    (
        /* Handle to Physical Video Device */
        ADI_DEV_VIDEO_PDD_HANDLE        hPhysicalDevice,
        /* Control command to be executed */
        uint32_t                        nCommand,
        /* Data specific to control command */
        void                           *pData
     );

} ADI_DEV_VIDEO_CLASS_ENTRY_POINT;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#else /* not _LANGUAGE_C */

/* Assembler-specific */

#endif /* not _LANGUAGE_C */

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */


#endif      /* __ADI_DEV_VIDEO_ENTRYPOINT_H__ */
