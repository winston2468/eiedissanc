/* $Id: adi_dev_audio_pdd.h 62861 2019-11-27 14:36:45Z dgath $
******************************************************************************
Copyright (c), 2019 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensors.
******************************************************************************

Title: ADI Physical Device Driver Interface

Description:

    Defines the Audio wrapper API

*****************************************************************************/

#ifndef __ADI_DEV_AUDIO_PDD_H__
#define __ADI_DEV_AUDIO_PDD_H__

#include <services/int/adi_int.h>
#include "adi_dev_audio_channel_id.h"

#if defined(_LANGUAGE_C)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Handle to a Physical Audio codec class device */
typedef void    *ADI_DEV_AUDIO_PDD_HANDLE;

/*
** Structure to get audio device specific parameters
*/
typedef struct  __AdiDevAudioDeviceParams
{

    /* 'true' when the device is forced to operate in
        bi-directional mode, 'false' when input and output
        channels are operated independently */
    bool                    bForceBiDirection;

    /* Input channel word length in bytes */
    uint8_t                 nInWordLengthInBytes;

    /* Output channel word length in bytes */
    uint8_t                 nOutWordLengthInBytes;

    /* Number of active Input channels */
    uint8_t                 nNumActiveInChannels;

    /* Number of active Output channels */
    uint8_t                 nNumActiveOutChannels;

    /* Input DMA Bus width */
    uint8_t                 nInDmaBusWidth;

    /* Output DMA Bus width */
    uint8_t                 nOutDmaBusWidth;

    /* Minimum number of input callback buffers (sub-buffers)
       to support (value between 3 and 10) */
    uint8_t                 nMinInSubBuffers;

    /* Minimum number of output callback buffers (sub-buffers)
       to support (value between 3 to 10) */
    uint8_t                 nMinOutSubBuffers;

    /* Default Input callback interval in milliseconds */
    uint16_t                nInCallbackInterval;

    /* Default Output callback interval in milliseconds */
    uint16_t                nOutCallbackInterval;

    /* Input channel sample rate */
    uint32_t                nInSampleRate;

    /* Output channel sample rate */
    uint32_t                nOutSampleRate;

    /* Physical Device Driver Handle to Audio input port */
    void *   hInPort;

    /* Physical Device Driver Handle to Audio output port */
    void *   hOutPort;

} ADI_DEV_AUDIO_DEVICE_PARAMS;


/*
**  Prototype: ADI_DEV_AUDIO_MGR_GETPARAMS_FN
**
**      Function Scope limited to Audio codec class manager/driver implementation
**
**      Prototype of the function called from the Audio Class Manager to
**      the underlying Audio Class Driver to get device specific parameters of
**      a physical audio device
**
**  Parameters:
**      hAudioClassDevice   - Handle to the Audio device instance to query
**      pDeviceParams       - Pointer to instance to hold
**                            audio device specific parameters
**
**  Returns:
**      ADI_DEV_RESULT_SUCCESS
**          - Successfully obtained audio device specific parameters
**      ADI_DEV_AUDIO_RESULT_HANDLE_INVALID
**          - Given audio device handle is invalid
**
*/

typedef uint32_t (*ADI_DEV_AUDIO_MGR_GETPARAMS_FN) (
    ADI_DEV_AUDIO_PDD_HANDLE        hAudioDevice,
    ADI_DEV_AUDIO_DEVICE_PARAMS     *pDeviceParams
);






/*
**
** Physical Audio codec class driver entry point structure
**
*/
typedef struct __AdiDevAudioEntryPoint
{
    /* Open Audio codec class device */
    uint32_t (*adi_pdd_Audio_Open)
    (
        /* Physical Device number to open */
        uint32_t                        nDeviceNumber,
        /* Pointer to application callback function */
        ADI_CALLBACK                     pfCallback,
        /* Callback parameter supplied by the application */
        void                            *pCallbackParam,
        /* Pointer to location to store handle to Audio codec class device */
        ADI_DEV_AUDIO_PDD_HANDLE        *phAudioClassDevice,
        /* Pointer to location to store address of function to be called by
           Audio Class Manager to get physical audio device specific parameters */
        ADI_DEV_AUDIO_MGR_GETPARAMS_FN  *pfGetParams
    );

    /* Close Audio codec class device */
    uint32_t (*adi_pdd_Audio_Close)
    (
        /* Handle to Audio codec class device to close */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice
    );

    /* Submit a TX Buffer */
    uint32_t (*adi_pdd_SubmitTxBuffer)(
    		ADI_DEV_AUDIO_PDD_HANDLE    const hDevice,
    		void                        *pBuffer,
    		uint32_t                    const nBufSize);

    /* Submit a TX Buffer */
    uint32_t (*adi_pdd_SubmitRxBuffer)(
    		ADI_DEV_AUDIO_PDD_HANDLE    const hDevice,
    		void                        *pBuffer,
    		uint32_t                    const nBufSize);


    /* Set/Sense Audio device parameters */
    uint32_t (*adi_pdd_Audio_Control)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* Command ID */
        uint32_t                        nCommandID,
        /* Command specific value */
        void                            *Value
    );

    /* Enable/Disable Audio Input (Receive) dataflow */
    uint32_t (*adi_pdd_AudioIn_EnableDataflow)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* 'true' to enable, 'false' to disable */
        bool                            bEnable
    );

    /* Enable/Disable Audio Output (Transmit) dataflow */
    uint32_t (*adi_pdd_AudioOut_EnableDataflow)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* 'true' to enable, 'false' to disable */
        bool                            bEnable
    );

    /* Set Audio Data Word length */
    uint32_t (*adi_pdd_Audio_SetWordLength)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* Logical Channel ID to which the word length applies */
        ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
        /* Audio data word length to set */
        uint32_t                        nWordLength
    );

    /* Set Audio Channel Volume */
    uint32_t (*adi_pdd_Audio_SetVolume)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* Logical Channel ID to set volume */
        ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
        /* Channel Volume range in linear steps */
        uint16_t                        rVolume
    );

    /* Get Audio Channel Volume */
    uint32_t (*adi_pdd_Audio_GetVolume)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* Logical Channel ID to query for volume */
        ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
        /* Location to store Channel Volume */
        uint16_t                        *prVolume
    );

    /* Set Audio Channel Sampling Rate */
    uint32_t (*adi_pdd_Audio_SetSampleRate)
    (
        /* Handle to Audio codec class device to work on */
        ADI_DEV_AUDIO_PDD_HANDLE        hAudioClassDevice,
        /* Logical Channel ID update */
        ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
        /* Sample rate in Hertz */
        uint32_t                        nSampleRateHz,
        /* 'true' to apply the sample rate to selected channel by
            updating hadware registers
           'false' to cache the sample rate with in the driver */
        bool                            bApply
    );

} ADI_DEV_AUDIO_ENTRY_POINT;

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* _LANGUAGE_C */

#endif  /* __ADI_DEV_AUDIO_MANAGER_H__ */

/*****/


