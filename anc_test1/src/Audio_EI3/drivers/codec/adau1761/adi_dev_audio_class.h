/* $Id: adi_dev_audio_class.h 62861 2019-11-27 14:36:45Z dgath $
******************************************************************************
Copyright (c), 2008-2019  - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensors.
******************************************************************************

Title: ADI Device Class Drivers - Audio CODEC Class

Description:
    This is the primary include file for Audio codec class drivers.

    This file defines enumerations, data structures and APIs common to all
    Audio codec class drivers

*****************************************************************************/

#ifndef __ADI_DEV_AUDIO_H__
#define __ADI_DEV_AUDIO_H__

/*=============  I N C L U D E S   =============*/

/* Generic Type Definition includes */
#include <stdint.h>
#include <stdbool.h>
/* Device class includes */
#include "adi_dev_class.h"
/* Logical Audio Channel ID includes */
#include "adi_dev_audio_channel_id.h"

/* Audio Wrapper API  */
#include "adi_dev_audio_pdd.h"
/* OS Abstraction Layer */
#include <adi_osal.h>

/* disable misra diagnostics as necessary */
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */



/** @addtogroup AudioClassDriver Audio Device Class
 *  @{
 */



/* IF (Build for C Langugage) */
#if defined(_LANGUAGE_C)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* C-specific definitions  */

/* Handle to an Audio codec class device */
typedef void    *ADI_DEV_AUDIO_HANDLE;


/*=======  P U B L I C   P R O T O T Y P E S  ========*/
/*            (globally-scoped functions)             */

/*  Opens an Audio codec class device */
uint32_t  adi_dev_Audio_Open (
    ADI_DEV_AUDIO_ENTRY_POINT       *pDeviceEntryPoint,
    uint32_t                        nDeviceNumber,
    ADI_DEV_AUDIO_HANDLE            *phAudioDevice
);

/* Closes an Audio codec class device */
uint32_t  adi_dev_Audio_Close (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice
);

/* Sets/Senses Audio codec class device parameters */
uint32_t  adi_dev_Audio_Control (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    uint32_t                        nCommandID,
    void                            *Value
);

/*
 * Gets address of an Audio Input (Receive) buffer of specified size
 * filled with valid audio samples. This function must be used in pair
 * with 'adi_dev_AudioIn_SubmitBuffer()' function
 */
uint32_t  adi_dev_AudioIn_GetBuffer (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    void                            **ppFullBuffer,
    uint32_t                        nRequiredBufferSize
);

/*
 * Submits an empty Audio Input (Receive) buffer by providing the
 * actual number of audio samples consumed by the application from
 * the Full Input Buffer obtained previously using
 * 'adi_dev_AudioIn_GetBuffer()' call
 */
uint32_t  adi_dev_AudioIn_SubmitBuffer (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    uint32_t                        nBytesUsed
);

/* Enables/Disables Audio Input (Receive) dataflow */
uint32_t  adi_dev_AudioIn_EnableDataflow (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    bool                            bEnable
);

/*
 * Gets address of an empty Audio Output (transmit) buffer of
 * specified size. This function must be used in pair with
 * 'adi_dev_AudioOut_SubmitBuffer()' function
 */
uint32_t  adi_dev_AudioOut_GetBuffer (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    void                            **ppEmptyBuffer,
    uint32_t                        nRequiredBufferSize
);

/*
 * Submits a full Audio Output (Transmit) buffer by providing the
 * actual number of valid samples (in bytes) written to the empty
 * output buffer previously obtained by the application using
 * 'adi_dev_AudioOut_GetBuffer()' call
 */
uint32_t  adi_dev_AudioOut_SubmitBuffer (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    uint32_t                        nBytesUsed
);

/* Enables/Disables Audio Output (Transmit) dataflow */
uint32_t  adi_dev_AudioOut_EnableDataflow (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    bool                            bEnable
);

/* Sets Audio Data Word length of a selected channel or device */
uint32_t  adi_dev_Audio_SetWordLength (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
    uint32_t                        nWordLength
);

/* Sets Volume of a selected Audio Channel */
uint32_t  adi_dev_Audio_SetVolume (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
    uint16_t                        rVolume
);

/* Gets Audio Channel volume */
uint32_t  adi_dev_Audio_GetVolume (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
    uint16_t                        *prVolume
);

/*
 * Sets Audio Sample rate of selected channel
 *
 * The supplied sampling rate will be applied only if 'bApply' flag is
 * set to TRUE. Otherwise the supplied sampling rate and its corresponding
 * Channel ID will be cached by Audio codec class driver and applied only in the
 * next call to this function with 'bApply' flag set to TRUE.
 */
uint32_t  adi_dev_Audio_SetSampleRate (
    ADI_DEV_AUDIO_HANDLE            hAudioDevice,
    ADI_DEV_AUDIO_CHANNEL_ID        eChannelID,
    uint32_t                        nSampleRateHz,
    bool                            bApply
);

/*=============  D A T A    T Y P E S   =============*/

/*
** Event IDs common to all Audio Codec Class drivers
*/

enum
{
    /* 0x50010000 - Start ID of event codes common to all Audio codec class drivers */
    ADI_DEV_AUDIO_EVENT_START = E_ADI_DEV_CLASS_AUDIO_START,

    /* 0x50010001 - Completed processing a Callback buffer sized
                    audio output buffer
                    Callback Argument = NULL */
    ADI_DEV_AUDIO_EVENT_OUT_BUFFER_PROCESSED,

    /* 0x50010002 - Completed processing a Callback buffer sized
                    audio input buffer
                    Callback Argument = NULL */
    ADI_DEV_AUDIO_EVENT_IN_BUFFER_PROCESSED,

    /* 0x50010003 - Output buffer underflow has occurred. This event is posted when
                    there is not enough valid samples available to transmit.
                    Audio device will output silence in such case
                    Callback Argument = NULL */
    ADI_DEV_AUDIO_EVENT_OUT_BUFFER_UNDERFLOW,

    /* 0x50010004 - Input buffer overflow has occurred. This event is posted when
                    there is no empty sub-buffer to receive inbound audio samples.
                    The last sub-buffer queued to DMA is used in such case
                    Callback Argument = NULL */
    ADI_DEV_AUDIO_EVENT_IN_BUFFER_OVERFLOW

};

/**
 * Command IDs common to all Audio codec class device drivers.
 * Each command is issue with an associated "Value" as outline below
 */
typedef enum
{
    ADI_DEV_AUDIO_CMD_TABLE,            /*!< Command to issue a table of control commands to an Audio codec class device.
                                             Value = ADI_DEV_AUDIO_CMD_VALUE_PAIR table.
                                             Table must be terminated with ADI_DEV_AUDIO_CMD_END.  */
    ADI_DEV_AUDIO_CMD_END,              /*!< Command to indicate end of a control command table.
                                             Value = NULL.  */
    ADI_DEV_AUDIO_CMD_PAIR,             /*!< Command to issue a single command-value pair (Address of a ADI_DEV_AUDIO_CMD_VALUE_PAIR instance).
                                             Value = ADI_DEV_AUDIO_CMD_VALUE_PAIR. */

    /*
    ** Audio Device Configuration Commands
    */

    ADI_DEV_AUDIO_CMD_ENABLE_CHANNEL,       /*!< Enables a particular audio channel. Command specific value = Enumeration of type ADI_DEV_AUDIO_CHANNEL_ID (Logical Audio channel ID of to enable)   */

    ADI_DEV_AUDIO_CMD_DISABLE_CHANNEL,      /*!<  Disables a particular audio channel. Command specific value = Enumeration of type ADI_DEV_AUDIO_CHANNEL_ID   (Logical Audio channel ID of to disable) */

    ADI_DEV_AUDIO_CMD_MUTE_CHANNEL,         /*!< Mutes a particular audio channel. Command specific value = Enumeration of type ADI_DEV_AUDIO_CHANNEL_ID Logical Audio channel ID of to mute)    */

    ADI_DEV_AUDIO_CMD_UNMUTE_CHANNEL,       /*!< Un-mutes a particular audio channel.   Command specific value = Enumeration of type ADI_DEV_AUDIO_CHANNEL_ID (Logical Audio channel ID of to un-mute) */

    ADI_DEV_AUDIO_CMD_SET_DATA_MODE,            /*!< Sets Audio Data Mode. Command specific value = Address of an instance of type ADI_DEV_AUDIO_CONFIG_DATA_MODE   */
    ADI_DEV_AUDIO_CMD_RESET,                    /*!< Resets Audio device hardware to power-up/default values. Command specific value = NULL     */

    ADI_DEV_AUDIO_CMD_SUBMIT_OUT_DATA_BUFFER,    /*!< Submits an Audio Receive (Input) data buffer for Audio codec class driver use. Command specific value = Pointer to an instance of type ADI_DEV_AUDIO_DATA_BUFFER    */

    ADI_DEV_AUDIO_CMD_SUBMIT_IN_DATA_BUFFER,    /*!< Submits an Audio Input (Receive) data buffer for Audio codec class driver use. Command specific value = Address of an instance of type ADI_DEV_AUDIO_DATA_BUFFER    */

    ADI_DEV_AUDIO_CMD_FLUSH_OUT_DATA_BUFFER,    /*!< Flushes Output data buffer by clearing leftover samples that are not transmitted/played yet.  Command specific value = NULL  */

    ADI_DEV_AUDIO_CMD_FLUSH_IN_DATA_BUFFER,    /*!< Flushes Input data buffer by clearing leftover samples that are not consumed by the application.  Command specific value = NULL  */


    ADI_DEV_AUDIO_CMD_OUT_BUFFER_PAD_ZERO,    /*!< Instructs Audio codec class driver to pad zeros to terminate an output (transmit) sub-buffer, there by eliminating noise at the end of playback.  Command specific value = NULL */

    ADI_DEV_AUDIO_CMD_ENABLE_IN_BUFFER_PAD_ZERO,    /*!< Enables/Disables Audio codec class driver to pad zeros to create an input (receive) buffer of application specified size   Command specific value = 'true' to enable, 'false' to disable     */

    ADI_DEV_AUDIO_CMD_GET_NUM_IN_VALID_DATA,    /*!<  Gets the count of number of valid data in bytes, currently available in audio input buffer. Command specific value = uint32_t * (Address of the location to hold number of valid bytes count) */

    ADI_DEV_AUDIO_CMD_SET_MAX_OUT_BUFFER_REQ_SIZE,    /*!< Sets the maximum output buffer size that can be  requested by application.  Command specific value = uint32_t (Maximum buffer size in bytes) */


    ADI_DEV_AUDIO_CMD_SET_MAX_IN_BUFFER_REQ_SIZE,    /*!< Sets the maximum input buffer size that can be requested by application. Command specific value = uint32_t (Maximum buffer size in bytes)   */

    /*
    ** Audio Data Port configuration commands
    */

    ADI_DEV_AUDIO_CMD_SET_IN_PORT_DEVICE_NUMBER,    /*!< Sets Audio Input Port Device number to use. Command specific value = uint32_t (Device Number)    */

    ADI_DEV_AUDIO_CMD_SET_OUT_PORT_DEVICE_NUMBER,    /*!< Sets Audio Output Port Device number.  Command specific value = uint32_t (Device Number)   */


    ADI_DEV_AUDIO_CMD_ENABLE_IN_PORT,    /*!< Enables/Disables Audio Input Data Port.   Command specific value = true/false (true to enable, false to disable)  */

    ADI_DEV_AUDIO_CMD_ENABLE_OUT_PORT,    /*!< nbles/Disables Audio Output Data Port. Command specific value = true/false (true to enable, false to disable)   */


    ADI_DEV_AUDIO_CMD_SET_SPI_DEVICE_NUMBER,    /*!< Sets SPI Device Number used to access Audio device registers. Command specific value = uint32_t (Device Number)     */


    ADI_DEV_AUDIO_CMD_SET_SPI_CHIP_SELECT,    /*!< Sets SPI Chip Select value to access Audio device registers. Command specific value = uint32_t (Chip select value)    */


    ADI_DEV_AUDIO_CMD_SET_TWI_DEVICE_NUMBER,    /*!<  Sets TWI Device Number used to access Audio device registers.  Command specific value = uint32_t (Device Number)   */

    ADI_DEV_AUDIO_CMD_SET_TWI_GLOBAL_ADDRESS,    /*!< Sets TWI Global Address used to access Audio device registers.\n Command specific value = uint32_t (TWI Global Address)   */

    /* 0x50010023 - Sets Pseudo TWI Flag ID to be used for TWI Serial Clock (SCL)
       Command Specific value = Enumerated value of type ADI_FLAG_ID */
    ADI_DEV_AUDIO_CMD_SET_PSEUDO_TWI_SCL_FLAG_ID,    /*!<     */

    /* 0x50010024 - Sets Pseudo TWI Flag ID to be used for TWI Serial Data (SDA)
       Command Specific value = Enumerated value of type ADI_FLAG_ID */
    ADI_DEV_AUDIO_CMD_SET_PSEUDO_TWI_SDA_FLAG_ID,    /*!<     */

    /* 0x50010025 - Sets Pseudo TWI Timer ID to be used to generate TWI Serial Clock (SCL)
       Command Specific value = uint32_t (Timer ID) */
    ADI_DEV_AUDIO_CMD_SET_PSEUDO_TWI_TIMER_ID,    /*!<     */

    /* 0x50010026 - Sets Peripheral interrupt ID of the Pseudo TWI Timer
                    allocated to generate TWI Serial Clock (SCL)
       Command Specific value = enumeration of type ADI_INT_PERIPHERAL_ID */
    ADI_DEV_AUDIO_CMD_SET_PSEUDO_TWI_TIMER_INT_ID,    /*!<     */



    /*
    ** Commands to set other Hardware specific configurations
    */

    ADI_DEV_AUDIO_CMD_SET_RESET_FLAG_ID

} ADI_DEV_AUDIO_CMD_IDS;


/*! Result codes common to all Audio Codec Class drivers */
typedef enum __AdiDevAudioClassResult
{
	ADI_DEV_AUDIO_RESULT_NOERROR,               /*!<  Success.    */
    ADI_DEV_AUDIO_RESULT_HANDLE_INVALID,        /*!<  Given Audio device handle is invalid.    */
    ADI_DEV_AUDIO_RESULT_DEV_OPEN_FAILED,       /*!< Could not open audio device. Device already opened       */

    ADI_DEV_AUDIO_RESULT_CMD_NOT_SUPPORTED,    /*!<   The command issued by the application is not supported.   */

    ADI_DEV_AUDIO_RESULT_CONFIG_INCOMPLETE,    /*!<  Cannot communicate with the Audio codec class/Device Driver as the device specific configuration is incomplete.  */

    ADI_DEV_AUDIO_RESULT_FN_NOT_SUPPORTED,     /*!< Requested Function is not supported by the Audio device.    */

    ADI_DEV_AUDIO_RESULT_FN_NOT_PERMITTED,     /*!<   Requested function not permitted in current state.   */

    ADI_DEV_AUDIO_RESULT_CHANNEL_INVALID,      /*!<    Supplied Audio Channel ID is invalid for this Audio device.  */

    ADI_DEV_AUDIO_RESULT_DATA_MODE_INVALID,    /*!<   Supplied data mode is invalid.   */

    ADI_DEV_AUDIO_RESULT_WORD_LENGTH_INVALID,    /*!<   Supplied word length is invalid.   */

    ADI_DEV_AUDIO_RESULT_SAMPLE_RATE_INVALID,    /*!<   Supplied Sampling Rate is invalid for the given Audio channel.   */

    ADI_DEV_AUDIO_RESULT_REGISTER_ACCESS_FAILED,    /*!<    Device Hardware Register access failed.  */

    ADI_DEV_AUDIO_RESULT_DMA_WIDTH_INVALID,    /*!< DMA Bus width of Input/Output port is invalid.     */

    ADI_DEV_AUDIO_RESULT_SEM_CREATE_FAILED,    /*!<   Failed to create semaphore(s) required to gain exclusive access to the Audio codec class device.  */

    ADI_DEV_AUDIO_RESULT_SEM_ACQUIRE_FAILED,    /*!<  Failed to acquire a semaphore required to gain exclusive access to the Audio codec class device.  */

    ADI_DEV_AUDIO_RESULT_DATAFLOW_ALREADY_ENABLED,    /*!<  Requested task can not be performed when the Audio dataflow is enabled.   */

    ADI_DEV_AUDIO_RESULT_BUFFER_SIZE_ERROR,    /*!<   Supplied buffer does not meet minimum size requirement of 4 bytes.  */

    ADI_DEV_AUDIO_RESULT_NO_EMPTY_BUFFER,    /*!<   Output buffer does not have enough empty bytes as requested by the application. */

    ADI_DEV_AUDIO_RESULT_NO_FULL_BUFFER,    /*!< Input buffer does not have the number of valid bytes requested by the application.    */

    ADI_DEV_AUDIO_RESULT_IN_EMPTY_BYTES_INSUFFICIENT,    /*!<   Input buffer does not have enough empty bytes to create a callback buffer and enable input dataflow. */

    ADI_DEV_AUDIO_RESULT_BUFFER_ALREADY_ISSUED,    /*!< Failed to get a new input/output buffer since the application hasn't submitted the previously obtained input/output buffer.   */

    ADI_DEV_AUDIO_RESULT_BUFFER_LIMIT_EXCEEDED,    /*!< Exceeded Input/Output buffer size limit set by the application (or)  Exceeded buffer size acquired from the Audio codec class driver.  */


    ADI_DEV_AUDIO_RESULT_EVENT_GROUP_CREATE_FAILED   /*!< Unable to create a static event group. */


} ADI_DEV_AUDIO_CLASS_RESULT;

/*! Enumerations of Audio data interface modes */

typedef enum __AdiDevAudioDataMode
{

    ADI_DEV_AUDIO_DATA_MODE_UNDEFINED,              /*!<  Data Mode undefined    */
    ADI_DEV_AUDIO_DATA_MODE_I2S_STEREO,             /*!<  I2S - Stereo Mode   */
    ADI_DEV_AUDIO_DATA_MODE_LJ_STEREO,              /*!<  Left Justified (LJ) - Stereo Mode */
    ADI_DEV_AUDIO_DATA_MODE_TDM_TRUE_MULTICHANNEL   /*!<  Time Division Multiplexing (TDM) */

    /* Add more data modes here */

} ADI_DEV_AUDIO_DATA_MODE;

/*!< Structure to pass Audio Data Mode */
typedef struct __AdiDevAudioConfigDataMode
{
    ADI_DEV_AUDIO_CHANNEL_ID        eChannelID;   /*!< Logical Channel ID to which the data mode applies. */
    ADI_DEV_AUDIO_DATA_MODE         eDataMode;    /*!< Audio device data mode to set. */
} ADI_DEV_AUDIO_CONFIG_DATA_MODE;

/*! Structure to pass Audio data buffer (input or output) The buffer will be used by the Audio codec class driver in circular fashion */
typedef struct __AdiDevAudioDataBuffer
{
    void            *pBuffer;           /*!< Pointer to audio data buffer */
    uint32_t        nBufferSize;        /*!< Data buffer size in bytes */
} ADI_DEV_AUDIO_DATA_BUFFER;

/*
** Structure to pass a control command and its value
** Pointer to instance or table of instances of this structure
** is used as command specific value for Batch processing commands
*/
typedef struct __AdiDevAudioCmdValuePair
{
    uint32_t    nCommandID;     /*!< Command ID to process */
    void        *Value;         /*!< Command specific value */
} ADI_DEV_AUDIO_CMD_VALUE_PAIR;

/*==============  D E F I N E S  ===============*/

/*
**
** Enumerations of Maximum and Minimum Volume range
**  Audio Channel volume can be configured in linear steps ranging from
**  0 to 0xFFFF, where
**
**      0       = Minimum volume/Minimum Gain/Maximum attenuation
**      0xFFFF  = Maximum volume/Maximum Gain/Minimum attenuation
**
**  Audio codec class driver converts the linear value to
**  corresponding device/channel register equivalent and vice-versa.
*/

/*
** Enumeration for Audio channel - minimum volume level
*/
#define E_ADI_DEV_AUDIO_MIN_VOLUME                0U

/*
** Enumeration for Audio channel - maximum volume level
*/
#define E_ADI_DEV_AUDIO_MAX_VOLUME                0xFFFFU


#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* _LANGUAGE_C */

/*@}*/

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#endif  /* __ADI_DEV_AUDIO_H__ */

