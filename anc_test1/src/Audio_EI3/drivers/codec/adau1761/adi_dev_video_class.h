/*******************************************************************************
Copyright(c) 2008-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*******************************************************************************/

/**
 * @file      adi_dev_video_class.h
 *
 * @brief     ADI video input/output device class header file.
 *
 * @version   $Revision: 62861 $
 *
 * @date      $Date: 2019-11-27 09:36:45 -0500 (Wed, 27 Nov 2019) $
 *
 * @details
 *            This is the primary include file for video class devices supported
 *            by ADI device class drivers. This header file contains the data
 *            types and function definitions pertaining to video class devices.
 */

/** @addtogroup Video_Device_Class Video Device Class
 *  @{
 */

#ifndef __ADI_DEV_VIDEO_CLASS_H__
#define __ADI_DEV_VIDEO_CLASS_H__

/*==========  I N C L U D E  ==========*/
#include "adi_dev_class.h"
#include "adi_dev_video_entrypoint.h"

/* disable misra diagnostics as necessary */
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */

#if defined(_LANGUAGE_C)
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*=============  D A T A    T Y P E S   =============*/

/*! Video class device handle */
typedef void*    ADI_DEV_VIDEO_HANDLE;

/*!
** Enumerations of return values for Video class devices
*/
typedef enum __AdiDevVideoResult
{
    /*! Generic Success. */
    ADI_DEV_VIDEO_RESULT_SUCCESS                    = 0u,

    /*! Generic Failure. */
    ADI_DEV_VIDEO_RESULT_FAILED                     = 1u,

    /*! Failed since the entry point passed is invalid. */
    ADI_DEV_VIDEO_RESULT_INVALID_ENTRY_POINT        = (E_ADI_DEV_CLASS_VIDEO_START +  0u),

    /*! Failed while communicating with the video device. */
    ADI_DEV_VIDEO_RESULT_DEVICE_ACCESS_FAILED       = (E_ADI_DEV_CLASS_VIDEO_START +  1u),

    /*! Failed to open the underlying driver. */
    ADI_DEV_VIDEO_RESULT_DEVICE_OPEN_FAILED         = (E_ADI_DEV_CLASS_VIDEO_START +  2u),

    /*! Failed since the underlying driver is not opened. */
    ADI_DEV_VIDEO_RESULT_DEVICE_NOT_OPENED          = (E_ADI_DEV_CLASS_VIDEO_START +  3u),

    /*! Return code to notify that some hardware error has occurred. */
    ADI_DEV_VIDEO_RESULT_HARDWARE_ERR               = (E_ADI_DEV_CLASS_VIDEO_START +  4u),

    /*! Return code to notify that some video event has occurred. */
    ADI_DEV_VIDEO_RESULT_VIDEO_EVENT                = (E_ADI_DEV_CLASS_VIDEO_START +  5u),

    /*! No empty frame in the queue */
    ADI_DEV_VIDEO_RESULT_FRAME_NOT_AVAILABLE        = (E_ADI_DEV_CLASS_VIDEO_START +  6u),

    /*! No frame entry available to store the given video frame buffer pointer */
    ADI_DEV_VIDEO_RESULT_NO_FRAME_ENTRY             = (E_ADI_DEV_CLASS_VIDEO_START +  7u),

    /*! Insufficient memory to complete the requested operation */
    ADI_DEV_VIDEO_RESULT_NO_MEMORY                  = (E_ADI_DEV_CLASS_VIDEO_START +  8u),

    /*! Given pointer is invalid or NULL */
    ADI_DEV_VIDEO_RESULT_INVALID_POINTER            = (E_ADI_DEV_CLASS_VIDEO_START +  9u),

    /*! One or more given parameters are invalid */
    ADI_DEV_VIDEO_RESULT_INVALID_PARAMETER          = (E_ADI_DEV_CLASS_VIDEO_START + 10u),

    /*! Given video device class handle is invalid */
    ADI_DEV_VIDEO_RESULT_INVALID_HANDLE             = (E_ADI_DEV_CLASS_VIDEO_START + 11u),

    /*! Given command is not supported by this Video Device */
    ADI_DEV_VIDEO_RESULT_UNSUPPORTED_COMMAND        = (E_ADI_DEV_CLASS_VIDEO_START + 12u),

    /*! Given command is not permitted at this state */
    ADI_DEV_VIDEO_RESULT_COMMAND_NOT_PERMITTED      = (E_ADI_DEV_CLASS_VIDEO_START + 13u),

    /*! Given video format is not supported by this Video device */
    ADI_DEV_VIDEO_RESULT_VIDEO_FORMAT_NOT_SUPPORTED = (E_ADI_DEV_CLASS_VIDEO_START + 14u),

    /*! Input video signal does not match with the configured video format */
    ADI_DEV_VIDEO_RESULT_VIDEO_FORMAT_MISMATCH      = (E_ADI_DEV_CLASS_VIDEO_START + 15u)

    /* Add more error codes here */

} ADI_DEV_VIDEO_RESULT;


/*! Enumerations of control commands supported by Video Device Class. */
typedef enum __AdiDevVideoCommands
{
    /*!
    **   Enable the video device. This causes the device class to submit frames to the
    **   driver, configure the driver for the given video format and enable the data flow.
    **
    **   Argument : bool bEnable
    */
    ADI_DEV_VIDEO_CMD_ENABLE = E_ADI_DEV_CLASS_VIDEO_START,

    /*!
    **   Get the Hardware Error Status. If multiple errors have occurred then
    **   the returned status will be OR of all the errors that occurred.
    **
    **   Argument : uint32_t *pErrCodes
    */
    ADI_DEV_VIDEO_CMD_GET_HW_ERRORS,

    /*!
    **   Get the Video Events that has occurred. If multiple events have occurred
    **   then the return event status will be OR of all the Video Events.
    **
    **   Argument : uint32_t *pVidEvents
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_EVENTS,

    /*!
    **   Sets properties for the video format.
    **
    **   Argument : ADI_DEV_VIDEO_FRAME_PROPERTIES *pData
    */
    ADI_DEV_VIDEO_CMD_SET_FRAME_PROPERTIES,

    /*!
    **   Gets video frame properties.
    **
    **   Argument : ADI_DEV_VIDEO_FRAME_PROPERTIES **ppData
    */
    ADI_DEV_VIDEO_CMD_GET_FRAME_PROPERTIES,

    /*!
    **   Sets video format (frame resolution)  to be used for video input/output.
    **
    **   Argument : ADI_DEV_VIDEO_FORMAT eFormat
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_FORMAT,

    /*!
    **   Gets video format (frame resolution) currently set in the video device class. 
    **
    **   Argument : ADI_DEV_VIDEO_FORMAT *peFormat
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_FORMAT,

    /*!
    **   Sets video frame format to be used by the video device class. This selects between
    **   different video frame formats like active only, entire frame, blanking only etc.
    **
    **   Argument : ADI_DEV_VIDEO_FRAME eFrameFormat
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_FRAME_FORMAT,

    /*!
    **   Gets video frame format used by the video device class. 
    **
    **   Argument : ADI_DEV_VIDEO_FRAME *peFrameFormat
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_FRAME_FORMAT,

    /*!
    **   Sets the video signal format.
    **
    **   Argument : ADI_DEV_VIDEO_SIGNAL  eVidSignal
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_SIGNAL_FORMAT,

    /*!
    **   Gets the current video signal format.
    **
    **   Argument : ADI_DEV_VIDEO_SIGNAL  *peVidSignal
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_SIGNAL_FORMAT,

    /*!
    **   Sets the video frame rate.
    **
    **   Argument : uint32_t nVidFrameRate
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_FRAME_RATE,

    /*!
    **    Gets the video frame rate.
    **
    **    Argument : uint32_t *pnVidFrameRate
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_FRAME_RATE,

    /*!
    **   Sets the data format for the video data frame.
    **
    **   Argument : ADI_DEV_VIDEO_DATA eVidDataFormat
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_DATA_FORMAT,

    /*!
    **   Gets the data format for the video data frame.
    **
    **   Argument : ADI_DEV_VIDEO_DATA *peVidDataFormat
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_DATA_FORMAT,

    /*!
    **   Sets the Video Port to be used by the device.
    **
    **   Argument : ADI_DEV_VIDEO_PORT eVidPort
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_PORT,

    /*!
    **   Gets the Video Port with is currently used by the device.
    **
    **   Argument : ADI_DEV_VIDEO_PORT *peVidPort
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_PORT,

    /*!
    **   Sets the port number to be used if multiple ports of the same type is
    **   available. For example if ports HDMI A and HDMI B is available, the
    **   port 0 and port 1 corresponds to HDMI A and HDMI B respectively.
    **
    **   Argument : uint32_t nPortNum
    */
    ADI_DEV_VIDEO_CMD_SET_VIDEO_PORT_NUMBER,

    /*!
    **   Gets the port number currently set in the video device class.
    **
    **   Argument : uint32_t *pnPortNum
    */
    ADI_DEV_VIDEO_CMD_GET_VIDEO_PORT_NUMBER,

    /*!
    **   Set the EDID information for the configured port.
    **
    **   Argument : ADI_DEV_VIDEO_EDID_INFO *pEdidInfo
    */
    ADI_DEV_VIDEO_CMD_SET_EDID_INFO,

    /*!
    **   Get the EDID information for the configured port.
    **
    **   Argument : ADI_DEV_VIDEO_EDID_INFO *pEdidInfo
    */
    ADI_DEV_VIDEO_CMD_GET_EDID_INFO,

    /*!
    **   Enable/Disable EDID information for the configured port.
    **
    **   Argument : bool bEnableEDID
    */
    ADI_DEV_VIDEO_CMD_ENABLE_EDID,

    /*!
    **   Convert the current video format into another.
    **
    **   Argument : ADI_DEV_VIDEO_CONVERT  eVidConv
    */
    ADI_DEV_VIDEO_CMD_CONVERT_VIDEO_FORMAT,

    /*!
    **   Sets the number of frame entries which holds the pointer to the buffer
    **   which is submitted by application.
    **
    **   Argument : uint32_t nFrameEntries
    */
    ADI_DEV_VIDEO_CMD_SET_NUM_FRAME_ENTRIES,

    /*!
    **   Gets the number of frame entries which holds the pointer to the buffer
    **   which is submitted by application.
    **
    **   Argument : uint32_t *pnFrameEntries
    */
    ADI_DEV_VIDEO_CMD_GET_NUM_FRAME_ENTRIES,

    /*!
    **   Gets the number of buffers that are available in the device class that
    **   can be retrieved by the application.
    **
    **   Argument : uint32_t *pnNumAvailBuffs
    */
    ADI_DEV_VIDEO_CMD_GET_NUM_AVAILABLE_BUFFERS,

    /*!
    **   Sets the PPI/EPPI number.
    **
    **   Argument : uint32_t nPPINum
    */
    ADI_DEV_VIDEO_CMD_SET_PPI_NUMBER,

    /*!
    **   Sets the TWI Device Number.
    **
    **   Argument : uint32_t nTWIDeviceNumber
    */
    ADI_DEV_VIDEO_CMD_SET_TWI_DEVICE_NUMBER,

    /*!
    **   Sets the TWI Global Address.
    **
    **   Argument : uint32_t nTWIGlobalAddress
    */
    ADI_DEV_VIDEO_CMD_SET_TWI_GLOBAL_ADDRESS,

    /*!
    **   Sets the SPI Device Number.
    **
    **   Argument : uint32_t nSPIDeviceNumber
    */
    ADI_DEV_VIDEO_CMD_SET_SPI_DEVICE_NUMBER,

    /*!
    **   Sets the SPI Chip Select.
    **
    **   Argument : uint32_t nSPIChipSelect
    */
    ADI_DEV_VIDEO_CMD_SET_SPI_CHIP_SELECT,

    /*!
    **   Gets the Number of frames for which buffer underflow/overflow occurred.
    **
    **   Argument : uint32_t *pNumErrBuffs
    */
    ADI_DEV_VIDEO_CMD_GET_NUM_ERROR_FRAMES,

    /*!
    **   Sets the DMA Transfer Size to/from PPI
    **
    **   Argument : ADI_DEV_VIDEO_DMA_TRANSFER_SIZE eDMATransferSize
    */
    ADI_DEV_VIDEO_CMD_SET_DMA_TRANSFER_SIZE,

    /*!
    **   Get the current DMA Transfer Size to/from PPI
    **
    **   Argument : ADI_DEV_VIDEO_DMA_TRANSFER_SIZE *peDMATransferSize
    */
    ADI_DEV_VIDEO_CMD_GET_DMA_TRANSFER_SIZE,

    /*!
    **   Set the X Modify of the DMA
    **
    **   Argument : int32_t nXModify
    */
    ADI_DEV_VIDEO_CMD_SET_X_MODIFY,

    /*!
    **   Set the Y Modify of the DMA.
    **   DMA moves the given number of bytes after bringing in a line.
    **
    **   Argument : int32_t nYModify
    */
    ADI_DEV_VIDEO_CMD_SET_Y_MODIFY,

    /*!
    **   Resets the video device.
    **
    **   Argument : None
    */
    ADI_DEV_VIDEO_CMD_RESET_DEVICE,

    /*!
    **   Set the Split Height of the Frame. This command is used if frame need to be cut to different slices. The maximum height of the slice
    **   will be the values set by this command. If the height of the frame is not a multiple of Split Height, then the last slice of the frame will
    **   have less number of lines than the split height. Setting a value of 0 to split height disables it.
    **
    **   Argument : uin32_t nSplitHeight
    */
    ADI_DEV_VIDEO_CMD_SET_SPLIT_HEIGHT



    /* Add more control commands here */

} ADI_DEV_VIDEO_CMD;

/*! Enumeration of Video Signal Format */
typedef enum __AdiDevVideoSignal
{
    ADI_DEV_VIDEO_SIGNAL_NONE,              /*!< No video signal detected */
    ADI_DEV_VIDEO_SIGNAL_UNKNOWN,           /*!< Video type unknown */
    ADI_DEV_VIDEO_SIGNAL_NOT_SUPPORTED,     /*!< Video type is not supported */
    ADI_DEV_VIDEO_SIGNAL_NTSC_M_J,          /*!< SD - NTSC M/J */
    ADI_DEV_VIDEO_SIGNAL_NTSC_443,          /*!< SD - NTSC 443 */
    ADI_DEV_VIDEO_SIGNAL_PAL_BGHID,         /*!< SD - PAL-B/G/H/I/D */
    ADI_DEV_VIDEO_SIGNAL_PAL_M,             /*!< SD - PAL-M */
    ADI_DEV_VIDEO_SIGNAL_PAL_60,            /*!< SD - PAL-60 */
    ADI_DEV_VIDEO_SIGNAL_PAL_N,             /*!< SD - PAL-N */
    ADI_DEV_VIDEO_SIGNAL_SECAM,             /*!< SD - SECAM */
    ADI_DEV_VIDEO_SIGNAL_SD_720X480I,       /*!< SD - 480i, 720x480i */
    ADI_DEV_VIDEO_SIGNAL_SD_720X576I,       /*!< SD - 576i, 720x576i */
    ADI_DEV_VIDEO_SIGNAL_ED_720X480P,       /*!< ED - 480p, 720x480p */
    ADI_DEV_VIDEO_SIGNAL_ED_720X576P,       /*!< ED - 576p, 720x576p */
    ADI_DEV_VIDEO_SIGNAL_HD_1280X720P,      /*!< HD - 720p, 1280x720p */
    ADI_DEV_VIDEO_SIGNAL_HD_1920X1080I,     /*!< HD - 1080i, 1920x1080i */
    ADI_DEV_VIDEO_SIGNAL_HD_1920X1080P,     /*!< HD - 1080p, 1920x1080p */
    ADI_DEV_VIDEO_SIGNAL_QVGA_320X240P,     /*!< QVGA - 320x240p */
    ADI_DEV_VIDEO_SIGNAL_WVGA_800X480P,     /*!< WVGA - 800x480p */
    ADI_DEV_VIDEO_SIGNAL_VGA_640X480P,      /*!< VGA - 640x480p */
    ADI_DEV_VIDEO_SIGNAL_SVGA_800X600P,     /*!< SVGA - 800x600p */
    ADI_DEV_VIDEO_SIGNAL_XGA_1024X768P      /*!< XGA - 1024x768p */

} ADI_DEV_VIDEO_SIGNAL;


/*! Enumerations of video formats for video class device */
typedef enum __AdiDevVideoFormat
{
    ADI_DEV_VIDEO_FORMAT_320X240P,          /*!< QVGA - 320x240p */
    ADI_DEV_VIDEO_FORMAT_800X480P,          /*!< WVGA - 800x480p */
    ADI_DEV_VIDEO_FORMAT_640X480P,          /*!< VGA - 640x480p */
    ADI_DEV_VIDEO_FORMAT_800X600P,          /*!< SVGA - 800x600p */
    ADI_DEV_VIDEO_FORMAT_1024X768P,         /*!< XGA - 1024x768p */
    ADI_DEV_VIDEO_FORMAT_720X480I,          /*!< SD - 480i, 720x480i */
    ADI_DEV_VIDEO_FORMAT_720X576I,          /*!< SD - 576i, 720x576i */
    ADI_DEV_VIDEO_FORMAT_720X480P,          /*!< ED - 480p, 720x480p */
    ADI_DEV_VIDEO_FORMAT_720X576P,          /*!< ED - 576p, 720x576p */
    ADI_DEV_VIDEO_FORMAT_1280X720P,         /*!< HD - 720p, 1280x720p */
    ADI_DEV_VIDEO_FORMAT_1920X1080I,        /*!< HD - 1080i, 1920x1080i */
    ADI_DEV_VIDEO_FORMAT_1920X1080P,        /*!< HD - 1080p, 1920x1080p */

    /* Add new Video Formats here */

    ADI_DEV_VIDEO_FORMAT_CUSTOM             /*!< Custom Video Format */

} ADI_DEV_VIDEO_FORMAT;

/*! Enumeration of video ports */
typedef enum __AdiDevVideoPort
{
    ADI_DEV_VIDEO_PORT_CVBS,               /*!< CVBS input (Composite Video, Blanking and Sync) */
    ADI_DEV_VIDEO_PORT_S_VIDEO,            /*!< S-Video (Separate Video) or Y/C */
    ADI_DEV_VIDEO_PORT_SCART,              /*!< SCART */
    ADI_DEV_VIDEO_PORT_COMPONENT_YPBPR,    /*!< Component - YPbPr */
    ADI_DEV_VIDEO_PORT_COMPONENT_RGB,      /*!< Component - RGB */
    ADI_DEV_VIDEO_PORT_VGA,                /*!< VGA */
    ADI_DEV_VIDEO_PORT_HDMI,               /*!< HDMI Port */
    ADI_DEV_VIDEO_PORT_LCD                 /*!< LCD Port */

} ADI_DEV_VIDEO_PORT;

/*! Enumeration of video format conversions */
typedef enum __AdiDevVideoConvert
{
    /*! No video format converstion */
    ADI_DEV_VIDEO_CONVERT_NONE,

    /*! Convert Interlace Video to Progressive at Interlace FPS */
    ADI_DEV_VIDEO_CONVERT_I_TO_P_IFPS,

    /*! Convert Interlace Video to Progressive at Progressive FPS */
    ADI_DEV_VIDEO_CONVERT_I_TO_P_PFPS,

    /*! Convert RGB888 Video to RGB666 Video Format */
    ADI_DEV_VIDEO_CONVERT_RGB888_TO_RGB666,

    /*! Convert RGB888 Video to RGB565 Video Format */
    ADI_DEV_VIDEO_CONVERT_RGB888_TO_RGB565

} ADI_DEV_VIDEO_CONVERT;

/*!
** Enumerations for Hardware Errors
*/
typedef enum __AdiDevVideoHWErrors
{
    /*! No Hardware Error Occurred */
    ADI_DEV_VIDEO_HW_ERR_NONE           = 0x00000000,

    /*! Preamble Error Detected but not corrected */
    ADI_DEV_VIDEO_HW_ERR_NC_PREAMBLE    = 0x00000001,

    /*! Preamble Error Detected */
    ADI_DEV_VIDEO_HW_ERR_PREAMBLE       = 0x00000002,

    /*! FIFO Error Detected */
    ADI_DEV_VIDEO_HW_ERR_FIFO           = 0x00000004,

    /*! Frame Track Error Detected */
    ADI_DEV_VIDEO_HW_ERR_FRAME_TRACK    = 0x00000008,

    /*! Line Track Error Detected */
    ADI_DEV_VIDEO_HW_ERR_LINE_TRACK     = 0x00000010,

    /*! DMA Error has occurred */
    ADI_DEV_VIDEO_HW_ERR_DMA_ERROR      = 0x00010000

    /* Add more HW Error codes here */

} ADI_DEV_VIDEO_HW_ERR;

/*!
** Enumeration for Video Events.
*/
typedef enum __AdiDevVideoEvents
{
    /*! No Video Event Occurred */
    ADI_DEV_VIDEO_EVENT_NONE              = 0x00000000,

    /*! Input Video Changed Event */
    ADI_DEV_VIDEO_EVENT_VIDEO_IN_CHANGED  = 0x00000001

    /* Add more Events here */

} ADI_DEV_VIDEO_EVENT;

/*!
** Enumeration for Video Frame Format.
*/
typedef enum __AdiDevVideoFrameFormats
{
    /*! Transmit/Receive active video data on Field 1 and Field 2 */
    ADI_DEV_VIDEO_FRAME_ACTIVE_FIELD1_FIELD2,

    /*! Receive active video data on Field 1 only */
    ADI_DEV_VIDEO_FRAME_ACTIVE_FIELD1_ONLY,

    /*! Transmit/Receive entire video frame including the blanking information */
    ADI_DEV_VIDEO_FRAME_ENTIRE_FRAME,

    /*! Receive blanking data only */
    ADI_DEV_VIDEO_FRAME_BLANKING_DATA

} ADI_DEV_VIDEO_FRAME;

/*!
** Enumeration for Video data color formats.
*/
typedef enum __AdiDevVideoDataFormats
{
    /*! UYVY 4:2:2 video data format */
    ADI_DEV_VIDEO_DATA_UYVY422,

    /*! RGB 8:8:8 video data format */
    ADI_DEV_VIDEO_DATA_RGB888,

    /*! RGB 6:6:6 video data format */
    ADI_DEV_VIDEO_DATA_RGB666,

    /*! RGB 5:6:5 video data format */
    ADI_DEV_VIDEO_DATA_RGB565

} ADI_DEV_VIDEO_DATA;

/*!
** Enumeration for Video DMA Transfer Sizes 
*/
typedef enum __AdiDevVideoDMATransferSize
{
    /*! Automatically choose the best available transfer size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_AUTO     = 0,

    /*! 8-bit(1 byte) DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_8_BIT    = 1,

    /*! 16-bit DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_16_BIT   = 2,

    /*! 32-bit DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_32_BIT   = 4,

    /*! 64-bit DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_64_BIT   = 8,

    /*! 128-bit DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_128_BIT  = 16,

    /*! 256-bit DMA Transfer Size */
    ADI_DEV_VIDEO_DMA_TRANSFER_SIZE_256_BIT  = 32

} ADI_DEV_VIDEO_DMA_TRANSFER_SIZE;
/*!
** Data structure to hold frame properties of the video device
*/
typedef struct __AdiDevVideoFrameProperties
{
    /*! Pixels Per Line for selected video format */
    uint32_t        nPixelsPerLine;

    /*! Lines Per Frame for selected video format */
    uint32_t        nLinesPerFrame;

    /*! Number of bytes required to store each pixel */
    uint32_t        nBytesPerPixel;

    /*! Number of Top Padding bytes required for selected video format */
    uint32_t        nNumTopPadBytes;

    /*! Number of Bottom Padding bytes required for selected video format */
    uint32_t        nNumBottomPadBytes;

    /*! Total Number of Lines Per Frame including top and
       bottom pad lines for selected video format */
    uint32_t        nTotalLinesPerFrame;

    /*! Total Frame Size in bytes required for selected video format */
    uint32_t        nFrameSizeInBytes;

    /*! Callback Interval between two frames in milliseconds,
       derived from the device operating frequency and resolution.
       It does not account for the amount of latency involved in
       processing the video device interrupt */
    uint32_t        nCallbackInterval;

} ADI_DEV_VIDEO_FRAME_PROPERTIES;

/*!
** Data structure to hold the EDID information
*/
typedef struct __AdiDevVideoEdidInfo
{
    /*! Pointer to the EDID information */
    void       *pEdidData;

    /*! Size of the EDID information in bytes */
    uint32_t    nEdidSizeInBytes;

    /*! Size of the buffer in bytes */
    uint32_t    nBufSizeInBytes;

} ADI_DEV_VIDEO_EDID_INFO;

/*!
 * Parameters that can be passed to the adi_dev_Video_Open_Ex function
 */
typedef enum __AdiDevVideoParam
{
	/*! Marks the end of the table */
	ADI_DEV_VIDEO_CONFIG_END      = E_ADI_DEV_CLASS_VIDEO_START,

	/*! Heap ID to be used for the memory allocation */
	ADI_DEV_VIDEO_CONFIG_HEAP_ID
} ADI_DEV_VIDEO_PARAM;

/*!
 * Data structure to hold the table row entry
 */
typedef struct __AdiDevVideoParamValuePair
{
	/*! Parameter name of the entry */
	ADI_DEV_VIDEO_PARAM  eParam;

	/*! Value to the corresponding parameter */
	void*                Value;
	
} ADI_DEV_VIDEO_PARAM_VALUE_PAIR;

/*=======  P U B L I C   P R O T O T Y P E S  ========*/
/*            (globally-scoped functions)             */

/* Opens a video class for Input/Output */
uint32_t  adi_dev_Video_Open (
    const ADI_DEV_VIDEO_CLASS_ENTRY_POINT  *pVideoEntryPoint,
    uint32_t                                nDeviceNumber,
    ADI_DEV_VIDEO_HANDLE                   *phVideoDevice
);

/* Extended version of adi_dev_Video_Open function */
uint32_t adi_dev_Video_Open_Ex (
	    const ADI_DEV_VIDEO_CLASS_ENTRY_POINT  *pVideoEntryPoint,
	    uint32_t                                nDeviceNumber,
	    ADI_DEV_VIDEO_HANDLE                   *phVideoDevice,
	    ADI_DEV_VIDEO_PARAM_VALUE_PAIR         *pParamValueTable
);


/* Closes a Video class device */
uint32_t  adi_dev_Video_Close(
    ADI_DEV_VIDEO_HANDLE        hVideoDevice
);

/* Submit a Frame to the Video Class driver */
uint32_t  adi_dev_Video_SubmitFrame(
    ADI_DEV_VIDEO_HANDLE        hVideoDevice,
    void                       *pFrameBuffer
);

/* Submit a repeat frame to the video class driver.
 * The given frame will be repeated the given number of times.
 * (Mainly for Video Output but can be used for Video Input in order to reduce
 * the frame rate)
 */
uint32_t  adi_dev_Video_SubmitRepeatFrame(
    ADI_DEV_VIDEO_HANDLE        hVideoDevice,
    void                       *pFrameBuffer,
    uint32_t                    nRepeatCount
);

/* Get a frame from the Video class driver. The function will pend if frame
 * not available.
 */
uint32_t  adi_dev_Video_GetFrame(
    ADI_DEV_VIDEO_HANDLE        hVideoDevice,
    void                      **ppFrameBuffer
);

/* Send commands to the video class driver */
uint32_t adi_dev_Video_Control(
    ADI_DEV_VIDEO_HANDLE        hVideoDevice,
    uint32_t                    nCommand,
    void                       *pData
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#else /* not _LANGUAGE_C */

/* Assembler-specific */

#endif /* not _LANGUAGE_C */

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#endif      /* __ADI_DEV_VIDEO_CLASS_H__ */

/* @} */
