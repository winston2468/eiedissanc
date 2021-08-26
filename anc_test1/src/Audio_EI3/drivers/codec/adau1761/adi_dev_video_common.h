/*******************************************************************************
Copyright(c) 2010-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

$Revision: 62861 $
$Date: 2019-11-27 09:36:45 -0500 (Wed, 27 Nov 2019) $

*******************************************************************************/

/*******************************************************************************
@file       adi_dev_video_common.h

@brief      Internal header file for video class driver containing common
            defines, data types and function definitions.

*******************************************************************************/

#ifndef __ADI_DEV_VIDEO_COMMON_H__
#define __ADI_DEV_VIDEO_COMMON_H__

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */

/*==============  D E F I N E S  ===============*/

/*
** NTSC Standard definitions
*/
/* Number of Active Pixels Per Line in NTSC Standard */
#define ADI_DEV_NTSC_ACTIVE_PIXELS_PER_LINE                     (720u)
/* Number of Pixels per Line in NTSC Standard */
#define ADI_DEV_NTSC_PIXELS_PER_LINE                            (858u)
/* Number of Active Video Lines Per Frame in NTSC Standard */
#define ADI_DEV_NTSC_ACTIVE_LINES_PER_FRAME                     (487u)
/* Number of Video Lines Per Frame in NTSC Standard */
#define ADI_DEV_NTSC_LINES_PER_FRAME                            (525u)
/* Number of blanking  lines for top field in NTSC Standard */
#define ADI_DEV_NTSC_TOP_FIELD_BLANK_LINES                       (18u)
/* Number of blanking  lines for bottom field in NTSC Standard */
#define ADI_DEV_NTSC_BOTTOM_FIELD_BLANK_LINES                    (20u)

/*
** PAL Standard definitions
*/
/* Number of Active Pixels Per Line in PAL Standard */
#define ADI_DEV_PAL_ACTIVE_PIXELS_PER_LINE                      (720u)
/* Number of Pixels per Line in PAL Standard */
#define ADI_DEV_PAL_PIXELS_PER_LINE                             (864u)
/* Number of Active Video Lines Per Frame in PAL Standard */
#define ADI_DEV_PAL_ACTIVE_LINES_PER_FRAME                      (576u)
/* Number of Video Lines Per Frame in PAL Standard */
#define ADI_DEV_PAL_LINES_PER_FRAME                             (625u)
/* Number of blanking  lines for top field in PAL Standard */
#define ADI_DEV_PAL_TOP_FIELD_BLANK_LINES                        (24u)
/* Number of blanking  lines for bottom field in PAL Standard */
#define ADI_DEV_PAL_BOTTOM_FIELD_BLANK_LINES                     (25u)

/*
** Standard Video Definitions
*/
/* Number of bytes per Pixel */
#define ADI_DEV_VIDEO_BYTES_PER_PIXEL                            (2u)

#if defined(_LANGUAGE_C)
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*=============  D A T A    T Y P E S   =============*/

/*
 * Structure to hold details of the bit mapping from driver HW Error to
 * device class HW Error
 */
typedef struct _ADI_DEV_VIDEO_BIT_MAP
{
    uint32_t nSrcBitMask;        /* Source Bit Mask */
    uint32_t nDstBitMask;        /* Destination Bit Mask */
} ADI_DEV_VIDEO_BIT_MAP;

/*==========  F U N C T I O N    D E C L A R A T I O N S ==========*/

/* Map device error codes to video class error codes based on mapping table */
uint32_t _adi_dev_Video_MapErrCodes (
                                     uint32_t               nDevErrCodes,
                                     ADI_DEV_VIDEO_BIT_MAP *pBitMapTable,
                                     uint32_t               nNumEntries
                                     );
/* Allocate memory */
uint32_t _adi_dev_Video_MemAlloc (
                                  void        *phApplication,
                                  uint32_t     nSize,
                                  void        *pArg,
                                  void       **ppData
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

#endif /* __ADI_DEV_VIDEO_COMMON_H__ */
