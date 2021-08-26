/* $Id: adi_dev_audio_channel_id.h 62861 2019-11-27 14:36:45Z dgath $
******************************************************************************
Copyright (c), 2008-2019  - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensors.
******************************************************************************

Title: ADI Device Class Drivers - Audio CODEC Class

Description:
    This is the include file that defines Logical Audio Channel IDs
    recogonised by Audio codec class drivers

*****************************************************************************/

#ifndef __ADI_DEV_AUDIO_CHANNEL_ID_H__
#define __ADI_DEV_AUDIO_CHANNEL_ID_H__


/* disable misra diagnostics as necessary */
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Identifiers (internal and external) may rely on the significance of more than 31 characters.")
#endif /* _MISRA_RULES */


/*==============  D E F I N E S  ===============*/

/*
**
**  Enumeration start values for Logical Audio channel IDs
**      0x00000000 to 0x03FFFFFF
**          - reserved for Logical Audio Output channels
**      0x04000000 to 0x07FFFFFF
**          - reserved for Logical Audio Input channels
**      0x08000000 to 0x7FFFFFFF
**          - Reserved
*/

/*
**  Enumeration start for Logical Audio Output channels
*/
#define E_ADI_DEV_AUDIO_CHANNEL_OUT_START         0x00000000U

/*
**  Enumeration start for Logical Audio Input channels
*/
#define E_ADI_DEV_AUDIO_CHANNEL_IN_START          0x04000000U

/* IF (Build for C Langugage) */
#if defined(_LANGUAGE_C)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* C-specific definitions  */

/*
** Enumerations of Logical Audio Channel ID's
** Note: Audio codec class drivers might not support all logical channels
**       Refer to corresponding Audio codec class driver manual for the
**       list of Logical channel ids supported by the driver
*/
typedef enum __AdiDevAudioChannelId
{
    /*****************************************************************
    Logical Channel IDs for Audio Output (DAC) channels
    *****************************************************************/
    /* ID to configure all Output channels in an Audio Device (0x00000000) */
    ADI_DEV_AUDIO_CHANNEL_ALL_OUT           = E_ADI_DEV_AUDIO_CHANNEL_OUT_START,
    /* Master Left Output Channel (0x00000001) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_L_OUT      = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+1U),
    /* Master Right Output Channel (0x00000002) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_R_OUT      = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+2U),
    /* ID for both Left and Right Master Output Channels (0x00000003) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+3U),
    /* Front Left Output Channel (0x00000004) */
    ADI_DEV_AUDIO_CHANNEL_FRONT_L_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+4U),
    /* Front Right Output Channel (0x00000005) */
    ADI_DEV_AUDIO_CHANNEL_FRONT_R_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+5U),
    /* ID for both Left and Right Front Output Channels (0x00000006) */
    ADI_DEV_AUDIO_CHANNEL_FRONT_OUT         = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+6U),
    /* Center Output Channel (0x00000007) */
    ADI_DEV_AUDIO_CHANNEL_CENTER_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+7U),
    /* LFE Output Channel (0x00000008) */
    ADI_DEV_AUDIO_CHANNEL_LFE_OUT           = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+8U),
    /* Surround Left Output Channel (0x00000009) */
    ADI_DEV_AUDIO_CHANNEL_SURROUND_L_OUT    = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+9U),
    /* Surround Right Output Channel (0x0000000A) */
    ADI_DEV_AUDIO_CHANNEL_SURROUND_R_OUT    = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+10U),
    /* ID for both Left and Right Surround Output Channels (0x0000000B) */
    ADI_DEV_AUDIO_CHANNEL_SURROUND_OUT      = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+11U),
    /* Rear Left Output Channel (0x0000000C) */
    ADI_DEV_AUDIO_CHANNEL_REAR_L_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+12U),
    /* Rear Right Output Channel (0x0000000D) */
    ADI_DEV_AUDIO_CHANNEL_REAR_R_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+13U),
    /* ID for both Left and Right Rear Output Channels (0x0000000E) */
    ADI_DEV_AUDIO_CHANNEL_REAR_OUT          = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+14U),
    /* Headphone Left Output Channel (0x0000000F) */
    ADI_DEV_AUDIO_CHANNEL_HP_L_OUT          = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+15U),
    /* Headphone Right Output Channel (0x00000010) */
    ADI_DEV_AUDIO_CHANNEL_HP_R_OUT          = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+16U),
    /* ID for both Left and Right Headphone Output Channels (0x00000011) */
    ADI_DEV_AUDIO_CHANNEL_HP_OUT            = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+17U),
    /* Line-Out Left Channel (0x00000012) */
    ADI_DEV_AUDIO_CHANNEL_LINE_L_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+18U),
    /* Line-Out Right Channel (0x00000013) */
    ADI_DEV_AUDIO_CHANNEL_LINE_R_OUT        = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+19U),
    /* ID for both Left and Right Line-Out Channels (0x00000014) */
    ADI_DEV_AUDIO_CHANNEL_LINE_OUT          = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+20U),
    /* Mono Output Channel (0x00000015) */
    ADI_DEV_AUDIO_CHANNEL_MONO_OUT          = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+21U),
    /* SPDIF Output Channel (0x00000016) */
    ADI_DEV_AUDIO_CHANNEL_SPDIF_OUT         = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+22U),
    /* Auxiliary Output 0 Left Channel (0x00000017) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_0_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+23U),
    /* Auxiliary Output 0 Right Channel (0x000000018) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_0_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+24U),
    /* ID for both Left and Right Auxiliary 0 Output Channels (0x00000019) */
    ADI_DEV_AUDIO_CHANNEL_AUX_0_OUT         = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+25U),
    /* Auxiliary Output 1 Left Channel (0x0000001A) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_1_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+26U),
    /* Auxiliary Output 1 Right Channel (0x0000001B) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_1_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+27U),
    /* ID for both Left and Right Auxiliary 1 Output Channels (0x0000001C) */
    ADI_DEV_AUDIO_CHANNEL_AUX_1_OUT         = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+28U),
    /* Auxiliary Output 2 Left Channel (0x0000001D) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_2_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+29U),
    /* Auxiliary Output 2 Right Channel (0x0000001E) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_2_OUT       = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+30U),
    /* ID for both Left and Right Auxiliary 2 Output Channels (0x0000001F) */
    ADI_DEV_AUDIO_CHANNEL_AUX_2_OUT         = (E_ADI_DEV_AUDIO_CHANNEL_OUT_START+31U),

    /* Define new Logical Audio Output (DAC) channel IDs here */

    /*****************************************************************
    Logical Channel IDs for Audio Input (ADC) channels
    *****************************************************************/
    /* ID to configure all Input channels in an Audio device (0x04000000) */
    ADI_DEV_AUDIO_CHANNEL_ALL_IN            = E_ADI_DEV_AUDIO_CHANNEL_IN_START,
    /* Master Left Input Channel (0x04000001) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_L_IN       = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+1U),
    /* Master Right Input Channel (0x04000002) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_R_IN       = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+2U),
    /* ID for both Left and Right Master Input Channels (0x04000003) */
    ADI_DEV_AUDIO_CHANNEL_MASTER_IN         = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+3U),
    /* Line-In Left Channel (0x04000004) */
    ADI_DEV_AUDIO_CHANNEL_LINE_L_IN         = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+4U),
    /* Line-In Right Channel (0x04000005) */
    ADI_DEV_AUDIO_CHANNEL_LINE_R_IN         = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+5U),
    /* ID for both Left and Right Line-In Channels (0x04000006) */
    ADI_DEV_AUDIO_CHANNEL_LINE_IN           = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+6U),
    /* MIC 0 Left Channel (0x04000007) */
    ADI_DEV_AUDIO_CHANNEL_MIC_0_L_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+7U),
    /* MIC 0 Right Channel (0x04000008) */
    ADI_DEV_AUDIO_CHANNEL_MIC_0_R_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+8U),
    /* ID for both Left and Right Mic 0 Channels (0x04000009) */
    ADI_DEV_AUDIO_CHANNEL_MIC_0_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+9U),
    /* MIC 1 Left Channel (0x0400000A) */
    ADI_DEV_AUDIO_CHANNEL_MIC_1_L_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+10U),
    /* MIC 1 Right Channel (0x0400000B) */
    ADI_DEV_AUDIO_CHANNEL_MIC_1_R_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+11U),
    /* ID for both Left and Right Mic 1 Channels (0x0400000C) */
    ADI_DEV_AUDIO_CHANNEL_MIC_1_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+12U),
    /* Phone-In Left Channel (0x0400000D) */
    ADI_DEV_AUDIO_CHANNEL_PHONE_L_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+13U),
    /* Phone-In Right Channel (0x0400000E) */
    ADI_DEV_AUDIO_CHANNEL_PHONE_R_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+14U),
    /* ID for both Left and Right Phone Input Channels (0x0400000F) */
    ADI_DEV_AUDIO_CHANNEL_PHONE_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+15U),
    /* CD-In Left Channel (0x04000010) */
    ADI_DEV_AUDIO_CHANNEL_CD_L_IN           = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+16U),
    /* CD-In Right Channel (0x04000011) */
    ADI_DEV_AUDIO_CHANNEL_CD_R_IN           = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+17U),
    /* ID for both Left and Right CD Input Channels (0x04000012) */
    ADI_DEV_AUDIO_CHANNEL_CD_IN             = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+18U),
    /* Auxiliary Input 0 Left Channel (0x04000013) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_0_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+19U),
    /* Auxiliary Input 0 Right Channel (0x04000014) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_0_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+20U),
    /* ID for both Left and Right Auxiliary 0 Input Channels (0x04000015) */
    ADI_DEV_AUDIO_CHANNEL_AUX_0_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+21U),
    /* Auxiliary Input 1 Left Channel (0x04000016) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_1_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+22U),
    /* Auxiliary Input 1 Right Channel (0x04000017) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_1_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+23U),
    /* ID for both Left and Right Auxiliary 1 Input Channels (0x04000018) */
    ADI_DEV_AUDIO_CHANNEL_AUX_1_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+24U),
    /* Auxiliary Input 2 Left Channel (0x04000019) */
    ADI_DEV_AUDIO_CHANNEL_AUX_L_2_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+25U),
    /* Auxiliary Input 2 Right Channel (0x0400001A) */
    ADI_DEV_AUDIO_CHANNEL_AUX_R_2_IN        = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+26U),
    /* ID for both Left and Right Auxiliary 2 Input Channels (0x0400001B) */
    ADI_DEV_AUDIO_CHANNEL_AUX_2_IN          = (E_ADI_DEV_AUDIO_CHANNEL_IN_START+27U)

    /* Add new Logical Audio Input (ADC) channel IDs here */

} ADI_DEV_AUDIO_CHANNEL_ID;

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* _LANGUAGE_C */


#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */


#endif  /* __ADI_DEV_AUDIO_CHANNEL_ID_H__ */

/*****/

