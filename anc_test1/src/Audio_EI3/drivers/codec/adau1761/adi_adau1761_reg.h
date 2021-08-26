/*********************************************************************************

Copyright(c) 2011 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/

/**
 * @file:    adi_adau1761_reg.h
 * @brief:   ADAU1761 low power stereo audio codec register definitions
 * @version: $Revision: 7177 $
 * @date:    $Date: 2012-02-15 14:53:41 -0500 (Wed, 15 Feb 2012) $
 *
 * @details
 *            This is the register definition header file for the Low power
 *            stereo audio codec.
 */

#ifndef ADI_ADAU1761__REG_H_
#define ADI_ADAU1761__REG_H_

/* disable misra diagnostics as necessary */
#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_2_4:"(advisory) Sections of code should not be commented out.")
#endif /* _MISRA_RULES */

/** @addtogroup ADAU1761_Driver ADAU1761 Low power stereo audio codec driver
 *  @{
 */

#define REG_PLL_CRL_ADDR        (0x4002u)

#define REG_MIC_CTRL_ADDR	    (0x4008u)

#define BITM_MIC_JDFUNC         (0x30u)
#define BITP_MIC_JDFUNC         (0x4u)
#define ENUM_MIC_ENABLE         (0x2u)
#define ENUM_MIC_DISABLE        (0x0u)

#define REG_MIC_BIAS_ADDR	    (0x4010u)

#define BITM_MIC_MBIEN          (0x1u)
#define BITP_MIC_MBIEN          (0x0u)
#define ENUM_MIC_BIAS_ENABLE    (0x1u)
#define ENUM_MIC_BIAS_DISABLE   (0x0u)

#define REG_ADC_CTRL_ADDR	    (0x4019u)

#define BITM_ADC_INSEL          (0x4u)
#define BITP_ADC_INSEL          (0x2u)
#define ENUM_ADC_MIC_IN_ENABLE  (0x1u)
#define ENUM_ADC_MIC_IN_DISABLE (0x0u)

#define REG_DIFF_IN_LEFT_ADDR   (0x400Eu)
#define REG_DIFF_IN_RIGHT_ADDR  (0x400Fu)

#define REG_ADC_MIC_LEFT_ADDR    (0x401Au)
#define REG_ADC_MIC_RIGHT_ADDR   (0x401Bu)

#define REG_HEADPHONE_LEFT_ADDR  (0x4023u)
#define REG_HEADPHONE_RIGHT_ADDR (0x4024u)

#define REG_LINE_OUT_LEFT_ADDR   (0x4025u)
#define REG_LINE_OUT_RIGHT_ADDR  (0x4026u)

#define REG_DAC_LEFT_ADDR        (0x402Bu)
#define REG_DAC_RIGHT_ADDR       (0x402Cu)

#define REG_CONV_CTRL_0_ADDR      (0x4017u)
#define REG_SERIAL_PORT_SAMP_ADDR (0x40F8u)
#define REG_DSP_SAMP_RATE_ADDR    (0x40EBu)

/**@}*/

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#endif /* ADI_ADAU1761_REG_H_ */

