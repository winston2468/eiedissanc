/*
 **
 ** sru_config.c source file generated on September 19, 2022 at 12:11:22.
 **
 ** Copyright (C) 2014-2022 Analog Devices Inc., All Rights Reserved.
 **
 ** This file is generated automatically based upon the options selected in
 ** the SRU Configuration editor. Changes to the SRU configuration should
 ** be made by changing the appropriate options rather than editing this file.
 **
 ** By default only the SRU registers with values different from their default
 ** reset value are initialized. Alternatively, macro SRU_FULL_INIT may be
 ** defined in the project settings to cause all SRU registers to be
 ** initialized.
 **
 */

#include <stdint.h>
#include <sys/platform.h>

int32_t adi_SRU_Init(void);

/*
 * Initialize the Signal Routing Unit
 */
int32_t adi_SRU_Init(void)
{
    /* Connections:
     *    DAI0_PB16_O - SPT2B_CLK_I (non-default)
     *    DAI0_PB06_O - SPT0B_CLK_I (non-default)
     *    DAI0_PB13_O - SPT1B_CLK_I
     *    DAI0_PB13_O - SPT1A_CLK_I
     *    DAI0_PB03_O - SPT0A_CLK_I
     *    DAI0_PB11_O - SPT2A_CLK_I (non-default)
     */
    *pREG_DAI0_CLK0 = 0x1ea630a2uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI0_LOW - SRC0_CLK_OP_I
     *    DAI0_LOW - SRC1_CLK_IP_I
     *    DAI0_LOW - SRC2_CLK_IP_I
     *    DAI0_LOW - SRC2_CLK_OP_I
     *    DAI0_LOW - SRC0_CLK_IP_I
     *    DAI0_LOW - SRC1_CLK_OP_I
     */
    *pREG_DAI0_CLK1 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPDIF0_TX_CLK_I
     *    DAI0_LOW - SRC3_CLK_IP_I
     *    DAI0_LOW - SRC3_CLK_OP_I
     */
    *pREG_DAI0_CLK2 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPDIF0_TX_HFCLK_I
     */
    *pREG_DAI0_CLK3 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPDIF0_TX_EXT_SYNC_I
     *    DAI0_LOW - PCG0_EXTCLKB_I
     *    DAI0_LOW - PCG0_SYNC_CLKA_I
     *    DAI0_LOW - PCG0_EXTCLKA_I
     *    DAI0_LOW - PCG0_SYNC_CLKB_I
     */
    *pREG_DAI0_CLK4 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPT3B_CLK_I
     *    DAI0_LOW - SPT3A_CLK_I
     */
    *pREG_DAI0_CLK5 = 0x3def7bdeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI0_PB09_O - SPT1A_D0_I
     *    DAI0_PB09_O - SPT0B_D0_I (non-default)
     *    DAI0_LOW - SPT0B_D1_I (non-default)
     *    DAI0_PB01_O - SPT0A_D0_I
     *    DAI0_PB02_O - SPT0A_D1_I
     */
    *pREG_DAI0_DAT0 = 0x08f88040uL;

    /* Connections:
     *    DAI0_LOW - SPT2A_D1_I (non-default)
     *    DAI0_PB10_O - SPT1A_D1_I
     *    DAI0_LOW - SPT1B_D0_I (non-default)
     *    DAI0_LOW - SPT2A_D0_I (non-default)
     *    DAI0_LOW - SPT1B_D1_I (non-default)
     */
    *pREG_DAI0_DAT1 = 0x3efbef89uL;

    /* Connections:
     *    DAI0_PB01_O - SRC1_DAT_IP_I
     *    DAI0_PB19_O - SPT2B_D0_I (non-default)
     *    DAI0_LOW - SPT2B_D1_I (non-default)
     *    DAI0_PB01_O - SRC2_DAT_IP_I
     *    DAI0_PB01_O - SRC0_DAT_IP_I
     */
    *pREG_DAI0_DAT2 = 0x00000f92uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI0_PB01_O - SRC3_DAT_IP_I
     *    DAI0_PB01_O - SRC0_TDM_OP_I
     *    DAI0_PB01_O - SRC2_TDM_OP_I
     *    DAI0_PB01_O - SRC3_TDM_OP_I
     *    DAI0_PB01_O - SRC1_TDM_OP_I
     */
    *pREG_DAI0_DAT3 = 0x00000000uL;

    /* Connections:
     *    DAI0_PB01_O - SPDIF0_TX_DAT_I
     */
    *pREG_DAI0_DAT4 = 0x00000000uL;

    /* Connections:
     *    DAI0_PB01_O - SPDIF0_RX_I
     */
    *pREG_DAI0_DAT5 = 0x00000000uL;

    /* Connections:
     *    DAI0_LOW - SPT3B_D0_I
     *    DAI0_LOW - SPT3A_D0_I
     *    DAI0_LOW - SPT3A_D1_I
     *    DAI0_LOW - SPT3B_D1_I
     */
    *pREG_DAI0_DAT6 = 0x00fbefbeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI0_LOW - SPT1A_FS_I (non-default)
     *    DAI0_PB12_O - SPT2A_FS_I (non-default)
     *    DAI0_PB07_O - SPT0B_FS_I (non-default)
     *    DAI0_PB17_O - SPT2B_FS_I (non-default)
     *    DAI0_PB04_O - SPT0A_FS_I
     *    DAI0_LOW - SPT1B_FS_I (non-default)
     */
    *pREG_DAI0_FS0 = 0x20bf78c3uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI0_LOW - SRC2_FS_IP_I
     *    DAI0_LOW - SRC0_FS_OP_I
     *    DAI0_LOW - SRC0_FS_IP_I
     *    DAI0_LOW - SRC1_FS_OP_I
     *    DAI0_LOW - SRC2_FS_OP_I
     *    DAI0_LOW - SRC1_FS_IP_I
     */
    *pREG_DAI0_FS1 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPDIF0_TX_FS_I
     *    DAI0_LOW - SRC3_FS_IP_I
     *    DAI0_LOW - SRC3_FS_OP_I
     */
    *pREG_DAI0_FS2 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - SPT3A_FS_I
     *    DAI0_LOW - SPT3B_FS_I
     */
    *pREG_DAI0_FS4 = 0x000003deuL;

    /* Connections:
     *    DAI0_LOW - DAI0_INT_8_I
     *    DAI0_LOW - DAI0_INT_7_I
     *    DAI0_LOW - DAI0_INT_6_I
     *    DAI0_LOW - DAI0_MISCA4_I
     *    DAI0_LOW - DAI0_MISCA5_I
     *    DAI0_LOW - DAI0_INV_MISCA4_I
     *    DAI0_LOW - DAI0_INT_9_I
     *    DAI0_LOW - DAI0_INV_MISCA5_I
     *    DAI0_LOW - DAI0_MISCA0_I
     *    DAI0_LOW - DAI0_MISCA1_I
     *    DAI0_LOW - DAI0_MISCA2_I
     *    DAI0_LOW - DAI0_MISCA3_I
     */
    *pREG_DAI0_MISC0 = 0x3def7bdeuL;

    /* Connections:
     *    DAI0_LOW - DAI0_INT_4_I
     *    DAI0_LOW - DAI0_INT_3_I
     *    DAI0_LOW - DAI0_INT_2_I
     *    DAI0_LOW - DAI0_INT_1_I
     *    DAI0_LOW - DAI0_INT_5_I
     *    DAI0_LOW - DAI0_INT_0_I
     */
    *pREG_DAI0_MISC1 = 0x3def7bdeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    SPT0A_CLK_PBEN_O - DAI0_PBEN03_I
     *    SPT0A_D1_PBEN_O - DAI0_PBEN02_I
     *    SPT0A_D0_PBEN_O - DAI0_PBEN01_I
     *    DAI0_LOW - DAI0_PBEN05_I (non-default)
     *    SPT0A_FS_PBEN_O - DAI0_PBEN04_I
     */
    *pREG_DAI0_PBEN0 = 0x002482cauL;

    /* Connections:
     *    DAI0_LOW - DAI0_PBEN07_I (non-default)
     *    DAI0_LOW - DAI0_PBEN06_I (non-default)
     *    SPT1A_D0_PBEN_O - DAI0_PBEN09_I
     *    DAI0_LOW - DAI0_PBEN08_I (non-default)
     *    SPT1A_D1_PBEN_O - DAI0_PBEN10_I
     */
    *pREG_DAI0_PBEN1 = 0x13480000uL;

    /* Connections:
     *    DAI0_LOW - DAI0_PBEN15_I (non-default)
     *    DAI0_LOW - DAI0_PBEN12_I (non-default)
     *    DAI0_LOW - DAI0_PBEN11_I (non-default)
     *    SPT1B_CLK_PBEN_O - DAI0_PBEN13_I
     *    DAI0_HIGH - DAI0_PBEN14_I (non-default)
     */
    *pREG_DAI0_PBEN2 = 0x00054000uL;

    /* Connections:
     *    DAI0_LOW - DAI0_PBEN16_I (non-default)
     *    DAI0_LOW - DAI0_PBEN17_I (non-default)
     *    DAI0_LOW - DAI0_PBEN18_I (non-default)
     *    DAI0_LOW - DAI0_PBEN20_I (non-default)
     *    DAI0_LOW - DAI0_PBEN19_I (non-default)
     */
    *pREG_DAI0_PBEN3 = 0x00000000uL;

    /* Connections:
     *    DAI0_LOW - DAI0_PB02_I (non-default)
     *    DAI0_LOW - DAI0_PB01_I (non-default)
     *    DAI0_LOW - DAI0_PB04_I (non-default)
     *    SPT0A_CLK_O - DAI0_PB03_I
     */
    *pREG_DAI0_PIN0 = 0x0fc83f7euL;

    /* Connections:
     *    DAI0_LOW - DAI0_PB06_I (non-default)
     *    DAI0_LOW - DAI0_PB05_I (non-default)
     *    DAI0_LOW - DAI0_PB08_I (non-default)
     *    DAI0_LOW - DAI0_PB07_I (non-default)
     */
    *pREG_DAI0_PIN1 = 0x0fdfbf7euL;

    /* Connections:
     *    DAI0_LOW - DAI0_PB12_I (non-default)
     *    SPT1A_D1_O - DAI0_PB10_I
     *    DAI0_LOW - DAI0_PB11_I (non-default)
     *    SPT1A_D0_O - DAI0_PB09_I
     */
    *pREG_DAI0_PIN2 = 0x0fdf8c98uL;

    /* Connections:
     *    DAI0_LOW - DAI0_PB16_I (non-default)
     *    SPT2A_D0_O - DAI0_PB14_I (non-default)
     *    DAI0_LOW - DAI0_PB15_I (non-default)
     *    SPT1B_CLK_O - DAI0_PB13_I
     */
    *pREG_DAI0_PIN3 = 0x0fdf8e23uL;

    /* Connections:
     *    DAI0_LOW - DAI0_PB18_I (non-default)
     *    DAI0_LOW - DAI0_PB19_I (non-default)
     *    DAI0_PB01_O - INV_DAI0_PB19_I
     *    DAI0_LOW - DAI0_PB17_I (non-default)
     *    DAI0_PB01_O - INV_DAI0_PB20_I
     *    DAI0_LOW - DAI0_PB20_I (non-default)
     */
    *pREG_DAI0_PIN4 = 0x0fdfbf7euL;

    /* Connections:
     *    DAI1_PB12_O - SPT4A_CLK_I (non-default)
     *    DAI1_PB13_O - SPT5A_CLK_I
     *    DAI1_PB02_O - SPT4B_CLK_I (non-default)
     *    DAI1_PB13_O - SPT5B_CLK_I
     *    DAI1_PB19_O - SPT6B_CLK_I
     *    DAI1_PB19_O - SPT6A_CLK_I
     */
    *pREG_DAI1_CLK0 = 0x2526302buL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI1_LOW - SRC4_CLK_IP_I
     *    DAI1_LOW - SRC6_CLK_OP_I
     *    DAI1_LOW - SRC4_CLK_OP_I
     *    DAI1_LOW - SRC5_CLK_IP_I
     *    DAI1_LOW - SRC6_CLK_IP_I
     *    DAI1_LOW - SRC5_CLK_OP_I
     */
    *pREG_DAI1_CLK1 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - SRC7_CLK_IP_I
     *    DAI1_LOW - SRC7_CLK_OP_I
     *    DAI1_LOW - SPDIF1_TX_CLK_I
     */
    *pREG_DAI1_CLK2 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - SPDIF1_TX_HFCLK_I
     */
    *pREG_DAI1_CLK3 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - PCG0_EXTCLKC_I
     *    DAI1_LOW - PCG0_EXTCLKD_I
     *    DAI1_LOW - SPDIF1_TX_EXT_SYNC_I
     *    DAI1_LOW - PCG0_SYNC_CLKC_I
     *    DAI1_LOW - PCG0_SYNC_CLKD_I
     */
    *pREG_DAI1_CLK4 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - SPT7B_CLK_I
     *    DAI1_LOW - SPT7A_CLK_I
     */
    *pREG_DAI1_CLK5 = 0x3def7bdeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI1_LOW - SPT4B_D1_I (non-default)
     *    DAI1_LOW - SPT4B_D0_I (non-default)
     *    DAI1_PB09_O - SPT5A_D0_I
     *    DAI1_PB07_O - SPT4A_D1_I (non-default)
     *    DAI1_PB06_O - SPT4A_D0_I (non-default)
     */
    *pREG_DAI1_DAT0 = 0x08fbe185uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI1_PB10_O - SPT5A_D1_I
     *    DAI1_PB11_O - SPT5B_D0_I
     *    DAI1_PB15_O - SPT6A_D0_I
     *    DAI1_PB12_O - SPT5B_D1_I
     *    DAI1_PB16_O - SPT6A_D1_I
     */
    *pREG_DAI1_DAT1 = 0x0f38b289uL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI1_LOW - SRC5_DAT_IP_I (non-default)
     *    DAI1_PB18_O - SPT6B_D1_I
     *    DAI1_PB17_O - SPT6B_D0_I
     *    DAI1_LOW - SRC4_DAT_IP_I (non-default)
     *    DAI1_LOW - SRC6_DAT_IP_I (non-default)
     */
    *pREG_DAI1_DAT2 = 0x3efbe450uL;

    /* Connections:
     *    DAI1_LOW - SRC4_TDM_OP_I (non-default)
     *    DAI1_LOW - SRC7_TDM_OP_I (non-default)
     *    DAI1_LOW - SRC6_TDM_OP_I (non-default)
     *    DAI1_LOW - SRC7_DAT_IP_I (non-default)
     *    DAI1_LOW - SRC5_TDM_OP_I (non-default)
     */
    *pREG_DAI1_DAT3 = 0x3efbefbeuL;

    /* Connections:
     *    DAI1_LOW - SPDIF1_TX_DAT_I (non-default)
     */
    *pREG_DAI1_DAT4 = 0x0000003euL;

    /* Connections:
     *    DAI1_LOW - SPDIF1_RX_I (non-default)
     */
    *pREG_DAI1_DAT5 = 0x3e000000uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI1_LOW - SPT7B_D1_I
     *    DAI1_LOW - SPT7A_D1_I
     *    DAI1_LOW - SPT7A_D0_I
     *    DAI1_LOW - SPT7B_D0_I
     */
    *pREG_DAI1_DAT6 = 0x00fbefbeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI1_LOW - SPT6B_FS_I (non-default)
     *    DAI1_PB04_O - SPT4B_FS_I (non-default)
     *    DAI1_PB14_O - SPT5A_FS_I
     *    DAI1_LOW - SPT6A_FS_I (non-default)
     *    DAI1_PB14_O - SPT5B_FS_I
     *    DAI1_PB20_O - SPT4A_FS_I (non-default)
     */
    *pREG_DAI1_FS0 = 0x3de6b473uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    DAI1_LOW - SRC4_FS_OP_I
     *    DAI1_LOW - SRC6_FS_IP_I
     *    DAI1_LOW - SRC5_FS_IP_I
     *    DAI1_LOW - SRC6_FS_OP_I
     *    DAI1_LOW - SRC4_FS_IP_I
     *    DAI1_LOW - SRC5_FS_OP_I
     */
    *pREG_DAI1_FS1 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - SPDIF1_TX_FS_I
     *    DAI1_LOW - SRC7_FS_OP_I
     *    DAI1_LOW - SRC7_FS_IP_I
     */
    *pREG_DAI1_FS2 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - SPT7B_FS_I
     *    DAI1_LOW - SPT7A_FS_I
     */
    *pREG_DAI1_FS4 = 0x000003deuL;

    /* Connections:
     *    DAI1_LOW - DAI1_INT_8_I
     *    DAI1_LOW - DAI1_INT_9_I
     *    DAI1_LOW - DAI1_MISCA5_I
     *    DAI1_LOW - DAI1_MISCA2_I
     *    DAI1_LOW - DAI1_INV_MISCA5_I
     *    DAI1_LOW - DAI1_MISCA1_I
     *    DAI1_LOW - DAI1_MISCA4_I
     *    DAI1_LOW - DAI1_MISCA3_I
     *    DAI1_LOW - DAI1_INV_MISCA4_I
     *    DAI1_LOW - DAI1_MISCA0_I
     *    DAI1_LOW - DAI1_INT_6_I
     *    DAI1_LOW - DAI1_INT_7_I
     */
    *pREG_DAI1_MISC0 = 0x3def7bdeuL;

    /* Connections:
     *    DAI1_LOW - DAI1_INT_0_I
     *    DAI1_LOW - DAI1_INT_1_I
     *    DAI1_LOW - DAI1_INT_2_I
     *    DAI1_LOW - DAI1_INT_3_I
     *    DAI1_LOW - DAI1_INT_4_I
     *    DAI1_LOW - DAI1_INT_5_I
     */
    *pREG_DAI1_MISC1 = 0x3def7bdeuL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI1_HIGH - DAI1_PBEN05_I (non-default)
     *    DAI1_HIGH - DAI1_PBEN01_I (non-default)
     *    DAI1_LOW - DAI1_PBEN02_I (non-default)
     *    DAI1_LOW - DAI1_PBEN03_I (non-default)
     *    DAI1_LOW - DAI1_PBEN04_I (non-default)
     */
    *pREG_DAI1_PBEN0 = 0x01000001uL;

    /* Connections:
     *    SPT5A_D1_PBEN_O - DAI1_PBEN10_I
     *    SPT5A_D0_PBEN_O - DAI1_PBEN09_I
     *    DAI1_LOW - DAI1_PBEN06_I (non-default)
     *    SPT4B_FS_PBEN_O - DAI1_PBEN08_I
     *    DAI1_LOW - DAI1_PBEN07_I (non-default)
     */
    *pREG_DAI1_PBEN1 = 0x1348d000uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    SPT5B_D0_PBEN_O - DAI1_PBEN11_I
     *    SPT5B_CLK_PBEN_O - DAI1_PBEN13_I
     *    SPT5B_D1_PBEN_O - DAI1_PBEN12_I
     *    SPT6A_D0_PBEN_O - DAI1_PBEN15_I
     *    SPT5B_FS_PBEN_O - DAI1_PBEN14_I
     */
    *pREG_DAI1_PBEN2 = 0x1a5545d6uL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    DAI1_LOW - DAI1_PBEN20_I (non-default)
     *    SPT6B_D0_PBEN_O - DAI1_PBEN17_I
     *    SPT6A_D1_PBEN_O - DAI1_PBEN16_I
     *    SPT6B_CLK_PBEN_O - DAI1_PBEN19_I
     *    SPT6B_D1_PBEN_O - DAI1_PBEN18_I
     */
    *pREG_DAI1_PBEN3 = 0x0071f79buL;

    /* Connections:
     *    DAI1_LOW - DAI1_PB04_I (non-default)
     *    DAI1_LOW - DAI1_PB02_I (non-default)
     *    DAI1_LOW - DAI1_PB03_I (non-default)
     *    SPT4B_D0_O - DAI1_PB01_I (non-default)
     */
    *pREG_DAI1_PIN0 = 0x0fdfbf16uL;

    /* Connections:
     *    DAI1_LOW - DAI1_PB08_I (non-default)
     *    DAI1_LOW - DAI1_PB06_I (non-default)
     *    DAI1_LOW - DAI1_PB07_I (non-default)
     *    SPT4B_D1_O - DAI1_PB05_I (non-default)
     */
    *pREG_DAI1_PIN1 = 0x0fdfbf17uL;

    /* Connections:
     *    SPT5A_D0_O - DAI1_PB09_I
     *    DAI1_LOW - DAI1_PB12_I (non-default)
     *    SPT5B_D0_O - DAI1_PB11_I
     *    SPT5A_D1_O - DAI1_PB10_I
     */
    *pREG_DAI1_PIN2 = 0x0fc68c98uL;

#if defined(SRU_FULL_INIT)
    /* Connections:
     *    SPT6A_D1_O - DAI1_PB16_I
     *    SPT6A_D0_O - DAI1_PB15_I
     *    SPT5B_FS_O - DAI1_PB14_I
     *    SPT5B_CLK_O - DAI1_PB13_I
     */
    *pREG_DAI1_PIN3 = 0x03a714a3uL;
#endif /* defined(SRU_FULL_INIT) */

    /* Connections:
     *    SPT6B_CLK_O - DAI1_PB19_I
     *    SPT6B_D1_O - DAI1_PB18_I
     *    DAI1_PB01_O - INV_DAI1_PB20_I
     *    SPT6B_D0_O - DAI1_PB17_I
     *    DAI1_PB01_O - INV_DAI1_PB19_I
     *    DAI1_LOW - DAI1_PB20_I (non-default)
     */
    *pREG_DAI1_PIN4 = 0x0fc94f9euL;
    /* PADS0 DAI0 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = BITM_PADS_DAI0_IE_VALUE;

    /* PADS0 DAI1 Port Input Enable Control Register */
    *pREG_PADS0_DAI1_IE = BITM_PADS_DAI1_IE_VALUE;

    return 0;
}

