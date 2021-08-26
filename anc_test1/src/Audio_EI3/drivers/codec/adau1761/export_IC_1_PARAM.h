/*
 * File:           C:\Development\swe\bsp\audio\branches\1.1.0\package\Audio_EI3\Blackfin\Examples\ADSP-BF609\RecordPlayback\SigmaStudio\export\export_IC_1_PARAM.h
 *
 * Created:        Monday, June 02, 2014 11:48:09 AM
 * Description:    RecordPlayback:IC 1 parameter RAM definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2014 Analog Devices, Inc. All rights reserved.
 */
#ifndef __EXPORT_IC_1_PARAM_H__
#define __EXPORT_IC_1_PARAM_H__


/* Module Modulo Size - Modulo Size*/
#define MOD_MODULOSIZE_COUNT                           1
#define MOD_MODULOSIZE_DEVICE                          "IC1"
#define MOD_MODULOSIZE_MODULO_SIZE_ADDR                0
#define MOD_MODULOSIZE_MODULO_SIZE_FIXPT               0x00001000
#define MOD_MODULOSIZE_MODULO_SIZE_VALUE               SIGMASTUDIOTYPE_INTEGER_CONVERT(4096)
#define MOD_MODULOSIZE_MODULO_SIZE_TYPE                SIGMASTUDIOTYPE_INTEGER

/* Module SW vol 1 - Single SW slew vol (adjustable)*/
#define MOD_SWVOL1_COUNT                               2
#define MOD_SWVOL1_DEVICE                              "IC1"
#define MOD_SWVOL1_ALG0_TARGET_ADDR                    8
#define MOD_SWVOL1_ALG0_TARGET_FIXPT                   0x00800000
#define MOD_SWVOL1_ALG0_TARGET_VALUE                   SIGMASTUDIOTYPE_FIXPOINT_CONVERT(1)
#define MOD_SWVOL1_ALG0_TARGET_TYPE                    SIGMASTUDIOTYPE_FIXPOINT
#define MOD_SWVOL1_ALG0_STEP_ADDR                      9
#define MOD_SWVOL1_ALG0_STEP_FIXPT                     0x00000800
#define MOD_SWVOL1_ALG0_STEP_VALUE                     SIGMASTUDIOTYPE_FIXPOINT_CONVERT(0.000244140625)
#define MOD_SWVOL1_ALG0_STEP_TYPE                      SIGMASTUDIOTYPE_FIXPOINT

#endif
