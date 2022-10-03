/*******************************************************************************

Copyright(c) 2014-2021 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*******************************************************************************/
/*!
* @file      page_table_mem.c
*
* @brief     Page table used by Memory Management Unit (MMU)
*
* @details
*            This contains the raw memory array that is used to hold the actual
*            first- and second-level page tables used by the hardware.
*/

#include <stdint.h> /* for uint32_h etc. */

#include <runtime/mmu/adi_mmu.h>

#if defined(__ADSPSC573_FAMILY__) || defined(__ADSPSC594_FAMILY__)
  #if __NUM_SHARC_CORES__ > 1
    #define TABLE_SIZE_IN_KBYTES 24
  #else
    #define TABLE_SIZE_IN_KBYTES 22
  #endif
#else
  #if __NUM_SHARC_CORES__ > 1
    #define TABLE_SIZE_IN_KBYTES 30
  #else
    #define TABLE_SIZE_IN_KBYTES 26
  #endif
#endif

#define KBYTES(x) ((x)*1024)

uint8_t _adi_mmu_tableMemory[KBYTES(TABLE_SIZE_IN_KBYTES)] __attribute__ ((aligned (KBYTES(16))));
const size_t _adi_mmu_tableMemSize = sizeof(_adi_mmu_tableMemory);
