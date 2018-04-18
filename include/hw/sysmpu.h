/*
 * sysmpu.h
 *
 *  Created on: 16 apr. 2018
 *      Author: SERAGUD
 */

#ifndef INCLUDE_HW_SYSMPU_H_
#define INCLUDE_HW_SYSMPU_H_

#include "common.h"

/// SYSMPU - Register Layout Typedef
typedef struct {
	/// Control/Error Status Register, offset: 0x0
	__IO uint32_t CESR;
	uint8_t RESERVED_0[12];
	struct {
		/// Error Address Register, slave port n, array offset: 0x10, array step: 0x8
		__I uint32_t EAR;
		/// Error Detail Register, slave port n, array offset: 0x14, array step: 0x8
		__I uint32_t EDR;
	} SP[5];
	uint8_t RESERVED_1[968];
	/// Region Descriptor n, Word 0..Region Descriptor n, Word 3, array offset: 0x400, array step: index*0x10, index2*0x4
	__IO uint32_t WORD[12][4];
	uint8_t RESERVED_2[832];
	/// Region Descriptor Alternate Access Control n, array offset: 0x800, array step: 0x4
	__IO uint32_t RGDAAC[12];
} __attribute__((__packed__)) HwSysMpu;

enum {
	HwSysMpu_CESR_VLD_Shift = 0,
	HwSysMpu_CESR_VLD_Mask = 1 << HwSysMpu_CESR_VLD_Shift,
};

volatile HwSysMpu * const volatile hwSysMpu = (HwSysMpu *) Memmap_SYSMPU_Addr;

#endif /* INCLUDE_HW_SYSMPU_H_ */
