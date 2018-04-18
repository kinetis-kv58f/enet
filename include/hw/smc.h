/*
 * smc.h
 *
 *  Created on: 17 jan. 2018
 *      Author: SERAGUD
 */

#ifndef INCLUDE_HW_SMC_H_
#define INCLUDE_HW_SMC_H_

#include "hw/common.h"
#include "types.h"

/** SMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t PMPROT;                             /**< Power Mode Protection register, offset: 0x0 */
  __IO uint8_t PMCTRL;                             /**< Power Mode Control register, offset: 0x1 */
  __IO uint8_t STOPCTRL;                           /**< Stop Control Register, offset: 0x2 */
  __I  uint8_t PMSTAT;                             /**< Power Mode Status register, offset: 0x3 */
} HwSMC;

enum {
	SMC_PMPPROT_AHSRUN_Shift = 7,
	SMC_PMPPROT_AHSRUN_Mask = 1 << SMC_PMPPROT_AHSRUN_Shift,

	SMC_PMCTRL_RUNM_Shift = 5,
	SMC_PMCTRL_RUNM_Mask = 0x3 << SMC_PMCTRL_RUNM_Shift,
	SMC_PMCTRL_RUNM_Normal = 0x0 << SMC_PMCTRL_RUNM_Shift,
	SMC_PMCTRL_RUNM_Vlpr = 0x2 << SMC_PMCTRL_RUNM_Shift,
	SMC_PMCTRL_RUNM_Hsrun = 0x3 << SMC_PMCTRL_RUNM_Shift,

	SMC_PMSTAT_Run = 0x01,
	SMC_PMSTAT_Stop = 0x02,
	SMC_PMSTAT_Vlpr = 0x04,
	SMC_PMSTAT_Vlpw = 0x08,
	SMC_PMSTAT_Vlps = 0x10,
	SMC_PMSTAT_Vlls = 0x40,
	SMC_PMSTAT_Hsrun = 0x80,
} hwsmc_fields_t;

volatile HwSMC * const volatile hw_smc = (HwSMC *) Memmap_SMC_Addr;

#endif /* INCLUDE_HW_SMC_H_ */
