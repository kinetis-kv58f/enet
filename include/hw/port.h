// © 2011. Ravi Teja Gudapati. All rights reserved.

#ifndef INCLUDE_HW_PORT_H_
#define INCLUDE_HW_PORT_H_

#include "hw/common.h"
#include "types.h"

/// PORT - Register Layout Typedef
typedef struct {
	/// Pin Control Register n, array offset: 0x0, array step: 0x4
	__IO uint32_t PCR[32];

	/// Global Pin Control Low Register, offset: 0x80
	__O uint32_t GPCLR;

	/// Global Pin Control High Register, offset: 0x84
	__O uint32_t GPCHR;

	uint8_t RESERVED_0[24];

	/// Interrupt Status Flag Register, offset: 0xA0
	__IO uint32_t ISFR;

	uint8_t RESERVED_1[28];

	/// Digital Filter Enable Register, offset: 0xC0
	__IO uint32_t DFER;

	/// Digital Filter Clock Register, offset: 0xC4
	__IO uint32_t DFCR;

	/// Digital Filter Width Register, offset: 0xC8
	__IO uint32_t DFWR;
} hw_port_t;

enum {
	HwPort_PinDisabledOrAnalog = 0U, /*!< Corresponding pin is disabled, but is used as an analog pin. */
	HwPort_MuxAsGpio = 1U, /*!< Corresponding pin is configured as GPIO. */
	HwPort_MuxAlt2 = 2U, /*!< Chip-specific */
	HwPort_MuxAlt3 = 3U, /*!< Chip-specific */
	HwPort_MuxAlt4 = 4U, /*!< Chip-specific */
	HwPort_MuxAlt5 = 5U, /*!< Chip-specific */
	HwPort_MuxAlt6 = 6U, /*!< Chip-specific */
	HwPort_MuxAlt7 = 7U, /*!< Chip-specific */
	HwPort_MuxAlt8 = 8U, /*!< Chip-specific */
	HwPort_MuxAlt9 = 9U, /*!< Chip-specific */
	HwPort_MuxAlt10 = 10U, /*!< Chip-specific */
	HwPort_MuxAlt11 = 11U, /*!< Chip-specific */
	HwPort_MuxAlt12 = 12U, /*!< Chip-specific */
	HwPort_MuxAlt13 = 13U, /*!< Chip-specific */
	HwPort_MuxAlt14 = 14U, /*!< Chip-specific */
	HwPort_MuxAlt15 = 15U, /*!< Chip-specific */
};

enum {
	HwPort_PCR_ISF_Shift = 24,
	HwPort_PCR_ISF_Mask = 1 << HwPort_PCR_ISF_Shift,

	HwPort_PCR_IRQC_Shift = 16,
	HwPort_PCR_IRQC_Mask = 0xF << HwPort_PCR_IRQC_Shift,

	HwPort_PCR_LK_Shift = 15,
	HwPort_PCR_LK_Mask = 1 << HwPort_PCR_LK_Shift,

	HwPort_PCR_MUX_Shift = 8,
	HwPort_PCR_MUX_Mask = 0xF << HwPort_PCR_MUX_Shift,

	HwPort_PCR_DSE_Shift = 6,
	HwPort_PCR_DSE_Mask = 1 << HwPort_PCR_DSE_Shift,

	HwPort_PCR_ODE_Shift = 5,
	HwPort_PCR_ODE_Mask = 1 << HwPort_PCR_ODE_Shift,

	HwPort_PCR_PFE_Shift = 4,
	HwPort_PCR_PFE_Mask = 1 << HwPort_PCR_PFE_Shift,

	HwPort_PCR_SRE_Shift = 2,
	HwPort_PCR_SRE_Mask = 1 << HwPort_PCR_SRE_Shift,

	HwPort_PCR_PE_Shift = 1,
	HwPort_PCR_PE_Mask = 1 << HwPort_PCR_PE_Shift,

	HwPort_PCR_PS_Shift = 0,
	HwPort_PCR_PS_Mask = 1 << HwPort_PCR_PS_Shift,
};

volatile hw_port_t * const volatile hw_portA = (hw_port_t *) Memmap_PORTA_Addr;

volatile hw_port_t * const volatile hw_portB = (hw_port_t *) Memmap_PORTB_Addr;

volatile hw_port_t * const volatile hw_portC = (hw_port_t *) Memmap_PORTC_Addr;

volatile hw_port_t * const volatile hw_portD = (hw_port_t *) Memmap_PORTD_Addr;

volatile hw_port_t * const volatile hw_portE = (hw_port_t *) Memmap_PORTE_Addr;

#endif /* INCLUDE_HW_PORT_H_ */
