/*
 * enet.c
 *
 *  Created on: 11 apr. 2018
 *      Author: SERAGUD
 */

#include "driv/enet/enet.h"
#include "hw/enet.h"
#include "hw/sim.h"
#include "driv/cpu/cpu.h"
#include "mii.h"
#include "hw/port.h"

/// Ethernet speed
typedef enum {
	/// Speed 10 Mbps.
	EnetSpeed10M = 0U,

	/// Speed 100 Mbps.
	EnetSpeed100M = 1U,
} EnetSpeed;

static Enet enet;

static uint32_t readSMI(uint8_t const phyAddr, uint8_t const phyReg) {
	// Clear interrupt
	hwEnet0->EIR = HwEnet_EIR_MII_Mask;

	hwEnet0->MMFR = (1 << HwEnet_MMFR_ST_Shift)
			| (MiiOpReadInc << HwEnet_MMFR_OP_Shift)
			| (phyAddr << HwEnet_MMFR_PA_Shift)
			| (phyReg << HwEnet_MMFR_RA_Shift) | (2 << HwEnet_MMFR_TA_Shift);

	{
		uint32_t timeout = 0;
		for (; timeout < 0xFFFFFU; timeout++) {
			if (hwEnet0->EIR & HwEnet_EIR_MII_Mask) {
				break;
			}
		}
		if (timeout == 0xFFFFFU) {
			return 0xFFFFFFFF;
		}
	}

	uint16_t data = (hwEnet0->MMFR & HwEnet_MMFR_DATA_Mask)
			>> HwEnet_MMFR_DATA_Shift;

	// Clear interrupt
	hwEnet0->EIR = HwEnet_EIR_MII_Mask;

	return data;
}

static bool_t writeSMI(uint8_t const phyAddr, uint8_t const phyReg,
		uint16_t data) {
	// Clear interrupt
	hwEnet0->EIR = HwEnet_EIR_MII_Mask;

	hwEnet0->MMFR = (1 << HwEnet_MMFR_ST_Shift)
			| (MiiOpWrite << HwEnet_MMFR_OP_Shift)
			| (phyAddr << HwEnet_MMFR_PA_Shift)
			| (phyReg << HwEnet_MMFR_RA_Shift) | (2 << HwEnet_MMFR_TA_Shift)
			| data;

	{
		uint32_t timeout = 0;
		for (; timeout < 0xFFFFFU; timeout++) {
			if (hwEnet0->EIR & HwEnet_EIR_MII_Mask) {
				break;
			}
		}
		if (timeout == 0xFFFFFU) {
			return FALSE;
		}
	}

	// Clear interrupt
	hwEnet0->EIR = HwEnet_EIR_MII_Mask;

	return TRUE;
}

static void setupPins() {
	hw_sim->SCGC5 |= SIM_SCGC5_PORTA_Mask;

	// TODO not needed
	hw_portA->PCR[29] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_COL
	hw_portA->PCR[27] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_CRS
	hw_portA->PCR[8] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_MDC
	hw_portA->PCR[7] = (HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift)
			| HwPort_PCR_SRE_Mask | HwPort_PCR_ODE_Mask;		// MII_MDIO
	hw_portA->PCR[11] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXCLK
	hw_portA->PCR[14] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXDV
	hw_portA->PCR[13] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXD0
	hw_portA->PCR[12] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXD1
	hw_portA->PCR[10] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXD2
	hw_portA->PCR[9] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_RXD3
	hw_portA->PCR[5] = HwPort_MuxAlt4 << HwPort_PCR_MUX_Shift;	// MII_RXER
	hw_portA->PCR[25] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXCLK
	hw_portA->PCR[16] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXD0
	hw_portA->PCR[17] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXD1
	hw_portA->PCR[24] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXD2
	hw_portA->PCR[26] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXD3
	hw_portA->PCR[15] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXEN
	hw_portA->PCR[28] = HwPort_MuxAlt5 << HwPort_PCR_MUX_Shift;	// MII_TXER
}

bool_t enet_init(void) {
	setupPins();

	const uint32_t clk = System_CoreCLK_Freq;

	// Enable clock to ENET
	hw_sim->SCGC2 |= SIM_SCGC2_ENET_Mask;

	{
		// Calculate the MII speed which controls the frequency of the MDC.
		uint32_t const speed = (clk / (2 * HwEnetMdcFrequency)) - 1;
		// Calculate the hold time on the MDIO output.
		// uint32_t clkCycle = (10 + EnetNanoSecInOneSecond / clk - 1)
		//		/ (EnetNanoSecInOneSecond / clk) - 1;
		uint32_t mscr = (speed << HwEnet_MSCR_MIISPEED_Shift)
				| HwEnet_MSCR_DISPRE_Mask
				/* | (clkCycle << HwEnet_MSCR_HOLDTIME_Shift) */;
		// Build the configuration for MDC/MDIO control.
		hwEnet0->MSCR = mscr;
	}

	{
		uint32_t timeout = 0;
		for (; timeout < 0xFFFFFU; timeout++) {
			uint32_t id = readSMI(0, MiiRegAddrPhyId1);
			if (id == MiiPhyControlId1) {
				break;
			}
		}
		if (timeout == 0xFFFFFU) {
			return FALSE;
		}
	}

	// Reset PHY
	if (!writeSMI(0, MiiRegAddrControl, MiiControlReg_Reset_Mask)) {
		return FALSE;
	}

	if (!writeSMI(0, MiiRegAddrAnAdvertise,
			(MiiAnAdvertiseReg_100FullDuplex_Mask
					| MiiAnAdvertiseReg_100HalfDuplex_Mask
					| MiiAnAdvertiseReg_10FullDuplex_Mask
					| MiiAnAdvertiseReg_10HalfDuplex_Mask | 1))) {
		return FALSE;
	}

	if (!writeSMI(0, MiiRegAddrControl,
			MiiControlReg_Autoneg_Mask | MiiControlReg_RestartAutoneg_Mask)) {
		return FALSE;
	}

	{
		uint32_t timeout = 0;
		for (; timeout < 0xFFFFFU; timeout++) {
			uint32_t read = readSMI(0, MiiRegAddrStatus);
			if (read == 0xFFFFFFFF)
				continue;
			if (read & MiiStatusReg_AutonegComp_Mask) {
				break;
			}
		}
		if (timeout == 0xFFFFFU) {
			return FALSE;
		}
	}

	for (uint32_t count = 0; count < 0xFFFFFU; count++) {
		count--;
		count++;
	}

	{
		uint32_t read = readSMI(0, MiiRegAddrStatus);
		if (read == 0xFFFFFFFF)
			return FALSE;
		if ((read & MiiStatusReg_LinkStatus_Mask) == 0) {
			return FALSE;
		}
	}

	{
		uint32_t data = readSMI(0, MiiRegAddrControl2);
		if (data == 0xFFFFFFFF) {
			return FALSE;
		}

		data = data & MiiControl2_SpeedDuplex_Mask;

		bool_t isFullDuplex = FALSE;
		bool_t is100 = FALSE;

		if (data == MiiControl2_10FullDuplex
				|| data == MiiControl2_100FullDuplex) {
			isFullDuplex = TRUE;
		}

		if (data == MiiControl2_100HalfDuplex
				|| data == MiiControl2_100FullDuplex) {
			is100 = TRUE;
		}

		if (!isFullDuplex) {
			return FALSE;
		}

		if (!is100) {
			return FALSE;
		}
	}

	hwEnet0->ECR |= HwEnet_ECR_RESET_Mask;

	{
		// Calculate the MII speed which controls the frequency of the MDC.
		uint32_t const speed = (clk / (2 * HwEnetMdcFrequency)) - 1;
		// Calculate the hold time on the MDIO output.
		uint32_t mscr = (speed << HwEnet_MSCR_MIISPEED_Shift)
				| HwEnet_MSCR_DISPRE_Mask;
		// Build the configuration for MDC/MDIO control.
		hwEnet0->MSCR = mscr;
	}

	// Set TX and RX buffers
	for (uint16_t count = 0; count < EnetNumTxBuffers; count++) {
		// Set data buffer address.
		enet.txBd[count].buffer = (uint8_t *) &enet.txBuf[count];
		// Initializes data length.
		enet.txBd[count].length = 0;
		// Initializes the buffer descriptors with empty bit.
		enet.txBd[count].control = HwEnet_TXBDControl_TC_Mask;
	}
	// Sets the last buffer descriptor with the wrap flag.
	enet.txBd[EnetNumTxBuffers - 1].control |= HwEnet_TXBDControl_W_Mask;

	for (uint16_t count = 0; count < EnetNumRxBuffers; count++) {
		// Set data buffer address.
		enet.rxBd[count].buffer = (uint8_t *) &enet.rxBuf[count];
		// Initializes data length.
		enet.rxBd[count].length = 0;
		// Initializes the buffer descriptors with empty bit.
		enet.rxBd[count].control = HwEnet_RXBDControl_E_Mask;
	}
	// Sets the last buffer descriptor with the wrap flag.
	enet.rxBd[EnetNumRxBuffers - 1].control |= HwEnet_RXBDControl_W_Mask;

	hwEnet0->RCR = HwEnet_RCR_MIIMODE_Mask | HwEnet_RCR_CRCFWD_Mask
			| (EnetMaxFrameLen << HwEnet_RCR_MAXFL_Shift) | HwEnet_RCR_PROM_Mask
			/* | HwEnet_RCR_LOOP_Mask */;

	hwEnet0->TCR |= HwEnet_TCR_FDN_Mask;

	hwEnet0->TACC = 0;
	hwEnet0->RACC = 0;

	hwEnet0->TFWR = HwEnet_TFWR_STRFWD_Mask;
	hwEnet0->RSFL = 0;

	hwEnet0->TDSR = (uint32_t) &enet.txBd;
	hwEnet0->RDSR = (uint32_t) &enet.rxBd;
	hwEnet0->MRBR = EnetBufferSize;

	// Set MAC address
	hwEnet0->PALR = uint32_make_from_uint8(macAddr[0], macAddr[1], macAddr[2],
			macAddr[3]);
	hwEnet0->PAUR = ((uint32_t) uint16_make_from_bytes(macAddr[4], macAddr[5]))
			<< HwEnet_PAUR_PADDR2_Shift;

	// TODO

	// TODO EIMR

	hwEnet0->ECR |= HwEnet_ECR_ETHEREN_Mask | HwEnet_ECR_DBSWP_Mask
			| HwEnet_ECR_EN1588_Mask;

	// TODO Set handler

	// Activate read
	hwEnet0->RDAR = HwEnet_RDAR_RDAR_Mask;

	// TODO

	return TRUE;
}

bool_t enet_sendFrame(uint8_t * data, uint16_t length) {
	EnetTxBd &bd = enet.txBd[enet.curTxBd];

	if (bd.control & HwEnet_TXBDControl_R_Mask) {
		return FALSE;
	}

	for (int i = 0; i < length; i++) {
		bd.buffer[i] = data[i];
	}
	bd.length = length;

	bd.control |= HwEnet_TXBDControl_R_Mask | HwEnet_TXBDControl_L_Mask
			| HwEnet_TXBDControl_TC_Mask;

	if (bd.control & HwEnet_TXBDControl_W_Mask) {
		enet.curTxBd = 0;
	} else {
		enet.curTxBd++;
	}

	hwEnet0->TDAR = HwEnet_TDAR_TDAR_Mask;

	return TRUE;
}

bool_t enet_receiveFrame() {
	EnetRxBd &bd = enet.rxBd[0];

	if (bd.control & HwEnet_RXBDControl_E_Mask) {
		return FALSE;
	}

	for (int i = 0; i < EnetNumRxBuffers; i++) {
		enet.rxBd[i].control |= HwEnet_RXBDControl_E_Mask;
		enet.rxBd[i].controlExtend2 &= ~0x8000U;
	}

	hwEnet0->RDAR = HwEnet_RDAR_RDAR_Mask;

	return TRUE;
}
