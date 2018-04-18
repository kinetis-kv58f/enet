#ifndef INCLUDE_HW_ENET_H_
#define INCLUDE_HW_ENET_H_

#include "common.h"

/// ENET - Register Layout
typedef struct {
	uint8_t RESERVED_0[4];
	/// Interrupt Event Register, offset: 0x4
	__IO uint32_t EIR;
	/// Interrupt Mask Register, offset: 0x8
	__IO uint32_t EIMR;
	uint8_t RESERVED_1[4];
	/// Receive Descriptor Active Register, offset: 0x10
	__IO uint32_t RDAR;
	/// Transmit Descriptor Active Register, offset: 0x14
	__IO uint32_t TDAR;
	uint8_t RESERVED_2[12];
	/// Ethernet Control Register, offset: 0x24
	__IO uint32_t ECR;
	uint8_t RESERVED_3[24];__IO
	/// MII Management Frame Register, offset: 0x40
	uint32_t MMFR;
	/// MII Speed Control Register, offset: 0x44
	__IO uint32_t MSCR;
	uint8_t RESERVED_4[28];
	/// MIB Control Register, offset: 0x64
	__IO uint32_t MIBC;
	uint8_t RESERVED_5[28];
	/// Receive Control Register, offset: 0x84
	__IO uint32_t RCR;
	uint8_t RESERVED_6[60];
	/// Transmit Control Register, offset: 0xC4
	__IO uint32_t TCR;
	uint8_t RESERVED_7[28];
	/// Physical Address Lower Register, offset: 0xE4
	__IO uint32_t PALR;
	/// Physical Address Upper Register, offset: 0xE8
	__IO uint32_t PAUR;
	/// Opcode/Pause Duration Register, offset: 0xEC
	__IO uint32_t OPD;
	uint8_t RESERVED_8[40];
	/// Descriptor Individual Upper Address Register, offset: 0x118
	__IO uint32_t IAUR;
	/// Descriptor Individual Lower Address Register, offset: 0x11C
	__IO uint32_t IALR;__IO uint32_t GAUR; /**< Descriptor Group Upper Address Register, offset: 0x120 */
	__IO uint32_t GALR; /**< Descriptor Group Lower Address Register, offset: 0x124 */
	uint8_t RESERVED_9[28];__IO uint32_t TFWR; /**< Transmit FIFO Watermark Register, offset: 0x144 */
	uint8_t RESERVED_10[56];__IO uint32_t RDSR; /**< Receive Descriptor Ring Start Register, offset: 0x180 */
	__IO uint32_t TDSR; /**< Transmit Buffer Descriptor Ring Start Register, offset: 0x184 */
	__IO uint32_t MRBR; /**< Maximum Receive Buffer Size Register, offset: 0x188 */
	uint8_t RESERVED_11[4];__IO uint32_t RSFL; /**< Receive FIFO Section Full Threshold, offset: 0x190 */
	__IO uint32_t RSEM; /**< Receive FIFO Section Empty Threshold, offset: 0x194 */
	__IO uint32_t RAEM; /**< Receive FIFO Almost Empty Threshold, offset: 0x198 */
	__IO uint32_t RAFL; /**< Receive FIFO Almost Full Threshold, offset: 0x19C */
	__IO uint32_t TSEM; /**< Transmit FIFO Section Empty Threshold, offset: 0x1A0 */
	__IO uint32_t TAEM; /**< Transmit FIFO Almost Empty Threshold, offset: 0x1A4 */
	__IO uint32_t TAFL; /**< Transmit FIFO Almost Full Threshold, offset: 0x1A8 */
	__IO uint32_t TIPG; /**< Transmit Inter-Packet Gap, offset: 0x1AC */
	__IO uint32_t FTRL; /**< Frame Truncation Length, offset: 0x1B0 */
	uint8_t RESERVED_12[12];__IO uint32_t TACC; /**< Transmit Accelerator Function Configuration, offset: 0x1C0 */
	__IO uint32_t RACC; /**< Receive Accelerator Function Configuration, offset: 0x1C4 */
	uint8_t RESERVED_13[56];
	uint32_t RMON_T_DROP; /**< Reserved Statistic Register, offset: 0x200 */
	__I uint32_t RMON_T_PACKETS; /**< Tx Packet Count Statistic Register, offset: 0x204 */
	__I uint32_t RMON_T_BC_PKT; /**< Tx Broadcast Packets Statistic Register, offset: 0x208 */
	__I uint32_t RMON_T_MC_PKT; /**< Tx Multicast Packets Statistic Register, offset: 0x20C */
	__I uint32_t RMON_T_CRC_ALIGN; /**< Tx Packets with CRC/Align Error Statistic Register, offset: 0x210 */
	__I uint32_t RMON_T_UNDERSIZE; /**< Tx Packets Less Than Bytes and Good CRC Statistic Register, offset: 0x214 */
	__I uint32_t RMON_T_OVERSIZE; /**< Tx Packets GT MAX_FL bytes and Good CRC Statistic Register, offset: 0x218 */
	__I uint32_t RMON_T_FRAG; /**< Tx Packets Less Than 64 Bytes and Bad CRC Statistic Register, offset: 0x21C */
	__I uint32_t RMON_T_JAB; /**< Tx Packets Greater Than MAX_FL bytes and Bad CRC Statistic Register, offset: 0x220 */
	__I uint32_t RMON_T_COL; /**< Tx Collision Count Statistic Register, offset: 0x224 */
	__I uint32_t RMON_T_P64; /**< Tx 64-Byte Packets Statistic Register, offset: 0x228 */
	__I uint32_t RMON_T_P65TO127; /**< Tx 65- to 127-byte Packets Statistic Register, offset: 0x22C */
	__I uint32_t RMON_T_P128TO255; /**< Tx 128- to 255-byte Packets Statistic Register, offset: 0x230 */
	__I uint32_t RMON_T_P256TO511; /**< Tx 256- to 511-byte Packets Statistic Register, offset: 0x234 */
	__I uint32_t RMON_T_P512TO1023; /**< Tx 512- to 1023-byte Packets Statistic Register, offset: 0x238 */
	__I uint32_t RMON_T_P1024TO2047; /**< Tx 1024- to 2047-byte Packets Statistic Register, offset: 0x23C */
	__I uint32_t RMON_T_P_GTE2048; /**< Tx Packets Greater Than 2048 Bytes Statistic Register, offset: 0x240 */
	__I uint32_t RMON_T_OCTETS; /**< Tx Octets Statistic Register, offset: 0x244 */
	uint32_t IEEE_T_DROP; /**< Reserved Statistic Register, offset: 0x248 */
	__I uint32_t IEEE_T_FRAME_OK; /**< Frames Transmitted OK Statistic Register, offset: 0x24C */
	__I uint32_t IEEE_T_1COL; /**< Frames Transmitted with Single Collision Statistic Register, offset: 0x250 */
	__I uint32_t IEEE_T_MCOL; /**< Frames Transmitted with Multiple Collisions Statistic Register, offset: 0x254 */
	__I uint32_t IEEE_T_DEF; /**< Frames Transmitted after Deferral Delay Statistic Register, offset: 0x258 */
	__I uint32_t IEEE_T_LCOL; /**< Frames Transmitted with Late Collision Statistic Register, offset: 0x25C */
	__I uint32_t IEEE_T_EXCOL; /**< Frames Transmitted with Excessive Collisions Statistic Register, offset: 0x260 */
	__I uint32_t IEEE_T_MACERR; /**< Frames Transmitted with Tx FIFO Underrun Statistic Register, offset: 0x264 */
	__I uint32_t IEEE_T_CSERR; /**< Frames Transmitted with Carrier Sense Error Statistic Register, offset: 0x268 */
	__I uint32_t IEEE_T_SQE; /**< Reserved Statistic Register, offset: 0x26C */
	__I uint32_t IEEE_T_FDXFC; /**< Flow Control Pause Frames Transmitted Statistic Register, offset: 0x270 */
	__I uint32_t IEEE_T_OCTETS_OK; /**< Octet Count for Frames Transmitted w/o Error Statistic Register, offset: 0x274 */
	uint8_t RESERVED_14[12];__I uint32_t RMON_R_PACKETS; /**< Rx Packet Count Statistic Register, offset: 0x284 */
	__I uint32_t RMON_R_BC_PKT; /**< Rx Broadcast Packets Statistic Register, offset: 0x288 */
	__I uint32_t RMON_R_MC_PKT; /**< Rx Multicast Packets Statistic Register, offset: 0x28C */
	__I uint32_t RMON_R_CRC_ALIGN; /**< Rx Packets with CRC/Align Error Statistic Register, offset: 0x290 */
	__I uint32_t RMON_R_UNDERSIZE; /**< Rx Packets with Less Than 64 Bytes and Good CRC Statistic Register, offset: 0x294 */
	__I uint32_t RMON_R_OVERSIZE; /**< Rx Packets Greater Than MAX_FL and Good CRC Statistic Register, offset: 0x298 */
	__I uint32_t RMON_R_FRAG; /**< Rx Packets Less Than 64 Bytes and Bad CRC Statistic Register, offset: 0x29C */
	__I uint32_t RMON_R_JAB; /**< Rx Packets Greater Than MAX_FL Bytes and Bad CRC Statistic Register, offset: 0x2A0 */
	uint32_t RMON_R_RESVD_0; /**< Reserved Statistic Register, offset: 0x2A4 */
	__I uint32_t RMON_R_P64; /**< Rx 64-Byte Packets Statistic Register, offset: 0x2A8 */
	__I uint32_t RMON_R_P65TO127; /**< Rx 65- to 127-Byte Packets Statistic Register, offset: 0x2AC */
	__I uint32_t RMON_R_P128TO255; /**< Rx 128- to 255-Byte Packets Statistic Register, offset: 0x2B0 */
	__I uint32_t RMON_R_P256TO511; /**< Rx 256- to 511-Byte Packets Statistic Register, offset: 0x2B4 */
	__I uint32_t RMON_R_P512TO1023; /**< Rx 512- to 1023-Byte Packets Statistic Register, offset: 0x2B8 */
	__I uint32_t RMON_R_P1024TO2047; /**< Rx 1024- to 2047-Byte Packets Statistic Register, offset: 0x2BC */
	__I uint32_t RMON_R_P_GTE2048; /**< Rx Packets Greater than 2048 Bytes Statistic Register, offset: 0x2C0 */
	__I uint32_t RMON_R_OCTETS; /**< Rx Octets Statistic Register, offset: 0x2C4 */
	__I uint32_t IEEE_R_DROP; /**< Frames not Counted Correctly Statistic Register, offset: 0x2C8 */
	__I uint32_t IEEE_R_FRAME_OK; /**< Frames Received OK Statistic Register, offset: 0x2CC */
	__I uint32_t IEEE_R_CRC; /**< Frames Received with CRC Error Statistic Register, offset: 0x2D0 */
	__I uint32_t IEEE_R_ALIGN; /**< Frames Received with Alignment Error Statistic Register, offset: 0x2D4 */
	__I uint32_t IEEE_R_MACERR; /**< Receive FIFO Overflow Count Statistic Register, offset: 0x2D8 */
	__I uint32_t IEEE_R_FDXFC; /**< Flow Control Pause Frames Received Statistic Register, offset: 0x2DC */
	__I uint32_t IEEE_R_OCTETS_OK; /**< Octet Count for Frames Received without Error Statistic Register, offset: 0x2E0 */
	uint8_t RESERVED_15[284];__IO uint32_t ATCR; /**< Adjustable Timer Control Register, offset: 0x400 */
	__IO uint32_t ATVR; /**< Timer Value Register, offset: 0x404 */
	__IO uint32_t ATOFF; /**< Timer Offset Register, offset: 0x408 */
	__IO uint32_t ATPER; /**< Timer Period Register, offset: 0x40C */
	__IO uint32_t ATCOR; /**< Timer Correction Register, offset: 0x410 */
	__IO uint32_t ATINC; /**< Time-Stamping Clock Period Register, offset: 0x414 */
	__I uint32_t ATSTMP; /**< Timestamp of Last Transmitted Frame, offset: 0x418 */
	uint8_t RESERVED_16[488];__IO uint32_t TGSR; /**< Timer Global Status Register, offset: 0x604 */
	struct { /* offset: 0x608, array step: 0x8 */
		__IO uint32_t TCSR; /**< Timer Control Status Register, array offset: 0x608, array step: 0x8 */
		__IO uint32_t TCCR; /**< Timer Compare Capture Register, array offset: 0x60C, array step: 0x8 */
	} CHANNEL[4];
} HwEnet;

enum {
	HwEnet_EIR_MII_Shift = 23,
	HwEnet_EIR_MII_Mask = 1 << HwEnet_EIR_MII_Shift,

	HwEnet_MMFR_ST_Shift = 30,
	HwEnet_MMFR_ST_Mask = 0x3 << HwEnet_MMFR_ST_Shift,

	HwEnet_MMFR_OP_Shift = 28,
	HwEnet_MMFR_OP_Mask = 0x3 << HwEnet_MMFR_OP_Shift,

	HwEnet_MMFR_PA_Shift = 23,
	HwEnet_MMFR_PA_Mask = 0x1F << HwEnet_MMFR_PA_Shift,

	HwEnet_MMFR_RA_Shift = 18,
	HwEnet_MMFR_RA_Mask = 0x1F << HwEnet_MMFR_RA_Shift,

	HwEnet_MMFR_TA_Shift = 16,
	HwEnet_MMFR_TA_Mask = 0x3 << HwEnet_MMFR_TA_Shift,

	HwEnet_MMFR_DATA_Shift = 0,
	HwEnet_MMFR_DATA_Mask = 0xFFFF << HwEnet_MMFR_DATA_Shift,

	HwEnet_PAUR_PADDR2_Shift = 16,
	HwEnet_PAUR_PADDR2_Mask = 0xFFFF << HwEnet_PAUR_PADDR2_Shift,

	HwEnet_PAUR_TYPE_Shift = 0,
	HwEnet_PAUR_TYPE_Mask = 0xFFFF << HwEnet_PAUR_TYPE_Shift,

	HwEnet_TFWR_STRFWD_Shift = 1,
	HwEnet_TFWR_STRFWD_Mask = 8 << HwEnet_TFWR_STRFWD_Shift,

	///
	HwEnet_RDAR_RDAR_Shift = 24,
	///
	HwEnet_RDAR_RDAR_Mask = 1 << HwEnet_RDAR_RDAR_Shift,

	///
	HwEnet_TDAR_TDAR_Shift = 24,
	HwEnet_TDAR_TDAR_Mask = 1 << HwEnet_TDAR_TDAR_Shift,
};

enum {
	HwEnet_ECR_DBSWP_Shift = 8,
	HwEnet_ECR_DBSWP_Mask = 1 << HwEnet_ECR_DBSWP_Shift,
	HwEnet_ECR_STOPEN_Shift = 7,
	HwEnet_ECR_STOPEN_Mask = 1 << HwEnet_ECR_STOPEN_Shift,
	HwEnet_ECR_DBGEN_Shift = 6,
	HwEnet_ECR_DBGEN_Mask = 1 << HwEnet_ECR_DBGEN_Shift,
	HwEnet_ECR_EN1588_Shift = 4,
	HwEnet_ECR_EN1588_Mask = 1 << HwEnet_ECR_EN1588_Shift,
	HwEnet_ECR_SLEEP_Shift = 3,
	HwEnet_ECR_SLEEP_Mask = 1 << HwEnet_ECR_SLEEP_Shift,
	HwEnet_ECR_MAGICEN_Shift = 2,
	HwEnet_ECR_MAGICEN_Mask = 1 << HwEnet_ECR_MAGICEN_Shift,
	HwEnet_ECR_ETHEREN_Shift = 1,
	HwEnet_ECR_ETHEREN_Mask = 1 << HwEnet_ECR_ETHEREN_Shift,
	HwEnet_ECR_RESET_Shift = 0,
	HwEnet_ECR_RESET_Mask = 1 << HwEnet_ECR_RESET_Shift,
};

enum {
	HwEnet_RCR_GRS_Shift = 31,
	HwEnet_RCR_GRS_Mask = 1 << HwEnet_RCR_GRS_Shift,

	HwEnet_RCR_NLC_Shift = 30,
	HwEnet_RCR_NLC_Mask = 1 << HwEnet_RCR_NLC_Shift,

	HwEnet_RCR_MAXFL_Shift = 16,
	HwEnet_RCR_MAXFL_Mask = 0x3FFF << HwEnet_RCR_MAXFL_Shift,

	HwEnet_RCR_CFEN_Shift = 15,
	HwEnet_RCR_CFEN_Mask = 1 << HwEnet_RCR_CFEN_Shift,

	HwEnet_RCR_CRCFWD_Shift = 14,
	HwEnet_RCR_CRCFWD_Mask = 1 << HwEnet_RCR_CRCFWD_Shift,

	HwEnet_RCR_PAUFWD_Shift = 13,
	HwEnet_RCR_PAUFWD_Mask = 1 << HwEnet_RCR_PAUFWD_Shift,

	HwEnet_RCR_PADEN_Shift = 12,
	HwEnet_RCR_PADEN_Mask = 1 << HwEnet_RCR_PADEN_Shift,

	HwEnet_RCR_RMII10T_Shift = 9,
	HwEnet_RCR_RMII10T_Mask = 1 << HwEnet_RCR_RMII10T_Shift,

	HwEnet_RCR_RMIIMODE_Shift = 8,
	HwEnet_RCR_RMIIMODE_Mask = 1 << HwEnet_RCR_RMIIMODE_Shift,

	HwEnet_RCR_FCE_Shift = 5,
	HwEnet_RCR_FCE_Mask = 1 << HwEnet_RCR_FCE_Shift,

	HwEnet_RCR_BCREJ_Shift = 4,
	HwEnet_RCR_BCREJ_Mask = 1 << HwEnet_RCR_BCREJ_Shift,

	HwEnet_RCR_PROM_Shift = 3,
	HwEnet_RCR_PROM_Mask = 1 << HwEnet_RCR_PROM_Shift,

	HwEnet_RCR_MIIMODE_Shift = 2,
	HwEnet_RCR_MIIMODE_Mask = 1 << HwEnet_RCR_MIIMODE_Shift,

	HwEnet_RCR_DRT_Shift = 1,
	HwEnet_RCR_DRT_Mask = 1 << HwEnet_RCR_DRT_Shift,

	HwEnet_RCR_LOOP_Shift = 0,
	HwEnet_RCR_LOOP_Mask = 1 << HwEnet_RCR_LOOP_Shift,
};

enum {
	HwEnet_TCR_CRCFWD_Shift = 9,
	HwEnet_TCR_CRCFWD_Mask = 1 << HwEnet_TCR_CRCFWD_Shift,

	HwEnet_TCR_ADDINS_Shift = 8,
	HwEnet_TCR_ADDINS_Mask = 1 << HwEnet_TCR_ADDINS_Shift,

	HwEnet_TCR_ADDSEL_Shift = 5,
	HwEnet_TCR_ADDSEL_Mask = 0x7 << HwEnet_TCR_ADDSEL_Shift,

	HwEnet_TCR_RFCPAUSE_Shift = 4,
	HwEnet_TCR_RFCPAUSE_Mask = 0x1 << HwEnet_TCR_RFCPAUSE_Shift,

	HwEnet_TCR_TFCPAUSE_Shift = 3,
	HwEnet_TCR_TFCPAUSE_Mask = 0x1 << HwEnet_TCR_TFCPAUSE_Shift,

	HwEnet_TCR_FDN_Shift = 2,
	HwEnet_TCR_FDN_Mask = 0x1 << HwEnet_TCR_FDN_Shift,

	HwEnet_TCR_GTS_Shift = 0,
	HwEnet_TCR_GTS_Mask = 0x1 << HwEnet_TCR_GTS_Shift,
};

enum {
	HwEnet_TACC_PROCHK_Shift = 4,
	HwEnet_TACC_PROCHK_Mask = 1 << HwEnet_TACC_PROCHK_Shift,

	HwEnet_TACC_IPCHK_Shift = 3,
	HwEnet_TACC_IPCHK_Mask = 1 << HwEnet_TACC_IPCHK_Shift,

	HwEnet_TACC_SHIFT16_Shift = 0,
	HwEnet_TACC_SHIFT16_Mask = 1 << HwEnet_TACC_SHIFT16_Shift,
};

enum {
	HwEnet_MSCR_HOLDTIME_Shift = 8,
	HwEnet_MSCR_HOLDTIME_Mask = 0x7 << HwEnet_MSCR_HOLDTIME_Shift,

	HwEnet_MSCR_DISPRE_Shift = 7,
	HwEnet_MSCR_DISPRE_Mask = 1 << HwEnet_MSCR_DISPRE_Shift,

	HwEnet_MSCR_MIISPEED_Shift = 1,
	HwEnet_MSCR_MIISPEED_Mask = 0x3F << HwEnet_MSCR_MIISPEED_Shift,
};

enum {
	HwEnet_TXBDControl_TC_Shift = 10,
	HwEnet_TXBDControl_TC_Mask = 1 << HwEnet_TXBDControl_TC_Shift,

	HwEnet_TXBDControl_L_Shift = 11,
	HwEnet_TXBDControl_L_Mask = 1 << HwEnet_TXBDControl_L_Shift,

	HwEnet_TXBDControl_TO2_Shift = 12,
	HwEnet_TXBDControl_TO2_Mask = 1 << HwEnet_TXBDControl_TO2_Shift,

	HwEnet_TXBDControl_W_Shift = 13,
	HwEnet_TXBDControl_W_Mask = 1 << HwEnet_TXBDControl_W_Shift,

	HwEnet_TXBDControl_TO1_Shift = 14,
	HwEnet_TXBDControl_TO1_Mask = 1 << HwEnet_TXBDControl_TO1_Shift,

	HwEnet_TXBDControl_R_Shift = 15,
	HwEnet_TXBDControl_R_Mask = 1 << HwEnet_TXBDControl_R_Shift,
};

enum {
	/// Empty bit mask.
	HwEnet_RXBDControl_E_Mask = 0x8000,
	/// Software owner one mask.
	HwEnet_RXBDControl_RO1_Mask = 0x4000,
	/// Next buffer descriptor is the start address.
	HwEnet_RXBDControl_W_Mask = 0x2000,
	/// Software owner two mask.
	HwEnet_RXBDControl_RO2_Mask = 0x1000,
	/// Last BD of the frame mask.
	HwEnet_RXBDControl_L_Mask = 0x0800,
	/// Received because of the promiscuous mode.
	HwEnet_RXBDControl_M_Mask = 0x0100,
	/// Broadcast packet mask.
	HwEnet_RXBDControl_BC_Mask = 0x0080,
	/// Multicast packet mask.
	HwEnet_RXBDControl_MC_Mask = 0x0040,
	/// Length violation mask.
	HwEnet_RXBDControl_LG_Mask = 0x0020,
	/// Non-octet aligned frame mask.
	HwEnet_RXBDControl_NO_Mask = 0x0010,
	/// CRC error mask.
	HwEnet_RXBDControl_CR_Mask = 0x0004,
	/// FIFO overrun mask.
	HwEnet_RXBDControl_OV_Mask = 0x0002,
	/// Frame is truncated mask.
	HwEnet_RXBDControl_TR_Mask = 0x0001
};

volatile HwEnet * const volatile hwEnet0 = (HwEnet *) Memmap_ENET0_Addr;

#endif /* INCLUDE_HW_ENET_H_ */
