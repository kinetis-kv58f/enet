/*
 * mii.h
 *
 *  Created on: 13 apr. 2018
 *      Author: SERAGUD
 */

#ifndef SRC_DRIV_ENET_MII_H_
#define SRC_DRIV_ENET_MII_H_

typedef enum {
	/// Address write
	MiiOpAddrWrite = 0U,
	/// Write operation
	MiiOpWrite = 1U,
	/// Read frame operation for a valid MII management frame.
	MiiOpReadInc = 2U,
	/// Read frame operation, but not MII-compliant.
	MiiOpRead = 3U,
} MiiOp;

typedef enum {
	MiiRegAddrControl = 0,
	MiiRegAddrStatus = 1,
	MiiRegAddrPhyId1 = 2,
	MiiRegAddrPhyId2 = 3,
	MiiRegAddrAnAdvertise = 4,
	MiiRegAddrAnLink = 5,
	MiiRegAddrAnExpansion = 6,
	MiiRegAddrAnNextPage = 7,
	MiiRegAddrAnLinkRxNextPage = 8,

	MiiRegAddrControl1 = 0x1E,
	MiiRegAddrControl2 = 0x1F,
} MiiRegAddr;

typedef enum {
	MiiControlReg_Reset_Mask = 0x8000,
	MiiControlReg_Loop_Mask = 0x4000,
	MiiControlReg_Speed_Mask = 0x2000,
	MiiControlReg_Autoneg_Mask = 0x1000,
	MiiControlReg_RestartAutoneg_Mask = 0x0200,
	MiiControlReg_Duplex_Mask = 0x0100,
	MiiControlReg_DisableTx_Mask = 0x0001,
} MiiControlReg;

typedef enum {
	MiiStatusReg_100BaseT4_Mask = 0x8000,
	MiiStatusReg_SpeedDuplex_Mask = 0x7800,
	MiiStatusReg_NoPreamble_Mask = 0x0040,
	MiiStatusReg_AutonegComp_Mask = 0x0020,
	MiiStatusReg_Autonegable_Mask = 0x0008,
	MiiStatusReg_LinkStatus_Mask = 0x0004,
} MiiStatusReg;

typedef enum {
	MiiAnAdvertiseReg_100FullDuplex_Mask = 0x100,
	MiiAnAdvertiseReg_100HalfDuplex_Mask = 0x080,
	MiiAnAdvertiseReg_10FullDuplex_Mask = 0x040,
	MiiAnAdvertiseReg_10HalfDuplex_Mask = 0x020,
} MiiAnAdvertiseReg;

enum {
	MiiControl2_SpeedDuplex_Mask = 0x001C,
	MiiControl2_10HalfDuplex = 0x0004U,
	MiiControl2_100HalfDuplex = 0x0008U,
	MiiControl2_10FullDuplex = 0x0014U,
	MiiControl2_100FullDuplex = 0x0018U,
};

/// Defines the receive buffer descriptor structure for the little endian system.
typedef struct {
	/// Buffer descriptor data length.
	uint16_t length;
	/// Buffer descriptor control and status.
	uint16_t control;
	/// Data buffer pointer.
	uint8_t *buffer;
	/// Extend buffer descriptor control0.
	uint16_t controlExtend0;
	/// Extend buffer descriptor control1.
	uint16_t controlExtend1;
	/// Internal payload checksum.
	uint16_t payloadCheckSum;
	/// Header length.
	uint8_t headerLength;
	/// Protocol type.
	uint8_t protocolTyte;
	uint16_t reserved0;
	/// Extend buffer descriptor control2.
	uint16_t controlExtend2;
	/// Timestamp.
	uint32_t timestamp;
	uint16_t reserved1;
	uint16_t reserved2;
	uint16_t reserved3;
	uint16_t reserved4;
} EnetRxBd;

/// Defines the enhanced transmit buffer descriptor structure for the little endian system.
typedef struct {
	/// Buffer descriptor data length.
	uint16_t length;
	/// Buffer descriptor control and status.
	uint16_t control;
	/// Data buffer pointer.
	uint8_t *buffer;
	/// Extend buffer descriptor control0.
	uint16_t controlExtend0;
	/// Extend buffer descriptor control1.
	uint16_t controlExtend1;
	uint16_t reserved0;
	uint16_t reserved1;
	uint16_t reserved2;
	/// Extend buffer descriptor control2.
	uint16_t controlExtend2;
	/// Timestamp.
	uint32_t timestamp;
	uint16_t reserved3;
	uint16_t reserved4;
	uint16_t reserved5;
	uint16_t reserved6;
} EnetTxBd;

enum {
	/// Default maximum Ethernet frame size.
	EnetMaxFrameLen = 1584,
	MiiPhyControlId1 = 0x22,
	EnetNumTxBuffers = 6,
	EnetNumRxBuffers = 6,
	EnetBufferSize = 1584,
	HwEnetMdcFrequency = 2500000,
	EnetNanoSecInOneSecond = 1000000000,
};

typedef struct {
	EnetRxBd rxBd[EnetNumRxBuffers];

	EnetTxBd txBd[EnetNumTxBuffers];

	uint8_t rxBuf[EnetNumTxBuffers][EnetBufferSize];

	uint8_t txBuf[EnetNumRxBuffers][EnetBufferSize];

	uint8_t curTxBd;
} Enet;

#endif /* SRC_DRIV_ENET_MII_H_ */
