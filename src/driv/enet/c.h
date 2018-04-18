/*
 * c.h
 *
 *  Created on: 12 apr. 2018
 *      Author: SERAGUD
 */

#ifndef SRC_DRIV_ENET_C_H_
#define SRC_DRIV_ENET_C_H_

/// Defines the receive buffer descriptor structure for the little endian system.
typedef struct {
	/// Buffer descriptor data length.
	uint16_t length;
	/// Buffer descriptor control and status.
	uint16_t control;
	/// Data buffer pointer.
	uint8_t *buffer;
#ifdef ENET_ENHANCEDBUFFERDESCRIPTOR_MODE

#endif
} EnetRxBd;

/// Defines the enhanced transmit buffer descriptor structure for the little endian system.
typedef struct {
/// Buffer descriptor data length.
uint16_t length;
/// Buffer descriptor control and status.
uint16_t control;
/// Data buffer pointer.
uint8_t *buffer;
#ifdef ENET_ENHANCEDBUFFERDESCRIPTOR_MODE
/// Extend buffer descriptor control0.
uint16_t controlExtend0;
/// Extend buffer descriptor control1.
uint16_t controlExtend1;
#if defined(FSL_FEATURE_ENET_HAS_AVB) && FSL_FEATURE_ENET_HAS_AVB
/// Transmit launch time.
int8_t *txLaunchTime;
#else
uint16_t reserved0;
uint16_t reserved1;
#endif /* FSL_FEATURE_ENET_HAS_AVB */
uint16_t reserved2;
/// Extend buffer descriptor control2.
uint16_t controlExtend2;
/// Timestamp.
uint32_t timestamp;
uint16_t reserved3;
uint16_t reserved4;
uint16_t reserved5;
uint16_t reserved6;
#endif /* ENET_ENHANCEDBUFFERDESCRIPTOR_MODE */
} EnetTxBd;

#endif /* SRC_DRIV_ENET_C_H_ */
