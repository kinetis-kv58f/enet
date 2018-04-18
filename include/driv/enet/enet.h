/*
 * enet.h
 *
 *  Created on: 11 apr. 2018
 *      Author: SERAGUD
 */

#ifndef INCLUDE_DRIV_ENET_ENET_H_
#define INCLUDE_DRIV_ENET_ENET_H_

#include "types.h"
#include "twin.h"

#ifdef __cplusplus
extern "C" {
#endif

static const uint8_t broadcastAddr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

#if MY_TWIN == TWIN_A
/// The MAC address for ENET device.
static const uint8_t macAddr[6] = { 0x64, 0xbe, 0xd9, 0x45, 0x22, 0x60 };
static const uint8_t macAddrOth[6] = { 0x65, 0xbe, 0xd9, 0x45, 0x22, 0x60 };
#elif MY_TWIN == TWIN_B
/// The MAC address for ENET device.
static const uint8_t macAddr[6] = { 0x65, 0xbe, 0xd9, 0x45, 0x22, 0x60 };
static const uint8_t macAddrOth[6] = { 0x64, 0xbe, 0xd9, 0x45, 0x22, 0x60 };
#else
#error "Invalid TWIN config!"
#endif

bool_t enet_init(void);

bool_t enet_sendFrame(uint8_t * data, uint16_t length);

bool_t enet_receiveFrame();

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_DRIV_ENET_ENET_H_ */
