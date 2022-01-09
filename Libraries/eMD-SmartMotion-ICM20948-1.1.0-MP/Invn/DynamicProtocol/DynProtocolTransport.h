/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#ifndef _DYN_PRO_TRANSPORT_H_
#define _DYN_PRO_TRANSPORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum DynProTransportEvent {
	DYN_PRO_TRANSPORT_EVENT_ERROR,
	DYN_PRO_TRANSPORT_EVENT_PKT_SIZE,
	DYN_PRO_TRANSPORT_EVENT_PKT_BYTE,
	DYN_PRO_TRANSPORT_EVENT_PKT_END,
	DYN_PRO_TRANSPORT_EVENT_TX_START,
	DYN_PRO_TRANSPORT_EVENT_TX_BYTE,
	DYN_PRO_TRANSPORT_EVENT_TX_END,
	DYN_PRO_TRANSPORT_EVENT_TX_START_DMA,
};

union DynProTransportEventData {
	int error;
	uint16_t pkt_size;
	uint8_t  pkt_byte;
	uint32_t tx_start;
	uint8_t  tx_byte;
	void *frame;
};

typedef void (*DynProTransportEvent_cb)(enum DynProTransportEvent e,
	union DynProTransportEventData data, void * cookie);

#ifdef __cplusplus
}
#endif

#endif /* _DYN_PRO_TRANSPORT_H_ */
