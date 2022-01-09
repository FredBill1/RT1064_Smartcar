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

#ifndef _DYN_PRO_TRANSPORT_UART_H_
#define _DYN_PRO_TRANSPORT_UART_H_

#include "DynProtocolTransport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct DynProTransportUart {
	DynProTransportEvent_cb event_cb;
	void *                   event_cb_cookie;


	enum DynProTransportRcvSm {
		RECEIVER_STATE_IDLE   = 0,
		RECEIVER_STATE_SYNC_0 = RECEIVER_STATE_IDLE,
		RECEIVER_STATE_SYNC_1,
		RECEIVER_STATE_SIZE_BYTE_0,
		RECEIVER_STATE_SIZE_BYTE_1,
		RECEIVER_STATE_PACKET_DATA
	} rx_sm_state;
	uint16_t rx_expected_bytes;
	uint16_t rx_received_bytes;
	uint8_t use_tx_dma; 
} DynProTransportUart_t;

typedef struct {
	uint8_t * header;
	uint8_t * payload_data;
	uint16_t payload_len;
	uint16_t max_payload_len;
	uint16_t len;
} DynProTransportUartFrame_t;

void DynProTransportUart_init(DynProTransportUart_t * self,
		DynProTransportEvent_cb event_cb, void * cookie);

void DynProTransportUart_enableTxDma(DynProTransportUart_t * self);

void DynProTransportUart_rxProcessReset(DynProTransportUart_t * self);
int DynProTransportUart_rxProcessByte(DynProTransportUart_t * self, uint8_t rcvByte);

int DynProTransportUart_txSendFrame(DynProTransportUart_t * self, DynProTransportUartFrame_t *frame);

int DynProTransportUart_tx(DynProTransportUart_t * self,
	const uint8_t * buffer, uint16_t size);

int DynProTransportUart_txAssignBuffer(DynProTransportUart_t * self, 
	DynProTransportUartFrame_t * frame, uint8_t * mem_buf, uint16_t buf_size);

int DynProTransportUart_txEncodeFrame(DynProTransportUart_t * self, 
	DynProTransportUartFrame_t * frame);
	
/** @brief Check 4-byte header validity pointed out by rcv_byte
 *  @param[in] rcv_byte	handler to header
 *  @return -1 if header is not valid, number of bytes still to be received otherwise
 */
int DynProTransportUart_checkHeader_fromISR(uint8_t * rcv_byte);

#ifdef __cplusplus
}
#endif

#endif /* _DYN_PRO_TRANSPORT_UART_H_ */