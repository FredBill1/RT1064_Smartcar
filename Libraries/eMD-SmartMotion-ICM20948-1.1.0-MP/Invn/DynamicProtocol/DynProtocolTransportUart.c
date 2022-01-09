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

#include "DynProtocolTransportUart.h"

#include "EmbUtils/Message.h"

#include <stdint.h>
#include <string.h>

/** TX/RX frames have the following format: 0x55 0xAA <NB_BYTES (2)> <PACKET (n)> */

#define SYNC_BYTE_0                         0x55
#define SYNC_BYTE_1                         0xAA
#define DYN_PRO_TRANSPORT_UART_OVERHEAD     4

static inline void DynProTransportUart_callEventCB(DynProTransportUart_t * self, 
		enum DynProTransportEvent event,
		union DynProTransportEventData data)
{
	if(self->event_cb) {
		self->event_cb(event, data, self->event_cb_cookie);
	}
}

void DynProTransportUart_init(DynProTransportUart_t * self,
		DynProTransportEvent_cb event_cb, void * cookie)
{
	self->event_cb        = event_cb;
	self->event_cb_cookie = cookie;
	self->rx_sm_state     = RECEIVER_STATE_IDLE;
	self->use_tx_dma        = 0;
}


/** @brief This function forces protocol to use DMA when transfering data on UART
 *
 * DMA is unused by default at startup. Once enabled DMA can not be disabled
 * until next POR.
 *
 *  @note : rx data is not impacted. Only tx data.
 *  @note : This function should be called after DynProTransportUart_init function. It was 
 *          not merged with DynProTransportUart_init() to avoid any API break.
 *
 * @param[in] self pointer on current DynProTransportUart_t transport object
 *
 */
void DynProTransportUart_enableTxDma(DynProTransportUart_t * self)
{
	self->use_tx_dma = 1;
}

void DynProTransportUart_rxProcessReset(DynProTransportUart_t * self)
{
	self->rx_sm_state = RECEIVER_STATE_IDLE;
}

int DynProTransportUart_checkHeader_fromISR(uint8_t * rcv_byte)
{
	uint16_t expected_size;
	
	if (rcv_byte[0] != SYNC_BYTE_0)
		return -1;
	if (rcv_byte[1] != SYNC_BYTE_1)
		return -1;
	
	expected_size = (uint16_t)rcv_byte[2] | ((uint16_t)rcv_byte[3] << 8U);
	
	if (expected_size > 128) // Arbitrary reasonable value to allow very quick check
		return -1;
	else
		return expected_size;
		
}

int DynProTransportUart_rxProcessByte(DynProTransportUart_t * self, uint8_t rcv_byte)
{
	union DynProTransportEventData udata;

	switch(self->rx_sm_state) {
	case RECEIVER_STATE_IDLE:
		if(rcv_byte == SYNC_BYTE_0) {
			self->rx_sm_state = RECEIVER_STATE_SYNC_1;
		}
		else {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "DynProTransportUart: unexpected SYNC0 byte %x recevied", rcv_byte);
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.error = -1;
			DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_ERROR, udata);
			return -1;
		}
		break;

	case RECEIVER_STATE_SYNC_1:
		if(rcv_byte == SYNC_BYTE_1) {
			self->rx_sm_state = RECEIVER_STATE_SIZE_BYTE_0;
		}
		else {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "DynProTransportUart: unexpected SYNC1 byte %x recevied", rcv_byte);
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.error = -1;
			DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_ERROR, udata);
			return -1;
		}
		break;

	case RECEIVER_STATE_SIZE_BYTE_0:
		self->rx_expected_bytes = (uint16_t)rcv_byte;
		self->rx_sm_state = RECEIVER_STATE_SIZE_BYTE_1;
		break;

	case RECEIVER_STATE_SIZE_BYTE_1:
		self->rx_expected_bytes |= ((uint16_t)rcv_byte << 8U);
		self->rx_received_bytes = 0;
		self->rx_sm_state = RECEIVER_STATE_PACKET_DATA;
		udata.pkt_size = self->rx_expected_bytes;
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_PKT_SIZE, udata);
		return 1;

	case RECEIVER_STATE_PACKET_DATA:
		self->rx_received_bytes++;
		udata.pkt_size = rcv_byte;
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_PKT_BYTE, udata);
		if(self->rx_received_bytes == self->rx_expected_bytes) {
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.pkt_size = self->rx_received_bytes;
			DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_PKT_END, udata);
		}
		return 1;
	}

	return 0;
}


/** @brief This function is used to send a frame on UART.
 *
 * To keep the protocol hardware independant, this function does not call any UART API. It just sends
 * some events to the transport callback. Transport callback is in charge of calling UART driver.
 * 
 * If DMA is enabled, function sends DYN_PRO_TRANSPORT_EVENT_TX_START_DMA event and frame pointer
 * to transport callback 
 * If DMA is disabled, function sends :
 *    - DYN_PRO_TRANSPORT_EVENT_TX_START event and frame size in bytes to the transport callback
 *    - DYN_PRO_TRANSPORT_EVENT_TX_BYTE event for each byte of the frame
 *    - DYN_PRO_TRANSPORT_EVENT_TX_END event and frame pointer to transport callback  
 *  
 * Finnally it sends DYN_PRO_TRANSPORT_EVENT_TX_END to the transport callback
 * 
 * @param[in] self : pointer on current DynProTransportUart_t transport object
 * @param[in] frame : pointer to frmae descriptor to be sent
 *
 * @return      0 on sucess, negative value on error
 */
int DynProTransportUart_txSendFrame(DynProTransportUart_t * self, DynProTransportUartFrame_t *frame)
{
	union DynProTransportEventData udata;
	uint16_t i;

	if(self->use_tx_dma) {
		udata.frame = (void*)frame;
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_START_DMA, udata);
	}
	else {
		udata.tx_start = frame->len;
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_START, udata);

		for(i = 0; i < frame->len; ++i) {
			udata.tx_byte = ((uint8_t*)frame->header)[i];
			DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);
		}
		
		// send frame pointer to callback in case some memory need to be free
		udata.frame = (void*)frame;
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_END, udata);
	}
	
	return 0;
}

/** @deprecated Deprecated function. Consider using DynProTransportUart_txSendFrame() instead.
 * 
 * @brief This function is used to send a buffer byte after byte on UART.
 *
 * To keep the protocol hardware independant, this function does not call any UART API. It just sends
 * some events to the transport callback. Transport callback is in charge of calling UART driver.
 * 
 * Function starts by sending DYN_PRO_TRANSPORT_EVENT_TX_START event to transport callback.
 * Then it sends DYN_PRO_TRANSPORT_EVENT_TX_BYTE event to the transport callback for each byte of :
 *    - the 4 bytes transport layer header
 *    - buffer given in paramters
 *  
 * Finnally it sends DYN_PRO_TRANSPORT_EVENT_TX_END to the transport callback
 * 
 * @param[in] self : pointer on current DynProTransportUart_t transport object
 * @param[in] buffer : pointer to the first byte to be sent
 * @param[in] size : number of bytes to be sent
 *
 * @return      0 on sucess, negative value on error
 */
int DynProTransportUart_tx(DynProTransportUart_t * self,
	const uint8_t * buffer, uint16_t size)
{
	union DynProTransportEventData udata;
	const uint32_t total_bytes = DYN_PRO_TRANSPORT_UART_OVERHEAD + size;
	uint16_t i;

	udata.tx_start = total_bytes;
	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_START, udata);
	udata.tx_byte = SYNC_BYTE_0;
	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = SYNC_BYTE_1;
	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = (size & 0x00FF);
	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = (size & 0xFF00) >> 8;
	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);

	for(i = 0; i < size; ++i) {
		udata.tx_byte = buffer[i];
		DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_BYTE, udata);
	}

	DynProTransportUart_callEventCB(self, DYN_PRO_TRANSPORT_EVENT_TX_END, udata);

	return 0;
}

int DynProTransportUart_txAssignBuffer(DynProTransportUart_t * self, 
	DynProTransportUartFrame_t * frame, uint8_t * mem_buf, uint16_t buf_size)
{
	(void)self;

	/* sanity check */
	if(buf_size < DYN_PRO_TRANSPORT_UART_OVERHEAD){
		return -1;
	}

	frame->header = mem_buf;
	frame->payload_data = mem_buf + DYN_PRO_TRANSPORT_UART_OVERHEAD;
	frame->payload_len = buf_size;
	frame->max_payload_len = buf_size - DYN_PRO_TRANSPORT_UART_OVERHEAD;
	frame->len = 0;

	return 0;
}

int DynProTransportUart_txEncodeFrame(DynProTransportUart_t * self, 
	DynProTransportUartFrame_t * frame)
{
	(void)self;

	/* sanity check */
	if((frame->payload_len <= 0) || (frame->payload_data != (frame->header + DYN_PRO_TRANSPORT_UART_OVERHEAD))) {
		return -1;
	}

	frame->header[0] = SYNC_BYTE_0;
	frame->header[1] = SYNC_BYTE_1;
	frame->header[2] = (frame->payload_len & 0x00FF);
	frame->header[3] = (frame->payload_len & 0xFF00) >> 8;

	frame->len = frame->payload_len + DYN_PRO_TRANSPORT_UART_OVERHEAD;

	return 0;
}
