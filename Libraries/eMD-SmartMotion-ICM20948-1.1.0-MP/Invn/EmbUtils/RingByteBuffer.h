/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
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

/** @defgroup RingByteBuffer RingByteBuffer
	@brief Function to mannage circular buffer of bytes
	@ingroup EmbUtils
	@{
*/

#ifndef _RING_BYTE_BUFFER_H_
#define _RING_BYTE_BUFFER_H_

#include <stdint.h>

#include "InvBool.h"
#include "InvAssert.h"


/** @brief 	RingByteBuffer object definitions
*/
typedef struct {
	uint8_t 	*buffer; 	/**< pointer to ring buffer data placeholder */
	uint16_t 	size;		/**< maximum size of the data buffer */
	uint16_t 	start;		/**< current start index  */
	uint16_t 	end;		/**< current end index  */
	uint8_t 	msbStart;	/**< current msb value for start index */
	uint8_t 	msbEnd;		/**< current msb value for end index */
} RingByteBuffer;

/** @brief 		Initialize and reset a ring buffer
	@param[in]	pBuffer		pointer to buffer placeholder
	@param[in]	sizeBuffer	size of buffer placeholder
	@return none
*/
void RingByteBuffer_init(RingByteBuffer *self, uint8_t *pBuffer,
                         uint16_t sizeBuffer);

/** @brief 		Clear a ring buffer
	@return 	none
*/
void RingByteBuffer_clear(RingByteBuffer *self);

/** @brief 		Get maximum size of a ring buffer
	@return 	Return maximum size of the ring buffer
*/
static inline uint16_t RingByteBuffer_maxSize(const RingByteBuffer *self)
{
	ASSERT(self);

	return self->size;
}

/** @brief 		Get current size of a ring buffer
	@return 	Return number of byte contained in the ring buffer
*/
uint16_t RingByteBuffer_size(const RingByteBuffer *self);

/** @brief 		Get number of empty slot of a ring buffer
	@return 	Return number of byte that can be stored in the ring buffer
*/
static inline uint16_t RingByteBuffer_available(const RingByteBuffer *self)
{
	ASSERT(self);

	return (RingByteBuffer_maxSize(self) - RingByteBuffer_size(self));
}

/** @brief 		Check for ring buffer fullness
	@return 	Return true if ring buffer is full, false otherwise
*/
static inline inv_bool_t RingByteBuffer_isFull(const RingByteBuffer *self)
{
	ASSERT(self);

	return (self->end == self->start && self->msbEnd != self->msbStart);
}

/** @brief 		Check for ring buffer emptyness
	@return 	Return true if ring buffer is empty, false otherwise
*/
static inline inv_bool_t RingByteBuffer_isEmpty(const RingByteBuffer *self)
{
	ASSERT(self);

	return (self->end == self->start && self->msbEnd == self->msbStart);
}

/** @brief 		Push a byte to a ring buffer
				Fullness test must be done by the caller
	@param[in] 	byte 	byte to push to the ring buffer
	@return 	none
*/
void RingByteBuffer_pushByte(RingByteBuffer *self, uint8_t byte);

/** @brief 		Pop a byte from a ring buffer
				Emptyness test must be done by the caller
	@return 	Byte pop from the ring buffer
*/
uint8_t RingByteBuffer_popByte(RingByteBuffer *self);

/** @brief 		Push a buffer of data to a ring buffer
				Check for available size must be done by the caller
	@param[in] 	data 	pointer to data to push to the ring buffer
	@param[in] 	size  	size of data to push to the ring buffer
	@return 	none
*/
void RingByteBuffer_pushBuffer(RingByteBuffer *self, const void *data,
                               uint16_t len);

/** @brief 		Pop a buffer of data to a ring buffer
				Check for size of the ring buffer must be done by the caller
	@param[in] 	data 	pointer to placeholder
	@param[in] 	size  	size of data to pop to the ring buffer
	@return 	none
*/
void RingByteBuffer_popBuffer(RingByteBuffer *self, void *data, uint16_t len);

/** @brief 		Return front item of a ring buffer
				Check for size of the ring buffer must be done by the caller
	@return 	front item of the buffer
*/
static inline uint8_t RingByteBuffer_front(const RingByteBuffer *self)
{
	ASSERT(self);

	return self->buffer[self->start];
}

#endif

/** @} */
