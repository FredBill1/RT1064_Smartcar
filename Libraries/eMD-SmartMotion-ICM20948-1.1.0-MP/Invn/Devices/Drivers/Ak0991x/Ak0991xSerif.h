/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

/** @defgroup DriverAk0991xSerif Ak0991x driver serif
 *  @brief Interface for low-level serial (I2C/SPI) access
 *  @ingroup  DriverAk0991x
 *  @{
 */

#ifndef _INV_AK0991X_SERIF_H_
#define _INV_AK0991X_SERIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../../../EmbUtils/InvBool.h"
#include "../../../EmbUtils/InvError.h"

#include <stdint.h>
#include <assert.h>

/** @brief Ak0991x serial interface
 */
struct inv_ak0991x_serif {
	void *     context;
	int      (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len);
	int      (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len);
	uint32_t   max_read;
	uint32_t   max_write;
	inv_bool_t is_spi;
};

static inline inv_bool_t inv_ak0991x_serif_is_spi(struct inv_ak0991x_serif * s)
{
	assert(s);

	return s->is_spi;
}

static inline uint32_t inv_ak0991x_serif_max_read(struct inv_ak0991x_serif * s)
{
	assert(s);

	return s->max_read;
}

static inline uint32_t inv_ak0991x_serif_max_write(struct inv_ak0991x_serif * s)
{
	assert(s);

	return s->max_write;
}

static inline int inv_ak0991x_serif_read_reg(struct inv_ak0991x_serif * s,
		uint8_t reg, uint8_t * buf, uint32_t len)
{
	assert(s);

	if(len > s->max_read)
		return INV_ERROR_SIZE;

	if(s->read_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

static inline int inv_ak0991x_serif_write_reg(struct inv_ak0991x_serif * s,
		uint8_t reg, const uint8_t * buf, uint32_t len)
{
	assert(s);

	if(len > s->max_write)
		return INV_ERROR_SIZE;

	if(s->write_reg(s->context, reg, buf, len) != 0)
		return INV_ERROR_TRANSPORT;

	return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_AK0991X_SERIF_H_ */

/** @} */
