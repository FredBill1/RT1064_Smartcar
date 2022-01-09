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

#include "DataConverter.h"

uint8_t * inv_dc_int32_to_little8(int32_t x, uint8_t * little8)
{
	little8[3] = (uint8_t)((x >> 24) & 0xff);
	little8[2] = (uint8_t)((x >> 16) & 0xff);
	little8[1] = (uint8_t)((x >> 8) & 0xff);
	little8[0] = (uint8_t)(x & 0xff);

	return little8;
}

uint8_t * inv_dc_int16_to_little8(int16_t x, uint8_t * little8)
{
	little8[0] = (uint8_t)(x & 0xff);
	little8[1] = (uint8_t)((x >> 8) & 0xff);

	return little8;
}

uint8_t * inv_dc_int32_to_big8(int32_t x, uint8_t * big8)
{
	big8[0] = (uint8_t)((x >> 24) & 0xff);
	big8[1] = (uint8_t)((x >> 16) & 0xff);
	big8[2] = (uint8_t)((x >> 8) & 0xff);
	big8[3] = (uint8_t)(x & 0xff);

	return big8;
}

uint8_t * inv_dc_int16_to_big8(int16_t x, uint8_t * big8)
{
	big8[0] = (uint8_t)((x >> 8) & 0xff);
	big8[1] = (uint8_t)(x & 0xff);

	return big8;
}

int32_t inv_dc_little8_to_int32(const uint8_t * little8)
{
	int32_t x = 0;

	x |= ((int32_t)little8[3] << 24);
	x |= ((int32_t)little8[2] << 16);
	x |= ((int32_t)little8[1] << 8);
	x |= ((int32_t)little8[0]);

	return x;
}

int16_t inv_dc_big16_to_int16(uint8_t * data)
{
	int16_t result;

	result  = (*data << 8);
	data++;
	result |= *data;

	return result;
}

int16_t inv_dc_le_to_int16(const uint8_t * little8)
{
	uint16_t x = 0;

	x |= ((uint16_t)little8[0]);
	x |= ((uint16_t)little8[1] << 8);

	return (int16_t)x;
}

void inv_dc_sfix32_to_float(const int32_t * in, uint32_t len, uint8_t qx, float * out)
{
	uint8_t i;

	for(i = 0; i < len; ++i) {
		out[i] = (float)in[i] / (1 << qx);
	}
}

void inv_dc_float_to_sfix32(const float * in, uint32_t len, uint8_t qx, int32_t * out)
{
	uint8_t i;

	for(i = 0; i < len; ++i) {
		out[i] = (int32_t)((in[i] * (1 << qx)) + ((in[i] >= 0) - 0.5f));
	}
}
