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

/** @defgroup DataConverter Data Converter
 *	@brief    Helper functions to convert integer
 *  @ingroup  EmbUtils
 *	@{
 */

#ifndef _INV_DATA_CONVERTER_H_
#define _INV_DATA_CONVERTER_H_

#include "InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Converts a 32-bit long to a little endian byte stream
 */
uint8_t INV_EXPORT * inv_dc_int32_to_little8(int32_t x, uint8_t * little8);

/** @brief Converts a 16-bit integer to a little endian byte stream
 */
uint8_t INV_EXPORT * inv_dc_int16_to_little8(int16_t x, uint8_t * little8);

/** @brief Converts a 32-bit long to a big endian byte stream
  */
uint8_t  INV_EXPORT * inv_dc_int32_to_big8(int32_t x, uint8_t *big8);

/** @brief Converts a 16-bit long to a big endian byte stream
  */
uint8_t  INV_EXPORT * inv_dc_int16_to_big8(int16_t x, uint8_t * big8);

/** @brief Converts a little endian byte stream into a 32-bit integer
 */
int32_t INV_EXPORT inv_dc_little8_to_int32(const uint8_t * little8);

/** @brief Converts a little endian byte stream into a 16-bit integer
 */
int16_t INV_EXPORT inv_dc_le_to_int16(const uint8_t * little8);

/** @brief Converts big endian on 16 bits into an unsigned short
 */
int16_t INV_EXPORT inv_dc_big16_to_int16(uint8_t * data);

/** @brief Converts an array of 32-bit signed fixed-point integers to an array of floats
 *  @param[in]   in    Pointer to the first element of the array of 32-bit signed fixed-point integers
 *  @param[in]   len   Length of the array
 *  @param[in]   qx    Number of bits used to represent the decimal part of the fixed-point integers
 *  @param[out]  out   Pointer to the memory area where the output will be stored
 */
void INV_EXPORT inv_dc_sfix32_to_float(const int32_t * in, uint32_t len, uint8_t qx, float * out);

/** @brief Converts an array of floats to an array of 32-bit signed fixed-point integers
 *  @param[in]   in    Pointer to the first element of the array of floats
 *  @param[in]   len   Length of the array
 *  @param[in]   qx    Number of bits used to represent the decimal part of the fixed-point integers
 *  @param[out]  out   Pointer to the memory area where the output will be stored
 */
void INV_EXPORT inv_dc_float_to_sfix32(const float * in, uint32_t len, uint8_t qx, int32_t * out);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DATA_CONVERTER_H_ */

/** @} */
