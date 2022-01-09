/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
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

/** @defgroup  VSensorConfig VSensorConfig
 *  @brief 	   Definitions of generic configuration data a VSensor can support
 *	@ingroup VSensorFwk
 *  @{
 */

#ifndef _V_SENSOR_CONFIG_H_
#define _V_SENSOR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "VSensorData.h"

/*
 * Configuration TYPE identifier
 */
#define VSENSOR_CONFIG_TYPE_RESERVED        0  /**< reserved - do not use */
#define VSENSOR_CONFIG_TYPE_REFERENCE_FRAME 1  /**< sensor reference frame (aka 'mouting matrix') */
#define VSENSOR_CONFIG_TYPE_GAIN            2  /**< sensor gain to be applied on sensor data */
#define VSENSOR_CONFIG_TYPE_OFFSET          3  /**< sensor offset to be applied on sensor data */
#define VSENSOR_CONFIG_TYPE_CONTEXT         4  /**< arbitray context buffer */
#define VSENSOR_CONFIG_TYPE_FSR             5  /**< sensor's full scale range */

#define VSENSOR_CONFIG_TYPE_CUSTOM          32 /**< base value to indicate custom config */

#define VSENSOR_CONFIG_TYPE_MAX             64 /**< absolute maximum value for config type */

/*
 * For backward compatibility - should not be used
 */
#define VSENSOR_CONFIG_TYPE_REFERANCE_FRAME VSENSOR_CONFIG_TYPE_REFERENCE_FRAME
 
/*
 * Configuration DATA defintion
 */

/** @brief Base VSensor configuration data structure
 *
 *  Any configuration data structure should inherit from it
 */
typedef struct VSensorConfig {
	uint32_t type; /**< configuration type identifier */
	uint32_t size; /**< configuration data size */
} VSensorConfig;

/** @brief Configuration data for OFFSET identifier
 */
typedef struct {
	VSensorConfig base;    /**< base */
	intq16_t      vect[3]; /**< 3x1 offset data */
} VSensorConfigOffset;

/** @brief Configuration data for GAIN identifier
 */
typedef struct {
	VSensorConfig base;      /**< base */
	intq30_t      matrix[9]; /**< 3x3 matrix gain data */
} VSensorConfigGain;

/** @brief Configuration data for REFERENCE_FRAME identifier
 */
typedef struct {
	VSensorConfig base;      /**< base */
	intq30_t      matrix[9]; /**< 3x3 mountig matrix */
} VSensorConfigReferenceFrame;

/** @brief Configuration data for CONTEXT identifier
 */
typedef struct {
	VSensorConfig base;       /**< base */
	uint8_t       buffer[64]; /**< buffer data */
} VSensorConfigContext;

/** @brief Configuration data for FSR identifier
 */
typedef struct {
	VSensorConfig base;       /**< base */
	int32_t       fsr;        /**< fsr value */
} VSensorConfigFullScaleRange;

#ifdef __cplusplus
}
#endif

#endif /* _V_SENSOR_CONFIG_H_ */

/** @} **/
