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

/** @defgroup DriverAk0991x Ak0991x driver
 *  @brief    Low-level driver for Ak0991x devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_AK0991X_H_
#define _INV_AK0991X_H_

#include "../../../EmbUtils/InvExport.h"
#include "../../../EmbUtils/InvBool.h"
#include "../../../EmbUtils/InvError.h"
#include "Ak0991xSerif.h"

#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AK09911_WHOAMI    0x5
#define AK09912_WHOAMI    0x4
#define AK09915_WHOAMI    0x10

/** @brief Supported auxiliary compass identifer
 */
enum inv_ak0991x_id {
	INV_AK09911_ID,  /**< AKM AK09911 */
	INV_AK09912_ID,  /**< AKM AK09912 */
	INV_AK09915_ID,  /**< AKM AK09915 */
};

/** @brief AK0991x driver states definition
 */
typedef struct inv_ak0991x {
	struct inv_ak0991x_serif serif;
	uint8_t compass_en;
	uint8_t compass_sens[3];
	enum inv_ak0991x_id compass_id;
	uint32_t min_delay_us;
} inv_ak0991x_t;

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 *  @details
 *  When running self-tests, this function is used to measure a duration.
 *  It can also be used as a way of getting the current time.
 */
extern uint64_t inv_ak0991x_get_time_us(void);

/** @brief Hook used for accurate timestamping, called by low-level driver after acquisition is complete
 *  @return timestamp to be reported with the last data
 *  @details
 *  The @see inv_ak0991x_get_time_us function is used to obtain the current time, this
 *  function has the slightly different purpose of marking the end of the sampling very
 *  accurately (relying on timers to achieve this). Host implementation will do the same
 *  as @see inv_ak0991x_get_time_us.
 */
extern uint64_t inv_ak0991x_get_dataready_interrupt_time_us(void);

/** @brief Hook called when magnetometer is started, implemented by upper layer which knows about timers
 *  @param[in] period sensor period in micro-second
 *  @return 0 on success, -1 on failure
 */
extern int inv_ak0991x_start_timer_us(uint32_t period);

/** @brief Hook called when magnetometer is stopped, implemented by upper layer which knows about timers
 *  @return 0 on success, -1 on failure
 */
extern int inv_ak0991x_stop_timer(void);

/** @brief Hook called when magnetometer odr is updated, implemented by upper layer which knows about timers
 *  @param[in] period sensor period in micro-second
 *  @return 0 on success, -1 on failure
 */
extern int inv_ak0991x_reconfigure_timer_us(uint32_t period);

/** @brief Reset and initialize driver states
 *  @param[in] s handle to driver states structure
 *  @param[in] serif handle to SERIF object for underlying register access
 */
static inline void inv_ak0991x_reset_states(struct inv_ak0991x * s,
		const struct inv_ak0991x_serif * serif)
{
	memset(s, 0, sizeof(*s));
	s->serif = *serif;
}

/** @brief Check and retrieve for new data
 *  @param[out] compass_data raw compass data
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_ak0991x_poll_data(struct inv_ak0991x * s, int16_t * compass_data);

/**
*  @brief      Perform hardware self-test for compass.
*  @return     0 on success, negative value on error
*/
int INV_EXPORT inv_ak0991x_run_selftest(struct inv_ak0991x * s);

/** @brief return WHOAMI value
 *  @param[out] whoami WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_ak0991x_get_whoami(struct inv_ak0991x * s, uint8_t * whoami);

/** @brief retrive ASA register values
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_ak0991x_retrieve_asa_values(struct inv_ak0991x * s);

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_ak0991x_soft_reset(struct inv_ak0991x * s);

/** @brief Enables / disables the compass sensor
 * @param[in] enable			0=off, 1=on
 * @return 0 in case of success, negative value on error
 */
int INV_EXPORT inv_ak0991x_enable_sensor(struct inv_ak0991x * s, inv_bool_t en);

#ifdef __cplusplus
}
#endif

#endif /* _INV_AK0991X_H_ */

/** @} */
