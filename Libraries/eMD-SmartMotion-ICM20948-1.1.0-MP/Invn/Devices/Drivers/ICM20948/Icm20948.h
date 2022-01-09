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

/** @defgroup DriverIcm20948 Icm20948 driver
 *  @brief    Low-level driver for ICM20948 devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICM20948_H_
#define _INV_ICM20948_H_

#include "../../../EmbUtils/InvExport.h"
#include "../../../EmbUtils/InvBool.h"
#include "../../../EmbUtils/InvError.h"


#include "Icm20948Setup.h"
#include "Icm20948Serif.h"
#include "Icm20948Transport.h"
#include "Icm20948DataConverter.h"
#include "Icm20948AuxCompassAkm.h"
#include "Icm20948SelfTest.h"


#include <stdint.h>
#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief States for the secondary device
 */
typedef enum inv_icm20948_compass_state
{
	INV_ICM20948_COMPASS_RESET = 0,
	INV_ICM20948_COMPASS_INITED,
	INV_ICM20948_COMPASS_SETUP,
}inv_icm20948_compass_state_t;

/** @brief ICM20948 driver states definition
 */
typedef struct sensor_type_icm20948{
	uint64_t odr_applied_us;
	uint64_t odr_us;
}sensor_type_icm20948_t;

typedef enum {
	CHIP_LOW_NOISE_ICM20948,
	CHIP_LOW_POWER_ICM20948,
}chip_lp_ln_mode_icm20948_t;

typedef struct inv_icm20948 {
	struct inv_icm20948_serif serif;
	/** @brief struct for the base_driver : this contains the Mems information */
	struct base_driver_t
	{
		unsigned char wake_state;
		chip_lp_ln_mode_icm20948_t chip_lp_ln_mode;
		unsigned char pwr_mgmt_1;
		unsigned char pwr_mgmt_2;
		unsigned char user_ctrl;
		unsigned char gyro_div;
		unsigned short secondary_div;
		short accel_div;
		unsigned char gyro_averaging;
		unsigned char accel_averaging;
		uint8_t gyro_fullscale; 
		uint8_t accel_fullscale;
		uint8_t lp_en_support:1;
		uint8_t firmware_loaded:1;
		uint8_t serial_interface;
		uint8_t timebase_correction_pll;
	}base_state;
	/* secondary device support */
	struct inv_icm20948_secondary_states {
		struct inv_icm20948_secondary_reg {
			uint16_t addr;
			uint16_t reg;
			uint16_t ctrl;
			uint16_t d0;
		} slv_reg[4];
		unsigned char sSavedI2cOdr;
		/* compass support */
		uint8_t compass_sens[3];
		long final_matrix[9];
		const int16_t *st_upper;
		const int16_t *st_lower;
		int scale;
		uint8_t dmp_on;
		uint8_t secondary_resume_compass_state;
		uint8_t mode_reg_addr;
		int compass_chip_addr;
		int compass_slave_id;
		inv_icm20948_compass_state_t compass_state;
	} secondary_state;
	/* self test */
	uint8_t selftest_done;
	uint8_t offset_done;
	uint8_t gyro_st_data[3];
	uint8_t accel_st_data[3];
	/* mpu fifo control */
	struct fifo_info_t
	{
		int fifoError;
		unsigned char fifo_overflow;
	} fifo_info;
	/* interface mapping */
	unsigned long sStepCounterToBeSubtracted;
	unsigned long sOldSteps;
	/* data converter */
	long s_quat_chip_to_body[4];
	/* base driver */
	uint8_t sAllowLpEn;
	uint8_t s_compass_available;
	uint8_t s_proximity_available;
	/* base sensor ctrl*/
	unsigned short inv_dmp_odr_dividers[37];//INV_SENSOR_NUM_MAX /!\ if the size change 
	unsigned short inv_dmp_odr_delays[37];//INV_SENSOR_NUM_MAX /!\ if the size change
	unsigned short bac_on; // indicates if ANDROID_SENSOR_ACTIVITY_CLASSIFICATON is on
	unsigned short pickup;
	unsigned short bac_status;
	unsigned short b2s_status;
	unsigned short flip_pickup_status;
	unsigned short inv_sensor_control;
	unsigned short inv_sensor_control2;
	unsigned long inv_androidSensorsOn_mask[2] ;// Each bit corresponds to a sensor being on
	unsigned short inv_androidSensorsOdr_boundaries[51][2];//GENERAL_SENSORS_MAX /!\ if the size change 
	unsigned char sGmrvIsOn; // indicates if GMRV was requested to be ON by end-user. Once this variable is set, it is either GRV or GMRV which is enabled internally
	unsigned short lLastHwSmplrtDividerAcc;
	unsigned short lLastHwSmplrtDividerGyr;
	unsigned char sBatchMode;
	uint8_t header2_count;
	char mems_put_to_sleep;
	unsigned short smd_status;
	unsigned short ped_int_status;
	unsigned short bac_request;
	uint8_t go_back_lp_when_odr_low; // set to 1 when we forced a switch from LP to LN mode to be able to reach 1kHz ODR, so we will need to go back to LP mode ASAP
	unsigned short odr_acc_ms; // ODR in ms requested for ANDROID_SENSOR_ACCELEROMETER
	//unsigned short odr_acc_wom_ms; // ODR in ms requested for ANDROID_SENSOR_WOM when using ACC
	unsigned short odr_racc_ms; // ODR in ms requested for ANDROID_SENSOR_RAW_ACCELEROMETER
	unsigned short odr_gyr_ms; // ODR in ms requested for ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED
	unsigned short odr_rgyr_ms; // ODR in ms requested for ANDROID_SENSOR_RAW_GYROSCOPE
	int bias[9];// dmp bias [0-2]:acc,[3-5]:gyr,[6-8]:mag
	/* Icm20948Fifo usage */
	signed char mounting_matrix[9];
	signed char mounting_matrix_secondary_compass[9];
	long soft_iron_matrix[9];
	uint8_t skip_sample[INV_ICM20948_SENSOR_MAX+1];
	uint64_t timestamp[INV_ICM20948_SENSOR_MAX+1];
	uint8_t sFirstBatch[INV_ICM20948_SENSOR_MAX+1];
	sensor_type_icm20948_t sensorlist[INV_ICM20948_SENSOR_MAX+1];
	unsigned short saved_count;
	/* Icm20948Transport*/
	unsigned char reg;
	unsigned char lastBank;
	unsigned char lLastBankSelected;
	/* augmented sensors*/
	unsigned short sGravityOdrMs;
	unsigned short sGrvOdrMs;
	unsigned short sLinAccOdrMs;
	unsigned short sGravityWuOdrMs;
	unsigned short sGrvWuOdrMs;
	unsigned short sLinAccWuOdrMs;
	unsigned short sRvOdrMs;
	unsigned short sOriOdrMs;
	unsigned short sRvWuOdrMs;
	unsigned short sOriWuOdrMs;
	/* Icm20649Setup */
	short set_accuracy;
	int new_accuracy;
} inv_icm20948_t;

/** @brief ICM20948 driver states singleton declaration
 *  Because of Low-level driver limitation only one insance of the driver is allowed
 */
extern struct inv_icm20948 * icm20948_instance;

/** @brief Hook for low-level system sleep() function to be implemented by upper layer
 *  @param[in] ms number of millisecond the calling thread should sleep
 */
extern void inv_icm20948_sleep_us(int us);

/** @brief Hook for low-level system time() function to be implemented by upper layer
 *  @return monotonic timestamp in us
 */
extern uint64_t inv_icm20948_get_time_us(void);

/** @brief Reset and initialize driver states
 *  @param[in] s             handle to driver states structure
 */
static inline void inv_icm20948_reset_states(struct inv_icm20948 * s,
		const struct inv_icm20948_serif * serif)
{
	assert(icm20948_instance == 0);

	memset(s, 0, sizeof(*s));
	s->serif = *serif;
	icm20948_instance = s;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20948_H_ */

/** @} */
