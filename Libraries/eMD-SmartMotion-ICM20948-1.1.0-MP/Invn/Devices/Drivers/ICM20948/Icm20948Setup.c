/** ________________________________________________________________________________________________________
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
#include "../../../Devices/SensorTypes.h"

#include "Icm20948Setup.h"
#include "Icm20948.h"
#include "Icm20948Defs.h"
#include "Icm20948DataBaseDriver.h"
#include "Icm20948DataBaseControl.h"
#include "Icm20948MPUFifoControl.h"
#include "Icm20948Augmented.h"
#include "Icm20948LoadFirmware.h"
#include "Icm20948Dmp3Driver.h"

#include "../../../EmbUtils/DataConverter.h"
#include "../../../EmbUtils/Message.h"

#include <assert.h>

/** @brief Set of flags for BAC state */
#define BAC_DRIVE   0x01
#define BAC_WALK    0x02
#define BAC_RUN     0x04
#define BAC_BIKE    0x08
#define BAC_TILT    0x10
#define BAC_STILL   0x20

/** @brief Conversion from DMP units to float format for compass scale */
#define DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION      (1/(float)(1UL<<16))
//! Convert the \a value from QN value to float. \ingroup invn_macro
#define INVN_FXP_TO_FLT(value, shift)	( (float)  (int32_t)(value) / (float)(1ULL << (shift)) )
static uint8_t sensor_type_2_android_sensor(enum inv_icm20948_sensor sensor)
{
	switch(sensor) {
	case INV_ICM20948_SENSOR_ACCELEROMETER:                 return ANDROID_SENSOR_ACCELEROMETER;
	case INV_ICM20948_SENSOR_GYROSCOPE:                     return ANDROID_SENSOR_GYROSCOPE;
	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:             return ANDROID_SENSOR_RAW_ACCELEROMETER;
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:                 return ANDROID_SENSOR_RAW_GYROSCOPE;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:   return ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:        return ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:        return ANDROID_SENSOR_ACTIVITY_CLASSIFICATON;
	case INV_ICM20948_SENSOR_STEP_DETECTOR:                 return ANDROID_SENSOR_STEP_DETECTOR;
	case INV_ICM20948_SENSOR_STEP_COUNTER:                  return ANDROID_SENSOR_STEP_COUNTER;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:          return ANDROID_SENSOR_GAME_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:               return ANDROID_SENSOR_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:   return ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:             return ANDROID_SENSOR_GEOMAGNETIC_FIELD;
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:     return ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:                   return ANDROID_SENSOR_FLIP_PICKUP;
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:          return ANDROID_SENSOR_WAKEUP_TILT_DETECTOR;
	case INV_ICM20948_SENSOR_GRAVITY:                       return ANDROID_SENSOR_GRAVITY;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:           return ANDROID_SENSOR_LINEAR_ACCELERATION;
	case INV_ICM20948_SENSOR_ORIENTATION:                   return ANDROID_SENSOR_ORIENTATION;
	case INV_ICM20948_SENSOR_B2S:                           return ANDROID_SENSOR_B2S;
	default:                                                return ANDROID_SENSOR_NUM_MAX;
	}
}

enum inv_icm20948_sensor inv_icm20948_sensor_android_2_sensor_type(int sensor)
{
	switch(sensor) {
	case ANDROID_SENSOR_ACCELEROMETER:                    return INV_ICM20948_SENSOR_ACCELEROMETER;
	case ANDROID_SENSOR_GYROSCOPE:                        return INV_ICM20948_SENSOR_GYROSCOPE;
	case ANDROID_SENSOR_RAW_ACCELEROMETER:                return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
	case ANDROID_SENSOR_RAW_GYROSCOPE:                    return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
	case ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:      return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
	case ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED:           return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
	case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON:           return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
	case ANDROID_SENSOR_STEP_DETECTOR:                    return INV_ICM20948_SENSOR_STEP_DETECTOR;
	case ANDROID_SENSOR_STEP_COUNTER:                     return INV_ICM20948_SENSOR_STEP_COUNTER;
	case ANDROID_SENSOR_GAME_ROTATION_VECTOR:             return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
	case ANDROID_SENSOR_ROTATION_VECTOR:                  return INV_ICM20948_SENSOR_ROTATION_VECTOR;
	case ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:      return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
	case ANDROID_SENSOR_GEOMAGNETIC_FIELD:                return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
	case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION:        return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
	case ANDROID_SENSOR_FLIP_PICKUP:                      return INV_ICM20948_SENSOR_FLIP_PICKUP;
	case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR:             return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
	case ANDROID_SENSOR_GRAVITY:                          return INV_ICM20948_SENSOR_GRAVITY;
	case ANDROID_SENSOR_LINEAR_ACCELERATION:              return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
	case ANDROID_SENSOR_ORIENTATION:                      return INV_ICM20948_SENSOR_ORIENTATION;
	case ANDROID_SENSOR_B2S:                              return INV_ICM20948_SENSOR_B2S;
	default:                                              return INV_ICM20948_SENSOR_MAX;
	}
}

static int skip_sensor(struct inv_icm20948 * s, unsigned char androidSensor)
{
	enum inv_icm20948_sensor icm20948_sensor_id = inv_icm20948_sensor_android_2_sensor_type(androidSensor);
	uint8_t skip_sample = s->skip_sample[icm20948_sensor_id];

	if (s->skip_sample[icm20948_sensor_id])
		s->skip_sample[icm20948_sensor_id]--;

	return skip_sample;
}

/* Identification related functions */
int inv_icm20948_get_whoami(struct inv_icm20948 * s, uint8_t * whoami)
{
	return inv_icm20948_read_reg_one(s, REG_WHO_AM_I, whoami);
}

void inv_icm20948_init_matrix(struct inv_icm20948 * s)
{
	// initialize chip to body
	s->s_quat_chip_to_body[0] = (1L<<30);
	s->s_quat_chip_to_body[1] = 0;
	s->s_quat_chip_to_body[2] = 0;
	s->s_quat_chip_to_body[3] = 0;
	//initialize mounting matrix
	memset(s->mounting_matrix, 0, sizeof(s->mounting_matrix));
	s->mounting_matrix[0] = 1;
	s->mounting_matrix[4] = 1;
	s->mounting_matrix[8] = 1;
	//initialize soft iron matrix
	s->soft_iron_matrix[0] = (1L<<30);
	s->soft_iron_matrix[4] = (1L<<30);
	s->soft_iron_matrix[8] = (1L<<30);

	inv_icm20948_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);
}

int inv_icm20948_init_structure(struct inv_icm20948 * s)
{
	int i;
	inv_icm20948_base_control_init(s);
	inv_icm20948_transport_init(s);
	inv_icm20948_augmented_init(s);
	//Init state
	s->set_accuracy = 0;
	s->new_accuracy = 0;
	for(i = 0; i < GENERAL_SENSORS_MAX; i ++)
		s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] = 0;

	return 0;
}

int inv_icm20948_initialize(struct inv_icm20948 * s, const uint8_t *dmp3_image, uint32_t dmp3_image_size)
{
	if(s->serif.is_spi) {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20948_initialize_lower_driver(s, SERIAL_INTERFACE_SPI, dmp3_image, dmp3_image_size)) {
			return -1;
		}
	}
	else {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20948_initialize_lower_driver(s, SERIAL_INTERFACE_I2C, dmp3_image, dmp3_image_size)) {
			return -1;
		}
	}
	return 0;
}

int inv_icm20948_init_scale(struct inv_icm20948 * s)
{
	/* Force accelero fullscale to 4g and gyr to 200dps */
	inv_icm20948_set_accel_fullscale(s, MPU_FS_4G);
	inv_icm20948_set_gyro_fullscale(s, MPU_FS_2000dps);

	return 0;
}

int inv_icm20948_set_fsr(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, const void * fsr)
{
	int result = 0;
	int * castedvalue = (int*) fsr;
	if((sensor == INV_ICM20948_SENSOR_RAW_ACCELEROMETER) ||
		(sensor == INV_ICM20948_SENSOR_ACCELEROMETER)){
			enum mpu_accel_fs afsr;
			if(*castedvalue == 2)
				afsr = MPU_FS_2G;
			else if(*castedvalue == 4)
				afsr = MPU_FS_4G;
			else if(*castedvalue == 8)
				afsr = MPU_FS_8G;
			else if(*castedvalue == 16)
				afsr = MPU_FS_16G;
			else
				return -1;
			result |= inv_icm20948_set_accel_fullscale(s, afsr);
	}
	else if((sensor == INV_ICM20948_SENSOR_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_RAW_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED)) {
			enum mpu_gyro_fs gfsr;
			if(*castedvalue == 250)
				gfsr = MPU_FS_250dps;
			else if(*castedvalue == 500)
				gfsr = MPU_FS_500dps;
			else if(*castedvalue == 1000)
				gfsr = MPU_FS_1000dps;
			else if(*castedvalue == 2000)
				gfsr = MPU_FS_2000dps;
			else
				return -1;
			result |= inv_icm20948_set_gyro_fullscale(s, gfsr);
	}
	return result;
}

int inv_icm20948_get_fsr(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, const void * fsr)
{

	if((sensor == INV_ICM20948_SENSOR_RAW_ACCELEROMETER) ||
	(sensor == INV_ICM20948_SENSOR_ACCELEROMETER)){
		unsigned char * castedvalue = (unsigned char*) fsr;
		int afsr = inv_icm20948_get_accel_fullscale(s);
		if(afsr == MPU_FS_2G)
			* castedvalue = 2;
		else if(afsr == MPU_FS_4G)
			* castedvalue = 4;
		else if(afsr == MPU_FS_8G)
			* castedvalue = 8;
		else if(afsr == MPU_FS_16G)
			* castedvalue = 16;
		else
			return -1;

		return 1;
	}
	else if((sensor == INV_ICM20948_SENSOR_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_RAW_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED)) {
			unsigned short * castedvalue = (unsigned short*) fsr;
			int gfsr = inv_icm20948_get_gyro_fullscale(s);
			if(gfsr == MPU_FS_250dps)
				* castedvalue = 250;
			else if(gfsr == MPU_FS_500dps)
				* castedvalue = 500;
			else if(gfsr == MPU_FS_1000dps)
				* castedvalue = 1000;
			else if(gfsr == MPU_FS_2000dps)
				* castedvalue = 2000;
			else
				return -1;

			return 2;
	}

	return 0;
}

int inv_icm20948_set_bias(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, const void * bias)
{
	int bias_q16[3];
	int bias_in[3];
	int rc = 0;
	short shift;
	switch(sensor) {
	case INV_ICM20948_SENSOR_ACCELEROMETER :
		memcpy(bias_q16, bias, sizeof(bias_q16));
		//convert from q16 to q25
		bias_in[0] = bias_q16[0] << (25 - 16);
		bias_in[1] = bias_q16[1] << (25 - 16);
		bias_in[2] = bias_q16[2] << (25 - 16);
		rc |= inv_icm20948_ctrl_set_acc_bias(s, bias_in);
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
	case INV_ICM20948_SENSOR_GYROSCOPE:
		memcpy(bias_q16, bias, sizeof(bias_q16));
		//convert from q16 to :
		//Q19 => 2000dps
		//Q20 => 1000dps
		//Q21 => 500dps
		//Q22 => 250dps
		shift = ((20 + (MPU_FS_1000dps - inv_icm20948_get_gyro_fullscale(s))) - 16);
		bias_in[0] = bias_q16[0] << shift;
		bias_in[1] = bias_q16[1] << shift;
		bias_in[2] = bias_q16[2] << shift;

		rc |= inv_icm20948_ctrl_set_gyr_bias(s, bias_in);
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		memcpy(bias_q16, bias, sizeof(bias_q16));
		// bias is already in q16
		rc |= inv_icm20948_ctrl_set_mag_bias(s, bias_q16);
		break;
	default :
		rc = -1;
		break;
	}
	return (rc == 0) ? 1 : rc;
}

int inv_icm20948_get_bias(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, void * bias)
{
	int bias_qx[3];
	int bias_out[3];
	int rc = 0;
	short shift;
	switch(sensor) {
	case INV_ICM20948_SENSOR_ACCELEROMETER :
		rc |= inv_icm20948_ctrl_get_acc_bias(s, bias_qx);
		//convert from q25 to q16
		bias_out[0] = bias_qx[0] >> (25 - 16);
		bias_out[1] = bias_qx[1] >> (25 - 16);
		bias_out[2] = bias_qx[2] >> (25 - 16);
		memcpy(bias, bias_out, sizeof(bias_out));
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
	case INV_ICM20948_SENSOR_GYROSCOPE:
		rc |= inv_icm20948_ctrl_get_gyr_bias(s, bias_qx);
		//convert from qn to q16:
		//Q19 => 2000dps
		//Q20 => 1000dps
		//Q21 => 500dps
		//Q22 => 250dps
		shift = ((20 + (MPU_FS_1000dps - inv_icm20948_get_gyro_fullscale(s))) - 16);
		bias_out[0] = bias_qx[0] >> shift;
		bias_out[1] = bias_qx[1] >> shift;
		bias_out[2] = bias_qx[2] >> shift;

		memcpy(bias, bias_out, sizeof(bias_out));
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		rc |= inv_icm20948_ctrl_get_mag_bias(s, bias_qx);
		// bias is already in q16
		memcpy(bias, bias_qx, sizeof(bias_qx));
		break;
	default:
		rc = -1;
		break;
	}
	return (rc == 0) ? 3*(int)sizeof(float) : rc;
}

int inv_icm20948_set_lowpower_or_highperformance(struct inv_icm20948 * s, uint8_t lowpower_or_highperformance)
{
	s->go_back_lp_when_odr_low = 0;
	if(lowpower_or_highperformance)
		return inv_icm20948_enter_low_noise_mode(s);
	else
		return inv_icm20948_enter_duty_cycle_mode(s);
}


int inv_icm20948_get_lowpower_or_highperformance(struct inv_icm20948 * s, uint8_t * lowpower_or_highperformance)
{
	(void)s;
	*lowpower_or_highperformance = CHIP_LOW_NOISE_ICM20948;
	return 1;
}

static void DmpDriver_convertion(signed char transformedtochar[9],
	const int32_t MatrixInQ30[9])
{
	// To convert Q30 to signed char value
	uint8_t iter;
	for (iter = 0; iter < 9; ++iter)
		transformedtochar[iter] = MatrixInQ30[iter] >> 30;
}

int inv_icm20948_set_matrix(struct inv_icm20948 * s, const float matrix[9], enum inv_icm20948_sensor sensor)
{
	int32_t mounting_mq30[9];
	int result = 0;
	int i;

	for(i = 0; i < 9; ++i)
		mounting_mq30[i] = (int32_t)(matrix[i] * (1 << 30));
	// Convert mounting matrix in char
	DmpDriver_convertion(s->mounting_matrix, mounting_mq30);
	//Apply new matrix
	inv_icm20948_set_chip_to_body_axis_quaternion(s, s->mounting_matrix, 0.0);

	if ((sensor == INV_ICM20948_SENSOR_RAW_ACCELEROMETER) ||
		(sensor == INV_ICM20948_SENSOR_ACCELEROMETER) ||
		(sensor == INV_ICM20948_SENSOR_RAW_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_GYROSCOPE) ||
		(sensor == INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED)) {
			//Update Dmp B2S according to new mmatrix in q30
			result |= dmp_icm20948_set_B2S_matrix(s, (int*)mounting_mq30);
	}

	return result;
}

int inv_icm20948_initialize_auxiliary(struct inv_icm20948 * s)
{
	if (inv_icm20948_set_slave_compass_id(s, s->secondary_state.compass_slave_id) )
		return -1;
	return 0;
}

int inv_icm20948_soft_reset(struct inv_icm20948 * s)
{
	//soft reset like
	int rc = inv_icm20948_write_single_mems_reg(s, REG_PWR_MGMT_1, BIT_H_RESET);
	// max start-up time is 100 msec
	inv_icm20948_sleep_us(100000);
	return rc;
}

int inv_icm20948_enable_sensor(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, inv_bool_t state)
{
	uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	if(0!=inv_icm20948_ctrl_enable_sensor(s, androidSensor, state))
		return -1;

	//In case we disable a sensor, we reset his timestamp
	if(state == 0)
		s->timestamp[sensor] = 0;

	return 0;
}

int inv_icm20948_set_sensor_period(struct inv_icm20948 * s, enum inv_icm20948_sensor sensor, uint32_t period)
{
	uint8_t androidSensor = sensor_type_2_android_sensor(sensor);

	if(0!=inv_icm20948_set_odr(s, androidSensor, period))
		return -1;

	// reset timestamp value and save current odr
	s->timestamp[sensor] = 0;
	s->sensorlist[sensor].odr_us = period * 1000;
	return 0;
}

int inv_icm20948_enable_batch_timeout(struct inv_icm20948 * s, unsigned short batchTimeoutMs)
{
	int rc;
	/* Configure batch timeout */
	if (inv_icm20948_ctrl_set_batch_timeout_ms(s, batchTimeoutMs) == 0) {
		/* If configuration was succesful then we enable it */
		if((rc = inv_icm20948_ctrl_enable_batch(s, 1)) != 0)
			return rc;
	} else {
		/* Else we disable it */
		if((rc = inv_icm20948_ctrl_enable_batch(s, 0)) != 0)
			return rc;
	}
	return 0;
}

int inv_icm20948_load(struct inv_icm20948 * s, const uint8_t * image, unsigned short size)
{
	return inv_icm20948_firmware_load(s, image, size, DMP_LOAD_START);
}

/** @brief Returns 1 if the sensor id is a streamed sensor and not an event-based sensor */
static int inv_icm20948_is_streamed_sensor(uint8_t id)
{
	switch(id)
	{
	case ANDROID_SENSOR_WAKEUP_TILT_DETECTOR :
	case ANDROID_SENSOR_ACTIVITY_CLASSIFICATON :
	case ANDROID_SENSOR_FLIP_PICKUP :
	case ANDROID_SENSOR_B2S :
	case ANDROID_SENSOR_STEP_COUNTER:
	case ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION :
	case ANDROID_SENSOR_STEP_DETECTOR :
		return 0;
	default :
		return 1;
	}
}

/** @brief Preprocess all timestamps so that they either contain very last time at which MEMS IRQ was fired
* or last time sent for the sensor + ODR */
static uint8_t inv_icm20948_updateTs(struct inv_icm20948 * s, int * data_left_in_fifo,
	unsigned short * total_sample_cnt, uint64_t * lastIrqTimeUs)
{
	/** @brief Very last time in us at which IRQ was fired since flushing FIFO process was started */
	unsigned short sample_cnt_array[GENERAL_SENSORS_MAX] = {0};
	uint8_t i;

	memset(sample_cnt_array, 0, sizeof(sample_cnt_array));
	if (inv_icm20948_fifo_swmirror(s, data_left_in_fifo, total_sample_cnt, sample_cnt_array)) {
		for(i = 0; i< GENERAL_SENSORS_MAX; i++) {
			if (inv_icm20948_is_streamed_sensor(i)) {
				s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] = *lastIrqTimeUs;
			}
		}
		return -1;
	}
	// we parse all senosr according to android type
	for (i = 0; i < GENERAL_SENSORS_MAX; i++) {
		if (inv_icm20948_is_streamed_sensor(i)) {
			if (sample_cnt_array[i]) {
				/** Number of samples present in MEMS FIFO last time we mirrored it */
				unsigned short fifo_sample_cnt = sample_cnt_array[i];

				/** In case of first batch we have less than the expected number of samples in the batch */
				/** To avoid a bad timestamping we recompute the startup time based on the theorical ODR and the number of samples */
				if (s->sFirstBatch[inv_icm20948_sensor_android_2_sensor_type(i)]) {
					s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] += *lastIrqTimeUs-s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)]
					- fifo_sample_cnt*s->sensorlist[inv_icm20948_sensor_android_2_sensor_type(i)].odr_us;
					s->sFirstBatch[inv_icm20948_sensor_android_2_sensor_type(i)] = 0;
				}

				/** In case it's the first time timestamp is set we create a factice one,
				In other cases, update timestamp for all streamed sensors depending on number of samples available in FIFO
				first time to be printed is t1+(t2-t1)/N
				- t1 is last time we sent data
				- t2 is when IRQ was fired so that we pop the FIFO
				- N is number of samples */

				if(s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] == 0) {
					s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] = *lastIrqTimeUs;
					s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] -= s->sensorlist[inv_icm20948_sensor_android_2_sensor_type(i)].odr_us*(fifo_sample_cnt);
					s->sensorlist[inv_icm20948_sensor_android_2_sensor_type(i)].odr_applied_us = s->sensorlist[inv_icm20948_sensor_android_2_sensor_type(i)].odr_us;
				}
				else {
					s->sensorlist[inv_icm20948_sensor_android_2_sensor_type(i)].odr_applied_us = (*lastIrqTimeUs-s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)])/fifo_sample_cnt;
				}
			}
		} else {
			/** update timestamp for all event sensors with time at which MEMS IRQ was fired */
			s->timestamp[inv_icm20948_sensor_android_2_sensor_type(i)] = *lastIrqTimeUs;
		}
	}

	return 0;
}

int inv_icm20948_poll_sensor(struct inv_icm20948 * s, void * context,
	void (*handler)(void * context, enum inv_icm20948_sensor sensor, uint64_t timestamp, const void * data, const void *arg))
{
	short int_read_back=0;
	unsigned short header=0, header2 = 0;
	int data_left_in_fifo=0;
	short short_data[3] = {0};
	signed long  long_data[3] = {0};
	signed long  long_quat[3] = {0};
	float gyro_raw_float[3];
	float gyro_bias_float[3];
	int gyro_accuracy = 0;
	int dummy_accuracy = 0;
	int accel_accuracy = 0;
	int compass_accuracy = 0;
	float rv_accuracy = 0;
	float gmrv_accuracy = 0;
	float accel_float[3];
	float grv_float[4];
	float gyro_float[3];
	float compass_float[3] = {0};
	float compass_raw_float[3];
	float rv_float[4];
	float gmrv_float[4];
	uint16_t pickup_state = 0;
	uint64_t lastIrqTimeUs;

	inv_icm20948_identify_interrupt(s, &int_read_back);

	if (int_read_back & (BIT_MSG_DMP_INT | BIT_MSG_DMP_INT_0)) {
		lastIrqTimeUs = inv_icm20948_get_time_us();
		do {
			unsigned short total_sample_cnt = 0;

			/* Mirror FIFO contents and stop processing FIFO if an error was detected*/
			if(inv_icm20948_updateTs(s, &data_left_in_fifo, &total_sample_cnt, &lastIrqTimeUs))
				break;
			while(total_sample_cnt--) {
				/* Read FIFO contents and parse it, and stop processing FIFO if an error was detected*/
				if (inv_icm20948_fifo_pop(s, &header, &header2, &data_left_in_fifo))
					break;

				/* Gyro sample available from DMP FIFO */
				if (header & GYRO_SET) {
					float lScaleDeg = (1 << inv_icm20948_get_gyro_fullscale(s)) * 250.f; // From raw to dps to degree per seconds
					float lScaleDeg_bias = 2000.f; // Gyro bias from FIFO is always in 2^20 = 2000 dps regardless of fullscale
					signed long  lRawGyroQ15[3] = {0};
					signed long  lBiasGyroQ20[3] = {0};

					/* Read raw gyro out of DMP FIFO and convert it from Q15 raw data format to radian per seconds in Android format */
					inv_icm20948_dmp_get_raw_gyro(short_data);
					lRawGyroQ15[0] = (long) short_data[0];
					lRawGyroQ15[1] = (long) short_data[1];
					lRawGyroQ15[2] = (long) short_data[2];
					inv_icm20948_convert_dmp3_to_body(s, lRawGyroQ15, lScaleDeg/(1L<<15), gyro_raw_float);

					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_GYROSCOPE) && !skip_sensor(s, ANDROID_SENSOR_RAW_GYROSCOPE)) {
						long out[3];
						inv_icm20948_convert_quat_rotate_fxp(s->s_quat_chip_to_body, lRawGyroQ15, out);
						s->timestamp[INV_ICM20948_SENSOR_RAW_GYROSCOPE] += s->sensorlist[INV_ICM20948_SENSOR_RAW_GYROSCOPE].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_RAW_GYROSCOPE, s->timestamp[INV_ICM20948_SENSOR_RAW_GYROSCOPE], out, &dummy_accuracy);
					}

					/* Read bias gyro out of DMP FIFO and convert it from Q20 raw data format to radian per seconds in Android format */
					inv_icm20948_dmp_get_gyro_bias(short_data);
					lBiasGyroQ20[0] = (long) short_data[0];
					lBiasGyroQ20[1] = (long) short_data[1];
					lBiasGyroQ20[2] = (long) short_data[2];
					inv_icm20948_convert_dmp3_to_body(s, lBiasGyroQ20, lScaleDeg_bias/(1L<<20), gyro_bias_float);

					/* Extract accuracy and calibrated gyro data based on raw/bias data if calibrated gyro sensor is enabled */
					gyro_accuracy = inv_icm20948_get_gyro_accuracy();
					/* If accuracy has changed previously we update the new accuracy the same time as bias*/
					if(s->set_accuracy){
						s->set_accuracy = 0;
						s->new_accuracy = gyro_accuracy;
					}
					/* gyro accuracy has changed, we will notify it the next time*/
					if(gyro_accuracy != s->new_accuracy){
						s->set_accuracy = 1;
					}
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE) && !skip_sensor(s, ANDROID_SENSOR_GYROSCOPE)) {
						// shift to Q20 to do all calibrated gyrometer operations in Q20
						// Gyro bias from FIFO is always in 2^20 = 2000 dps regardless of fullscale
						// Raw gyro from FIFO is in 2^15 = gyro fsr (250/500/1000/2000).
						lRawGyroQ15[0] <<= 5 - (MPU_FS_2000dps - inv_icm20948_get_gyro_fullscale(s));
						lRawGyroQ15[1] <<= 5 - (MPU_FS_2000dps - inv_icm20948_get_gyro_fullscale(s));
						lRawGyroQ15[2] <<= 5 - (MPU_FS_2000dps - inv_icm20948_get_gyro_fullscale(s));
						/* Compute calibrated gyro data based on raw and bias gyro data and convert it from Q20 raw data format to radian per seconds in Android format */
						inv_icm20948_dmp_get_calibrated_gyro(long_data, lRawGyroQ15, lBiasGyroQ20);
						inv_icm20948_convert_dmp3_to_body(s, long_data, lScaleDeg_bias/(1L<<20), gyro_float);
						s->timestamp[INV_ICM20948_SENSOR_GYROSCOPE] += s->sensorlist[INV_ICM20948_SENSOR_GYROSCOPE].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_GYROSCOPE, s->timestamp[INV_ICM20948_SENSOR_GYROSCOPE], gyro_float, &s->new_accuracy);
					}
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED)  && !skip_sensor(s, ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED)) {
						float raw_bias_gyr[6];
						raw_bias_gyr[0] = gyro_raw_float[0];
						raw_bias_gyr[1] = gyro_raw_float[1];
						raw_bias_gyr[2] = gyro_raw_float[2];
						raw_bias_gyr[3] = gyro_bias_float[0];
						raw_bias_gyr[4] = gyro_bias_float[1];
						raw_bias_gyr[5] = gyro_bias_float[2];
						s->timestamp[INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED] += s->sensorlist[INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED].odr_applied_us;
						/* send raw float and bias for uncal gyr*/
						handler(context, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, s->timestamp[INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED], raw_bias_gyr, &s->new_accuracy);
					}
				}
				/* Calibrated accel sample available from DMP FIFO */
				if (header & ACCEL_SET) {
					float scale;
					/* Read calibrated accel out of DMP FIFO and convert it from Q25 raw data format to m/s² in Android format */
					inv_icm20948_dmp_get_accel(long_data);

					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_RAW_ACCELEROMETER) && !skip_sensor(s, ANDROID_SENSOR_RAW_ACCELEROMETER)) {
						long out[3];
						inv_icm20948_convert_quat_rotate_fxp(s->s_quat_chip_to_body, long_data, out);
						
						/* convert to raw data format to Q12/Q11/Q10/Q9 depending on full scale applied,
						so that it fits on 16bits so that it can go through any protocol, even the one which have raw data on 16b */
						out[0] = out[0] >> 15;
						out[1] = out[1] >> 15;
						out[2] = out[2] >> 15;
						s->timestamp[INV_ICM20948_SENSOR_RAW_ACCELEROMETER] += s->sensorlist[INV_ICM20948_SENSOR_RAW_ACCELEROMETER].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, s->timestamp[INV_ICM20948_SENSOR_RAW_ACCELEROMETER], out, &dummy_accuracy);
					}
					if((inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER) && !skip_sensor(s, ANDROID_SENSOR_ACCELEROMETER)) ||
						(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_LINEAR_ACCELERATION))) {
							accel_accuracy = inv_icm20948_get_accel_accuracy();
							scale = (1 << inv_icm20948_get_accel_fullscale(s)) * 2.f / (1L<<30); // Convert from raw units to g's

							inv_icm20948_convert_dmp3_to_body(s, long_data, scale, accel_float);

							if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ACCELEROMETER)) {
								s->timestamp[INV_ICM20948_SENSOR_ACCELEROMETER] += s->sensorlist[INV_ICM20948_SENSOR_ACCELEROMETER].odr_applied_us;
								handler(context, INV_ICM20948_SENSOR_ACCELEROMETER, s->timestamp[INV_ICM20948_SENSOR_ACCELEROMETER], accel_float, &accel_accuracy);
							}
					}
				}
				/* Calibrated compass sample available from DMP FIFO */
				if (header & CPASS_CALIBR_SET) {
					float scale;

					/* Read calibrated compass out of DMP FIFO and convert it from Q16 raw data format to µT in Android format */
					inv_icm20948_dmp_get_calibrated_compass(long_data);

					compass_accuracy = inv_icm20948_get_mag_accuracy();
					scale = DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
					inv_icm20948_convert_dmp3_to_body(s, long_data, scale, compass_float);
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_FIELD) && !skip_sensor(s, ANDROID_SENSOR_GEOMAGNETIC_FIELD)) {
						s->timestamp[INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD] += s->sensorlist[INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, s->timestamp[INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD], compass_float, &compass_accuracy);
					}
				}

				/* Raw compass sample available from DMP FIFO */
				if (header & CPASS_SET) {
					/* Read calibrated compass out of DMP FIFO and convert it from Q16 raw data format to µT in Android format */
					inv_icm20948_dmp_get_raw_compass(long_data);
					compass_raw_float[0] = long_data[0] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
					compass_raw_float[1] = long_data[1] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
					compass_raw_float[2] = long_data[2] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) && !skip_sensor(s, ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED)) {
						float raw_bias_mag[6];
						int mag_bias[3];

						raw_bias_mag[0] = compass_raw_float[0];
						raw_bias_mag[1] = compass_raw_float[1];
						raw_bias_mag[2] = compass_raw_float[2];
						inv_icm20948_ctrl_get_mag_bias(s, mag_bias);
						//calculate bias
						raw_bias_mag[3] = mag_bias[0] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
						raw_bias_mag[4] = mag_bias[1] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;
						raw_bias_mag[5] = mag_bias[2] * DMP_UNIT_TO_FLOAT_COMPASS_CONVERSION;

						compass_accuracy = inv_icm20948_get_mag_accuracy();
						s->timestamp[INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED] += s->sensorlist[INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED].odr_applied_us;
						/* send raw float and bias for uncal mag*/
						handler(context, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, s->timestamp[INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED],
							raw_bias_mag, &compass_accuracy);
					}
				}
				/* 6axis AG orientation quaternion sample available from DMP FIFO */
				if (header & QUAT6_SET) {
					long gravityQ16[3];
					float ref_quat[4];
					/* Read 6 axis quaternion out of DMP FIFO in Q30 */
					inv_icm20948_dmp_get_6quaternion(long_quat);
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR) && !skip_sensor(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR)) {
						/* and convert it from Q30 DMP format to Android format only if GRV sensor is enabled */
						inv_icm20948_convert_rotation_vector(s, long_quat, grv_float);
						ref_quat[0] = grv_float[3];
						ref_quat[1] = grv_float[0];
						ref_quat[2] = grv_float[1];
						ref_quat[3] = grv_float[2];
						s->timestamp[INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR] += s->sensorlist[INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, s->timestamp[INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR], ref_quat, 0);
					}

					/* Compute gravity sensor data in Q16 in g based on 6 axis quaternion in Q30 DMP format */
					inv_icm20948_augmented_sensors_get_gravity(s, gravityQ16, long_quat);
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GRAVITY) && !skip_sensor(s, ANDROID_SENSOR_GRAVITY)) {
						float gravity_float[3];
						/* Convert gravity data from Q16 to float format in g */
						gravity_float[0] = INVN_FXP_TO_FLT(gravityQ16[0], 16);
						gravity_float[1] = INVN_FXP_TO_FLT(gravityQ16[1], 16);
						gravity_float[2] = INVN_FXP_TO_FLT(gravityQ16[2], 16);
						s->timestamp[INV_ICM20948_SENSOR_GRAVITY] += s->sensorlist[INV_ICM20948_SENSOR_GRAVITY].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_GRAVITY, s->timestamp[INV_ICM20948_SENSOR_GRAVITY], gravity_float, &accel_accuracy);
					}

					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_LINEAR_ACCELERATION) && !skip_sensor(s, ANDROID_SENSOR_LINEAR_ACCELERATION)) {
						float linacc_float[3];
						long linAccQ16[3];
						long accelQ16[3];

						/* Compute linear acceleration data based on accelerometer data in Q16 g and on gravity data in Q16 g */
						accelQ16[0] = (int32_t)  ((float)(accel_float[0])*(1ULL << 16) + ( (accel_float[0]>=0)-0.5f ));
						accelQ16[1] = (int32_t)  ((float)(accel_float[1])*(1ULL << 16) + ( (accel_float[1]>=0)-0.5f ));
						accelQ16[2] = (int32_t)  ((float)(accel_float[2])*(1ULL << 16) + ( (accel_float[2]>=0)-0.5f ));

						inv_icm20948_augmented_sensors_get_linearacceleration(linAccQ16, gravityQ16, accelQ16);
						linacc_float[0] = INVN_FXP_TO_FLT(linAccQ16[0], 16);
						linacc_float[1] = INVN_FXP_TO_FLT(linAccQ16[1], 16);
						linacc_float[2] = INVN_FXP_TO_FLT(linAccQ16[2], 16);
						s->timestamp[INV_ICM20948_SENSOR_LINEAR_ACCELERATION] += s->sensorlist[INV_ICM20948_SENSOR_LINEAR_ACCELERATION].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_LINEAR_ACCELERATION, s->timestamp[INV_ICM20948_SENSOR_LINEAR_ACCELERATION], linacc_float, &accel_accuracy);
					}
				}
				/* 9axis orientation quaternion sample available from DMP FIFO */
				if (header & QUAT9_SET) {
					float ref_quat[4];
					/* Read 9 axis quaternion out of DMP FIFO in Q30 */
					inv_icm20948_dmp_get_9quaternion(long_quat);
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ROTATION_VECTOR) && !skip_sensor(s, ANDROID_SENSOR_ROTATION_VECTOR)) {
						/* and convert it from Q30 DMP format to Android format only if RV sensor is enabled */
						inv_icm20948_convert_rotation_vector(s, long_quat, rv_float);
						/* Read rotation vector heading accuracy out of DMP FIFO in Q29*/
						{
							float rv_accur = inv_icm20948_get_rv_accuracy();
							rv_accuracy = rv_accur/(float)(1ULL << (29));
						}
						ref_quat[0] = rv_float[3];
						ref_quat[1] = rv_float[0];
						ref_quat[2] = rv_float[1];
						ref_quat[3] = rv_float[2];
						s->timestamp[INV_ICM20948_SENSOR_ROTATION_VECTOR] += s->sensorlist[INV_ICM20948_SENSOR_ROTATION_VECTOR].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_ROTATION_VECTOR, s->timestamp[INV_ICM20948_SENSOR_ROTATION_VECTOR], ref_quat, &rv_accuracy);
					}

					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_ORIENTATION) && !skip_sensor(s, ANDROID_SENSOR_ORIENTATION)) {
						long orientationQ16[3];
						float orientation_float[3];
						/* Compute Android-orientation sensor data based on rotation vector data in Q30 */
						inv_icm20948_augmented_sensors_get_orientation(orientationQ16, long_quat);
						orientation_float[0] = INVN_FXP_TO_FLT(orientationQ16[0], 16);
						orientation_float[1] = INVN_FXP_TO_FLT(orientationQ16[1], 16);
						orientation_float[2] = INVN_FXP_TO_FLT(orientationQ16[2], 16);
						s->timestamp[INV_ICM20948_SENSOR_ORIENTATION] += s->sensorlist[INV_ICM20948_SENSOR_ORIENTATION].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_ORIENTATION, s->timestamp[INV_ICM20948_SENSOR_ORIENTATION], orientation_float, 0);
					}
				}
				/* 6axis AM orientation quaternion sample available from DMP FIFO */
				if (header & GEOMAG_SET) {
					float ref_quat[4];
					/* Read 6 axis quaternion out of DMP FIFO in Q30 and convert it to Android format */
					inv_icm20948_dmp_get_gmrvquaternion(long_quat);
					if(inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) && !skip_sensor(s, ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR)) {
						inv_icm20948_convert_rotation_vector(s, long_quat, gmrv_float);
						/* Read geomagnetic rotation vector heading accuracy out of DMP FIFO in Q29*/
						{
							float gmrv_acc = inv_icm20948_get_gmrv_accuracy();
							gmrv_accuracy = gmrv_acc/(float)(1ULL << (29));
						}
						ref_quat[0] = gmrv_float[3];
						ref_quat[1] = gmrv_float[0];
						ref_quat[2] = gmrv_float[1];
						ref_quat[3] = gmrv_float[2];
						s->timestamp[INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR] += s->sensorlist[INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR].odr_applied_us;
						handler(context, INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, s->timestamp[INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR],
							ref_quat, &gmrv_accuracy);
					}
				}
				/* Activity recognition sample available from DMP FIFO */
				if (header2 & ACT_RECOG_SET) {
					uint16_t bac_state = 0;
					long bac_ts = 0;
					int bac_event = 0;
					struct bac_map{
						uint8_t act_id;
						enum inv_sensor_bac_event sensor_bac;
					} map[] = {
						{ BAC_DRIVE, INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN},
						{ BAC_WALK, INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN},
						{ BAC_RUN, INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN},
						{ BAC_BIKE, INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN},
						{ BAC_STILL, INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN},
						{ BAC_TILT, INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN},
					};
					int i = 0;
					/* Read activity type and associated timestamp out of DMP FIFO
					activity type is a set of 2 bytes :
					- high byte indicates activity start
					- low byte indicates activity end */
					inv_icm20948_dmp_get_bac_state(&bac_state);
					inv_icm20948_dmp_get_bac_ts(&bac_ts);
					//Map according to dmp bac events
					for(i = 0; i < 6; i++) {
						if ((bac_state >> 8) & map[i].act_id){
							//Check if BAC is enabled
							if (inv_icm20948_ctrl_get_activitiy_classifier_on_flag(s)) {
								/* Start detected */
								bac_event = map[i].sensor_bac;
								handler(context, INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON, s->timestamp[INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON], &bac_event, 0);
							}
							//build event TILT only if enabled
							if((map[i].act_id == BAC_TILT) && inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_WAKEUP_TILT_DETECTOR))
								handler(context, INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR, s->timestamp[INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR], 0, 0);
						}
						/* Check if bit tilt is set for activity end byte */
						else if (bac_state & map[i].act_id) {
							//Check if BAC is enabled
							if (inv_icm20948_ctrl_get_activitiy_classifier_on_flag(s)) {
								/* End detected */
								bac_event = -map[i].sensor_bac;
								handler(context, INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON, s->timestamp[INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON], &bac_event, 0);
							}
						}
					}
				}
				/* Pickup sample available from DMP FIFO */
				if (header2 & FLIP_PICKUP_SET) {
					/* Read pickup type and associated timestamp out of DMP FIFO */
					inv_icm20948_dmp_get_flip_pickup_state(&pickup_state);
					handler(context, INV_ICM20948_SENSOR_FLIP_PICKUP, s->timestamp[INV_ICM20948_SENSOR_FLIP_PICKUP], &pickup_state, 0);
				}

				/* Step detector available from DMP FIFO and step counter sensor is enabled*/
				// If step detector enabled => step counter started too
				// So don't watch the step counter data if the user doesn't start the sensor
				if((header & PED_STEPDET_SET) && (inv_icm20948_ctrl_androidSensor_enabled(s, ANDROID_SENSOR_STEP_COUNTER))) {
					unsigned long steps;
					unsigned long lsteps;
					uint64_t stepc = 0;
					/* Read amount of steps counted out of DMP FIFO and notify them only if updated */
					dmp_icm20948_get_pedometer_num_of_steps(s, &lsteps);
					// need to subtract the steps accumulated while Step Counter sensor is not active.
					steps = lsteps - s->sStepCounterToBeSubtracted;
					stepc = steps;
					if(stepc != s->sOldSteps) {
						s->sOldSteps = steps;
						handler(context, INV_ICM20948_SENSOR_STEP_COUNTER, s->timestamp[INV_ICM20948_SENSOR_STEP_COUNTER], &stepc, 0);
					}
				}
			}
		} while(data_left_in_fifo);

		/* SMD detected by DMP */
		if (int_read_back & BIT_MSG_DMP_INT_2) {
			uint8_t event = 0;
			handler(context, INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION, s->timestamp[INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION], &event, 0);
		}
		/* Step detector triggered by DMP */
		if (int_read_back & BIT_MSG_DMP_INT_3) {
			uint8_t event = 0;
			handler(context, INV_ICM20948_SENSOR_STEP_DETECTOR, s->timestamp[INV_ICM20948_SENSOR_STEP_DETECTOR], &event, 0);
		}
		/* Bring to see detected by DMP */
		if (int_read_back & BIT_MSG_DMP_INT_5) {
			uint8_t event = 0;
			handler(context, INV_ICM20948_SENSOR_B2S, s->timestamp[INV_ICM20948_SENSOR_B2S], &event, 0);
		}
	}

	/* Sometimes, the chip can be put in sleep mode even if there is data in the FIFO. If we poll at this moment, the transport layer will wake-up the chip, but never put it back in sleep. */
	if (s->mems_put_to_sleep) {
		inv_icm20948_sleep_mems(s);
	}

	return 0;
}
