/*
* ________________________________________________________________________________________________________
* Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.
* This software, related documentation and any modifications thereto (collectively “Software”) is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
* ________________________________________________________________________________________________________
*/

#include "Icm20948.h"
#include "Icm20948Augmented.h"
#include "Icm20948DataConverter.h"

#include "Icm20948DataBaseControl.h"

// Determine the fastest ODR for all gravity-based sensors
#define AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_GRAVITY)) \
		newOdr = MIN(s->sGravityOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_GAME_ROTATION_VECTOR)) \
		newOdr = MIN(s->sGrvOdrMs,newOdr);  \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_LINEAR_ACCELERATION)) \
		newOdr = MIN(s->sLinAccOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_GRAVITY)) \
		newOdr = MIN(s->sGravityWuOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR)) \
		newOdr = MIN(s->sGrvWuOdrMs,newOdr);  \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION)) \
		newOdr = MIN(s->sLinAccWuOdrMs,newOdr);

// Determine the fastest ODR for all rotation vector-based sensors
#define AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_ORIENTATION)) \
		newOdr = MIN(s->sOriOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_ROTATION_VECTOR)) \
		newOdr = MIN(s->sRvOdrMs,newOdr);
#define AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, newOdr) \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_ORIENTATION)) \
		newOdr = MIN(s->sOriWuOdrMs,newOdr); \
	if	(inv_icm20948_ctrl_androidSensor_enabled	(s, ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR)) \
		newOdr = MIN(s->sRvWuOdrMs,newOdr);

int inv_icm20948_augmented_init(struct inv_icm20948 * s)
{
	// ODR expected for gravity-based sensors
	s->sGravityOdrMs = 0xFFFF;
	s->sGrvOdrMs = 0xFFFF;
	s->sLinAccOdrMs = 0xFFFF;
	s->sGravityWuOdrMs = 0xFFFF;
	s->sGrvWuOdrMs = 0xFFFF;
	s->sLinAccWuOdrMs = 0xFFFF;
	// ODR expected for rotation vector-based sensors
	s->sRvOdrMs = 0xFFFF;
	s->sOriOdrMs = 0xFFFF;
	s->sRvWuOdrMs = 0xFFFF;
	s->sOriWuOdrMs = 0xFFFF;
	
	return 0;
}

int inv_icm20948_augmented_sensors_get_gravity(struct inv_icm20948 * s, long gravity[3], const long quat6axis_3e[3])
{
	long quat6axis_4e[4];
	long quat6axis_4e_body_to_world[4];

	if(!gravity) return -1;
	if(!quat6axis_3e) return -1;

	// compute w element
	inv_icm20948_convert_compute_scalar_part_fxp(quat6axis_3e, quat6axis_4e);
	// apply mounting matrix
	inv_icm20948_q_mult_q_qi(quat6axis_4e, s->s_quat_chip_to_body, quat6axis_4e_body_to_world);

	gravity[0] = ( 2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[1], quat6axis_4e_body_to_world[3], 30) - 
	               2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[0], quat6axis_4e_body_to_world[2], 30) ) >> (30 - 16);
	gravity[1] = ( 2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[2], quat6axis_4e_body_to_world[3], 30) + 
	               2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[0], quat6axis_4e_body_to_world[1], 30) ) >> (30 - 16);
	gravity[2] = ( (1 << 30) - 2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[1], quat6axis_4e_body_to_world[1], 30) - 
	                2 * inv_icm20948_convert_mult_qfix_fxp(quat6axis_4e_body_to_world[2], quat6axis_4e_body_to_world[2], 30) ) >> (30 - 16);

	return MPU_SUCCESS;
}

int inv_icm20948_augmented_sensors_get_linearacceleration(long linacc[3], const long gravity[3], const long accel[3])
{
    if(!linacc) return -1;
    if(!gravity) return -1;
    if(!accel) return -1;
    
    linacc[0] = accel[0] - gravity[0];
    linacc[1] = accel[1] - gravity[1];
    linacc[2] = accel[2] - gravity[2];
                    
    return MPU_SUCCESS;
}


int inv_icm20948_augmented_sensors_get_orientation(long orientation[3], const long quat9axis_3e[4])
{
    long lQuat9axis4e[4];
	long lMatrixQ30[9];       
	long lMatrixQ30Square; 
	long lRad2degQ16 = 0x394BB8; // (float)(180.0 / 3.14159265358979) in Q16
    
    if(!orientation) return -1;
    if(!quat9axis_3e) return -1;
    
    // compute w element
	inv_icm20948_convert_compute_scalar_part_fxp(quat9axis_3e, lQuat9axis4e);
    
	// quaternion to a rotation matrix, q30 to q30
	inv_icm20948_convert_quat_to_col_major_matrix_fxp((const long *)lQuat9axis4e, (long *)lMatrixQ30);

	// compute orientation in q16
	// orientationFlt[0] = atan2f(-matrixFlt[1][0], matrixFlt[0][0]) * rad2deg;
	orientation[0] = inv_icm20948_math_atan2_q15_fxp(-lMatrixQ30[3] >> 15, lMatrixQ30[0] >> 15) << 1;
	orientation[0] = inv_icm20948_convert_mult_qfix_fxp(orientation[0], lRad2degQ16, 16);

	// orientationFlt[1] = atan2f(-matrixFlt[2][1], matrixFlt[2][2]) * rad2deg;
	orientation[1] = inv_icm20948_math_atan2_q15_fxp(-lMatrixQ30[7] >> 15, lMatrixQ30[8] >> 15) << 1;
	orientation[1] = inv_icm20948_convert_mult_qfix_fxp(orientation[1], lRad2degQ16, 16);

	// orientationFlt[2] = asinf ( matrixFlt[2][0]) * rad2deg;
	// asin(x) = atan (x/sqrt(1-x²))
	// atan2(y,x) = atan(y/x)
	// asin(x) = atan2(x, sqrt(1-x²))
	lMatrixQ30Square = inv_icm20948_convert_mult_qfix_fxp(lMatrixQ30[6], lMatrixQ30[6], 30); // x²
	lMatrixQ30Square = (1UL << 30) - lMatrixQ30Square; // 1-x²
	lMatrixQ30Square = inv_icm20948_convert_fast_sqrt_fxp(lMatrixQ30Square); // sqrt(1-x²)
	orientation[2] = inv_icm20948_math_atan2_q15_fxp(lMatrixQ30[6] >> 15,  lMatrixQ30Square >> 15) << 1; // atan2(x, sqrt(1-x²))
	orientation[2] = inv_icm20948_convert_mult_qfix_fxp(orientation[2], lRad2degQ16, 16); // * rad2deg

	if (orientation[0] < 0)
		orientation[0] += 360UL << 16;

    return MPU_SUCCESS;
}

unsigned short inv_icm20948_augmented_sensors_set_odr(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short delayInMs)
{
	switch(androidSensor)
	{
		case ANDROID_SENSOR_GRAVITY:
			s->sGravityOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
			s->sGrvOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
			s->sLinAccOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_ORIENTATION:
			s->sOriOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_ROTATION_VECTOR:
			s->sRvOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, delayInMs);
			break;
		case ANDROID_SENSOR_WAKEUP_GRAVITY:
			s->sGravityWuOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
			s->sGrvWuOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
			s->sLinAccWuOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_WAKEUP_ORIENTATION:
			s->sOriWuOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, delayInMs);
			break;
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
			s->sRvWuOdrMs = delayInMs;
			AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, delayInMs);
			break;
		default :
			break;
	}

	return delayInMs;
}


void inv_icm20948_augmented_sensors_update_odr(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short * updatedDelayPtr)
{
	unsigned short lDelayInMs = 0xFFFF; // max value of uint16_t, so that we can get min value of all enabled sensors
	switch(androidSensor)
	{
		case ANDROID_SENSOR_GRAVITY:
        case ANDROID_SENSOR_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_LINEAR_ACCELERATION:
			AUGMENTED_SENSOR_GET_6QUAT_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_WAKEUP_GRAVITY:
        case ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR:
        case ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION:
			AUGMENTED_SENSOR_GET_6QUATWU_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_ORIENTATION:
        case ANDROID_SENSOR_ROTATION_VECTOR:
			AUGMENTED_SENSOR_GET_9QUAT_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		case ANDROID_SENSOR_WAKEUP_ORIENTATION:
        case ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR:
			AUGMENTED_SENSOR_GET_9QUATWU_MIN_ODR(s, lDelayInMs);
			*updatedDelayPtr = lDelayInMs;
			break;
		default :
			break;
	}

}
