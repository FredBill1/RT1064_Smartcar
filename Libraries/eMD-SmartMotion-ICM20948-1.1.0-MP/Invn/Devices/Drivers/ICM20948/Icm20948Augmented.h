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
/** @defgroup icm20948_augmented_sensors augmented_sensors
	@ingroup  SmartSensor_driver
	@{
*/
#ifndef INV_ICM20948_AUGMENTED_SENSORS__
#define INV_ICM20948_AUGMENTED_SENSORS__


#ifdef __cplusplus
extern "C"
{
#endif

#include "Icm20948Defs.h"

/* forward declaration */
struct inv_icm20948;

/** @brief Initialize structure values
* @param[in] base state structre
*/
int INV_EXPORT inv_icm20948_augmented_init(struct inv_icm20948 * s);
/** @brief Gets the 3 axis gravity value based on GRV quaternion
* @param[out] gravity   3 components resulting gravity in Q16 in m/s2
* @param[in] quat 3 components input AG-based quaternion in Q30
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_augmented_sensors_get_gravity(struct inv_icm20948 * s, long gravity[3], const long quat6axis_3e[3]);

/** @brief Gets the 3 axis linear acceleration value based on gravity and accelerometer values
* @param[out] linacc  3 components resulting linear acceleration in Q16 in m/s2
* @param[in] gravity 3 components gravity in Q16 in m/s2
* @param[in] accel 3 components acceleration in Q16 in m/s2
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_augmented_sensors_get_linearacceleration(long linacc[3], const long gravity[3], const long accel[3]);

/** @brief Gets the 3 axis orientation value based on RV quaternion
* @param[out] orientation  3 components resulting orientation in Q16 in degrees
				The x field is azimuth, the angle between the magnetic north direction and the y axis around the the z axis.
				The y field is pitch, the rotation arounf x axis, positive when the z axis moves toward the y axis.
				The z field is roll, the rotation arount the y axis, positive when the x axis moves toward the z axis.
* @param[in] quat9axis_3e 3 components input AGM-based quaternion in Q30
* @return 0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_augmented_sensors_get_orientation(long orientation[3], const long quat9axis_3e[4]);

/** @brief Set ODR for one of the augmented sensor-related Android sensor
* @param[in] androidSensor   Android sensor ID for which a new delay in to be applied
* @param[in] delayInMs the new delay in ms requested for androidSensor
* @return the delay in ms to be applied to quat6 output
*/
unsigned short INV_EXPORT inv_icm20948_augmented_sensors_set_odr(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short delayInMs);

/** @brief Update ODR when an augmented sensor-related Android sensor was enabled or disable with ODR unchanged
* @param[in] androidSensor   Android sensor ID for which status was updated
* @param[out] updatedDelayPtr Handler where should be written new delay to be applied
* @return None
*/
void INV_EXPORT inv_icm20948_augmented_sensors_update_odr(struct inv_icm20948 * s, unsigned char androidSensor, unsigned short * updatedDelayPtr);

#ifdef __cplusplus
}
#endif
#endif // INV_ICM20948_AUGMENTED_SENSORS__

/** @} */
