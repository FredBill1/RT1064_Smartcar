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
/** @defgroup icm20948_base_driver base_driver
	@ingroup  SmartSensor_driver
	@{
*/	
#ifndef INV_ICM20948_BASE_DRIVER_H__HWDFWQ__
#define INV_ICM20948_BASE_DRIVER_H__HWDFWQ__

#include "Icm20948Defs.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* forward declaration */
struct inv_icm20948;

// Standard Functions
/** @brief Initializes the platform
* @param[in] type 				Define the interface for communicate : SERIAL_INTERFACE_I2C or SERIAL_INTERFACE_SPI
* @param[out] dmp_image_sram 4 	The image to be load 
* @return 						0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_initialize_lower_driver(struct inv_icm20948 * s, enum SMARTSENSOR_SERIAL_INTERFACE type, 
	const uint8_t *dmp3_image, uint32_t dmp3_image_size);

/** @brief Initializes the compass and the id address
* @param[in] id 	address of compass component
* @return 			0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_slave_compass_id(struct inv_icm20948 * s, int id);

/** @brief Selects the interface of communication with the board
* @param[in] type 	Define the interface for communicate : SERIAL_INTERFACE_I2C or SERIAL_INTERFACE_SPI
* @return 			0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_serial_comm(struct inv_icm20948 * s, enum SMARTSENSOR_SERIAL_INTERFACE type);

/** @brief Wakes up mems platform
* @return 	0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_wakeup_mems(struct inv_icm20948 * s);

/** @brief Sleeps up mems platform
* @return 	0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_sleep_mems(struct inv_icm20948 * s);

/** @brief Sets the power state of the Ivory chip loop
* @param[in] func  		CHIP_AWAKE, CHIP_LP_ENABLE
* @param[in] on_off 	The functions are enabled if previously disabled and 
*                		disabled if previously enabled based on the value of On/Off.
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_chip_power_state(struct inv_icm20948 * s, unsigned char func, unsigned char on_off);

/** @brief Current wake status of the Mems chip
* @return the wake status
*/
uint8_t INV_EXPORT inv_icm20948_get_chip_power_state(struct inv_icm20948 * s);

/** @brief Sets up dmp start address and firmware
* @return  0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_dmp_address(struct inv_icm20948 * s);

/** @brief Sets up the secondary i2c bus
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_secondary(struct inv_icm20948 * s);

/** @brief Enables accel and/or gyro and/or pressure if integrated with gyro and accel.
* @param[in] bit_mask 	A mask where 2 means turn on accel, 1 means turn on gyro, 4 is for pressure.
*            			By default, this only turns on a sensor if all sensors are off otherwise the DMP controls 
*            			this register including turning off a sensor. To override this behavior add in a mask of 128.
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_enable_hw_sensors(struct inv_icm20948 * s, int bit_mask);

/** @brief Sets the dmp for a particular gyro configuration.
* @param[in] gyro_div 	Value written to GYRO_SMPLRT_DIV register, where
*            			0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
*           			10=102.2727Hz sample rate, ... etc.
* @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_gyro_sf(struct inv_icm20948 * s, unsigned char div, int gyro_level);

/** @brief Sets the gyro sample rate
* @param[in] div 		Value written to GYRO_SMPLRT_DIV register
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_gyro_divider(struct inv_icm20948 * s, unsigned char div);

/** @brief Returns the gyro sample rate
* @return Value written to GYRO_SMPLRT_DIV register.
*/
unsigned char INV_EXPORT inv_icm20948_get_gyro_divider(struct inv_icm20948 * s);

/** @brief Returns the real odr in Milliseconds, Micro Seconds or Ticks.
* @param[in] odrInDivider 	Odr In divider 
* @param[in] odr_units 		Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks
* @return Odr in fucntion of enum.
*/
uint32_t INV_EXPORT inv_icm20948_get_odr_in_units(struct inv_icm20948 * s, unsigned short odrInDivider, unsigned char odr_units );

/** @brief Sets the accel sample rate
* @param[in] div 		Value written to ACCEL_SMPLRT_DIV register
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_accel_divider(struct inv_icm20948 * s, short div);

/** @brief Returns the accel sample rate
* @return the divider for the accel
*/
short INV_EXPORT inv_icm20948_get_accel_divider(struct inv_icm20948 * s);

/** @brief Sets the I2C secondary device sample rate
* @param[in] div 		Value written to REG_I2C_MST_ODR_CONFIG register
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_secondary_divider(struct inv_icm20948 * s, unsigned char div);

/** @brief Returns the I2C secondary device sample rate
* @return the divider for the I2C secondary device interface
*/
unsigned short INV_EXPORT inv_icm20948_get_secondary_divider(struct inv_icm20948 * s);

/** @brief Sets fullscale range of gyro in hardware.
* @param[in]  level  See mpu_gyro_fs.
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_gyro_fullscale(struct inv_icm20948 * s, int level);

/** @brief Returns fullscale range of gyrometer in hardware
* @return the fullscale range
*/
uint8_t INV_EXPORT inv_icm20948_get_gyro_fullscale(struct inv_icm20948 * s);

/** @brief Sets fullscale range of gyro in hardware.
* @param[in]  level  See mpu_gyro_fs.
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_icm20948_gyro_fullscale(struct inv_icm20948 * s, int level);

/** @brief Sets fullscale range of accel in hardware.
* @param[in]  	level  See mpu_accel_fs.
* @return 		0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_accel_fullscale(struct inv_icm20948 * s, int level);

/** @brief Returns fullscale range of accelerometer in hardware
* @return the fullscale range
*/
uint8_t INV_EXPORT inv_icm20948_get_accel_fullscale(struct inv_icm20948 * s);

/** @brief Sets fullscale range of accel in hardware.
* @param[in]  	level  See mpu_accel_fs.
* @return 		0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_icm20948_accel_fullscale(struct inv_icm20948 * s, int level);

/** @brief Asserts int1 interrupt when DMP execute INT1 cmd
* @param[in] enable		0=off, 1=on
* @return 				0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_set_int1_assertion(struct inv_icm20948 * s, int enable);

/** @brief Reads accelerometer data stored in hardware register
* @param[in] accel_hw_reg_data 	variable to be recuperated the accelerometer data 
* @return 						0 on success, negative value on error.
*/
int INV_EXPORT inv_icm20948_accel_read_hw_reg_data(struct inv_icm20948 * s, short accel_hw_reg_data[3]);

/** @brief Prevent LP_EN from being set to 1 again, this speeds up transaction
*/
void INV_EXPORT inv_icm20948_prevent_lpen_control(struct inv_icm20948 * s);

/** @brief Allow LP_EN to be set to 1 again and sets it to 1 again if supported by chip
*/
void INV_EXPORT inv_icm20948_allow_lpen_control(struct inv_icm20948 * s);

/** @brief Determine if compass could be successfully found and inited on board
* @return	1 on success, 0 if not available.
*/
int INV_EXPORT inv_icm20948_get_compass_availability(struct inv_icm20948 * s);

/** @brief Determine if pressure could be successfully found and inited on board
* @return	1 on success, 0 if not available.
*/
int INV_EXPORT inv_icm20948_get_pressure_availability(struct inv_icm20948 * s);

/** @brief Determine if proximity could be successfully found and inited on board
* @return	1 on success, 0 if not available.
*/
int INV_EXPORT inv_icm20948_get_proximity_availability(struct inv_icm20948 * s);

/** @brief Have the chip to enter stand duty cycled mode, also called low-power mode
*	where max reporting frequency is 562Hz
*/
int INV_EXPORT inv_icm20948_enter_duty_cycle_mode(struct inv_icm20948 * s);

/** @brief Have the chip to enter low-noise mode
*/
int INV_EXPORT inv_icm20948_enter_low_noise_mode(struct inv_icm20948 * s);
#ifdef __cplusplus
}
#endif
#endif // INV_ICM20948_BASE_DRIVER_H__HWDFWQ__

/** @} */
