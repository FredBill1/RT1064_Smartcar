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
/** @defgroup	inv_icm20948_slave_compass	inv_slave_compass
    @ingroup 	SmartSensor_driver
    @{
*/
#ifndef INV_ICM20948_SLAVE_COMPASS_H_SDFWQN__
#define INV_ICM20948_SLAVE_COMPASS_H_SDFWQN__

#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_icm20948;

/** @brief Supported auxiliary compass identifer
 */
enum inv_icm20948_compass_id {
	INV_ICM20948_COMPASS_ID_NONE = 0, /**< no compass */
	INV_ICM20948_COMPASS_ID_AK09911,  /**< AKM AK09911 */
	INV_ICM20948_COMPASS_ID_AK09912,  /**< AKM AK09912 */
	INV_ICM20948_COMPASS_ID_AK09916,  /**< AKM AK09916 */
	INV_ICM20948_COMPASS_ID_AK08963,  /**< AKM AK08963 */
};

/** @brief Register AUX compass
 *
 *  Will only set internal states and won't perform any transaction on the bus.
 *  Must be called before inv_icm20948_initialize().
 *
 *  @param[in]  compass_id 	        Compass ID
 *  @param[in]  compass_i2c_addr 	Compass I2C address
 *  @return     0 on success, negative value on error
 */
void INV_EXPORT inv_icm20948_register_aux_compass(struct inv_icm20948 * s,
		enum inv_icm20948_compass_id compass_id, uint8_t compass_i2c_addr);

/** @brief Initializes the compass
* @return 	0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_setup_compass_akm(struct inv_icm20948 * s);

/** @brief Self test for the compass
* @return 	0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_check_akm_self_test(struct inv_icm20948 * s);

/** @brief Changes the scale of the compass
* @param[in] data  	new scale for the compass
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_write_akm_scale(struct inv_icm20948 * s, int data);

/** @brief Reads the scale of the compass
* @param[out] scale  	pointer to recuperate the scale
* @return 	   			0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_read_akm_scale(struct inv_icm20948 * s, int *scale);

/** @brief Stops the compass
* @return 	0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_suspend_akm(struct inv_icm20948 * s);

/** @brief Starts the compass
* @return 	0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_resume_akm(struct inv_icm20948 * s);

/** @brief Get compass power status
* @return 	1 in case compass is enabled, 0 if not started
*/
char INV_EXPORT inv_icm20948_compass_getstate(struct inv_icm20948 * s);

/** @brief detects if the compass is connected
* @return 	1 if the compass is connected, 0 otherwise
*/
int INV_EXPORT inv_icm20948_compass_isconnected(struct inv_icm20948 * s);

/** @brief Calibrates the data
* @param[in] m  			pointer to the raw compass data  
* @param[out] compass_m 	pointer to the calibrated compass data
* @return 	   				0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_compass_dmp_cal(struct inv_icm20948 * s, const signed char *m, const signed char *compass_m);

/**
* @brief Applies mounting matrix and scaling to raw compass data.
* @param[in] raw_data	 		Raw compass data
* @param[in] compensated_out   Compensated compass data
* @return 	   					0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_apply_raw_compass_matrix(struct inv_icm20948 * s, short *raw_data, long *compensated_out);

#ifdef __cplusplus
}
#endif

#endif // INV_ICM20948_SLAVE_COMPASS_H_SDFWQN__

/** @} */
