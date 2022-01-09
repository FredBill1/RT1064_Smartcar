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
/** @defgroup	inv_icm20948_secondary_transport	inv_secondary_transport
    @ingroup 	SmartSensor_driver
    @{
*/
#ifndef INV_ICM20948_SECONDARY_TRANSPORT_H__
#define INV_ICM20948_SECONDARY_TRANSPORT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_icm20948;

/** @brief I2C from secondary device can stand on up to 4 channels. To perform automatic read and feed DMP :
- channel 0 is reserved for compass reading data
- channel 1 is reserved for compass writing one-shot acquisition register
- channel 2 is reserved for als reading data */
#define COMPASS_I2C_SLV_READ		0
#define COMPASS_I2C_SLV_WRITE		1
#define ALS_I2C_SLV					2

/** @brief Initializes the register for the i2c communication*/
void INV_EXPORT inv_icm20948_init_secondary(struct inv_icm20948 * s);

/** @brief Reads data in i2c a secondary device
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be read on the secondary device
* @param[in] len 	Size of data to be read
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_read_secondary(struct inv_icm20948 * s, int index, unsigned char addr, unsigned char reg, char len);

/** @brief Reads data in i2c a secondary device directly 
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be read on the secondary device
* @param[in] len 	Size of data to be read
* @param[out] d 	pointer to the data to be read
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_execute_read_secondary(struct inv_icm20948 * s, int index, unsigned char addr, int reg, int len, uint8_t *d);

/** @brief Writes data in i2c a secondary device
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be write on the secondary device
* @param[in] v 		the data to be written
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_write_secondary(struct inv_icm20948 * s, int index, unsigned char addr, unsigned char reg, char v);

/** @brief Writes data in i2c a secondary device directly
* @param[in] index  The i2c slave what you would use 
* @param[in] addr  	i2c address slave of the secondary slave
* @param[in] reg 	the register to be write on the secondary device
* @param[in] v 		the data to be written
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_execute_write_secondary(struct inv_icm20948 * s, int index, unsigned char addr, int reg, uint8_t v);

/** @brief Save current secondary I2C ODR configured
*/
void INV_EXPORT inv_icm20948_secondary_saveI2cOdr(struct inv_icm20948 * s);

/** @brief Restore secondary I2C ODR configured based on the one saved with inv_icm20948_secondary_saveI2cOdr()
*/
void INV_EXPORT inv_icm20948_secondary_restoreI2cOdr(struct inv_icm20948 * s);

/** @brief Stop one secondary I2C channel by writing 0 in its control register
* @param[in] index  	the channel id to be stopped
* @return 	   		0 in case of success, -1 for any error
* @warning It does not stop I2C secondary interface, just one channel
*/
int INV_EXPORT inv_icm20948_secondary_stop_channel(struct inv_icm20948 * s, int index);

/** @brief Enable secondary I2C interface
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_secondary_enable_i2c(struct inv_icm20948 * s);

/** @brief Stop secondary I2C interface
* @return 	   		0 in case of success, -1 for any error
* @warning It stops all I2C transactions, whatever the channel status
*/
int INV_EXPORT inv_icm20948_secondary_disable_i2c(struct inv_icm20948 * s);

/** @brief Changes the odr of the I2C master
* @param[in] divider  	frequency divider to BASE_SAMPLE_RATE
* @param[out] effectiveDivider  	divider finally applied to base sample rate, at which data will be actually read on I2C bus
* @return 	   		0 in case of success, -1 for any error
*/
int INV_EXPORT inv_icm20948_secondary_set_odr(struct inv_icm20948 * s, int divider, unsigned int* effectiveDivider);

#ifdef __cplusplus
}
#endif

#endif // INV_ICM20948_SECONDARY_TRANSPORT_H__

/** @} */
