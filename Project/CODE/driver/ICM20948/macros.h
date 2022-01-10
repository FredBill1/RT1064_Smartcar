#ifndef _ICM20948_macros_h
#define _ICM20948_macros_h
// clang-format off

#define AK0991x_DEFAULT_I2C_ADDR 0x0C /* The default I2C address for AK0991x Magnetometers */

#define READ_BIT_MASK			0x80
#define WRITE_BIT_MASK			0x7F

#define WHO_AM_I                0xEA

#define DEF_ST_ACCEL_FS                 2
#define DEF_ST_GYRO_FS_DPS              250
#define DEF_ST_SCALE                    32768
#define DEF_SELFTEST_GYRO_SENS			(DEF_ST_SCALE / DEF_ST_GYRO_FS_DPS)
#define DEF_ST_ACCEL_FS_MG				2000

#ifndef INV20948_ABS
#define INV20948_ABS(x) (((x) < 0) ? -(x) : (x))
#endif

#endif  // _ICM20948_macros_h