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

#include "Icm20948AuxCompassAkm.h"

#include "Icm20948Defs.h"
#include "Icm20948DataConverter.h"
#include "Icm20948AuxTransport.h"
#include "Icm20948Dmp3Driver.h"

/* AKM definitions */
#define REG_AKM_ID               0x00
#define REG_AKM_INFO             0x01
#define REG_AKM_STATUS           0x02
#define REG_AKM_MEASURE_DATA     0x03
#define REG_AKM_MODE             0x0A
#define REG_AKM_ST_CTRL          0x0C
#define REG_AKM_SENSITIVITY      0x10
#define REG_AKM8963_CNTL1        0x0A

#if (MEMS_CHIP == HW_ICM20648)
/* AK09911 register definition */
#define REG_AK09911_DMP_READ    0x3
#define REG_AK09911_STATUS1     0x10
#define REG_AK09911_CNTL2       0x31
#define REG_AK09911_SENSITIVITY 0x60
#define REG_AK09911_MEASURE_DATA     0x11

/* AK09912 register definition */
#define REG_AK09912_DMP_READ    0x3
#define REG_AK09912_STATUS1     0x10
#define REG_AK09912_CNTL1       0x30
#define REG_AK09912_CNTL2       0x31
#define REG_AK09912_SENSITIVITY 0x60
#define REG_AK09912_MEASURE_DATA     0x11
#endif

/* AK09916 register definition */
#define REG_AK09916_DMP_READ    0x3
#define REG_AK09916_STATUS1     0x10
#define REG_AK09916_STATUS2     0x18
#define REG_AK09916_CNTL2       0x31
#define REG_AK09916_CNTL3       0x32
#define REG_AK09916_MEASURE_DATA     0x11
#define REG_AK09916_TEST        0x33

//-- REG WIA
#define DATA_AKM_ID              0x48
//-- REG CNTL2
#define DATA_AKM_MODE_PD	 0x00
#define DATA_AKM_MODE_SM	 0x01
#define DATA_AKM_MODE_ST	 0x08
#define DATA_AK09911_MODE_ST	 0x10
#define DATA_AK09912_MODE_ST	 0x10
#define DATA_AK09916_MODE_ST	 0x10
#define DATA_AKM_MODE_FR	 0x0F
#define DATA_AK09911_MODE_FR     0x1F
#define DATA_AK09912_MODE_FR     0x1F
// AK09916 doesn't support Fuse ROM access
#define DATA_AKM_SELF_TEST       0x40
//-- REG Status 1
#define DATA_AKM_DRDY            0x01
#define DATA_AKM9916_DOR         0x01
#define DATA_AKM8963_BIT         0x10

#if (MEMS_CHIP == HW_ICM20648)
/* 0.3 uT * (1 << 30) */
#define DATA_AKM8975_SCALE       322122547
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8972_SCALE       644245094
/* 0.6 uT * (1 << 30) */
#define DATA_AKM8963_SCALE0      644245094
/* 0.6 uT * (1 << 30) */
#define DATA_AK09911_SCALE       644245094
/* 0.15 uT * (1 << 30) */
#define DATA_AK09912_SCALE       161061273
#endif
/* 0.15 uT * (1 << 30) */
#define DATA_AKM8963_SCALE1      161061273
/* 0.15 uT * (1 << 30) */
#define DATA_AK09916_SCALE       161061273

#define DATA_AKM8963_SCALE_SHIFT      4
#define DATA_AKM_MIN_READ_TIME            (9 * NSEC_PER_MSEC)

/* AK09912C NSF */
/* 0:disable, 1:Low, 2:Middle, 3:High */
#define DATA_AK9912_NSF  1
#define DATA_AK9912_NSF_SHIFT 5

#define DEF_ST_COMPASS_WAIT_MIN     (10 * 1000)
#define DEF_ST_COMPASS_WAIT_MAX     (15 * 1000)
#define DEF_ST_COMPASS_TRY_TIMES    10
#define DEF_ST_COMPASS_8963_SHIFT   2
#define DEF_ST_COMPASS_9916_SHIFT   2
#define X                           0
#define Y                           1
#define Z                           2

/* milliseconds between each access */
#define AKM_RATE_SCALE       10

#define DATA_AKM_99_BYTES_DMP   10
#define DATA_AKM_89_BYTES_DMP   9

#if (MEMS_CHIP == HW_ICM20648)
static const short AKM8975_ST_Lower[3] = {-100, -100, -1000};
static const short AKM8975_ST_Upper[3] = {100, 100, -300};

static const short AKM8972_ST_Lower[3] = {-50, -50, -500};
static const short AKM8972_ST_Upper[3] = {50, 50, -100};

static const short AKM8963_ST_Lower[3] = {-200, -200, -3200};
static const short AKM8963_ST_Upper[3] = {200, 200, -800};

static const short AK09911_ST_Lower[3] = {-30, -30, -400};
static const short AK09911_ST_Upper[3] = {30, 30, -50};

static const short AK09912_ST_Lower[3] = {-200, -200, -1600};
static const short AK09912_ST_Upper[3] = {200, 200, -400};
#endif

static const short AK09916_ST_Lower[3] = {-200, -200, -1000};
static const short AK09916_ST_Upper[3] = {200, 200, -200};

void inv_icm20948_register_aux_compass(struct inv_icm20948 * s,
		enum inv_icm20948_compass_id compass_id, uint8_t compass_i2c_addr)
{
	switch(compass_id) {
	case INV_ICM20948_COMPASS_ID_AK09911:
		s->secondary_state.compass_slave_id = HW_AK09911;
		s->secondary_state.compass_chip_addr = compass_i2c_addr;
		s->secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;
		/* initialise mounting matrix of compass to identity akm9911 */
		s->mounting_matrix_secondary_compass[0] = -1 ;
		s->mounting_matrix_secondary_compass[4] = -1;
		s->mounting_matrix_secondary_compass[8] = 1;
		break;
	case INV_ICM20948_COMPASS_ID_AK09912:
		s->secondary_state.compass_slave_id = HW_AK09912;
		s->secondary_state.compass_chip_addr = compass_i2c_addr;
		s->secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;
		/* initialise mounting matrix of compass to identity akm9912 */
		s->mounting_matrix_secondary_compass[0] = 1 ;
		s->mounting_matrix_secondary_compass[4] = 1;
		s->mounting_matrix_secondary_compass[8] = 1;
		break;
	case INV_ICM20948_COMPASS_ID_AK08963:
		s->secondary_state.compass_slave_id = HW_AK8963;
		s->secondary_state.compass_chip_addr = compass_i2c_addr;
		s->secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;
		/* initialise mounting matrix of compass to identity akm8963 */
		s->mounting_matrix_secondary_compass[0] = 1;
		s->mounting_matrix_secondary_compass[4] = 1;
		s->mounting_matrix_secondary_compass[8] = 1;
		break;
	case INV_ICM20948_COMPASS_ID_AK09916:
		s->secondary_state.compass_slave_id = HW_AK09916;
		s->secondary_state.compass_chip_addr = compass_i2c_addr;
		s->secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;
		/* initialise mounting matrix of compass to identity akm9916 */
		s->mounting_matrix_secondary_compass[0] = 1 ;
		s->mounting_matrix_secondary_compass[4] = -1;
		s->mounting_matrix_secondary_compass[8] = -1;
		break;
	default:
		s->secondary_state.compass_slave_id  = 0;
		s->secondary_state.compass_chip_addr = 0;
		s->secondary_state.compass_state = INV_ICM20948_COMPASS_RESET;
	}
}

/*
 *  inv_icm20948_setup_compass_akm() - Configure akm series compass.
 */
int inv_icm20948_setup_compass_akm(struct inv_icm20948 * s)
{
	int result;
	unsigned char data[4];
#if (MEMS_CHIP != HW_ICM20948)
	uint8_t sens, cmd;
#endif
	//reset variable to initial values
	memset(s->secondary_state.final_matrix, 0, sizeof(s->secondary_state.final_matrix));
	memset(s->secondary_state.compass_sens, 0, sizeof(s->secondary_state.compass_sens));
	s->secondary_state.scale = 0;
	s->secondary_state.dmp_on = 1;
	s->secondary_state.secondary_resume_compass_state = 0;

	/* Read WHOAMI through I2C SLV for compass */
	result = inv_icm20948_execute_read_secondary(s, COMPASS_I2C_SLV_READ, s->secondary_state.compass_chip_addr, REG_AKM_ID, 1, data);
	if (result) {
        // inv_log("Read secondary error: Compass.\r\n");
		return result;
    }
	if (data[0] != DATA_AKM_ID) {
        // inv_log("Compass not found!!\r\n");
		return -1;
    }
    // inv_log("Compass found.\r\n");

	/* setup upper and lower limit of self-test */
#if (MEMS_CHIP == HW_ICM20948)
	s->secondary_state.st_upper = AK09916_ST_Upper;
	s->secondary_state.st_lower = AK09916_ST_Lower;
#else
	if (HW_AK8975 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AKM8975_ST_Upper;
		s->secondary_state.st_lower = AKM8975_ST_Lower;
	} else if (HW_AK8972 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AKM8972_ST_Upper;
		s->secondary_state.st_lower = AKM8972_ST_Lower;
	} else if (HW_AK8963 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AKM8963_ST_Upper;
		s->secondary_state.st_lower = AKM8963_ST_Lower;
	} else if (HW_AK09911 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AK09911_ST_Upper;
		s->secondary_state.st_lower = AK09911_ST_Lower;
	} else if (HW_AK09912 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AK09912_ST_Upper;
		s->secondary_state.st_lower = AK09912_ST_Lower;
	} else if (HW_AK09916 == s->secondary_state.compass_slave_id) {
		s->secondary_state.st_upper = AK09916_ST_Upper;
		s->secondary_state.st_lower = AK09916_ST_Lower;
	} else {
		return -1;
	}
#endif


#if (MEMS_CHIP == HW_ICM20948)
	/* Read conf and configure compass through I2C SLV for compass and subsequent channel */
	s->secondary_state.mode_reg_addr = REG_AK09916_CNTL2;
	// no sensitivity adjustment value
	s->secondary_state.compass_sens[0] = 128;
	s->secondary_state.compass_sens[1] = 128;
	s->secondary_state.compass_sens[2] = 128;
#else
	/* Read conf and configure compass through I2C SLV for compass and subsequent channel */
	if (HW_AK09916 == s->secondary_state.compass_slave_id) {
		s->secondary_state.mode_reg_addr = REG_AK09916_CNTL2;
		// no sensitivity adjustment value
		s->secondary_state.compass_sens[0] = 128;
		s->secondary_state.compass_sens[1] = 128;
		s->secondary_state.compass_sens[2] = 128;
	}
	else {
		// Fuse ROM access not possible for ak9916
		/* set AKM to Fuse ROM access mode */
		if (HW_AK09911 == s->secondary_state.compass_slave_id) {
			s->secondary_state.mode_reg_addr = REG_AK09911_CNTL2;
			sens = REG_AK09911_SENSITIVITY;
			cmd = DATA_AK09911_MODE_FR;
		} else if (HW_AK09912 == s->secondary_state.compass_slave_id) {
			s->secondary_state.mode_reg_addr = REG_AK09912_CNTL2;
			sens = REG_AK09912_SENSITIVITY;
			cmd = DATA_AK09912_MODE_FR;
		} else {
			s->secondary_state.mode_reg_addr = REG_AKM_MODE;
			sens = REG_AKM_SENSITIVITY;
			cmd = DATA_AKM_MODE_FR;
		}

		result = inv_icm20948_read_secondary(s, COMPASS_I2C_SLV_READ, s->secondary_state.compass_chip_addr, sens, THREE_AXES);
		if (result)
			return result;
		// activate FUSE_ROM mode to CNTL2
		result = inv_icm20948_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE, s->secondary_state.compass_chip_addr,
				s->secondary_state.mode_reg_addr, cmd);

		if (result)
			return result;
		// read sensitivity
		result = inv_icm20948_read_mems_reg(s, REG_EXT_SLV_SENS_DATA_00, THREE_AXES, s->secondary_state.compass_sens);
		if (result)
			return result;
	}
	//aply noise suppression filter (only available for 9912)
	if (HW_AK09912 == s->secondary_state.compass_slave_id) {
		result = inv_icm20948_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE, s->secondary_state.compass_chip_addr, REG_AK09912_CNTL1,
                                     DATA_AK9912_NSF << DATA_AK9912_NSF_SHIFT);
		if (result)
			return result;
	}
#endif
	/* Set compass in power down through I2C SLV for compass */
	result = inv_icm20948_execute_write_secondary(s, COMPASS_I2C_SLV_WRITE, s->secondary_state.compass_chip_addr, s->secondary_state.mode_reg_addr, DATA_AKM_MODE_PD);
	if (result)
		return result;

	s->secondary_state.secondary_resume_compass_state = 1;
	s->secondary_state.compass_state = INV_ICM20948_COMPASS_SETUP;
	return inv_icm20948_suspend_akm(s);
}

int inv_icm20948_check_akm_self_test(struct inv_icm20948 * s)
{
	int result;
	unsigned char data[6], mode, addr;
	unsigned char counter;
	short x, y, z;
	unsigned char *sens;
	int shift;
	unsigned char slv_ctrl[2];
	unsigned char odr_cfg;
#if (MEMS_CHIP != HW_ICM20948)
	unsigned char cntl;
#endif
	addr = s->secondary_state.compass_chip_addr;
	sens = s->secondary_state.compass_sens;

	/* back up registers */
	/* SLV0_CTRL */
	result = inv_icm20948_read_mems_reg(s, REG_I2C_SLV0_CTRL, 1, &slv_ctrl[0]);
	if (result)
		return result;
	result = inv_icm20948_write_single_mems_reg(s, REG_I2C_SLV0_CTRL, 0);
	if (result)
		return result;
	/* SLV1_CTRL */
	result = inv_icm20948_read_mems_reg(s, REG_I2C_SLV1_CTRL, 1, &slv_ctrl[1]);
	if (result)
		return result;
	result = inv_icm20948_write_single_mems_reg(s, REG_I2C_SLV1_CTRL, 0);
	if (result)
		return result;
	/* I2C_MST ODR */
	result = inv_icm20948_read_mems_reg(s, REG_I2C_MST_ODR_CONFIG, 1, &odr_cfg);
	if (result)
		return result;
	result = inv_icm20948_write_single_mems_reg(s, REG_I2C_MST_ODR_CONFIG, 0);
	if (result)
		return result;

#if (MEMS_CHIP == HW_ICM20948)
	mode = REG_AK09916_CNTL2;
#else
	if (HW_AK09911 == s->secondary_state.compass_slave_id)
		mode = REG_AK09911_CNTL2;
	else if (HW_AK09912 == s->secondary_state.compass_slave_id)
		mode = REG_AK09912_CNTL2;
	else if (HW_AK09916 == s->secondary_state.compass_slave_id)
		mode = REG_AK09916_CNTL2;
	else
		mode = REG_AKM_MODE;
#endif
	/* set to power down mode */
	result = inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AKM_MODE_PD);
	if (result)
		goto AKM_fail;

	/* write 1 to ASTC register */
	if ((HW_AK09911 != s->secondary_state.compass_slave_id) &&
		(HW_AK09912 != s->secondary_state.compass_slave_id)) {
		result = inv_icm20948_execute_write_secondary(s, 0, addr, REG_AKM_ST_CTRL, DATA_AKM_SELF_TEST);
		if (result)
			goto AKM_fail;
	}
#if (MEMS_CHIP == HW_ICM20948)
	result = inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AK09916_MODE_ST);
#else
	/* set self test mode */
	if (HW_AK09911 == s->secondary_state.compass_slave_id)
		result = inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AK09911_MODE_ST);
	else if (HW_AK09912 == s->secondary_state.compass_slave_id)
		result = inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AK09912_MODE_ST);
	else if (HW_AK09916 == s->secondary_state.compass_slave_id)
		result = inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AK09916_MODE_ST);
	else
		result = inv_icm20948_execute_write_secondary(s, 0, addr, mode,	DATA_AKM_MODE_ST);
#endif
	if (result)
		goto AKM_fail;
	counter = DEF_ST_COMPASS_TRY_TIMES;
	while (counter > 0) {
//		usleep_range(DEF_ST_COMPASS_WAIT_MIN, DEF_ST_COMPASS_WAIT_MAX);
        inv_icm20948_sleep_us(15000);

#if (MEMS_CHIP == HW_ICM20948)
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09916_STATUS1, 1, data);
#else
		if (HW_AK09911 == s->secondary_state.compass_slave_id)
			result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09911_STATUS1, 1, data);
		else if (HW_AK09912 == s->secondary_state.compass_slave_id)
			result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09912_STATUS1, 1, data);
		else if (HW_AK09916 == s->secondary_state.compass_slave_id)
			result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09916_STATUS1, 1, data);
		else
			result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AKM_STATUS, 1, data);
#endif
		if (result)
			goto AKM_fail;
		if ((data[0] & DATA_AKM_DRDY) == 0)
			counter--;
		else
			counter = 0;
	}
	if ((data[0] & DATA_AKM_DRDY) == 0) {
		result = -1;
		goto AKM_fail;
	}
#if (MEMS_CHIP == HW_ICM20948)
	result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09916_MEASURE_DATA, BYTES_PER_SENSOR, data);
#else
	if (HW_AK09911 == s->secondary_state.compass_slave_id) {
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09911_MEASURE_DATA, BYTES_PER_SENSOR, data);
	} else if (HW_AK09912 == s->secondary_state.compass_slave_id) {
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09912_MEASURE_DATA, BYTES_PER_SENSOR, data);
	} else if (HW_AK09916 == s->secondary_state.compass_slave_id) {
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AK09916_MEASURE_DATA, BYTES_PER_SENSOR, data);
	} else {
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AKM_MEASURE_DATA, BYTES_PER_SENSOR, data);
	}
#endif
	if (result)
		goto AKM_fail;

    x = ((short)data[1])<<8|data[0];
    y = ((short)data[3])<<8|data[2];
    z = ((short)data[5])<<8|data[4];

	if (HW_AK09911 == s->secondary_state.compass_slave_id)
		shift = 7;
	else
		shift = 8;
	x = ((x * (sens[0] + 128)) >> shift);
	y = ((y * (sens[1] + 128)) >> shift);
	z = ((z * (sens[2] + 128)) >> shift);
#if (MEMS_CHIP == HW_ICM20648)
	if (HW_AK8963 == s->secondary_state.compass_slave_id) {
		result = inv_icm20948_execute_read_secondary(s, 0, addr, REG_AKM8963_CNTL1, 1, &cntl);
		if (result)
			goto AKM_fail;
		if (0 == (cntl & DATA_AKM8963_BIT)) {
			x <<= DEF_ST_COMPASS_8963_SHIFT;
			y <<= DEF_ST_COMPASS_8963_SHIFT;
			z <<= DEF_ST_COMPASS_8963_SHIFT;
		}
	}
#endif

	result = -1;
	if (x > s->secondary_state.st_upper[0] || x < s->secondary_state.st_lower[0])
		goto AKM_fail;
	if (y > s->secondary_state.st_upper[1] || y < s->secondary_state.st_lower[1])
		goto AKM_fail;
	if (z > s->secondary_state.st_upper[2] || z < s->secondary_state.st_lower[2])
		goto AKM_fail;
	result = 0;
AKM_fail:
	/*write 0 to ASTC register */
	if ((HW_AK09911 != s->secondary_state.compass_slave_id) &&
		(HW_AK09912 != s->secondary_state.compass_slave_id) &&
		(HW_AK09916 != s->secondary_state.compass_slave_id)) {
		result |= inv_icm20948_execute_write_secondary(s, 0, addr, REG_AKM_ST_CTRL, 0);
	}
	/*set to power down mode */
	result |= inv_icm20948_execute_write_secondary(s, 0, addr, mode, DATA_AKM_MODE_PD);

    return result;
}

/*
 *  inv_icm20948_write_akm_scale() - Configure the akm scale range.
 */
int inv_icm20948_write_akm_scale(struct inv_icm20948 * s, int data)
{
	char d, en;
	int result;

	if (HW_AK8963 != s->secondary_state.compass_slave_id)
		return 0;
	en = !!data;
	if (s->secondary_state.scale == en)
		return 0;
	d = (DATA_AKM_MODE_SM | (en << DATA_AKM8963_SCALE_SHIFT));

	result = inv_icm20948_write_single_mems_reg(s, REG_I2C_SLV1_DO, d);
	if (result)
		return result;

	s->secondary_state.scale = en;

	return 0;
}

/*
 *  inv_icm20948_read_akm_scale() - show AKM scale.
 */
int inv_icm20948_read_akm_scale(struct inv_icm20948 * s, int *scale)
{
#if (MEMS_CHIP == HW_ICM20948)
	(void)s;
	*scale = DATA_AK09916_SCALE;
#else
	if (HW_AK8975 == s->secondary_state.compass_slave_id)
		*scale = DATA_AKM8975_SCALE;
	else if (HW_AK8972 == s->secondary_state.compass_slave_id)
		*scale = DATA_AKM8972_SCALE;
	else if (HW_AK8963 == s->secondary_state.compass_slave_id)
		if (s->secondary_state.scale)
			*scale = DATA_AKM8963_SCALE1;
		else
			*scale = DATA_AKM8963_SCALE0;
	else if (HW_AK09911 == s->secondary_state.compass_slave_id)
		*scale = DATA_AK09911_SCALE;
	else if (HW_AK09912 == s->secondary_state.compass_slave_id)
		*scale = DATA_AK09912_SCALE;
	else if (HW_AK09916 == s->secondary_state.compass_slave_id)
		*scale = DATA_AK09916_SCALE;
	else
		return -1;
#endif
	return 0;
}

int inv_icm20948_suspend_akm(struct inv_icm20948 * s)
{
	int result;

	if (!s->secondary_state.secondary_resume_compass_state)
		return 0;

	/* slave 0 is disabled */
	result = inv_icm20948_secondary_stop_channel(s, COMPASS_I2C_SLV_READ);
	/* slave 1 is disabled */
	result |= inv_icm20948_secondary_stop_channel(s, COMPASS_I2C_SLV_WRITE);
	if (result)
		return result;

	// Switch off I2C Interface as compass is alone
	result |= inv_icm20948_secondary_disable_i2c(s);

	s->secondary_state.secondary_resume_compass_state = 0;

	return result;
}

int inv_icm20948_resume_akm(struct inv_icm20948 * s)
{
	int result;
	uint8_t reg_addr, bytes;
    unsigned char lDataToWrite;

	if (s->secondary_state.secondary_resume_compass_state)
		return 0;

	/* slave 0 is used to read data from compass */
	/*read mode */
#if (MEMS_CHIP == HW_ICM20948)
	if (s->secondary_state.dmp_on) {
		reg_addr = REG_AK09916_DMP_READ;
		bytes = DATA_AKM_99_BYTES_DMP;
	} else {
		reg_addr = REG_AK09916_STATUS1;
		bytes = DATA_AKM_99_BYTES_DMP - 1;
	}
#else
	/* AKM status register address is 1 */
	if (HW_AK09911 == s->secondary_state.compass_slave_id) {
		if (s->secondary_state.dmp_on) {
			reg_addr = REG_AK09911_DMP_READ;
			bytes = DATA_AKM_99_BYTES_DMP;
		} else {
			reg_addr = REG_AK09911_STATUS1;
			bytes = DATA_AKM_99_BYTES_DMP - 1;
		}
	} else if (HW_AK09912 == s->secondary_state.compass_slave_id) {
		if (s->secondary_state.dmp_on) {
			reg_addr = REG_AK09912_DMP_READ;
			bytes = DATA_AKM_99_BYTES_DMP;
		} else {
			reg_addr = REG_AK09912_STATUS1;
			bytes = DATA_AKM_99_BYTES_DMP - 1;
		}
	} else if (HW_AK09916 == s->secondary_state.compass_slave_id) {
		if (s->secondary_state.dmp_on) {
			reg_addr = REG_AK09916_DMP_READ;
			bytes = DATA_AKM_99_BYTES_DMP;
		} else {
			reg_addr = REG_AK09916_STATUS1;
			bytes = DATA_AKM_99_BYTES_DMP - 1;
		}
	} else {
		if (s->secondary_state.dmp_on) {
			reg_addr = REG_AKM_INFO;
			bytes = DATA_AKM_89_BYTES_DMP;
		} else {
			reg_addr = REG_AKM_STATUS;
			bytes = DATA_AKM_89_BYTES_DMP - 1;
		}
	}
#endif
	/* slave 0 is enabled, read 10 or 8 bytes from here depending on compass type, swap bytes to feed DMP */
	result = inv_icm20948_read_secondary(s, COMPASS_I2C_SLV_READ, s->secondary_state.compass_chip_addr, reg_addr, INV_MPU_BIT_GRP | INV_MPU_BIT_BYTE_SW | bytes);
	if (result)
		return result;
#if (MEMS_CHIP == HW_ICM20948)
	lDataToWrite = DATA_AKM_MODE_SM;
#else
	/* slave 1 is used to write one-shot accquisition configuration to compass */
	/* output data for slave 1 is fixed, single measure mode */
	s->secondary_state.scale = 1;
	if (HW_AK8975 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else if (HW_AK8972 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else if (HW_AK8963 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM |
			(s->secondary_state.scale << DATA_AKM8963_SCALE_SHIFT);
	}  else if (HW_AK09911 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	}  else if (HW_AK09912 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	}  else if (HW_AK09916 == s->secondary_state.compass_slave_id) {
		lDataToWrite = DATA_AKM_MODE_SM;
	} else {
		return -1;
	}
#endif
	result = inv_icm20948_write_secondary(s, COMPASS_I2C_SLV_WRITE, s->secondary_state.compass_chip_addr, s->secondary_state.mode_reg_addr, lDataToWrite);
	if (result)
		return result;

	result |= inv_icm20948_secondary_enable_i2c(s);

    s->secondary_state.secondary_resume_compass_state = 1;

	return result;
}

char inv_icm20948_compass_getstate(struct inv_icm20948 * s)
{
	return s->secondary_state.secondary_resume_compass_state;
}

int inv_icm20948_compass_isconnected(struct inv_icm20948 * s)
{
	if(s->secondary_state.compass_state == INV_ICM20948_COMPASS_SETUP) {
		return 1;
	} else {
		return 0;
	}
}

/**
*  @brief      Set up the soft-iron matrix for compass in DMP.
*  @param[in]  Accel/Gyro mounting matrix
*  @param[in]  Compass mounting matrix
*  @return     0 if successful.
*/

int inv_icm20948_compass_dmp_cal(struct inv_icm20948 * s, const signed char *m, const signed char *compass_m)
{
	int8_t trans[NINE_ELEM];
	int tmp_m[NINE_ELEM];
	int i, j, k;
	int sens[THREE_AXES];
	int scale;
	int shift;
    int current_compass_matrix[NINE_ELEM];

	for (i = 0; i < THREE_AXES; i++)
		for (j = 0; j < THREE_AXES; j++)
			trans[THREE_AXES * j + i] = m[THREE_AXES * i + j];

    switch (s->secondary_state.compass_slave_id)
    {
#if (MEMS_CHIP == HW_ICM20648)
        case HW_AK8972:
            scale = DATA_AKM8972_SCALE;
            shift = AK89XX_SHIFT;
            break;
        case HW_AK8975:
            scale = DATA_AKM8975_SCALE;
            shift = AK89XX_SHIFT;
            break;
        case HW_AK8963:
            scale = DATA_AKM8963_SCALE1;
            shift = AK89XX_SHIFT;
            break;
        case HW_AK09911:
            scale = DATA_AK09911_SCALE;
            shift = AK99XX_SHIFT;
            break;
        case HW_AK09912:
            scale = DATA_AK09912_SCALE;
            shift = AK89XX_SHIFT;
            break;
#else
        case HW_AK09916:
            scale = DATA_AK09916_SCALE;
            shift = AK89XX_SHIFT;
            break;
#endif
		default:
				scale = DATA_AKM8963_SCALE1;
				shift = AK89XX_SHIFT;
				break;
    }

	for (i = 0; i < THREE_AXES; i++) {
		sens[i] = s->secondary_state.compass_sens[i] + 128;
		sens[i] = inv_icm20948_convert_mult_q30_fxp(sens[i] << shift, scale);
	}
	for (i = 0; i < NINE_ELEM; i++) {
		current_compass_matrix[i] = compass_m[i] * sens[i % THREE_AXES];
		tmp_m[i] = 0;
	}

    for (i = 0; i < THREE_AXES; i++) {
		for (j = 0; j < THREE_AXES; j++) {
			s->secondary_state.final_matrix[i * THREE_AXES + j] = 0;
			for (k = 0; k < THREE_AXES; k++)
				s->secondary_state.final_matrix[i * THREE_AXES + j] +=
					inv_icm20948_convert_mult_q30_fxp(s->soft_iron_matrix[i * THREE_AXES + k],
                                 current_compass_matrix[j + k * THREE_AXES]);
		}
	}

    for (i = 0; i < THREE_AXES; i++)
		for (j = 0; j < THREE_AXES; j++)
			for (k = 0; k < THREE_AXES; k++)
				tmp_m[THREE_AXES * i + j] +=
					trans[THREE_AXES * i + k] *
						s->secondary_state.final_matrix[THREE_AXES * k + j];

    return dmp_icm20948_set_compass_matrix(s, tmp_m);
}

/**
*  @brief      Apply mounting matrix and scaling to raw compass data.
*  @param[in]  Raw compass data
*  @param[in]  Compensated compass data
*  @return     0 if successful.
*/

int inv_icm20948_apply_raw_compass_matrix(struct inv_icm20948 * s, short *raw_data, long *compensated_out)
{
	int i, j;
	long long tmp;

	for (i = 0; i < THREE_AXES; i++) {
		tmp = 0;
		for (j = 0; j < THREE_AXES; j++)
			tmp  +=
			(long long)s->secondary_state.final_matrix[i * THREE_AXES + j] * (((int)raw_data[j]) << 16);
		compensated_out[i] = (long)(tmp >> 30);
	}

	return 0;
}

