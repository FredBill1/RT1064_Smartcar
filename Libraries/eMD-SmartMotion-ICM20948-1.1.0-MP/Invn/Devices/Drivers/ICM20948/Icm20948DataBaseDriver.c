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
#include "Icm20948DataBaseDriver.h"

#include "Icm20948Defs.h"
#include "Icm20948DataBaseControl.h"

#include "Icm20948AuxCompassAkm.h"
#include "Icm20948AuxTransport.h"
#include "Icm20948Dmp3Driver.h"

static unsigned char inv_is_gyro_enabled(struct inv_icm20948 * s);

void inv_icm20948_prevent_lpen_control(struct inv_icm20948 * s)
{
	s->sAllowLpEn = 0;
}
void inv_icm20948_allow_lpen_control(struct inv_icm20948 * s)
{
	s->sAllowLpEn = 1;
	inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);
}
static uint8_t inv_icm20948_get_lpen_control(struct inv_icm20948 * s)
{
	return s->sAllowLpEn;
}

/*!
 ******************************************************************************
 *   @brief     This function sets the power state of the Ivory chip 
 *				loop
 *   @param[in] Function - CHIP_AWAKE, CHIP_LP_ENABLE
 *   @param[in] On/Off - The functions are enabled if previously disabled and 
                disabled if previously enabled based on the value of On/Off.
 ******************************************************************************
 */ 
int inv_icm20948_set_chip_power_state(struct inv_icm20948 * s, unsigned char func, unsigned char on_off)
{
	int status = 0;

	switch(func) {

		case CHIP_AWAKE:    
			if(on_off){
				if((s->base_state.wake_state & CHIP_AWAKE) == 0) {// undo sleep_en
					s->base_state.pwr_mgmt_1 &= ~BIT_SLEEP;
					status = inv_icm20948_write_single_mems_reg_core(s, REG_PWR_MGMT_1, s->base_state.pwr_mgmt_1);
					s->base_state.wake_state |= CHIP_AWAKE;
					inv_icm20948_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
				}
			} else {
				if(s->base_state.wake_state & CHIP_AWAKE) {// set sleep_en
					s->base_state.pwr_mgmt_1 |= BIT_SLEEP;
					status = inv_icm20948_write_single_mems_reg_core(s, REG_PWR_MGMT_1, s->base_state.pwr_mgmt_1);
					s->base_state.wake_state &= ~CHIP_AWAKE;
					inv_icm20948_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
				}
			}
		break;

		case CHIP_LP_ENABLE:
			if(s->base_state.lp_en_support == 1) {
				if(on_off) {
					if( (inv_icm20948_get_lpen_control(s)) && ((s->base_state.wake_state & CHIP_LP_ENABLE) == 0)){
						s->base_state.pwr_mgmt_1 |= BIT_LP_EN; // lp_en ON
						status = inv_icm20948_write_single_mems_reg_core(s, REG_PWR_MGMT_1, s->base_state.pwr_mgmt_1);
						s->base_state.wake_state |= CHIP_LP_ENABLE;
					}
				} else {
					if(s->base_state.wake_state & CHIP_LP_ENABLE){
						s->base_state.pwr_mgmt_1 &= ~BIT_LP_EN; // lp_en off
						status = inv_icm20948_write_single_mems_reg_core(s, REG_PWR_MGMT_1, s->base_state.pwr_mgmt_1);
						s->base_state.wake_state &= ~CHIP_LP_ENABLE;
						inv_icm20948_sleep_100us(1); // after writing the bit wait 100 Micro Seconds
					}
				}
			}
		break;

		default:
		break;

	}// end switch

	return status;
}

/*!
 ******************************************************************************
 *   @return    Current wake status of the Ivory chip.
 ******************************************************************************
 */
uint8_t inv_icm20948_get_chip_power_state(struct inv_icm20948 * s)
{
	return s->base_state.wake_state;
}

/** Wakes up DMP3 (SMARTSENSOR).
*/
int inv_icm20948_wakeup_mems(struct inv_icm20948 * s)
{
	unsigned char data;
	int result = 0;

	result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
		s->base_state.user_ctrl |= BIT_I2C_IF_DIS;
		inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl);  
	}

	data = 0x47;	// FIXME, should set up according to sensor/engines enabled.
	result |= inv_icm20948_write_mems_reg(s, REG_PWR_MGMT_2, 1, &data);

	if(s->base_state.firmware_loaded == 1) {
		s->base_state.user_ctrl |= BIT_DMP_EN | BIT_FIFO_EN;
		result |= inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl);  
	}

	result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);
	return result;
}

/** Puts DMP3 (SMARTSENSOR) into the lowest power state. Assumes sensors are all off.
*/
int inv_icm20948_sleep_mems(struct inv_icm20948 * s)
{
	int result;
	unsigned char data;

	data = 0x7F;
	result = inv_icm20948_write_mems_reg(s, REG_PWR_MGMT_2, 1, &data);

	result |= inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 0);

	return result;
}

int inv_icm20948_set_dmp_address(struct inv_icm20948 * s)
{
	int result;
	unsigned char dmp_cfg[2] = {0};
	unsigned short config;

	// Write DMP Start address
	inv_icm20948_get_dmp_start_address(s, &config);
	/* setup DMP start address and firmware */
	dmp_cfg[0] = (unsigned char)((config >> 8) & 0xff);
	dmp_cfg[1] = (unsigned char)(config & 0xff);

	result = inv_icm20948_write_mems_reg(s, REG_PRGM_START_ADDRH, 2, dmp_cfg);
	return result;
}

/**
*  @brief      Set up the secondary I2C bus on 20630.
*  @param[in]  MPU state varible
*  @return     0 if successful.
*/

int inv_icm20948_set_secondary(struct inv_icm20948 * s)
{
	int r = 0;
	static uint8_t lIsInited = 0;

	if(lIsInited == 0) {
		r  = inv_icm20948_write_single_mems_reg(s, REG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR);
		r |= inv_icm20948_write_single_mems_reg(s, REG_I2C_MST_ODR_CONFIG, MIN_MST_ODR_CONFIG);

		lIsInited = 1;
	}
	return r;
}

int inv_icm20948_enter_duty_cycle_mode(struct inv_icm20948 * s)
{
	/* secondary cycle mode should be set all the time */
	unsigned char data  = BIT_I2C_MST_CYCLE|BIT_ACCEL_CYCLE|BIT_GYRO_CYCLE;

	s->base_state.chip_lp_ln_mode = CHIP_LOW_POWER_ICM20948;
	return inv_icm20948_write_mems_reg(s, REG_LP_CONFIG, 1, &data);
}

int inv_icm20948_enter_low_noise_mode(struct inv_icm20948 * s)
{
	/* secondary cycle mode should be set all the time */
	unsigned char data  = BIT_I2C_MST_CYCLE;

	s->base_state.chip_lp_ln_mode = CHIP_LOW_NOISE_ICM20948;
	return inv_icm20948_write_mems_reg(s, REG_LP_CONFIG, 1, &data);
}

/** Should be called once on power up. Loads DMP3, initializes internal variables needed 
*   for other lower driver functions.
*/
int inv_icm20948_initialize_lower_driver(struct inv_icm20948 * s, enum SMARTSENSOR_SERIAL_INTERFACE type, 
	const uint8_t *dmp3_image, uint32_t dmp3_image_size)
{
	int result = 0;
	static unsigned char data;
	// set static variable
	s->sAllowLpEn = 1;
	s->s_compass_available = 0;
	// ICM20948 do not support the proximity sensor for the moment.
	// s_proximity_available variable is nerver changes
	s->s_proximity_available = 0;

	// Set varialbes to default values
	memset(&s->base_state, 0, sizeof(s->base_state));
	s->base_state.pwr_mgmt_1 = BIT_CLK_PLL;
	s->base_state.pwr_mgmt_2 = BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY | BIT_PWR_PRESSURE_STBY;
	s->base_state.serial_interface = type;
	result |= inv_icm20948_read_mems_reg(s, REG_USER_CTRL, 1, &s->base_state.user_ctrl);

	result |= inv_icm20948_wakeup_mems(s);

	result |= inv_icm20948_read_mems_reg(s, REG_WHO_AM_I, 1, &data);

	/* secondary cycle mode should be set all the time */
	data = BIT_I2C_MST_CYCLE|BIT_ACCEL_CYCLE|BIT_GYRO_CYCLE;

	// Set default mode to low power mode
	result |= inv_icm20948_set_lowpower_or_highperformance(s, 0);
	
	// Disable Ivory DMP.
	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI)   
		s->base_state.user_ctrl = BIT_I2C_IF_DIS;
	else
		s->base_state.user_ctrl = 0;

	result |= inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl);

	//Setup Ivory DMP.
	result |= inv_icm20948_load_firmware(s, dmp3_image, dmp3_image_size);
	if(result)
		return result;
	else
		s->base_state.firmware_loaded = 1;
	result |= inv_icm20948_set_dmp_address(s);
	// Turn off all sensors on DMP by default.
	//result |= dmp_set_data_output_control1(0);   // FIXME in DMP, these should be off by default.
	result |= dmp_icm20948_reset_control_registers(s);
	
	// set FIFO watermark to 80% of actual FIFO size
	result |= dmp_icm20948_set_FIFO_watermark(s, 800);

	// Enable Interrupts.
	data = 0x2;
	result |= inv_icm20948_write_mems_reg(s, REG_INT_ENABLE, 1, &data); // Enable DMP Interrupt
	data = 0x1;
	result |= inv_icm20948_write_mems_reg(s, REG_INT_ENABLE_2, 1, &data); // Enable FIFO Overflow Interrupt

	// TRACKING : To have accelerometers datas and the interrupt without gyro enables.
	data = 0XE4;
	result |= inv_icm20948_write_mems_reg(s, REG_SINGLE_FIFO_PRIORITY_SEL, 1, &data);

	// Disable HW temp fix
	inv_icm20948_read_mems_reg(s, REG_HW_FIX_DISABLE,1,&data);
	data |= 0x08;
	inv_icm20948_write_mems_reg(s, REG_HW_FIX_DISABLE,1,&data);

	// Setup MEMs properties.
	s->base_state.accel_averaging = 1; //Change this value if higher sensor sample avergaing is required.
	s->base_state.gyro_averaging = 1;  //Change this value if higher sensor sample avergaing is required.
	inv_icm20948_set_gyro_divider(s, FIFO_DIVIDER);       //Initial sampling rate 1125Hz/19+1 = 56Hz.
	inv_icm20948_set_accel_divider(s, FIFO_DIVIDER);      //Initial sampling rate 1125Hz/19+1 = 56Hz.

	// Init the sample rate to 56 Hz for BAC,STEPC and B2S
	dmp_icm20948_set_bac_rate(s, DMP_ALGO_FREQ_56);
	dmp_icm20948_set_b2s_rate(s, DMP_ALGO_FREQ_56);

	// FIFO Setup.
	result |= inv_icm20948_write_single_mems_reg(s, REG_FIFO_CFG, BIT_SINGLE_FIFO_CFG); // FIFO Config. fixme do once? burst write?
	result |= inv_icm20948_write_single_mems_reg(s, REG_FIFO_RST, 0x1f); // Reset all FIFOs.
	result |= inv_icm20948_write_single_mems_reg(s, REG_FIFO_RST, 0x1e); // Keep all but Gyro FIFO in reset.
	result |= inv_icm20948_write_single_mems_reg(s, REG_FIFO_EN, 0x0); // Slave FIFO turned off.
	result |= inv_icm20948_write_single_mems_reg(s, REG_FIFO_EN_2, 0x0); // Hardware FIFO turned off.
    
	s->base_state.lp_en_support = 1;
	
	if(s->base_state.lp_en_support == 1)
		inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	result |= inv_icm20948_sleep_mems(s);   
        
	return result;
}

static void activate_compass(struct inv_icm20948 * s)
{
	s->s_compass_available = 1;
}

static void desactivate_compass(struct inv_icm20948 * s)
{
	s->s_compass_available = 0;
}

int inv_icm20948_get_compass_availability(struct inv_icm20948 * s)
{
	return s->s_compass_available;
}

// return true 1 if gyro was enabled, otherwise false 0
static unsigned char inv_is_gyro_enabled(struct inv_icm20948 * s)
{
	if ((s->inv_androidSensorsOn_mask[0] & INV_NEEDS_GYRO_MASK) || (s->inv_androidSensorsOn_mask[1] & INV_NEEDS_GYRO_MASK1))
		return 1;
	return 0;
}

int inv_icm20948_get_proximity_availability(struct inv_icm20948 * s)
{
	return s->s_proximity_available;
}

int inv_icm20948_set_slave_compass_id(struct inv_icm20948 * s, int id)
{
	int result = 0;
	(void)id;

	//result = inv_icm20948_wakeup_mems(s);
	//if (result)
	//	return result;
		
	inv_icm20948_prevent_lpen_control(s);
	activate_compass(s);
	
	inv_icm20948_init_secondary(s);

	// Set up the secondary I2C bus on 20630.
	inv_icm20948_set_secondary(s);

	//Setup Compass
	result = inv_icm20948_setup_compass_akm(s);

	//Setup Compass mounting matrix into DMP
	result |= inv_icm20948_compass_dmp_cal(s, s->mounting_matrix, s->mounting_matrix_secondary_compass);
	
	if (result)
		desactivate_compass(s);

	//result = inv_icm20948_sleep_mems(s);
	inv_icm20948_allow_lpen_control(s);
	return result;
}

int inv_icm20948_set_gyro_divider(struct inv_icm20948 * s, unsigned char div)
{
	s->base_state.gyro_div = div;
	return inv_icm20948_write_mems_reg(s, REG_GYRO_SMPLRT_DIV, 1, &div);
}

unsigned char inv_icm20948_get_gyro_divider(struct inv_icm20948 * s)
{
	return s->base_state.gyro_div;
}

int inv_icm20948_set_secondary_divider(struct inv_icm20948 * s, unsigned char div)
{
	s->base_state.secondary_div = 1UL<<div;
	return inv_icm20948_write_single_mems_reg(s, REG_I2C_MST_ODR_CONFIG, div);
}

unsigned short inv_icm20948_get_secondary_divider(struct inv_icm20948 * s)
{
	return s->base_state.secondary_div;
}

int inv_icm20948_set_accel_divider(struct inv_icm20948 * s, short div)
{
	unsigned char data[2] = {0};

	s->base_state.accel_div = div;
	data[0] = (unsigned char)(div >> 8);
	data[1] = (unsigned char)(div & 0xff);

	return inv_icm20948_write_mems_reg(s, REG_ACCEL_SMPLRT_DIV_1, 2, data);
}

short inv_icm20948_get_accel_divider(struct inv_icm20948 * s)
{
	return s->base_state.accel_div;
}

/*
 You can obtain the real odr in Milliseconds, Micro Seconds or Ticks.
 Use the enum values: ODR_IN_Ms, ODR_IN_Us or ODR_IN_Ticks,
 when calling inv_icm20948_get_odr_in_units().
*/
uint32_t inv_icm20948_get_odr_in_units(struct inv_icm20948 * s, unsigned short odrInDivider, unsigned char odr_units )
{
	unsigned long odr=0;
	unsigned long Us=0;
	unsigned char PLL=0, gyro_is_on=0;

	if(s->base_state.timebase_correction_pll == 0)
		inv_icm20948_read_mems_reg(s, REG_TIMEBASE_CORRECTION_PLL, 1, &s->base_state.timebase_correction_pll);
	
	PLL = s->base_state.timebase_correction_pll;

	// check if Gyro is currently enabled
	gyro_is_on = inv_is_gyro_enabled(s);

	if( PLL < 0x80 ) { // correction positive
		// In Micro Seconds
		Us = (odrInDivider*1000000L/1125L) * (1270L)/(1270L+ (gyro_is_on ? PLL : 0));
	} 
	else {
		PLL &= 0x7F;

		// In Micro Seconds 
		Us = (odrInDivider*1000000L/1125L) * (1270L)/(1270L-(gyro_is_on ? PLL : 0));
	}

	switch( odr_units ) {
		// ret in Milliseconds 
		case ODR_IN_Ms:
			odr = Us/1000;
			break;

		// ret in Micro
		case ODR_IN_Us:
			odr = Us;
			break;

		// ret in Ticks
		case ODR_IN_Ticks:
			odr = (Us/1000) * (32768/1125);// According to Mars
			break;
	}

	return odr;
}
 
/**
* Sets the DMP for a particular gyro configuration.
* @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
*            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
*            10=102.2727Hz sample rate, ... etc.
* @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
*/
int inv_icm20948_set_gyro_sf(struct inv_icm20948 * s, unsigned char div, int gyro_level)
{
	long gyro_sf;
	static long lLastGyroSf = 0;
	int result = 0;

	// gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
	gyro_level = 4;

	if(s->base_state.timebase_correction_pll == 0)
		result |= inv_icm20948_read_mems_reg(s, REG_TIMEBASE_CORRECTION_PLL, 1, &s->base_state.timebase_correction_pll);

	{
		unsigned long long const MagicConstant = 264446880937391LL;
		unsigned long long const MagicConstantScale = 100000LL;
		unsigned long long ResultLL;

		if (s->base_state.timebase_correction_pll & 0x80) {
			ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 - (s->base_state.timebase_correction_pll & 0x7F)) / MagicConstantScale);
		}
		else {
			ResultLL = (MagicConstant * (long long)(1ULL << gyro_level) * (1 + div) / (1270 + s->base_state.timebase_correction_pll) / MagicConstantScale);
		}
		/*
		    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
		    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
		    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
		*/
		if  (ResultLL > 0x7FFFFFFF) 
			gyro_sf = 0x7FFFFFFF;
		else
			gyro_sf = (long)ResultLL;
	}

	if (gyro_sf != lLastGyroSf) {
		result |= dmp_icm20948_set_gyro_sf(s, gyro_sf);
		lLastGyroSf = gyro_sf;
	}

	return result;
}

int inv_icm20948_set_gyro_fullscale(struct inv_icm20948 * s, int level)
{
	int result;
	s->base_state.gyro_fullscale = level;
	result = inv_icm20948_set_icm20948_gyro_fullscale(s, level);
	result |= inv_icm20948_set_gyro_sf(s, s->base_state.gyro_div, level);
	result |= dmp_icm20948_set_gyro_fsr(s, 250<<level);

	return result;
}

uint8_t inv_icm20948_get_gyro_fullscale(struct inv_icm20948 * s)
{
	return s->base_state.gyro_fullscale;
}


int inv_icm20948_set_icm20948_gyro_fullscale(struct inv_icm20948 * s, int level)
{
	int result = 0;
	unsigned char gyro_config_1_reg;
	unsigned char gyro_config_2_reg;
	unsigned char dec3_cfg;
	if (level >= NUM_MPU_GFS)
		return -1;

	result |= inv_icm20948_read_mems_reg(s, REG_GYRO_CONFIG_1, 1, &gyro_config_1_reg);
	gyro_config_1_reg &= 0xC0;
	gyro_config_1_reg |= (level << 1) | 1;  //fchoice = 1, filter = 0.
	result |= inv_icm20948_write_mems_reg(s, REG_GYRO_CONFIG_1, 1, &gyro_config_1_reg);

	result |= inv_icm20948_read_mems_reg(s, REG_GYRO_CONFIG_2, 1, &gyro_config_2_reg);
	gyro_config_2_reg &= 0xF8;
	
	switch(s->base_state.gyro_averaging) {
		case 1:
			dec3_cfg = 0;
			break;

		case 2:
			dec3_cfg = 1;
			break;

		case 4:
			dec3_cfg = 2;
			break;

		case 8:
			dec3_cfg = 3;
			break;

		case 16:
			dec3_cfg = 4;
			break;

		case 32:
			dec3_cfg = 5;
			break;

		case 64:
			dec3_cfg = 6;
			break;

		case 128:
			dec3_cfg = 7;
			break;

		default:
			dec3_cfg = 0;
			break;
	}
	gyro_config_2_reg |= dec3_cfg;  
	result |= inv_icm20948_write_single_mems_reg(s, REG_GYRO_CONFIG_2, gyro_config_2_reg);
	return result;
}


int inv_icm20948_set_accel_fullscale(struct inv_icm20948 * s, int level)
{
	int result;
	s->base_state.accel_fullscale = level;
	result = inv_icm20948_set_icm20948_accel_fullscale(s, level);
	result |= dmp_icm20948_set_accel_fsr(s, 2<<level);
	result |= dmp_icm20948_set_accel_scale2(s, 2<<level);
	return result;
}

uint8_t inv_icm20948_get_accel_fullscale(struct inv_icm20948 * s)
{
	return s->base_state.accel_fullscale;
}


int inv_icm20948_set_icm20948_accel_fullscale(struct inv_icm20948 * s, int level)
{
	int result = 0;
	unsigned char accel_config_1_reg;
	unsigned char accel_config_2_reg;
	unsigned char dec3_cfg;

	if (level >= NUM_MPU_AFS)
		return -1;

	result |= inv_icm20948_read_mems_reg(s, REG_ACCEL_CONFIG, 1, &accel_config_1_reg);
	accel_config_1_reg &= 0xC0;

	if(s->base_state.accel_averaging > 1)
		accel_config_1_reg |= (7 << 3) | (level << 1) | 1;   //fchoice = 1, filter = 7.
	else
		accel_config_1_reg |= (level << 1) | 0;  //fchoice = 0, filter = 0.
	/* /!\ FCHOICE=0 considers we are in low power mode always and allows us to have correct values on raw data since not averaged,
	in case low noise mode is to be supported for 20649, please reconsider this value and update base sample rate from 1125 to 4500...
	*/
	result |= inv_icm20948_write_single_mems_reg(s, REG_ACCEL_CONFIG, accel_config_1_reg);

	switch(s->base_state.accel_averaging) {
		case 1:
			dec3_cfg = 0;
			break;

		case 4:
			dec3_cfg = 0;
			break;
		
		case 8:
			dec3_cfg = 1;
			break;
	
		case 16:
			dec3_cfg = 2;
			break;
		
		case 32:
			dec3_cfg = 3;
			break;

		default:
			dec3_cfg = 0;
			break;
	}

	result |= inv_icm20948_read_mems_reg(s, REG_ACCEL_CONFIG_2, 1, &accel_config_2_reg);
	accel_config_2_reg &= 0xFC;

	accel_config_2_reg |=  dec3_cfg;
	result |= inv_icm20948_write_single_mems_reg(s, REG_ACCEL_CONFIG_2, accel_config_2_reg);

	return result;
}


int inv_icm20948_enable_hw_sensors(struct inv_icm20948 * s, int bit_mask)
{
	int rc = 0;

	if ((s->base_state.pwr_mgmt_2 == (BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY | BIT_PWR_PRESSURE_STBY)) | (bit_mask & 0x80)) {
		// All sensors off, or override is on
		s->base_state.pwr_mgmt_2 = 0; // Zero means all sensors are on
		// Gyro and Accel were off
		if ((bit_mask & 2) == 0) {
			s->base_state.pwr_mgmt_2 = BIT_PWR_ACCEL_STBY; // Turn off accel
		}
		if ((bit_mask & 1) == 0) {
			s->base_state.pwr_mgmt_2 |= BIT_PWR_GYRO_STBY; // Turn off gyro
		}
		if ((bit_mask & 4) == 0) {
			s->base_state.pwr_mgmt_2 |= BIT_PWR_PRESSURE_STBY; // Turn off pressure
		}

		rc |= inv_icm20948_write_mems_reg(s, REG_PWR_MGMT_2, 1, &s->base_state.pwr_mgmt_2);
	}

	if (bit_mask & SECONDARY_COMPASS_AVAILABLE) {
		rc |= inv_icm20948_resume_akm(s);
	} 
	else {
		rc |= inv_icm20948_suspend_akm(s);
	}

	return rc;
}

int inv_icm20948_set_serial_comm(struct inv_icm20948 * s, enum SMARTSENSOR_SERIAL_INTERFACE type)
{
	s->base_state.serial_interface = type;

	return 0;
}


int inv_icm20948_set_int1_assertion(struct inv_icm20948 * s, int enable)
{
	int   result = 0;
	// unsigned char reg_pin_cfg;
	unsigned char reg_int_enable;

	// INT1 held until interrupt status is cleared
	/*
	result         |= inv_icm20948_read_mems_reg(s, REG_INT_PIN_CFG, 1, &reg_pin_cfg);
	reg_pin_cfg    |= BIT_INT_LATCH_EN ;	// Latchen : BIT5 held the IT until register is read
	result         |= inv_icm20948_write_single_mems_reg(s, REG_INT_PIN_CFG, reg_pin_cfg);
	*/

	// Set int1 enable
	result |= inv_icm20948_read_mems_reg(s, REG_INT_ENABLE, 1, &reg_int_enable);

	if(enable) { // Enable bit
		reg_int_enable |= BIT_DMP_INT_EN;
	}
	else { // Disable bit
		reg_int_enable &= ~BIT_DMP_INT_EN;
	}

	result |= inv_icm20948_write_single_mems_reg(s, REG_INT_ENABLE, reg_int_enable);

	return result;
}


/**
*  @brief      Read accel data stored in hw reg
*  @param[in]  level  See mpu_accel_fs
*  @return     0 if successful
*/
int inv_icm20948_accel_read_hw_reg_data(struct inv_icm20948 * s, short accel_hw_reg_data[3])
{
	int result = 0;
	uint8_t accel_data[6]; // Store 6 bytes for that

	// read mem regs
	result = inv_icm20948_read_mems_reg(s, REG_ACCEL_XOUT_H_SH, 6, (unsigned char *) &accel_data);

	// Assign axys !
	accel_hw_reg_data[0] = (accel_data[0] << 8) + accel_data[1];
	accel_hw_reg_data[1] = (accel_data[2] << 8) + accel_data[3];
	accel_hw_reg_data[2] = (accel_data[4] << 8) + accel_data[5];

	return result;
}

