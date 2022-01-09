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
#include "Icm20948Defs.h"
#include "Icm20948DataBaseDriver.h"

#include "Icm20948AuxTransport.h"

void inv_icm20948_init_secondary(struct inv_icm20948 * s)
{
	s->secondary_state.slv_reg[0].addr = REG_I2C_SLV0_ADDR;
	s->secondary_state.slv_reg[0].reg  = REG_I2C_SLV0_REG;
	s->secondary_state.slv_reg[0].ctrl = REG_I2C_SLV0_CTRL;
	s->secondary_state.slv_reg[0].d0   = REG_I2C_SLV0_DO;
    
    s->secondary_state.slv_reg[1].addr = REG_I2C_SLV1_ADDR;
	s->secondary_state.slv_reg[1].reg  = REG_I2C_SLV1_REG;
	s->secondary_state.slv_reg[1].ctrl = REG_I2C_SLV1_CTRL;
	s->secondary_state.slv_reg[1].d0   = REG_I2C_SLV1_DO;
    
    s->secondary_state.slv_reg[2].addr = REG_I2C_SLV2_ADDR;
	s->secondary_state.slv_reg[2].reg  = REG_I2C_SLV2_REG;
	s->secondary_state.slv_reg[2].ctrl = REG_I2C_SLV2_CTRL;
	s->secondary_state.slv_reg[2].d0   = REG_I2C_SLV2_DO;
    
	s->secondary_state.slv_reg[3].addr = REG_I2C_SLV3_ADDR;
	s->secondary_state.slv_reg[3].reg  = REG_I2C_SLV3_REG;
	s->secondary_state.slv_reg[3].ctrl = REG_I2C_SLV3_CTRL;
	s->secondary_state.slv_reg[3].d0   = REG_I2C_SLV3_DO;
	
	/* Make sure that by default all channels are disabled 
	To not inherit from a previous configuration from a previous run*/
	inv_icm20948_secondary_stop_channel(s, 0);
	inv_icm20948_secondary_stop_channel(s, 1);
	inv_icm20948_secondary_stop_channel(s, 2);
	inv_icm20948_secondary_stop_channel(s, 3);
}

/* the following functions are used for configuring the secondary devices */

/*
* inv_configure_secondary_read(): set secondary registers for reading.
The chip must be set as bank 3 before calling.
* This is derived from inv_icm20948_read_secondary in linux...
* for now, uses a very simple data struct for the registers
* 
* index gives the mapping to the particular SLVx registers
* addr is the physical address of the device to be accessed
* reg is the device register we wish to access
* len is the number of bytes to be read
* 
*/
int inv_icm20948_read_secondary(struct inv_icm20948 * s, int index, unsigned char addr, unsigned char reg, char len)
{
	int result = 0;
    unsigned char data;

    data = INV_MPU_BIT_I2C_READ | addr;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].addr, 1, &data);

    data = reg;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].reg, 1, &data);
    
    data = INV_MPU_BIT_SLV_EN | len;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].ctrl, 1, &data);
    
	return result;
}

int inv_icm20948_execute_read_secondary(struct inv_icm20948 * s, int index, unsigned char addr, int reg, int len, uint8_t *d)
{
	int result = 0;

	result |= inv_icm20948_read_secondary(s, index, addr, reg, len);
	
	result |= inv_icm20948_secondary_enable_i2c(s);
    
	inv_icm20948_sleep_us(SECONDARY_INIT_WAIT*1000);
    
	result |= inv_icm20948_secondary_disable_i2c(s);

    result |= inv_icm20948_read_mems_reg(s, REG_EXT_SLV_SENS_DATA_00, len, d); 

	result |= inv_icm20948_secondary_stop_channel(s, index);

	return result;
}

/*
* inv_icm20948_write_secondary(): set secondary registers for writing?.
The chip must be set as bank 3 before calling.
* This is derived from inv_icm20948_write_secondary in linux...
* for now, uses a very simple data struct for the registers
* 
* index gives the mapping to the particular SLVx registers
* addr is the physical address of the device to be accessed
* reg is the device register we wish to access
* len is the number of bytes to be read
* 
*/
int inv_icm20948_write_secondary(struct inv_icm20948 * s, int index, unsigned char addr, unsigned char reg, char v)
{
	int result = 0;
    unsigned char data;
    
    data = (unsigned char)addr;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].addr, 1, &data);

    data = reg;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].reg, 1, &data);

    data = v;
    result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].d0, 1, &data);
    
    data = INV_MPU_BIT_SLV_EN | 1;
	result |= inv_icm20948_write_mems_reg(s, s->secondary_state.slv_reg[index].ctrl, 1, &data);
    
    return result;
}

int inv_icm20948_execute_write_secondary(struct inv_icm20948 * s, int index, unsigned char addr, int reg, uint8_t v)
{
	int result = 0;

	result |= inv_icm20948_write_secondary(s, index, addr, reg, v);
	
	result |= inv_icm20948_secondary_enable_i2c(s);
    
	inv_icm20948_sleep_us(SECONDARY_INIT_WAIT*1000);
    
	result |= inv_icm20948_secondary_disable_i2c(s);

	result |= inv_icm20948_secondary_stop_channel(s, index);

	return result;
}

void inv_icm20948_secondary_saveI2cOdr(struct inv_icm20948 * s)
{
	inv_icm20948_read_mems_reg(s, REG_I2C_MST_ODR_CONFIG,1,&s->secondary_state.sSavedI2cOdr);
}

void inv_icm20948_secondary_restoreI2cOdr(struct inv_icm20948 * s)
{
	inv_icm20948_write_single_mems_reg(s, REG_I2C_MST_ODR_CONFIG,s->secondary_state.sSavedI2cOdr);
}

int inv_icm20948_secondary_stop_channel(struct inv_icm20948 * s, int index)
{
	return inv_icm20948_write_single_mems_reg(s, s->secondary_state.slv_reg[index].ctrl, 0);
}

int inv_icm20948_secondary_enable_i2c(struct inv_icm20948 * s)
{
	s->base_state.user_ctrl |= BIT_I2C_MST_EN;
	return inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl); 
}

int inv_icm20948_secondary_disable_i2c(struct inv_icm20948 * s)
{
	s->base_state.user_ctrl &= ~BIT_I2C_MST_EN;
	return inv_icm20948_write_single_mems_reg(s, REG_USER_CTRL, s->base_state.user_ctrl); 
}


int inv_icm20948_secondary_set_odr(struct inv_icm20948 * s, int divider, unsigned int* effectiveDivider)
{
	int mst_odr_config = 0;

    // find 2^x = divider to fit BASE_SAMPLE_RATE/2^REG_I2C_MST_ODR_CONFIG
    do
    {
		divider>>=1;
		mst_odr_config++;
    } while(divider>>1);
    
	if (mst_odr_config < MIN_MST_ODR_CONFIG)
		mst_odr_config = MIN_MST_ODR_CONFIG;

	*effectiveDivider = 1<<mst_odr_config;

	return	inv_icm20948_set_secondary_divider(s, (unsigned char)mst_odr_config);
}
