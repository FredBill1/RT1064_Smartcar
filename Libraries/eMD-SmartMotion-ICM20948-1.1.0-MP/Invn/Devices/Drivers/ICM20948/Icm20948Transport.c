/*
* ________________________________________________________________________________________________________
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

#include "Icm20948Transport.h"
#include "Icm20948Serif.h"
#include "Icm20948.h"

struct inv_icm20948 * icm20948_instance;

int inv_icm20948_read_reg(struct inv_icm20948 * s, uint8_t reg,	uint8_t * buf, uint32_t len)
{
	return inv_icm20948_serif_read_reg(&s->serif, reg, buf, len);
}

int inv_icm20948_write_reg(struct inv_icm20948 * s, uint8_t reg, const uint8_t * buf, uint32_t len)
{
	return inv_icm20948_serif_write_reg(&s->serif, reg, buf, len);
}

void inv_icm20948_sleep_100us(unsigned long nHowMany100MicroSecondsToSleep)  // time in 100 us
{
	inv_icm20948_sleep_us(nHowMany100MicroSecondsToSleep * 100);
}

long inv_icm20948_get_tick_count(void)
{
	return (long)inv_icm20948_get_time_us();
}

/* driver transport function */

#include "Icm20948Defs.h"
#include "Icm20948DataBaseDriver.h"
#include "Icm20948DataBaseControl.h"

void inv_icm20948_transport_init(struct inv_icm20948 * s)
{
	s->lastBank = 0x7E;
	s->lLastBankSelected = 0xFF;
}

static uint8_t check_reg_access_lp_disable(struct inv_icm20948 * s, unsigned short reg)
{
	switch(reg){
	case REG_LP_CONFIG:      /** (BANK_0 | 0x05) */
	case REG_PWR_MGMT_1:     /** (BANK_0 | 0x06) */
	case REG_PWR_MGMT_2:     /** (BANK_0 | 0x07) */
	case REG_INT_PIN_CFG:    /** (BANK_0 | 0x0F) */
	case REG_INT_ENABLE:     /** (BANK_0 | 0x10) */
	case REG_FIFO_COUNT_H:   /** (BANK_0 | 0x70) */
	case REG_FIFO_COUNT_L:   /** (BANK_0 | 0x71) */
	case REG_FIFO_R_W:       /** (BANK_0 | 0x72) */
		return inv_icm20948_ctrl_get_batch_mode_status(s);
	case REG_FIFO_CFG:       /** (BANK_0 | 0x76) */
	case REG_MEM_BANK_SEL:   /** (BANK_0 | 0x7E) */
	case REG_BANK_SEL:       /** 0x7F */
	case REG_INT_STATUS:     /** (BANK_0 | 0x19) */
	case REG_DMP_INT_STATUS: /** (BANK_0 | 0x18) */
		return 0;
		break;
	default:
		break;
	}
	return 1;
}

/**
*  @brief      Set up the register bank register for accessing registers in 20630.
*  @param[in]  register bank number
*  @return     0 if successful.
*/

static int inv_set_bank(struct inv_icm20948 * s, unsigned char bank)
{
	int result;
	//if bank reg was set before, just return
	if(bank==s->lastBank) 
		return 0;
	else 
		s->lastBank = bank;

	result = inv_icm20948_read_reg(s, REG_BANK_SEL, &s->reg, 1);

	if (result)
		return result;

	s->reg &= 0xce;
	s->reg |= (bank << 4);
	result = inv_icm20948_write_reg(s, REG_BANK_SEL, &s->reg, 1);

	return result;
}

/* the following functions are used for configuring the secondary devices */

/**
*  @brief      Write data to a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Length of data
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
int inv_icm20948_write_mems_reg(struct inv_icm20948 * s, uint16_t reg, unsigned int length, const unsigned char *data)
{
	int result = 0;
	unsigned int bytesWrite = 0;
	unsigned char regOnly = (unsigned char)(reg & 0x7F);

	unsigned char power_state = inv_icm20948_get_chip_power_state(s);

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(check_reg_access_lp_disable(s, reg))    // Check if register needs LP_EN to be disabled   
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);  //Disable LP_EN

	result |= inv_set_bank(s, reg >> 7);

	while (bytesWrite<length) 
	{
		int thisLen = min(INV_MAX_SERIAL_WRITE, length-bytesWrite);

		result |= inv_icm20948_write_reg(s, regOnly+bytesWrite,&data[bytesWrite], thisLen);

		if (result)
			return result;

		bytesWrite += thisLen;
	}

	if(check_reg_access_lp_disable(s, reg))   //Enable LP_EN since we disabled it at begining of this function.
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	return result;
}

/**
*  @brief      Write single byte of data to a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
int inv_icm20948_write_single_mems_reg(struct inv_icm20948 * s, uint16_t reg, const unsigned char data)
{
	int result = 0;
	unsigned char regOnly = (unsigned char)(reg & 0x7F);


	unsigned char power_state = inv_icm20948_get_chip_power_state(s);

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(check_reg_access_lp_disable(s, reg))   // Check if register needs LP_EN to be disabled
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);  //Disable LP_EN

	result |= inv_set_bank(s, reg >> 7);
	result |= inv_icm20948_write_reg(s, regOnly, &data, 1);

	if(check_reg_access_lp_disable(s, reg))   //Enable LP_EN since we disabled it at begining of this function.
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	return result;
}

/**
*  @brief      Read data from a register on MEMs.
*  @param[in]  Register address
*  @param[in]  Length of data
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
int inv_icm20948_read_mems_reg(struct inv_icm20948 * s, uint16_t reg, unsigned int length, unsigned char *data)
{
	int result = 0;
	unsigned int bytesRead = 0;
	unsigned char regOnly = (unsigned char)(reg & 0x7F);
	unsigned char i, dat[INV_MAX_SERIAL_READ];
	unsigned char power_state = inv_icm20948_get_chip_power_state(s);

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(check_reg_access_lp_disable(s, reg))   // Check if register needs LP_EN to be disabled
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);  //Disable LP_EN

	result |= inv_set_bank(s, reg >> 7);

	while (bytesRead<length) 
	{
		int thisLen = min(INV_MAX_SERIAL_READ, length-bytesRead);
		if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
			result |= inv_icm20948_read_reg(s, regOnly+bytesRead, &dat[bytesRead], thisLen);
		} else {
			result |= inv_icm20948_read_reg(s, regOnly+bytesRead, &data[bytesRead],thisLen);
		}

		if (result)
			return result;

		bytesRead += thisLen;
	}

	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
		for (i=0; i< length; i++) {
			*data= dat[i];
			data++;
		}
	}

	if(check_reg_access_lp_disable(s, reg))    // Check if register needs LP_EN to be enabled  
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);  //Enable LP_EN

	return result;
}

/**
*  @brief      Read data from a register in DMP memory 
*  @param[in]  DMP memory address
*  @param[in]  number of byte to be read
*  @param[in]  input data from the register
*  @return     0 if successful.
*/
int inv_icm20948_read_mems(struct inv_icm20948 * s, unsigned short reg, unsigned int length, unsigned char *data)
{
	int result=0;
	unsigned int bytesWritten = 0;
	unsigned int thisLen;
	unsigned char i, dat[INV_MAX_SERIAL_READ] = {0};
	unsigned char power_state = inv_icm20948_get_chip_power_state(s);
	unsigned char lBankSelected;
	unsigned char lStartAddrSelected;

	if(!data)
		return -1;

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	if(check_reg_access_lp_disable(s, reg))
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);

	result |= inv_set_bank(s, 0);

	lBankSelected = (reg >> 8);
	if (lBankSelected != s->lLastBankSelected)
	{
		result |= inv_icm20948_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
		if (result)
			return result;
		s->lLastBankSelected = lBankSelected;
	}

	while (bytesWritten < length) 
	{
		lStartAddrSelected = (reg & 0xff);
		/* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		Contents are changed after read or write of the selected memory.
		This register must be written prior to each access to initialize the register to the proper starting address.
		The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
		result |= inv_icm20948_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
		if (result)
			return result;

		thisLen = min(INV_MAX_SERIAL_READ, length-bytesWritten);
		/* Write data */
		if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
			result |= inv_icm20948_read_reg(s, REG_MEM_R_W, &dat[bytesWritten], thisLen);
		} else {
			result |= inv_icm20948_read_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
		}
		if (result)
			return result;

		bytesWritten += thisLen;
		reg += thisLen;
	}

	if(s->base_state.serial_interface == SERIAL_INTERFACE_SPI) {
		for (i=0; i< length; i++) {
			*data= dat[i];
			data++;
		}
	}

	//Enable LP_EN if we disabled it at begining of this function.
	if(check_reg_access_lp_disable(s, reg))
		result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	return result;
}

/**
*  @brief       Write data to a register in DMP memory 
*  @param[in]   DMP memory address
*  @param[in]   number of byte to be written
*  @param[out]  output data from the register
*  @return     0 if successful.
*/
int inv_icm20948_write_mems(struct inv_icm20948 * s, unsigned short reg, unsigned int length, const unsigned char *data)
{
	int result=0;
	unsigned int bytesWritten = 0;
	unsigned int thisLen;
	unsigned char lBankSelected;
	unsigned char lStartAddrSelected;

	unsigned char power_state = inv_icm20948_get_chip_power_state(s);

	if(!data)
		return -1;

	if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		result = inv_icm20948_set_chip_power_state(s, CHIP_AWAKE, 1);

	result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 0);

	result |= inv_set_bank(s, 0);

	lBankSelected = (reg >> 8);
	if (lBankSelected != s->lLastBankSelected)
	{
		result |= inv_icm20948_write_reg(s, REG_MEM_BANK_SEL, &lBankSelected, 1);
		if (result)
			return result;
		s->lLastBankSelected = lBankSelected;
	}

	while (bytesWritten < length) 
	{
		lStartAddrSelected = (reg & 0xff);
		/* Sets the starting read or write address for the selected memory, inside of the selected page (see MEM_SEL Register).
		Contents are changed after read or write of the selected memory.
		This register must be written prior to each access to initialize the register to the proper starting address.
		The address will auto increment during burst transactions.  Two consecutive bursts without re-initializing the start address would skip one address. */
		result |= inv_icm20948_write_reg(s, REG_MEM_START_ADDR, &lStartAddrSelected, 1);
		if (result)
			return result;

		thisLen = min(INV_MAX_SERIAL_WRITE, length-bytesWritten);

		/* Write data */ 
		result |= inv_icm20948_write_reg(s, REG_MEM_R_W, &data[bytesWritten], thisLen);
		if (result)
			return result;

		bytesWritten += thisLen;
		reg += thisLen;
	}

	//Enable LP_EN since we disabled it at begining of this function.
	result |= inv_icm20948_set_chip_power_state(s, CHIP_LP_ENABLE, 1);

	return result;
}

/**
*  @brief      Write single byte of data to a register on MEMs with no power control
*  @param[in]  Register address
*  @param[in]  Data to be written
*  @return     0 if successful.
*/
int inv_icm20948_write_single_mems_reg_core(struct inv_icm20948 * s, uint16_t reg, const uint8_t data)
{
	int result = 0;
	unsigned char regOnly = (unsigned char)(reg & 0x7F);

	result |= inv_set_bank(s, reg >> 7);
	result |= inv_icm20948_write_reg(s, regOnly, &data, 1);

	return result;
}
