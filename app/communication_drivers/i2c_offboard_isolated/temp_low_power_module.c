/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file temp_low_power_module.c
 * @brief Temperature sensor module.
 *
 * @author joao.rosa
 *
 * @date 26/10/2016
 *
 */

#include "communication_drivers/control/control.h"
#include "communication_drivers/ipc/ipc_lib.h"

#include "i2c_offboard_isolated.h"
#include "temp_low_power_module.h"

// TMP100 temperature sensor
#define I2C_SLV_ADDR_TEMP_SENSE_PS1 0x48 // 7 bits Power Supply 1 Address
#define I2C_SLV_ADDR_TEMP_SENSE_PS2 0x49 // 7 bits Power Supply 2 Address
#define I2C_SLV_ADDR_TEMP_SENSE_PS3 0x4A // 7 bits Power Supply 3 Address
#define I2C_SLV_ADDR_TEMP_SENSE_PS4 0x4C // 7 bits Power Supply 4 Address

#define TEMP_REGISTER		0x00
#define CONFIG_REGISTER		0x01
#define TLOW_REGISTER		0x02
#define THIGH_REGISTER		0x03

/*
 * Shutdown Mode (SD) = 0
 * Thermostat Mode (TM) = 0
 * Polarity (POL) = 0
 * Fault Queue (F1, F0) = 0
 * Converter Resolution (R1, R0) = 0 (9bits)
 */
#define CONFIG_REGISTER_VALUE	0x00

volatile uint8_t Temp1 = 0;
volatile float *tmp1;

volatile uint8_t Temp2 = 0;
volatile float *tmp2;

volatile uint8_t Temp3 = 0;
volatile float *tmp3;

volatile uint8_t Temp4 = 0;
volatile float *tmp4;

uint8_t data_temp[10];

void power_supply_1_temp_init(void)
{
	data_temp[0] = CONFIG_REGISTER;
	data_temp[1] = CONFIG_REGISTER_VALUE;
	write_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS1, 0x02, data_temp);

	tmp1 = &g_controller_mtoc.net_signals[13].f;
	*tmp1 = 0.0;
}

void power_supply_2_temp_init(void)
{
	data_temp[0] = CONFIG_REGISTER;
	data_temp[1] = CONFIG_REGISTER_VALUE;
	write_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS2, 0x02, data_temp);

	tmp2 = &g_controller_mtoc.net_signals[14].f;
	*tmp2 = 0.0;
}

void power_supply_3_temp_init(void)
{
	data_temp[0] = CONFIG_REGISTER;
	data_temp[1] = CONFIG_REGISTER_VALUE;
	write_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS3, 0x02, data_temp);

	tmp3 = &g_controller_mtoc.net_signals[15].f;
	*tmp3 = 0.0;
}

void power_supply_4_temp_init(void)
{
	data_temp[0] = CONFIG_REGISTER;
	data_temp[1] = CONFIG_REGISTER_VALUE;
	write_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS4, 0x02, data_temp);

	tmp4 = &g_controller_mtoc.net_signals[16].f;
	*tmp4 = 0.0;
}

void power_supply_1_temp_read(void)
{
	data_temp[0] = TEMP_REGISTER;  // Temperature Register
	read_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS1, SINGLE_ADDRESS,
	                                                       0x02, data_temp);

	data_temp[5] = data_temp[0] << 1;
	data_temp[6] = data_temp[1] >> 7;
	data_temp[5] = data_temp[5] | data_temp[6];

	Temp1 = (uint8_t) data_temp[5] * 0.5;

	*tmp1 = (float)Temp1;
}

void power_supply_2_temp_read(void)
{
	data_temp[0] = TEMP_REGISTER;  // Temperature Register
	read_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS2, SINGLE_ADDRESS,
	                                                       0x02, data_temp);

	data_temp[5] = data_temp[0] << 1;
	data_temp[6] = data_temp[1] >> 7;
	data_temp[5] = data_temp[5] | data_temp[6];

	Temp2 = (uint8_t) data_temp[5] * 0.5;

	*tmp2 = (float)Temp2;
}

void power_supply_3_temp_read(void)
{
	data_temp[0] = TEMP_REGISTER;  // Temperature Register
	read_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS3, SINGLE_ADDRESS,
	                                                       0x02, data_temp);

	data_temp[5] = data_temp[0] << 1;
	data_temp[6] = data_temp[1] >> 7;
	data_temp[5] = data_temp[5] | data_temp[6];

	Temp3 = (uint8_t) data_temp[5] * 0.5;

	*tmp3 = (float)Temp3;
}

void power_supply_4_temp_read(void)
{
	data_temp[0] = TEMP_REGISTER;  // Temperature Register
	read_i2c_offboard_isolated(I2C_SLV_ADDR_TEMP_SENSE_PS4, SINGLE_ADDRESS,
	                                                       0x02, data_temp);

	data_temp[5] = data_temp[0] << 1;
	data_temp[6] = data_temp[1] >> 7;
	data_temp[5] = data_temp[5] | data_temp[6];

	Temp4 = (uint8_t) data_temp[5] * 0.5;

	*tmp4 = (float)Temp4;
}

uint8_t power_supply_1_temp(void)
{
	return Temp1;
}

uint8_t power_supply_2_temp(void)
{
	return Temp2;
}

uint8_t power_supply_3_temp(void)
{
	return Temp3;
}

uint8_t power_supply_4_temp(void)
{
	return Temp4;
}
