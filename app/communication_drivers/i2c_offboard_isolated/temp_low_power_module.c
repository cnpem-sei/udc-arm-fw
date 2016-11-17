/*
 * temp_low_power_module.c
 *
 *  Created on: 26/10/2016
 *      Author: joao.rosa
 */


#include "temp_low_power_module.h"
#include "i2c_offboard_isolated.h"


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

uint8_t Temp1 = 0;

uint8_t data_temp[10];

void PowerSupplyTempInit(void)
{
	data_temp[0] = CONFIG_REGISTER;
	data_temp[1] = CONFIG_REGISTER_VALUE;
	WriteI2COffboardIsolated(I2C_SLV_ADDR_TEMP_SENSE_PS1, 0x02, data_temp);

	//
}

void PowerSupply1TempRead(void)
{
	data_temp[0] = TEMP_REGISTER;  // Temperature Register
	ReadI2COffboardIsolated(I2C_SLV_ADDR_TEMP_SENSE_PS1, SINGLE_ADDRESS, 0x02, data_temp);

	data_temp[5] = data_temp[0] << 1;
	data_temp[6] = data_temp[1] >> 7;
	data_temp[5] = data_temp[5] | data_temp[6];

	Temp1 = (uint8_t) data_temp[5] * 0.5;
}

uint8_t PowerSupply1Temp(void)
{
	return Temp1;
}
