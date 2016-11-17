/*
 * i2c_onboard_isolated.c
 *
 *  Created on: 28/05/2015
 *      Author: joao.rosa
 *
 *      Routines working fine!!
 *      Tested 15/07/2015
 *
 *      To work in the future: in single byte read, the I2C read function call 2 bytes, spending more time in it.
 */

#include "i2c_offboard_isolated.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"

#include <stdint.h>

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_udc_v2.0.h"

#define I2C_READ true
#define I2C_WRITE false
#define I2C_MESSAGE_SIZE 2

#define I2CWhileMasterBusy while (I2CMasterBusy(I2C_OFFBOARD_ISO_MASTER_BASE)) {}

void
ReadI2COffboardIsolated(uint8_t SLAVE_ADDR, uint8_t TYPE_REGISTER_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data)
{

	I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_WRITE);

	// Dummy Write to set the future read address
	I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data); // address zero in the EEPROM memory
	I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CWhileMasterBusy

	if(TYPE_REGISTER_ADDR == DOUBLE_ADDRESS)
	{
		data++; // Increase the pointer
		I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data); // Send second byte address
		I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		I2CWhileMasterBusy
		data--; // Decrease the pointer
	}

	// Start reading the address
	I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_READ);

	int i;
	for (i = 0; i < MESSAGE_SIZE; i++)
	{

		if(i == 0) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		else if(i == (MESSAGE_SIZE - 1)) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		else I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

		I2CWhileMasterBusy
		*data = I2CMasterDataGet(I2C_OFFBOARD_ISO_MASTER_BASE);
		data++;

	}

}

void
WriteI2COffboardIsolated(uint8_t SLAVE_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data)
{

	I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_WRITE);

	int i;
	for (i = 0; i < MESSAGE_SIZE; i++) { // +2 for the address byte
		I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data);

		if(i == 0) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		else if(i == (MESSAGE_SIZE - 1)) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		else I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
		data++;

		I2CWhileMasterBusy
	}
}

void
InitI2COffboardIsolated(void)
{
	// I2C0 configuration (EEPROM memory, IO expander e Temperature sensor.)
	// Data rate is set to 400kbps
	I2CMasterInitExpClk(I2C_OFFBOARD_ISO_MASTER_BASE, SysCtlClockGet(
		                       SYSTEM_CLOCK_SPEED), true);

	//I2C enable
	I2CMasterEnable(I2C_OFFBOARD_ISO_MASTER_BASE);

}


