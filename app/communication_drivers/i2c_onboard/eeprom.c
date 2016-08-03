/*
 * eeprom.c
 *
 *  Created on: 13/07/2015
 *      Author: joao.rosa
 *
 *      Memory PN: AT24C64D-SSHM-T
 *
 *      Routines working fine!!
 *      Tested 15/07/2015
 */


#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"


#include "eeprom.h"
#include "i2c_onboard.h"

#include "set_pinout_udc_v2.0.h"

#include <stdint.h>

//***********************************************************************************
//  The memory address is compose of 13bits (2 bytes)
//  Byte MSB with 5 bits [---1 1111] and the LSB with 8 bits [1111 1111]
//  The 8 MSB bits (5 in the MSB byte + 3 in the LSB byte) correspond to the page address and the lasts 5 LSB bits are used to point one of 32bytes available in a memory page.
//
//  Memory page [8 bits], Byte in the page [5 bits]
//
//***********************************************************************************

#define I2C_SLV_ADDR_EEPROM	0x50 // Endereço 7 bits

// Memory map
#define	IP_ADDR			0x0140
#define IPMASK_ADDR		0x0144

#define	RS485_ADDR		0x0160
#define	RS485_BR		0x0161
#define RS485BKP_ADDR	0x0165
#define	RS485BKP_BR		0x0166

#define GAIN_KP			0x0200
#define	GAIN_KI			0x0204
#define	GAIN_KD			0x0208

#define PSMODEL			0x0100


uint8_t data_eeprom[32];

// Split float in bytes
union
{
   float f;
   char c[4];
} floatNchars;

volatile unsigned long ulLoop;


//***********************************************************************************
//                            IP DATA
//***********************************************************************************
uint32_t
EepromReadIP(void)
{
	uint32_t IP = 0;
	data_eeprom[0] = IP_ADDR >> 8; //Memory address MSB
	data_eeprom[1] = IP_ADDR; //Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	IP = data_eeprom[0];
	IP = IP << 8;
	IP |= data_eeprom[1];
	IP = IP << 8;
	IP |= data_eeprom[2];
	IP = IP << 8;
	IP |= data_eeprom[3];

	return IP;

}

void
EepromWriteIP(uint32_t IP)
{
	data_eeprom[0] = IP_ADDR >> 8; //Memory address MSB
	data_eeprom[1] = IP_ADDR; //Memory address LSB
	data_eeprom[2] = IP >> 24;
	data_eeprom[3] = IP >> 16;
	data_eeprom[4] = IP >> 8;
	data_eeprom[5] = IP;

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

}

uint32_t
EepromReadIPMask(void)
{
	uint32_t IPMASK = 0;
	data_eeprom[0] = IPMASK_ADDR >> 8; // Memory address MSB
	data_eeprom[1] = IPMASK_ADDR; // Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	IPMASK = data_eeprom[0];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[1];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[2];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[3];

	return IPMASK;
}

void
EepromWriteIPMask(uint32_t IPMASK)
{
	data_eeprom[0] = IPMASK_ADDR >> 8; // Memory address MSB
	data_eeprom[1] = IPMASK_ADDR; // Memory address LSB
	data_eeprom[2] = IPMASK >> 24;
	data_eeprom[3] = IPMASK >> 16;
	data_eeprom[4] = IPMASK >> 8;
	data_eeprom[5] = IPMASK;

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

}

//***********************************************************************************

//***********************************************************************************
//                            RS-485 DATA
//***********************************************************************************

uint8_t
EepromReadRs485Add(void)
{
	data_eeprom[0] = RS485_ADDR >> 8; //Memory address MSB
	data_eeprom[1] = RS485_ADDR; //Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x02, data_eeprom);

	return data_eeprom[0];
}

void
EepromWriteRs485Add(uint8_t ADD_RS485)
{

	data_eeprom[0] = RS485_ADDR >> 8; //Memory address MSB
	data_eeprom[1] = RS485_ADDR; //Memory address LSB
	data_eeprom[2] = ADD_RS485;

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection
}

uint32_t
EepromReadRs485BaudRate(void)
{
	uint32_t BAUD = 0;
	data_eeprom[0] = RS485_BR >> 8; // Memory address MSB
	data_eeprom[1] = RS485_BR; // Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	BAUD = data_eeprom[0];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[1];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[2];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[3];

	return BAUD;
}

void
EepromWriteRs485BaudRate(uint32_t BAUD)
{
	data_eeprom[0] = RS485_BR >> 8; // Memory address MSB
	data_eeprom[1] = RS485_BR; // Memory address LSB
	data_eeprom[2] = BAUD >> 24;
	data_eeprom[3] = BAUD >> 16;
	data_eeprom[4] = BAUD >> 8;
	data_eeprom[5] = BAUD;

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

}

//***********************************************************************************

//***********************************************************************************
//                            Control Law DATA
//***********************************************************************************

float
EepromReadKp(void)
{
	data_eeprom[0] = GAIN_KP >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KP; // Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

void
EepromWriteKp(float KP)
{
	floatNchars.f = KP;

	data_eeprom[0] = GAIN_KP >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KP; // Memory address LSB

	data_eeprom[2] = floatNchars.c[0];
	data_eeprom[3] = floatNchars.c[1];
	data_eeprom[4] = floatNchars.c[2];
	data_eeprom[5] = floatNchars.c[3];

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection
}

float
EepromReadKi(void)
{
	data_eeprom[0] = GAIN_KI >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KI; // Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

void
EepromWriteKi(float KI)
{
	floatNchars.f = KI;

	data_eeprom[0] = GAIN_KI >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KI; // Memory address LSB

	data_eeprom[2] = floatNchars.c[0];
	data_eeprom[3] = floatNchars.c[1];
	data_eeprom[4] = floatNchars.c[2];
	data_eeprom[5] = floatNchars.c[3];

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection
}

float
EepromReadKd(void)
{
	data_eeprom[0] = GAIN_KD >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KD; // Memory address LSB
	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

void
EepromWriteKd(float KD)
{
	floatNchars.f = KD;

	data_eeprom[0] = GAIN_KD >> 8; // Memory address MSB
	data_eeprom[1] = GAIN_KD; // Memory address LSB

	data_eeprom[2] = floatNchars.c[0];
	data_eeprom[3] = floatNchars.c[1];
	data_eeprom[4] = floatNchars.c[2];
	data_eeprom[5] = floatNchars.c[3];

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

}

//***********************************************************************************
//                            Power Supply Model
//***********************************************************************************

uint8_t
EepromReadPSModel(void)
{

	data_eeprom[0] = PSMODEL >> 8; // Memory address MSB
	data_eeprom[1] = PSMODEL; // Memory address LSB

	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x02, data_eeprom);

	return data_eeprom[0];
}

void
EepromWritePSModel(uint8_t ps_model)
{
	data_eeprom[0] = PSMODEL >> 8; // Memory address MSB
	data_eeprom[1] = PSMODEL;      // Memory address LSB
	data_eeprom[2] = ps_model;

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

	WriteI2C(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

	for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

	GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection,

}


//***********************************************************************************

