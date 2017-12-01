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
 * @file eeprom.c
 * @brief EEPROM module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"

#include "i2c_onboard.h"
#include "hardware_def.h"
#include "eeprom.h"

//***********************************************************************************
//  The memory address is compose of 13bits (2 bytes)
//  Byte MSB with 5 bits [---1 1111] and the LSB with 8 bits [1111 1111]
//  The 8 MSB bits (5 in the MSB byte + 3 in the LSB byte) correspond to the page address and the lasts 5 LSB bits are used to point one of 32bytes available in a memory page.
//
//  Memory page [8 bits], Byte in the page [5 bits]
//
//***********************************************************************************

#define I2C_SLV_ADDR_EEPROM	0x50 // Endere�o 7 bits

// Memory map
#define	IP_ADDR			    0x0140
#define IPMASK_ADDR		    0x0144

//#define	RS485_ADDR		0x0160
//#define	RS485_BR		0x0161
//#define RS485BKP_ADDR	0x0165
//#define	RS485BKP_BR		0x0166

#define GAIN_KP1		    0x0200
#define	GAIN_KI1		    0x0204
#define	GAIN_KD1		    0x0208

#define GAIN_KP2		    0x0210
#define	GAIN_KI2		    0x0214
#define	GAIN_KD2		    0x0218

#define GAIN_KP3		    0x0220
#define	GAIN_KI3		    0x0224
#define	GAIN_KD3		    0x0228

#define GAIN_KP4		    0x0230
#define	GAIN_KI4		    0x0234
#define	GAIN_KD4		    0x0238

#define RS485_CH_0_ADDR     0x0300
#define RS485_CH_1_ADDR     0x0301
#define RS485_CH_2_ADDR     0x0302
#define RS485_CH_3_ADDR     0x0303
#define RS485_BR            0x0304
#define RS485BKP_ADDR       0x0308
#define RS485BKP_BR         0x0309

#define PSMODEL			0x0100


uint8_t data_eeprom[32];
uint16_t add = 0;

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
eeprom_read_ip(void)
{
	uint32_t IP = 0;
	data_eeprom[0] = IP_ADDR >> 8; //Memory address MSB
	data_eeprom[1] = (uint8_t)IP_ADDR; //Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	IP = data_eeprom[0];
	IP = IP << 8;
	IP |= data_eeprom[1];
	IP = IP << 8;
	IP |= data_eeprom[2];
	IP = IP << 8;
	IP |= data_eeprom[3];

	return IP;

}

static uint32_t ip_address_data = 0;
static uint8_t ip_new_data = 0;

void
save_ip_address(uint32_t IP)
{
	ip_address_data = IP;
	ip_new_data = 1;
}

void
eeprom_write_ip(void)
{
	if(ip_new_data)
	{
		data_eeprom[0] = IP_ADDR >> 8; //Memory address MSB
		data_eeprom[1] = (uint8_t)IP_ADDR; //Memory address LSB
		data_eeprom[2] = ip_address_data >> 24;
		data_eeprom[3] = ip_address_data >> 16;
		data_eeprom[4] = ip_address_data >> 8;
		data_eeprom[5] = ip_address_data;

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ip_new_data = 0;
	}

}

//**********

uint32_t eeprom_read_ip_mask(void)
{
	uint32_t IPMASK = 0;
	data_eeprom[0] = IPMASK_ADDR >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)IPMASK_ADDR; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	IPMASK = data_eeprom[0];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[1];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[2];
	IPMASK = IPMASK << 8;
	IPMASK |= data_eeprom[3];

	return IPMASK;
}

static uint32_t ip_mask_data = 0;
static uint8_t	ip_mask_new_data = 0;

void save_ip_mask(uint32_t IP_MASK)
{
	ip_mask_data = IP_MASK;
	ip_mask_new_data = 1;
}

void eeprom_write_ip_mask(void)
{
	if(ip_mask_new_data)
	{
		data_eeprom[0] = IPMASK_ADDR >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)IPMASK_ADDR; // Memory address LSB
		data_eeprom[2] = ip_mask_data >> 24;
		data_eeprom[3] = ip_mask_data >> 16;
		data_eeprom[4] = ip_mask_data >> 8;
		data_eeprom[5] = ip_mask_data;

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ip_mask_new_data = 0;
	}

}

//***********************************************************************************

//***********************************************************************************
//                            RS-485 DATA
//***********************************************************************************

//uint8_t
//EepromReadRs485Add(void)
//{
//	data_eeprom[0] = RS485_ADDR >> 8; //Memory address MSB
//	data_eeprom[1] = (uint8_t)RS485_ADDR; //Memory address LSB
//	ReadI2C(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x02, data_eeprom);
//
//	return data_eeprom[0];
//}

uint8_t eeprom_read_rs485_add(uint8_t ch)
{
    switch (ch)
    {

        case 1:
            data_eeprom[0] = RS485_CH_1_ADDR >> 8; //Memory address MSB
            data_eeprom[1] = (uint8_t)RS485_CH_1_ADDR; //Memory address LSB
            break;

        case 2:
            data_eeprom[0] = RS485_CH_2_ADDR >> 8; //Memory address MSB
            data_eeprom[1] = (uint8_t)RS485_CH_2_ADDR; //Memory address LSB
            break;

        case 3:
            data_eeprom[0] = RS485_CH_3_ADDR >> 8; //Memory address MSB
            data_eeprom[1] = (uint8_t)RS485_CH_3_ADDR; //Memory address LSB
            break;

        default:
            data_eeprom[0] = RS485_CH_0_ADDR >> 8; //Memory address MSB
            data_eeprom[1] = (uint8_t)RS485_CH_0_ADDR; //Memory address LSB
    }

    read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x02, data_eeprom);

    return data_eeprom[0];
}

//static uint8_t rs485_add_data = 0;
//static uint8_t	rs485_add_new_data = 0;
static uint8_t rs485_add_ch_0_data = 0;
static uint8_t rs485_add_ch_1_data = 0;
static uint8_t rs485_add_ch_2_data = 0;
static uint8_t rs485_add_ch_3_data = 0;

static uint8_t   rs485_add_ch_0_new_data = 0;
static uint8_t   rs485_add_ch_1_new_data = 0;
static uint8_t   rs485_add_ch_2_new_data = 0;
static uint8_t   rs485_add_ch_3_new_data = 0;


//void
//SaveRs485Add(uint32_t RS485_ADD)
//{
//	rs485_add_data = RS485_ADD;
//	rs485_add_new_data = 1;
//}

void save_rs485_ch_0_add(uint32_t add)
{
    rs485_add_ch_0_data = add;
    rs485_add_ch_0_new_data = 1;
}

void save_rs485_ch_1_add(uint32_t add)
{
    rs485_add_ch_1_data = add;
    rs485_add_ch_1_new_data = 1;
}

void save_rs485_ch_2_add(uint32_t add)
{
    rs485_add_ch_2_data = add;
    rs485_add_ch_2_new_data = 1;
}

void save_rs485_ch_3_add(uint32_t add)
{
    rs485_add_ch_3_data = add;
    rs485_add_ch_3_new_data = 1;
}

//void
//EepromWriteRs485Add(void)
//{
//
//	if(rs485_add_new_data)
//	{
//		data_eeprom[0] = RS485_ADDR >> 8; //Memory address MSB
//		data_eeprom[1] = (uint8_t)RS485_ADDR; //Memory address LSB
//		data_eeprom[2] = rs485_add_data;
//
//		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection
//
//		WriteI2C(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);
//
//		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms
//
//		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection
//
//		rs485_add_new_data = 0;
//	}
//
//}

void eeprom_write_rs485_add(void)
{

    if(rs485_add_ch_0_new_data)
    {
        data_eeprom[0] = RS485_CH_0_ADDR >> 8; //Memory address MSB
        data_eeprom[1] = (uint8_t)RS485_CH_0_ADDR; //Memory address LSB
        data_eeprom[2] = rs485_add_ch_0_data;

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

        write_i2c(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

        for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

        rs485_add_ch_0_new_data = 0;
    }

    if(rs485_add_ch_1_new_data)
    {
        data_eeprom[0] = RS485_CH_1_ADDR >> 8; //Memory address MSB
        data_eeprom[1] = (uint8_t)RS485_CH_1_ADDR; //Memory address LSB
        data_eeprom[2] = rs485_add_ch_1_data;

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

        write_i2c(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

        for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

        rs485_add_ch_1_new_data = 0;
    }

    if(rs485_add_ch_2_new_data)
    {
        data_eeprom[0] = RS485_CH_2_ADDR >> 8; //Memory address MSB
        data_eeprom[1] = (uint8_t)RS485_CH_2_ADDR; //Memory address LSB
        data_eeprom[2] = rs485_add_ch_2_data;

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

        write_i2c(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

        for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

        rs485_add_ch_2_new_data = 0;
    }

    if(rs485_add_ch_3_new_data)
    {
        data_eeprom[0] = RS485_CH_3_ADDR >> 8; //Memory address MSB
        data_eeprom[1] = (uint8_t)RS485_CH_3_ADDR; //Memory address LSB
        data_eeprom[2] = rs485_add_ch_3_data;

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

        write_i2c(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

        for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

        GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

        rs485_add_ch_3_new_data = 0;
    }

}

//*********

uint32_t eeprom_read_rs485_baudrate(void)
{
	uint32_t BAUD = 0;
	data_eeprom[0] = RS485_BR >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)RS485_BR; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	BAUD = data_eeprom[0];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[1];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[2];
	BAUD = BAUD << 8;
	BAUD |= data_eeprom[3];

	return BAUD;
}

static uint32_t rs485_baud_data = 0;
static uint8_t	rs485_baud_new_data = 0;

void save_rs485_baud(uint32_t RS485_BAUD)
{
	rs485_baud_data = RS485_BAUD;
	rs485_baud_new_data = 1;
}

void eeprom_write_rs485_baudrate(void)
{
	if(rs485_baud_new_data)
	{
		data_eeprom[0] = RS485_BR >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)RS485_BR; // Memory address LSB
		data_eeprom[2] = rs485_baud_data >> 24;
		data_eeprom[3] = rs485_baud_data >> 16;
		data_eeprom[4] = rs485_baud_data >> 8;
		data_eeprom[5] = rs485_baud_data;

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		rs485_baud_new_data = 0;
	}



}

//***********************************************************************************

//***********************************************************************************
//                            Control Law DATA - PID1
//***********************************************************************************

float eeprom_read_kp1(void)
{
	data_eeprom[0] = GAIN_KP1 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KP1; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kp1_data = 0;
static uint8_t kp1_new_data = 0;

void save_kp1_gain(float KP1)
{
	kp1_data = KP1;
	kp1_new_data = 1;
}

void eeprom_write_Kp1(void)
{
	if(kp1_new_data)
	{
		floatNchars.f = kp1_data;

		data_eeprom[0] = GAIN_KP1 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KP1; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kp1_new_data = 0;
	}

}

//***********

float eeprom_read_ki1(void)
{
	data_eeprom[0] = GAIN_KI1 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KI1; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float ki1_data = 0;
static uint8_t	ki1_new_data = 0;

void save_ki1_gain(float KI1)
{
	ki1_data = KI1;
	ki1_new_data = 1;
}

void eeprom_write_ki1(void)
{
	if(ki1_new_data)
	{
		floatNchars.f = ki1_data;

		data_eeprom[0] = GAIN_KI1 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KI1; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ki1_new_data = 0;
	}

}

//******

float eeprom_read_kd1(void)
{
	data_eeprom[0] = GAIN_KD1 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KD1; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kd1_data = 0;
static uint8_t	kd1_new_data = 0;

void save_kd1_gain(float KD1)
{
	kd1_data = KD1;
	kd1_new_data = 1;
}

void eeprom_write_kd1(void)
{
	if(kd1_new_data)
	{
		floatNchars.f = kd1_data;

		data_eeprom[0] = GAIN_KD1 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KD1; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kd1_new_data = 0;
	}

}

//***********************************************************************************
//                            Control Law DATA - PID2
//***********************************************************************************

float eeprom_read_kp2(void)
{
	data_eeprom[0] = GAIN_KP2 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KP2; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kp2_data = 0;
static uint8_t	kp2_new_data = 0;

void save_kp2_gain(float KP2)
{
	kp2_data = KP2;
	kp2_new_data = 1;
}

void eeprom_write_kp2(void)
{
	if(kp2_new_data)
	{
		floatNchars.f = kp2_data;

		data_eeprom[0] = GAIN_KP2 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KP2; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kp2_new_data = 0;
	}

}

float eeprom_read_ki2(void)
{
	data_eeprom[0] = GAIN_KI2 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KI2; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float ki2_data = 0;
static uint8_t	ki2_new_data = 0;

void save_ki2_gain(float KI2)
{
	ki2_data = KI2;
	ki2_new_data = 1;
}

void eeprom_write_ki2(void)
{
	if(ki2_new_data)
	{
		floatNchars.f = ki2_data;

		data_eeprom[0] = GAIN_KI2 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KI2; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ki2_new_data = 0;
	}

}

//******

float eeprom_read_kd2(void)
{
	data_eeprom[0] = GAIN_KD2 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KD2; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kd2_data = 0;
static uint8_t	kd2_new_data = 0;

void save_kd2_gain(float KD2)
{
	kd2_data = KD2;
	kd2_new_data = 1;
}

void eeprom_write_kd2(void)
{
	if(kd2_new_data)
	{
		floatNchars.f = kd2_data;

		data_eeprom[0] = GAIN_KD2 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KD2; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kd2_new_data = 0;
	}



}

//***********************************************************************************
//                            Control Law DATA - PID3
//***********************************************************************************

float eeprom_read_kp3(void)
{
	data_eeprom[0] = GAIN_KP3 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KP3; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kp3_data = 0;
static uint8_t	kp3_new_data = 0;

void save_kp3_gain(float KP3)
{
	kp3_data = KP3;
	kp3_new_data = 1;
}

void eeprom_write_kp3(void)
{
	if(kp3_new_data)
	{
		floatNchars.f = kp3_data;

		data_eeprom[0] = GAIN_KP3 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KP3; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kp3_new_data = 0;
	}

}

//*******

float eeprom_read_ki3(void)
{
	data_eeprom[0] = GAIN_KI3 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KI3; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float ki3_data = 0;
static uint8_t	ki3_new_data = 0;

void save_ki3_gain(float KI3)
{
	ki3_data = KI3;
	ki3_new_data = 1;
}

void eeprom_write_ki3(void)
{
	if(ki3_new_data)
	{
		floatNchars.f = ki3_data;

		data_eeprom[0] = GAIN_KI3 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KI3; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ki3_new_data = 0;
	}

}

//*******

float eeprom_read_kd3(void)
{
	data_eeprom[0] = GAIN_KD3 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KD3; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kd3_data = 0;
static uint8_t	kd3_new_data = 0;

void save_kd3_gain(float KD3)
{
	kd3_data = KD3;
	kd3_new_data = 1;
}

void eeprom_write_kd3(void)
{
	if(kd3_new_data)
	{
		floatNchars.f = kd3_data;

		data_eeprom[0] = GAIN_KD3 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KD3; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kd3_new_data = 0;
	}

}

//***********************************************************************************
//                            Control Law DATA - PID4
//***********************************************************************************

float eeprom_read_kp4(void)
{
	data_eeprom[0] = GAIN_KP4 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KP4; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kp4_data = 0;
static uint8_t	kp4_new_data = 0;

void save_kp4_gain(float KP4)
{
	kp4_data = KP4;
	kp4_new_data = 1;
}

void eeprom_write_kp4(void)
{
	if(kp4_new_data)
	{
		floatNchars.f = kp4_data;

		data_eeprom[0] = GAIN_KP4 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KP4; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kp4_new_data = 0;
	}

}

//*****
float eeprom_read_ki4(void)
{
	data_eeprom[0] = GAIN_KI4 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KI4; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float ki4_data = 0;
static uint8_t	ki4_new_data = 0;

void save_ki4_gain(float KI4)
{
	ki4_data = KI4;
	ki4_new_data = 1;
}

void eeprom_write_ki4(void)
{
	if(ki4_new_data)
	{
		floatNchars.f = ki4_data;

		data_eeprom[0] = GAIN_KI4 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KI4; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		ki4_new_data = 0;
	}

}

//*****

float eeprom_read_kd4(void)
{
	data_eeprom[0] = GAIN_KD4 >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)GAIN_KD4; // Memory address LSB
	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x04, data_eeprom);

	floatNchars.c[0] = data_eeprom[0];
	floatNchars.c[1] = data_eeprom[1];
	floatNchars.c[2] = data_eeprom[2];
	floatNchars.c[3] = data_eeprom[3];

	return floatNchars.f;
}

static float kd4_data = 0;
static uint8_t kd4_new_data = 0;

void save_kd4_gain(float KD4)
{
	kd4_data = KD4;
	kd4_new_data = 1;
}

void eeprom_write_kd4(void)
{
	if(kd4_new_data)
	{
		floatNchars.f = kd4_data;

		data_eeprom[0] = GAIN_KD4 >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)GAIN_KD4; // Memory address LSB

		data_eeprom[2] = floatNchars.c[0];
		data_eeprom[3] = floatNchars.c[1];
		data_eeprom[4] = floatNchars.c[2];
		data_eeprom[5] = floatNchars.c[3];

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x06, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection

		kd4_new_data = 0;
	}

}

//***********************************************************************************
//                            Power Supply Model
//***********************************************************************************

uint8_t eeprom_read_ps_model(void)
{

	data_eeprom[0] = PSMODEL >> 8; // Memory address MSB
	data_eeprom[1] = (uint8_t)PSMODEL; // Memory address LSB

	read_i2c(I2C_SLV_ADDR_EEPROM, DOUBLE_ADDRESS, 0x02, data_eeprom);

	return data_eeprom[0];
}

static uint8_t ps_model_data = 0;
static uint8_t ps_model_new_data = 0;

void save_ps_model(uint8_t PS_MODEL)
{
	ps_model_data = PS_MODEL;
	ps_model_new_data = 1;
}

void eeprom_write_ps_model(void)
{
	if(ps_model_new_data)
	{
		data_eeprom[0] = PSMODEL >> 8; // Memory address MSB
		data_eeprom[1] = (uint8_t)PSMODEL;      // Memory address LSB
		data_eeprom[2] = ps_model_data;

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, OFF); // Disable Write protection

		write_i2c(I2C_SLV_ADDR_EEPROM, 0x03, data_eeprom);

		for (ulLoop=0;ulLoop<100000;ulLoop++){}; // wait 5ms

		GPIOPinWrite(EEPROM_WP_BASE, EEPROM_WP_PIN, ON); // Enable Write protection,

		ps_model_new_data = 0;
	}

}


//***********************************************************************************

void eeprom_write_request_check(void)
{
    eeprom_write_ip();
    eeprom_write_ip_mask();

    eeprom_write_rs485_add();
    eeprom_write_rs485_baudrate();

    eeprom_write_Kp1();
    eeprom_write_ki1();
    eeprom_write_kd1();

    eeprom_write_kp2();
    eeprom_write_ki2();
    eeprom_write_kd2();

    eeprom_write_kp3();
    eeprom_write_ki3();
    eeprom_write_kd3();

    eeprom_write_kp4();
	eeprom_write_ki4();
	eeprom_write_kd4();

	eeprom_write_ps_model();
}

