/*
 * exio.c
 *
 *  Created on: 13/07/2015
 *      Author: joao.rosa
 *
 *      Routines working fine!!
 *      Tested 15/07/2015
 */

#include "eeprom.h"
#include "i2c_onboard.h"

#include <stdint.h>

#define I2C_SLV_ADDR_EXIO 0x70 // Endereço 7 bits

uint8_t data_exio[10];

static uint8_t StsExpIO = 0;

//*********************************************************************************************************************
// IO expander configuration pins for UDC V2.0, Register 3
//	P0 = 1 Input	DISPLAY_OVERCURRENT
//	P1 = 0 Output	DISPLAY_ENABLE
//	P2 = 1 Input	SD_ATT
//	P3 = 1 Input	DC-DCStatus
//	P4 = 0 Output	PWM_E_OUT_CTR
//	P5 = 0 Output	HRADC_RESET
//	P6 = 0 Output	DC/DC-Control
//	P7 = 0 Output	PWM_EN_FIBER
//
//	Register = 0x0D = 00001101
//
//*********************************************************************************************************************


void
ExIOInit(void)
{
	data_exio[0] = 0x03; // Input/Output configuration register
	data_exio[1] = 0x0D; // Expander configuration
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);

	data_exio[0] = 0x01; // Output state register
	data_exio[1] = StsExpIO = 0x40; // Output state pins. Turn off: Display, Dcdc, PWM Fiber and PWM Electric
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}

void
DisplayPwrCtrl(uint8_t sts)
{
	if(sts) StsExpIO |= 0b00000010;
	else StsExpIO &= 0b11111101;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO;
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}

uint8_t
DisplayPwrOCSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	Sts &= 0b00000001;
	if(Sts) return(1);
	else return(0);
}

uint8_t
SdAttSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	Sts &= 0b00000100;
	if(Sts) return(1);
	else return(0);
}

void
DcdcPwrCtrl(uint8_t sts)
{
	if(sts) StsExpIO &= 0b10111111;
	else StsExpIO |= 0b01000000;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO;
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}

uint8_t
DcdcSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	Sts &= 0b00001000;
	if(Sts) return(0);
	else return(1);
}

void
HradcRstCtrl(uint8_t sts)
{
	if(sts) StsExpIO |= 0b00100000;
	else StsExpIO &= 0b11011111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO;
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}

void
PwmFiberCtrl(uint8_t sts)
{
	if(sts) StsExpIO |= 0b10000000;
	else StsExpIO &= 0b01111111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO;
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}

void
PwmEletrCtrl(uint8_t sts)
{
	if(sts) StsExpIO |= 0b00010000;
	else StsExpIO &= 0b11101111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO;
	WriteI2C(I2C_SLV_ADDR_EXIO, 0x02, data_exio);
}
