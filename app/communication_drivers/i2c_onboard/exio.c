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

#include "../board_drivers/hardware_def.h"

#include <stdint.h>

#define I2C_SLV_ADDR_EXIO1 0x70 // Endereço 7 bits

#define I2C_SLV_ADDR_EXIO2 0x71 // Endereço 7 bits

//#define HardwareVer	0x20

uint8_t data_exio[10];

static uint8_t StsExpIO1 = 0;
static uint8_t StsExpIO2 = 0;

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

//*********************************************************************************************************************
// IO expanders configuration pins for UDC V2.1, Register 3
//
//	IO expander 1
//
//	P0 = 1 Input	DISPLAY_OVERCURRENT
//	P1 = 0 Output	DISPLAY_ENABLE
//	P2 = 1 Input	SD_ATT
//	P3 = 0 Output	RS485-TERMINATION
//	P4 = 1 Input	DC-DCStatus
//	P5 = 1 Input	DISP_ATT
//	P6 = 0 Output	HRADC_RESET
//	P7 = 1 Input	NO CONNECTED
//
//	Register = 0xB5 = 10110101
//
//	IO expander 2
//
//	P0 = 1 Input	NO CONNECTED
//	P1 = 0 Output	DC/DC-Control
//	P2 = 0 Output	BUFFER_ENABLE
//	P3 = 0 Output	ITLK_LED_CTRL
//	P4 = 0 Output	STATUS_LED
//	P5 = 0 Output	SOUND_SELECT
//	P6 = 1 Input	NO CONNECTED
//	P7 = 1 Input	NO CONNECTED
//
//	Register = 0xC1 = 11000001
//
//*********************************************************************************************************************


// !!!!
/*
 *
 * How to detect the V2.1 hardware version?
 * Try to configure the second IO expander (address 0x71)
 * Read the configuration register
 * If the data collected from the IC is equal, than the board is a 2.1 release
 *
 */

uint8_t
HardwareVersionTest(uint8_t ExNumber)
{
	uint8_t Ex1 = 0;
	uint8_t Ex2 = 0;

	// Board version test section

	// Expander number 2 detection routine
	// Try to configure the second expander
	data_exio[0] = 0x03; // Input/Output configuration register
	data_exio[1] = 0xC1; // Expander 2 configuration (Test Configuration)
	WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);

	// Try to read back the configuration byte from the second expander
	data_exio[0] = 0x03;
	ReadI2C(I2C_SLV_ADDR_EXIO2, SINGLE_ADDRESS, 0x02, data_exio);

	if(data_exio[0] == 0xC1) Ex2 = 0x01; //Expander 2 detected
	else Ex2 = 0xEE; //Expander 2 not detected

	// Expander number 1 detection routine
	// Try to configure the second expander
	data_exio[0] = 0x03; // Input/Output configuration register
	data_exio[1] = 0x7F; // Expander 1 configuration (Test Configuration)
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);

	// Try to read back the configuration byte from the first expander
	data_exio[0] = 0x03;
	ReadI2C(I2C_SLV_ADDR_EXIO1, SINGLE_ADDRESS, 0x02, data_exio);

	if(data_exio[0] == 0x7F) Ex1 = 0x01; //Expander 1 detected
	else Ex1 = 0xEE; //Expander 1 not detected

	if(Ex1 == 0x01 && Ex2 == 0x01)
	{
		// V2.1 hardware version
		return 0x21;
	}
	else if(Ex1 == 0x01 && Ex2 == 0xEE)
	{
		// V2.0 hardware version
		// Check second time?
		return 0x20;
	}
	else
	{
		// Hardware problem!!!
		return 0xEE;
	}
}

void
ExIOInit(void)
{

	switch(HARDWARE_VERSION)
	{
	case 0x20:
		data_exio[0] = 0x03; // Input/Output configuration register
		data_exio[1] = 0x0D; // Expander configuration
		WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);

		data_exio[0] = 0x01; // Output state register
		data_exio[1] = StsExpIO1 = 0x40; // Output state pins. Turn off: Display, Dcdc, PWM Fiber and PWM Electric
		WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);

		break;
	case 0x21:
		// First expander
		data_exio[0] = 0x03; // Input/Output configuration register
		data_exio[1] = 0xB5; // Expander configuration
		WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);

		data_exio[0] = 0x01; // Output state register
		data_exio[1] = StsExpIO1 = 0x00; // Output state pins. Turn off: Display, Dcdc, PWM Fiber and PWM Electric
		WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);

		// Second expander
		data_exio[0] = 0x03; // Input/Output configuration register
		data_exio[1] = 0xC1; // Expander configuration
		WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);

		data_exio[0] = 0x01; // Output state register
		data_exio[1] = StsExpIO2 = 0x00; // Output state pins. Turn off: Display, Dcdc, PWM Fiber and PWM Electric
		WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);

		break;
	}

}

void
DisplayPwrCtrl(uint8_t sts)
{
	if(sts) StsExpIO1 |= 0b00000010;
	else StsExpIO1 &= 0b11111101;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO1;
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
}

uint8_t
DisplayPwrOCSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO1, SINGLE_ADDRESS, 0x02, data_exio);
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
	ReadI2C(I2C_SLV_ADDR_EXIO1, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	Sts &= 0b00000100;
	if(Sts) return(1);
	else return(0);
}

void
DcdcPwrCtrl(uint8_t sts)
{
	switch(HARDWARE_VERSION)
	{
	case 0x20:
		if(sts) StsExpIO1 &= 0b10111111;
		else StsExpIO1 |= 0b01000000;
		data_exio[0] = 0x01;
		data_exio[1] = StsExpIO1;
		WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
		break;
	case 0x21:
		if(sts) StsExpIO2 &= 0b11111101;
		else StsExpIO2 |= 0b00000010;
		data_exio[0] = 0x01;
		data_exio[1] = StsExpIO2;
		WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);
		break;
	}

}

uint8_t
DcdcSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO1, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	switch(HARDWARE_VERSION)
	{
	case 0x20:
		Sts &= 0b00001000;
		break;
	case 0x21:
		Sts &= 0b00010000;
		break;
	}

	if(Sts) return(0);
	else return(1);
}

void
HradcRstCtrl(uint8_t sts)
{
	switch(HARDWARE_VERSION)
	{
	case 0x20:
		if(sts) StsExpIO1 |= 0b00100000;
		else StsExpIO1 &= 0b11011111;
		break;
	case 0x21:
		if(sts) StsExpIO1 |= 0b01000000;
		else StsExpIO1 &= 0b10111111;
		break;
	}

	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO1;
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
}


// Available only on 2.0 hardware release
void
PwmFiberCtrl(uint8_t sts)
{
	if(sts) StsExpIO1 |= 0b10000000;
	else StsExpIO1 &= 0b01111111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO1;
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
}


// Available only on 2.0 hardware release
void
PwmEletrCtrl(uint8_t sts)
{
	if(sts) StsExpIO1 |= 0b00010000;
	else StsExpIO1 &= 0b11101111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO1;
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
}


// Available only on 2.1 hardware release
void
Rs485TermCtrl(uint8_t sts)
{
	if(sts) StsExpIO1 |= 0b00001000;
	else StsExpIO1 &= 0b11110111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO1;
	WriteI2C(I2C_SLV_ADDR_EXIO1, 0x02, data_exio);
}

// Available only on 2.1 hardware release
uint8_t
DisplayAttSts(void)
{
	uint8_t Sts = 0;
	data_exio[0] = 0x00;
	ReadI2C(I2C_SLV_ADDR_EXIO1, SINGLE_ADDRESS, 0x02, data_exio);
	Sts = data_exio[0];

	Sts &= 0b00100000;
	if(Sts) return(1);
	else return(0);
}

// Available only on 2.1 hardware release
void
BuffersCtrl(uint8_t sts)
{
	if(sts) StsExpIO2 |= 0b00000100;
	else StsExpIO2 &= 0b11111011;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO2;
	WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);
}

// Available only on 2.1 hardware release
void
LedItlkCtrl(uint8_t sts)
{
	if(sts) StsExpIO2 |= 0b00001000;
	else StsExpIO2 &= 0b11110111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO2;
	WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);
}

// Available only on 2.1 hardware release
void
LedStsCtrl(uint8_t sts)
{
	if(sts) StsExpIO2 |= 0b00010000;
	else StsExpIO2 &= 0b11101111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO2;
	WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);
}

// Available only on 2.1 hardware release
void
SoundSelCtrl(uint8_t sts)
{
	if(sts) StsExpIO2 |= 0b00100000;
	else StsExpIO2 &= 0b11011111;
	data_exio[0] = 0x01;
	data_exio[1] = StsExpIO2;
	WriteI2C(I2C_SLV_ADDR_EXIO2, 0x02, data_exio);
}
