/*
 * system.c
 *
 *  Created on: 22/07/2015
 *      Author: joao.rosa
 */

#include "system.h"

#include "../board_drivers/hardware_def.h"

#include "../i2c_onboard/i2c_onboard.h"
#include "../i2c_onboard/rtc.h"
#include "../i2c_onboard/eeprom.h"
#include "../i2c_onboard/exio.h"

#include "../i2c_offboard_isolated/i2c_offboard_isolated.h"
#include "../i2c_offboard_isolated/external_devices.h"

#include "../adcp/adcp.h"

#include "../timer/timer.h"

#include "../system_task/system_task.h"

#include "../../flash/flash_mem.h"

#include "ethernet_uip.h"

#include "../rs485/rs485.h"

#include "../rs485_bkp/rs485_bkp.h"

#include "../can/can_bkp.h"

#include "../shared_memory/ctrl_law.h"

#include "../usb_device/superv_cmd.h"

#include "../ihm/ihm.h"

#include "../bsmp/bsmp_lib.h"

#include "../shared_memory/ctrl_law.h"

#include "../ipc/ipc_lib.h"

#include "../usb_to_serial/usb_to_serial.h"

#include "../epi/sdram_mem.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>



/*
 *  This function test if the EEPROM memory is fully new and doesn't have data.
 *  if this is true, default data is write to initialize the system
 */
void
TestEepromMemory(void)
{
	uint8_t var8 = 0;
	uint32_t var32 = 0;

	// Read RS485 address from EEPROM
	// If data is equal to 0xFF than this is a new memory and needs parameterization
	var8 = EepromReadRs485Add();

	if(var8 == 0xFF)
	{

		// Write default IP address 10.0.28.203
		var32 = 0x0A;		// 10
		var32 = var32 << 8;
		var32 |= 0x00;		// 0
		var32 = var32 << 8;
		var32 |= 0x1C;		// 28
		var32 = var32 << 8;
		var32 |= 0xCB;		// 203

		SaveIpAddress(var32);

		// Write default IP MASK 255.255.255.0
		var32 = 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0x00;		// 0

		SaveIpMask(var32);

		// Write default RS485 address
		SaveRs485Add(0x01);

		// Write default RS485 Baud Rate
		SaveRs485Baud(115200);

		// Write default Kp gain 0.0
		SaveKp1Gain(0.0);

		// Write default Ki gain 0.0
		SaveKi1Gain(0.0);

		// Write default Ki gain 0.0
		SaveKd1Gain(0.0);

		// Write default PS_Model as 0 (FBP)
		SavePsModel(0);

		EepromWriteRequestCheck();
	}
}

void
SystemConfig(void)
{
	InitI2COnboard();

	TestEepromMemory();

	ExIOInit();

	if(HARDWARE_VERSION == 0x21) BuffersCtrl(0);

	IPCInit();

}

void
SystemInit(void)
{

	InitI2COffboardIsolated();

	FlashMemInit();

	DcdcPwrCtrl(true);

	// Não necessita da configuração da malha de controle, o dsp ja possui os dados
	//CtrllawInit();

	InitDisplay();

	InitRS485();

	InitRS485BKP();

	if(HARDWARE_VERSION == 0x21) InitUsb2Serial();

	InitCanBkp();

	BSMPInit();

	EthernetInit();

	//InitUSBSerialDevice();

	DisplayPwrCtrl(true);

	RTCInit();

	AdcpInit();

	if(HARDWARE_VERSION == 0x20)
	{
		PwmFiberCtrl(true);
		PwmEletrCtrl(true);
	}

	//SdramInit();

	InitI2cOffboardExternalDevices();

	GlobalTimerInit();

}
