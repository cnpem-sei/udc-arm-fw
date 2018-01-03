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
 * @file system.c
 * @brief System module.
 *
 * @author joao.rosa
 *
 * @date 22/07/2015
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>

#include "board_drivers/hardware_def.h"
#include "communication_drivers/i2c_onboard/i2c_onboard.h"
#include "communication_drivers/i2c_onboard/rtc.h"
#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/i2c_onboard/exio.h"
#include "communication_drivers/i2c_offboard_isolated/i2c_offboard_isolated.h"
#include "communication_drivers/i2c_offboard_isolated/external_devices.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/timer/timer.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers//flash/flash_mem.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/rs485_bkp/rs485_bkp.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/usb_device/superv_cmd.h"
#include "communication_drivers/ihm/ihm.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/usb_to_serial/usb_to_serial.h"
#include "communication_drivers/epi/sdram_mem.h"
#include "communication_drivers/control/control.h"

#include "ethernet_uip.h"

#include "system.h"

/*
 *  This function test if the EEPROM memory is fully new and doesn't have data.
 *  if this is true, default data is write to initialize the system
 */
void test_eeprom_memory(void)
{
	uint8_t var8 = 0;
	uint32_t var32 = 0;

	// Read RS485 address from EEPROM
	// If data is equal to 0xFF than this is a new memory and needs parameterization
	var8 = eeprom_read_rs485_add(1);

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

		save_ip_address(var32);

		// Write default IP MASK 255.255.255.0
		var32 = 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0xFF;		// 255
		var32 = var32 << 8;
		var32 |= 0x00;		// 0

		save_ip_mask(var32);

		// Write default RS485 address
        save_rs485_ch_0_add(01);
		save_rs485_ch_1_add(33);
		save_rs485_ch_2_add(34);
		save_rs485_ch_3_add(35);

		// Write default RS485 Baud Rate
		save_rs485_baud(115200);

		// Write default Kp gain 0.0
		save_kp1_gain(0.0);

		// Write default Ki gain 0.0
		save_ki1_gain(0.0);

		// Write default Ki gain 0.0
		save_kd1_gain(0.0);

		// Write default PS_Model as 0 (FBP)
		save_ps_model(0);

		eeprom_write_request_check();
	}
}

void system_config(void)
{
    init_i2c_onboard();

    test_eeprom_memory();

	extern_io_init();

	if(HARDWARE_VERSION == 0x21) buffers_ctrl(1);

	hradc_rst_ctrl(1);

	ipc_init();
}

void system_init(void)
{

    init_i2c_offboard_isolated();

	flash_mem_init();

	dcdc_pwr_ctrl(true);
	// Não necessita da configuração da malha de controle, o dsp ja possui os dados
	//CtrllawInit();

	//init_display();

	init_rs485();

	init_rs485_bkp();

	if(HARDWARE_VERSION == 0x21) init_usb_to_serial();

	//init_can_bkp();

	//BSMPInit();

	ethernet_init();

	//InitUSBSerialDevice();

	display_pwr_ctrl(true);

	rtc_init();

	adcp_init();

	if(HARDWARE_VERSION == 0x20)
	{
	    pwm_fiber_ctrl(true);
	    pwm_eletr_ctrl(true);
	}

	//SdramInit();

	init_i2c_offboard_external_devices();

	global_timer_init();

}
