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
#include "communication_drivers/flash/flash_mem.h"
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
#include "communication_drivers/parameters/ps_parameters.h"

#include "ethernet_uip.h"

#include "system.h"

void init_system(void)
{
    init_i2c_onboard();

	init_extern_io();

	if(HARDWARE_VERSION == 0x21)
    {
	    buffers_ctrl(1);
    }

	hradc_rst_ctrl(1);

	init_parameters_bank();

	init_ipc();

	load_param_bank();

	load_dsp_modules_eeprom();

    init_i2c_offboard_isolated();

	flash_mem_init();

	dcdc_pwr_ctrl(true);

	//CtrllawInit();

	//init_display();

	init_rs485();

	init_rs485_bkp();

	if(HARDWARE_VERSION == 0x21) init_usb_to_serial();

	//init_can_bkp();

	bsmp_init(0);
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
