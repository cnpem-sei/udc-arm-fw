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
    /**
     * Enable I2C interface for the following onboard componentes (therefore,
     * it must be initialized before them):
     *
     *      - Board temperature sensor
     *      - EEPROM memory
     *      - I/O Expander
     *      - RTC Timer
     */

    init_i2c_onboard();

    /**
     * Initialize I/O expander. It provides digital I/O for the following
     * functionalities (therefore, it must be initialized before them):
     *
     *      - 5V buffers
     *      - Buzzer
     *      - DC/DC converter for isolated circuits
     *      - Front panel LEDs
     *      - HRADC
     *      - IHM
     *      - RS485
     *      - SD card
     */
	init_extern_io();

	/**
	 * Initialize 5V buffers. It provides signal path for the following
     * functionalities (therefore, it must be initialized before them):
     *
     *      - Backplane RS485 interface
     *      - Backplane UART*
     *      - CAN interface
     *      - External interruptions and synchronization signals
     *      - HRADC
     *      - Offboard I2C interface
     *      - Non-isolated PWM
	 */
	if(HARDWARE_VERSION == 0x21)
    {
	    buffers_ctrl(1);
	    init_usb_to_serial();
    }
	else if(HARDWARE_VERSION == 0x20)
    {
        pwm_fiber_ctrl(true);
        pwm_eletr_ctrl(true);
    }

	hradc_rst_ctrl(1);

	init_i2c_offboard_isolated();

    init_i2c_offboard_external_devices();

	init_parameters_bank();

	//load_param_bank();

	init_ipc();

	init_control_framework(&g_controller_mtoc);

	load_dsp_modules_eeprom(Onboard_EEPROM);

	flash_mem_init();

	dcdc_pwr_ctrl(true);

	init_rs485();

	init_rs485_bkp();

	bsmp_init(0);

	ethernet_init();

	display_pwr_ctrl(true);

	rtc_init();

	adcp_init();

	ihm_init();

	/**
	 * TODO: Initialization of CAN, USB and SDRAM
	 */
	init_can_bkp();
	//InitUSBSerialDevice();
    //SdramInit();

	global_timer_init();
}
