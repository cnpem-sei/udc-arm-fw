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
 * @file external_devices.c
 * @brief External devices module.
 *
 * @author joao.rosa
 *
 * @date 25/01/2017
 *
 */

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/rs485/rs485.h"
#include "temp_low_power_module.h"
#include "external_devices.h"

void init_i2c_offboard_external_devices(void)
{
    switch (g_ipc_mtoc.ps_module[0].ps_status.bit.model)
	{
		case FBP:
            power_supply_1_temp_init();
            power_supply_2_temp_init();
            power_supply_3_temp_init();
            power_supply_4_temp_init();
            break;
	}
}
