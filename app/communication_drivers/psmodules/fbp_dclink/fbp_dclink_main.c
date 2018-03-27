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
 * @file fbp_dclink.c
 * @brief FBP DCLINK module
 *
 * Main module for FBP DCLINK operation.
 *
 * @author paulo.santos
 * @date 15/03/2018
 *
 */

#include <communication_drivers/psmodules/fbp_dclink/fbp_dclink_main.h>
#include <communication_drivers/psmodules/fbp_dclink/fbp_dclink_system.h>
#include<stdint.h>
#include<stdbool.h>

#include "board_drivers/version.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/control/control.h"

#include "driverlib/ipc.h"

/**
* @brief Main function for FBP.
*
* Entry point for FAC operation.
*
*/
void main_fbp_dclink(void)
{
    volatile uint32_t uiloop;
    fbp_dclink_system_config();
    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    for (;;)
    {
        for (uiloop = 0; uiloop < 1000; uiloop++)
        {
            TaskCheck();
        }

    }

    SysCtlDelay(75000);
    get_firmwares_version();
}
