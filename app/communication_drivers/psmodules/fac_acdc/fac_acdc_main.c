/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_acdc_main.c
 * @brief Main module for FAC ACDC operation
 *
 * @author gabriel.brunheira
 * @date 23/04/2018
 *
 */

#include <communication_drivers/psmodules/fac_acdc/fac_acdc_main.h>
#include <communication_drivers/psmodules/fac_acdc/fac_acdc_system.h>
#include <stdint.h>
#include <stdbool.h>

#include "board_drivers/version.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/parameters/ps_parameters.h"

#include "inc/hw_types.h"

#include "driverlib/ipc.h"
#include "driverlib/sysctl.h"

/**
* @brief Main function for FBP.
*
* Entry point for FAC operation.
*
*/
void fac_acdc_main(void)
{
    volatile uint32_t uiloop;
    fac_acdc_system_config();

    //IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    for (uiloop = 0; uiloop < 1000; uiloop++)
    {
        TaskCheck();
    }

    SysCtlDelay(75000);

    get_firmwares_version();

    for (;;)
    {
        for (uiloop = 0; uiloop < 1000; uiloop++)
        {
            TaskCheck();
        }

    }
}
