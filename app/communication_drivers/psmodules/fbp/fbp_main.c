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
 * @file fbpmain.c
 * @brief FBP module
 *
 * Main module for FBP operation.
 *
 * @author allef.silva
 * @date 18/10/2017
 *
 */

#include<stdint.h>
#include<stdbool.h>

#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"

#include "fbp_main.h"
#include "fbp_system.h"
#include "driverlib/ipc.h"

/**
* @brief Main function for FBP.
*
* Entry point for FAC operation.
*
*/
void fbp_main(void)
{
    volatile uint32_t uiloop;
    fbp_system_config();
    IPCMtoCBootControlSystem(CBROM_MTOC_BOOTMODE_BOOT_FROM_FLASH);

    for (;;)
    {
        for (uiloop = 0; uiloop < 1000; uiloop++)
        {
            TaskCheck();
        }

    }
}
