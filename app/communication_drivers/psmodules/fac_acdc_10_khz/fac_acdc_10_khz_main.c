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
 * @file fapmain.c
 * @brief FAP module
 * 
 * Main module for FAP operation.
 *
 * @author allef.silva
 * @date 30/10/2017
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"

#include "fac_acdc_10_khz_main.h"
#include "fac_acdc_10_khz_system.h"

/**
* @brief Main function for FAP.
*
* Entry point for FAP operation.
*
*/
void fac_acdc_10_khz_main(void)
{
    volatile uint32_t uiloop;
    fac_acdc_10_khz_system_config();

    for (;;)
    {
        for (uiloop = 0; uiloop < 1000; uiloop++)
        {
            TaskCheck();
        }

    }
}
