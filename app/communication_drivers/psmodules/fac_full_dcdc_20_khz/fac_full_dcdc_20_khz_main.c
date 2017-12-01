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
 * @file facmain.c
 * @brief FAC module
 * 
 * Main module for FAC operation.
 *
 * @author allef.silva
 * @date 30/10/2017
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"

#include "fac_full_dcdc_20_khz_system.h"
#include "fac_full_dcdc_20_khz_main.h"

/**
* @brief Main function for FAC.
*
* Entry point for FAC operation.
*
*/
void fac_full_dcdc_20_khz_main(void)
{
    volatile uint32_t uiloop;
    fac_full_dcdc_20_khz_system_config();

    for (;;)
    {
        for (uiloop = 0; uiloop < 1000; uiloop++)
        {
            TaskCheck();
        }

    }
}
