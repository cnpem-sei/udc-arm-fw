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
 * @file fapsystem.c
 * @brief System setup for operation as FAP
 *
 * @author allef.silva
 * @date 30/10/2017
 *
 */

#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "driverlib/interrupt.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"

#include "fac_acdc_10_khz_system.h"

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    ipc_init();

    /**
     * Do nothing
     */
}

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAP operation.
*
*/
static void adcp_channel_config(void)
{
    /**
     * Do nothing
     */
}

/**
* @brief Initialize BSMP server.
*
* Setup BSMP server for FAP operation.
*
*/
static void bsmp_init_server(void)
{
    /**
     * Enable just server 0.
     * Considered FAP_DCDC_20KHz
     */
    bsmp_init(0);

    Init_BSMP_var(2, 0, DP_Framework.NetSignals[5].u8);
    Init_BSMP_var(11, 0, DP_Framework.NetSignals[7].u8);
}

/**
* @brief System configuration for FAP.
*
* Initialize specific parameters e configure peripherals for FAP operation.
*
*/
void fac_acdc_10_khz_system_config(void)
{
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
