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
 * @file facsystem.c
 * @brief System setup for operation as FAC
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

#include "fac_full_acdc_10_khz_system.h"

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    ipc_init();

    g_ipc_mtoc_msg[0].WfmRef.SyncMode.enu = OneShot; //Default
}

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC operation.
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
* Setup BSMP server for FAC operation.
*
*/
static void bsmp_init_server(void)
{
    /**
     * Enable just server 0.
     * Considered FAC_Full_DCDC_20KHz
     */
    bsmp_init(0);

    Init_BSMP_var(2, 0, DP_Framework.NetSignals[4].u8);         // Iin1
    Init_BSMP_var(3, 0, DP_Framework.NetSignals[5].u8);         // Iin2
    Init_BSMP_var(4, 0, Buck.Iout1.u8);                         // Iout1
    Init_BSMP_var(5, 0, Buck.Iout2.u8);                         // Iout2
    Init_BSMP_var(7, 0, Buck.Vin1.u8);                          // Vin1
    Init_BSMP_var(8, 0, Buck.Vin2.u8);                          // Vin2
    Init_BSMP_var(11, 0, DP_Framework.NetSignals[6].u8);        // Vout1
    Init_BSMP_var(12, 0, DP_Framework.NetSignals[7].u8);        // Vout2
    Init_BSMP_var(15, 0, Buck.TempL1.u8);                       // Temp L1
    Init_BSMP_var(16, 0, Buck.TempL2.u8);                       // Temp L2

}

/**
* @brief System configuration for FAC.
*
* Initialize specific parameters e configure peripherals for FAC operation.
*
*/
void fac_full_acdc_10_khz_system_config(void)
{
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
