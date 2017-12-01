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

#include "fac_full_dcdc_20_khz_system.h"

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
     * Just IPC for module 0.
     */
    g_ipc_mtoc_msg[0].WfmRef.SyncMode.enu = OneShot;
    memcpy(0x20014000, get_wfm_ref_data_fac(), 8192);

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
     * Iout module 1: 10V = 1000A
     */
    g_analog_ch_1.Enable    = 1;
    g_analog_ch_1.Gain      = 1000.0 / 2048.0;
    g_analog_ch_1.Value     = &DP_Framework_MtoC.NetSignals[0].f;

    /**
     * Iout module 2: 10V = 1000A
     */
    g_analog_ch_2.Enable    = 1;
    g_analog_ch_2.Gain      = 1000.0 / 2048.0;
    g_analog_ch_2.Value     = &DP_Framework_MtoC.NetSignals[1].f;

    g_analog_ch_0.Enable    = 0;
    g_analog_ch_3.Enable    = 0;
    g_analog_ch_4.Enable    = 0;
    g_analog_ch_5.Enable    = 0;
    g_analog_ch_6.Enable    = 0;
    g_analog_ch_7.Enable    = 0;
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

    Init_BSMP_var(1, 0, DP_Framework.NetSignals[11].u8);        // dDuty
    Init_BSMP_var(2, 0, DP_Framework_MtoC.NetSignals[0].u8);    // Iout1
    Init_BSMP_var(3, 0, DP_Framework_MtoC.NetSignals[1].u8);    // Iout2
    Init_BSMP_var(7, 0, DP_Framework.NetSignals[17].u8);        // V DC Link Mod1
    Init_BSMP_var(8, 0, DP_Framework.NetSignals[19].u8);        // V DC Link Mod2
    Init_BSMP_var(11, 0, DP_Framework.DutySignals[0].u8);       // Duty Mod 1
    Init_BSMP_var(12, 0, DP_Framework.DutySignals[1].u8);       // Duty Mod 2
    Init_BSMP_var(13, 0, Mod1Q4.RH);                            // Relative humidity Mod1
    Init_BSMP_var(14, 0, Mod2Q4.RH);                            // Relative humidity Mod2
    Init_BSMP_var(15, 0, Mod1Q4.TempIGBT1.u8);                  // Temp IGBTs 1+4 Mod1
    Init_BSMP_var(16, 0, Mod1Q4.TempIGBT2.u8);                  // Temp IGBTs 2+3 Mod1
    Init_BSMP_var(17, 0, Mod2Q4.TempIGBT1.u8);                  // Temp IGBTs 1+4 Mod2
    Init_BSMP_var(18, 0, Mod2Q4.TempIGBT2.u8);                  // Temp IGBTs 2+3 Mod2

}

/**
* @brief System configuration for FAC.
*
* Initialize specific parameters e configure peripherals for FAC operation.
*
*/
void fac_full_dcdc_20_khz_system_config(void)
{
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
