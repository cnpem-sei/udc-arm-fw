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

#include "fap_acdc_system.h"

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    ipc_init();
    g_ipc_mtoc_msg[0].WfmRef.SyncMode.enu = OneShot;
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
     * Vout Rectifier 1: 10V = 60V
     */
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 60.0/2048.0;
    g_analog_ch_0.Value = &DP_Framework_MtoC.NetSignals[9].f;

    /**
     * Vout Rectifier 2: 10V = 60V
     */
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = -60.0/2048.0;
    g_analog_ch_1.Value = &DP_Framework_MtoC.NetSignals[10].f;

    g_analog_ch_2.Enable = 0;
    g_analog_ch_3.Enable = 0;
    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
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
     */
    bsmp_init(0);

    Init_BSMP_var(2, 0, Rectifier.IoutRectf1.u8);               // IoutRectf1
    Init_BSMP_var(3, 0, Rectifier.IoutRectf2.u8);               // IoutRectf2
    Init_BSMP_var(4, 0, Rectifier.LeakageCurrent.u8);           // LeakageCurrent
    Init_BSMP_var(11, 0, DP_Framework_MtoC.NetSignals[9].u8);   // VoutRectf1
    Init_BSMP_var(12, 0, DP_Framework_MtoC.NetSignals[10].u8);  // VoutRectf2
    Init_BSMP_var(13, 0, Rectifier.TempHeatSink.u8);            // TempHeatSink
    Init_BSMP_var(15, 0, Rectifier.TempModule1.u8);             // TempRectf1
    Init_BSMP_var(16, 0, Rectifier.TempModule2.u8);             // TempRectf2
    Init_BSMP_var(17, 0, Rectifier.TempL1.u8);                  // TempL1
    Init_BSMP_var(18, 0, Rectifier.TempL2.u8);                  // TempL2

}

/**
* @brief System configuration for FAP.
*
* Initialize specific parameters e configure peripherals for FAP operation.
*
*/
void fap_acdc_system_config(void)
{
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
