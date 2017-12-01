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

#include "fap_dcdc_20_khz_system.h"

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    ipc_init();

    g_ipc_mtoc_msg[0].SigGen.Enable.u16 = 0;
    g_ipc_mtoc_msg[0].SigGen.Amplitude[0].f = 180.0;
    g_ipc_mtoc_msg[0].SigGen.Aux.f = 2.0;
    g_ipc_mtoc_msg[0].SigGen.Freq.f = 0.0;
    g_ipc_mtoc_msg[0].SigGen.Ncycles.u16 = 1;
    g_ipc_mtoc_msg[0].SigGen.Offset.f = 0.0;
    g_ipc_mtoc_msg[0].SigGen.PhaseEnd.f = 30.0;
    g_ipc_mtoc_msg[0].SigGen.PhaseStart.f = 30.0;
    g_ipc_mtoc_msg[0].SigGen.Type.enu = Trapezoidal;
    g_ipc_mtoc_msg[0].WfmRef.SyncMode.enu = SampleBySample;

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
     * Iout module 1: 10V = 200A
     */
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 200.0/2048.0;
    g_analog_ch_0.Value = &DP_Framework_MtoC.NetSignals[0].f;

    /**
     * Iout module 2: 10V = 200A
     */
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 200.0/2048.0;
    g_analog_ch_1.Value = &DP_Framework_MtoC.NetSignals[1].f;

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
     * Considered FAP_DCDC_20KHz
     */
    bsmp_init(0);

    Init_BSMP_var(1, 0, DP_Framework.NetSignals[18].u8);        // Max Iload measured
    Init_BSMP_var(2, 0, DP_Framework_MtoC.NetSignals[2].u8);    // Imod1
    Init_BSMP_var(3, 0, DP_Framework_MtoC.NetSignals[3].u8);    // Imod2
    Init_BSMP_var(4, 0, DP_Framework_MtoC.NetSignals[20].u8);   // Imod1_MAX
    Init_BSMP_var(5, 0, DP_Framework_MtoC.NetSignals[21].u8);   // Imod2 MAX
    Init_BSMP_var(6, 0, DP_Framework_MtoC.NetSignals[9].u8);    // Vload
    Init_BSMP_var(7, 0, DP_Framework.NetSignals[13].u8);        // Vdclink (C28)
    Init_BSMP_var(8, 0, DP_Framework_MtoC.NetSignals[5].u8);    // Vdclink (IIB)
    Init_BSMP_var(9, 0, DP_Framework_MtoC.NetSignals[22].u8);   // Vdclink MAX (IIB)
    Init_BSMP_var(11, 0, DP_Framework.DutySignals[0].u8);       // Duty Mod1
    Init_BSMP_var(12, 0, DP_Framework.DutySignals[1].u8);       // Duty Mod2
    Init_BSMP_var(13, 0, DP_Framework_MtoC.NetSignals[23].u8);  // Vload MAX
    Init_BSMP_var(14, 0, Mod1Q1.TempHeatSink.u8);               // TempHeatSink
    Init_BSMP_var(15, 0, Mod1Q1.TempIGBT1.u8);                  // TempIGBT1
    Init_BSMP_var(16, 0, Mod1Q1.TempIGBT2.u8);                  // TempIGBT2
    Init_BSMP_var(17, 0, Mod1Q1.TempL1.u8);                     // TempL1
    Init_BSMP_var(18, 0, Mod1Q1.TempL2.u8);                     // TempL2

}

/**
* @brief System configuration for FAP.
*
* Initialize specific parameters e configure peripherals for FAP operation.
*
*/
void fap_dcdc_20_khz_system_config(void)
{
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
