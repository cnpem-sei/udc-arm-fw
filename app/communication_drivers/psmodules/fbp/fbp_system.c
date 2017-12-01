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
 * @file fbpsystem.c
 * @brief System setup for operation as FBP
 *
 * @author allef.silva
 * @date 18/10/2017
 *
 */

#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/psmodules/ps_modules.h"
#include "communication_drivers/psmodules/fbp/fbp_system.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"

#define PS1_ID                    0x0000

#define PS1_LOAD_CURRENT          g_controller_ctom.net_signals[0].u8    // HRADC0
#define PS1_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[0].u8    // ANI2
#define PS1_LOAD_VOLTAGE          g_controller_mtoc.net_signals[4].u8    // ANI6
#define PS1_TEMPERATURE           g_controller_mtoc.net_signals[8].u8   // I2C Add 0x48

#define PS2_ID                    0x0001

#define PS2_LOAD_CURRENT          g_controller_ctom.net_signals[1].u8    // HRADC1
#define PS2_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[1].u8    // ANI1
#define PS2_LOAD_VOLTAGE          g_controller_mtoc.net_signals[5].u8   // ANI7
#define PS2_TEMPERATURE           g_controller_mtoc.net_signals[9].u8   // I2C Add 0x49

#define PS3_ID                    0x0002

#define PS3_LOAD_CURRENT          g_controller_ctom.net_signals[2].u8    // HRADC2
#define PS3_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[2].u8    // ANI4
#define PS3_LOAD_VOLTAGE          g_controller_mtoc.net_signals[6].u8   // ANI3
#define PS3_TEMPERATURE           g_controller_mtoc.net_signals[10].u8   // I2C Add 0x4A

#define PS4_ID                    0x0003

#define PS4_LOAD_CURRENT          g_controller_ctom.net_signals[3].u8   // HRADC3
#define PS4_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[3].u8    // ANI0
#define PS4_LOAD_VOLTAGE          g_controller_mtoc.net_signals[7].u8   // ANI5
#define PS4_TEMPERATURE           g_controller_mtoc.net_signals[11].u8   // I2C Add 0x4C

/**
 * Number of power supplies
 */
static const uint8_t fbp_qtd = 4;

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    ipc_init(); //TODO: Refactor IPC module

    volatile uint8_t uiloop;

    for (uiloop = 0; uiloop < fbp_qtd; uiloop++)
    {
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.model = FBP;
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.active = 1;
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.openloop = 1;
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.state = Off;
//        g_ipc_mtoc_msg[uiloop].WfmRef.SyncMode.enu = SampleBySample_Continuous;
//        memcpy(0x20014000, get_wfm_ref_data_fbp(), 8192);
    }
}

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FBP operation.
*
*/
static void adcp_channel_config(void)
{

    // PS1 VdcLink: 10V = 20V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 20.0/2048.0;
    g_analog_ch_2.Value = &g_controller_mtoc.net_signals[0].f;

    // PS2 VdcLink: 10V = 20V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 20.0/2048.0;
    g_analog_ch_1.Value = &g_controller_mtoc.net_signals[1].f;

    // PS3 VdcLink: 10V = 20V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 20.0/2048.0;
    g_analog_ch_4.Value = &g_controller_mtoc.net_signals[2].f;

    // PS4 VdcLink: 10V = 20V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 20.0/2048.0;
    g_analog_ch_0.Value = &g_controller_mtoc.net_signals[3].f;

    // PS1 Vload: 10V = 20.2V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 20.2/2048.0;
    g_analog_ch_6.Value = &g_controller_mtoc.net_signals[4].f;

    // PS2 Vload: 10V = 20.2V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 20.2/2048.0;
    g_analog_ch_7.Value = &g_controller_mtoc.net_signals[5].f;

    // PS3 Vload: 10V = 20.2V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 20.2/2048.0;
    g_analog_ch_3.Value = &g_controller_mtoc.net_signals[6].f;

    // PS4 Vload: 10V = 20.2V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 20.2/2048.0;
    g_analog_ch_5.Value = &g_controller_mtoc.net_signals[7].f;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    bsmp_init(PS1_ID);
    set_bsmp_var_pointer(25, PS1_ID, g_ipc_ctom.ps_module[PS1_ID].ps_soft_interlock.u8);
    set_bsmp_var_pointer(26, PS1_ID, g_ipc_ctom.ps_module[PS1_ID].ps_hard_interlock.u8);
    set_bsmp_var_pointer(27, PS1_ID, PS1_LOAD_CURRENT);
    set_bsmp_var_pointer(28, PS1_ID, PS1_LOAD_VOLTAGE);
    set_bsmp_var_pointer(29, PS1_ID, PS1_DCLINK_VOLTAGE);
    set_bsmp_var_pointer(30, PS1_ID, PS1_TEMPERATURE);

    bsmp_init(PS2_ID);
    set_bsmp_var_pointer(25, PS2_ID, g_ipc_ctom.ps_module[PS2_ID].ps_soft_interlock.u8);
    set_bsmp_var_pointer(26, PS2_ID, g_ipc_ctom.ps_module[PS2_ID].ps_hard_interlock.u8);
    set_bsmp_var_pointer(27, PS2_ID, PS2_LOAD_CURRENT);
    set_bsmp_var_pointer(28, PS2_ID, PS2_LOAD_VOLTAGE);
    set_bsmp_var_pointer(29, PS2_ID, PS2_DCLINK_VOLTAGE);
    set_bsmp_var_pointer(30, PS2_ID, PS2_TEMPERATURE);

    bsmp_init(PS3_ID);
    set_bsmp_var_pointer(25, PS3_ID, g_ipc_ctom.ps_module[PS3_ID].ps_soft_interlock.u8);
    set_bsmp_var_pointer(26, PS3_ID, g_ipc_ctom.ps_module[PS3_ID].ps_hard_interlock.u8);
    set_bsmp_var_pointer(27, PS3_ID, PS3_LOAD_CURRENT);
    set_bsmp_var_pointer(28, PS3_ID, PS3_LOAD_VOLTAGE);
    set_bsmp_var_pointer(29, PS3_ID, PS3_DCLINK_VOLTAGE);
    set_bsmp_var_pointer(30, PS3_ID, PS3_TEMPERATURE);

    bsmp_init(PS4_ID);
    set_bsmp_var_pointer(25, PS4_ID, g_ipc_ctom.ps_module[PS4_ID].ps_soft_interlock.u8);
    set_bsmp_var_pointer(26, PS4_ID, g_ipc_ctom.ps_module[PS4_ID].ps_hard_interlock.u8);
    set_bsmp_var_pointer(27, PS4_ID, PS4_LOAD_CURRENT);
    set_bsmp_var_pointer(28, PS4_ID, PS4_LOAD_VOLTAGE);
    set_bsmp_var_pointer(29, PS4_ID, PS4_DCLINK_VOLTAGE);
    set_bsmp_var_pointer(30, PS4_ID, PS4_TEMPERATURE);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fbp_system_config()
{
    init_control_framework(&g_controller_mtoc);
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
