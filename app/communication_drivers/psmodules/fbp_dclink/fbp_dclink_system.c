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
#include "communication_drivers/psmodules/fbp_dclink/fbp_dclink_system.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"

#include "fbp_dclink_system.h"

#define ID  0x0000

#define SOUR01_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[0]    // ANI1
#define SOUR02_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[1]    // ANI3
#define SOUR03_DCLINK_VOLTAGE       g_controller_mtoc.net_signals[2]    // ANI2
#define DIGITAL_POT_VOLTAGE         g_controller_mtoc.net_signals[3]
#define SOUROUT_DCLINK_VOLTAGE      g_controller_mtoc.net_signals[4]    // ANI0

#define PIN_STATUS_FAIL_PS_ALL      g_controller_ctom.net_signals[0]

//volatile float g_digital_pot_value = 0.0;

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    init_ipc(); //TODO: Refactor IPC module

    g_ipc_mtoc.ps_module[0].ps_status.bit.model = FBP_DCLink;
    g_ipc_mtoc.ps_module[0].ps_status.bit.active = 1;
    g_ipc_mtoc.ps_module[0].ps_status.bit.openloop = 1;
    g_ipc_mtoc.ps_module[0].ps_status.bit.state = Off;

//    g_ipc_mtoc_msg[uiloop].WfmRef.SyncMode.enu = SampleBySample_Continuous;
//    memcpy(0x20014000, get_wfm_ref_data_fbp(), 8192);
}

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FBP operation.
*
*/
static void adcp_channel_config(void)
{
    //PS_OUT VdcLink: 10V = 20V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 20.0/2048.0;
    g_analog_ch_0.Value = &(SOUROUT_DCLINK_VOLTAGE.f);

    //PS1 VdcLink: 10V = 20V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 20.0/2048.0;
    g_analog_ch_1.Value = &(SOUR01_DCLINK_VOLTAGE.f);

    //PS3 VdcLink: 10V = 20V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 20.0/2048.0;
    g_analog_ch_2.Value = &(SOUR03_DCLINK_VOLTAGE.f);

    //PS2 VdcLink: 10V = 20V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 20.0/2048.0;
    g_analog_ch_3.Value = &(SOUR02_DCLINK_VOLTAGE.f);

    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    bsmp_init(ID);

    set_bsmp_var_pointer(26, ID, g_ipc_ctom.ps_module[ID].ps_hard_interlock.u8);
    set_bsmp_var_pointer(31, ID, SOUR01_DCLINK_VOLTAGE.u8);
    set_bsmp_var_pointer(32, ID, SOUR02_DCLINK_VOLTAGE.u8);
    set_bsmp_var_pointer(33, ID, SOUR03_DCLINK_VOLTAGE.u8);
    set_bsmp_var_pointer(34, ID, PIN_STATUS_FAIL_PS_ALL.u8);
    set_bsmp_var_pointer(35, ID, DIGITAL_POT_VOLTAGE.u8);
    set_bsmp_var_pointer(36, ID, SOUROUT_DCLINK_VOLTAGE.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP_DCLINK operation.
*
*/
void fbp_dclink_system_config()
{
    init_control_framework(&g_controller_mtoc);//
    ipc_init_parameters();
    bsmp_init_server();
    adcp_channel_config();
}
