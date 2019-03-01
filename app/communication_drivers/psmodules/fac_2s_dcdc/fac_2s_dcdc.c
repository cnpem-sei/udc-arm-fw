/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_2s_dcdc.c
 * @brief FAC-2S DC/DC Stage module
 *
 * Module for control of two DC/DC modules of FAC power supplies for focusing
 * quadrupoles from booster. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 27/02/2019
 *
 */

#include <communication_drivers/psmodules/fac_2s_dcdc/fac_2s_dcdc.h>
#include <communication_drivers/psmodules/ps_modules.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"

/// DSP Net Signals
#define I_LOAD_1                        g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                        g_controller_ctom.net_signals[1]    // HRADC1
#define V_CAPBANK_MOD_1                 g_controller_ctom.net_signals[2]    // HRADC2
#define V_CAPBANK_MOD_2                 g_controller_ctom.net_signals[3]    // HRADC3

#define I_LOAD_REFERENCE_WFMREF         g_controller_ctom.net_signals[4]

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[5]
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[6]

#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[7]
#define DUTY_REFERENCE_FF               g_controller_ctom.net_signals[8]
#define DUTY_MEAN                       g_controller_ctom.net_signals[9]

#define V_OUT_DIFF                      g_controller_ctom.net_signals[10]
#define DUTY_DIFF                       g_controller_ctom.net_signals[11]

#define V_CAPBANK_MOD_1_FILTERED        g_controller_ctom.net_signals[12]
#define V_CAPBANK_MOD_2_FILTERED        g_controller_ctom.net_signals[13]

#define IN_FF_V_CAPBANK_MOD_1           g_controller_ctom.net_signals[14]
#define IN_FF_V_CAPBANK_MOD_2           g_controller_ctom.net_signals[15]

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[16]
#define V_LOAD                          g_controller_ctom.net_signals[17]

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define V_OUT_MOD_1                     g_controller_mtoc.net_signals[0]
#define V_OUT_MOD_2                     g_controller_mtoc.net_signals[1]

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P4S DCDC operation. Note that module 3
* and module 4 don't follow the sequence of ADCP channels, in order to keep
* cables installation on EXB more intuitive.
*/
static void adcp_channel_config(void)
{
    g_analog_ch_0.Enable = 0;
    g_analog_ch_1.Enable = 0;
    g_analog_ch_2.Enable = 0;
    g_analog_ch_3.Enable = 0;
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
    create_bsmp_var(25, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(26, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

    create_bsmp_var(27, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(28, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(29, 0, 4, false, I_LOAD_2.u8);

    create_bsmp_var(30, 0, 4, false, V_LOAD.u8);

    create_bsmp_var(31, 0, 4, false, V_OUT_MOD_1.u8);
    create_bsmp_var(32, 0, 4, false, V_OUT_MOD_2.u8);

    create_bsmp_var(33, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(34, 0, 4, false, V_CAPBANK_MOD_2.u8);

    create_bsmp_var(35, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(36, 0, 4, false, DUTY_CYCLE_MOD_2.u8);
    create_bsmp_var(37, 0, 4, false, DUTY_DIFF.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2s_dcdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
