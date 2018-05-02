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
 * @file fac_dcdc.c
 * @brief FAC DC/DC Stage module
 *
 * Module for control of DC/DC module of FAC power supplies. It implements the
 * controller for load current.
 *
 * @author gabriel.brunheira
 * @date 01/05/2018
 *
 */


#include <communication_drivers/psmodules/fac_dcdc/fac_dcdc.h>
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

#define I_LOAD_1            g_controller_ctom.net_signals[0]
#define I_LOAD_2            g_controller_ctom.net_signals[1]
#define I_LOAD_MEAN         g_controller_ctom.net_signals[2]
#define I_LOAD_DIFF         g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[4]

#define v_LOAD              g_controller_mtoc.net_signals[0]
#define V_CAPBANK           g_controller_mtoc.net_signals[1]
#define TEMP_INDUCTORS      g_controller_mtoc.net_signals[2]
#define TEMP_IGBT           g_controller_mtoc.net_signals[3]

#define DUTY_CYCLE          g_controller_ctom.output_signals[0]

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC ACDC operation.
*
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
    create_bsmp_var(27, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(28, 0, 4, false, I_LOAD_2.u8);
    create_bsmp_var(29, 0, 4, false, v_LOAD.u8);
    create_bsmp_var(30, 0, 4, false, V_CAPBANK.u8);
    create_bsmp_var(31, 0, 4, false, TEMP_INDUCTORS.u8);
    create_bsmp_var(32, 0, 4, false, TEMP_IGBT.u8);
    create_bsmp_var(33, 0, 4, false, DUTY_CYCLE.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_dcdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
