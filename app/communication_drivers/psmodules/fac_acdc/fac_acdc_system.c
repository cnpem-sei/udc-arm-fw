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
 * @file fac_acdc_system.h
 * @brief System setup for operation as FAC ACDC
 *
 * @author gabriel.brunheira
 * @date 23/04/2018
 *
 */

#include <communication_drivers/psmodules/fac_acdc/fac_acdc_system.h>
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

#define V_CAPBANK                       g_controller_ctom.net_signals[0]    // HRADC0
#define IOUT_RECT                       g_controller_ctom.net_signals[1]    // HRADC1

#define VOUT_RECT                       g_controller_mtoc.net_signals[0]
#define TEMP_HEATSINK                   g_controller_mtoc.net_signals[1]
#define TEMP_INDUCTORS                  g_controller_mtoc.net_signals[2]

#define DUTY_CYCLE                      g_controller_ctom.output_signals[0]

static uint8_t dummy_u8;


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
    create_bsmp_var(27, 0, 4, false, V_CAPBANK.u8);
    create_bsmp_var(28, 0, 4, false, VOUT_RECT.u8);
    create_bsmp_var(29, 0, 4, false, IOUT_RECT.u8);
    create_bsmp_var(30, 0, 4, false, TEMP_HEATSINK.u8);
    create_bsmp_var(31, 0, 4, false, TEMP_INDUCTORS.u8);
    create_bsmp_var(32, 0, 4, false, DUTY_CYCLE.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_acdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
