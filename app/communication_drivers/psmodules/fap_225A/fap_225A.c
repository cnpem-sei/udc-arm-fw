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
 * @file fap_225A.c
 * @brief FAP 225 A module
 *
 * Module for control of FAP power supply used by Magnets Group for magnets
 * characterization. It implements the controller for load current.
 *
 * @author gabriel.brunheira
 * @date 22/10/2018
 *
 */

#include <communication_drivers/psmodules/fap_225A/fap_225A.h>
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

/**
 * Controller defines
 */
#define I_LOAD                  g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_ERROR            g_controller_ctom.net_signals[1]
#define DUTY_MEAN               g_controller_ctom.net_signals[2]


#define I_IGBT_1                g_controller_mtoc.net_signals[0]    // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1]    // ANI1

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[3]
#define DUTY_DIFF               g_controller_ctom.net_signals[4]

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1]

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC ACDC operation.
*
*/
static void adcp_channel_config(void)
{
    // IGBT 1 current: 10 V = 200 A
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 200.0/2048.0;
    g_analog_ch_0.Value = &(I_IGBT_1.f);

    // IGBT 2 current: 10 V = 200 A
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 200.0/2048.0;
    g_analog_ch_1.Value = &(I_IGBT_2.f);

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
    create_bsmp_var(27, 0, 4, false, I_LOAD.u8);
    create_bsmp_var(28, 0, 4, false, I_IGBT_1.u8);
    create_bsmp_var(29, 0, 4, false, I_IGBT_2.u8);
    create_bsmp_var(30, 0, 4, false, DUTY_CYCLE_IGBT_1.u8);
    create_bsmp_var(31, 0, 4, false, DUTY_CYCLE_IGBT_2.u8);
    create_bsmp_var(32, 0, 4, false, DUTY_DIFF.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fap_225A_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
