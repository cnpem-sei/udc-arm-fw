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
 * @file fac_2s_acdc.c
 * @brief FAC-2S AC/DC Stage module
 *
 * Module for control of two AC/DC modules of FAC power supplies for focusing
 * quadrupoles from booster. It implements the individual controllers for input
 * current and capacitor bank voltage of each AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 27/02/2019
 *
 */

#include <communication_drivers/psmodules/fac_2s_acdc/fac_2s_acdc.h>
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
 * Defines for module A variables
 */
#define MOD_A_ID                    0x0

#define V_CAPBANK_MOD_A             g_controller_ctom.net_signals[0]  // HRADC0
#define IOUT_RECT_MOD_A             g_controller_ctom.net_signals[1]  // HRADC1

#define VOUT_RECT_MOD_A             g_controller_mtoc.net_signals[0]
#define TEMP_HEATSINK_MOD_A         g_controller_mtoc.net_signals[1]
#define TEMP_INDUCTORS_MOD_A        g_controller_mtoc.net_signals[2]

#define DUTY_CYCLE_MOD_A            g_controller_ctom.output_signals[0]

/**
 * Defines for module B variables
 */
#define MOD_B_ID                    0x1

#define V_CAPBANK_MOD_B             g_controller_ctom.net_signals[2]  // HRADC2
#define IOUT_RECT_MOD_B             g_controller_ctom.net_signals[3]  // HRADC3

#define VOUT_RECT_MOD_B             g_controller_mtoc.net_signals[3]
#define TEMP_HEATSINK_MOD_B         g_controller_mtoc.net_signals[4]
#define TEMP_INDUCTORS_MOD_B        g_controller_mtoc.net_signals[5]

#define DUTY_CYCLE_MOD_B            g_controller_ctom.output_signals[1]

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P4S ACDC operation.
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
* Setup BSMP servers for FAC-2P4S AC/DC operation.
*
*/
static void bsmp_init_server(void)
{
    /**
     * Create module A specific variables
     */
    create_bsmp_var(25, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, MOD_A_ID, 4, false, V_CAPBANK_MOD_A.u8);
    create_bsmp_var(28, MOD_A_ID, 4, false, VOUT_RECT_MOD_A.u8);
    create_bsmp_var(29, MOD_A_ID, 4, false, IOUT_RECT_MOD_A.u8);
    create_bsmp_var(30, MOD_A_ID, 4, false, TEMP_HEATSINK_MOD_A.u8);
    create_bsmp_var(31, MOD_A_ID, 4, false, TEMP_INDUCTORS_MOD_A.u8);
    create_bsmp_var(32, MOD_A_ID, 4, false, DUTY_CYCLE_MOD_A.u8);

    /**
     * Create module B specific variables
     */

    /// Module A BSMP server already initialized
    bsmp_init(MOD_B_ID);

    /// Both modules share these variables
    modify_bsmp_var(0, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_status.u8);
    modify_bsmp_var(1, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_setpoint.u8);
    modify_bsmp_var(2, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_reference.u8);

    create_bsmp_var(25, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, MOD_B_ID, 4, false, V_CAPBANK_MOD_B.u8);
    create_bsmp_var(28, MOD_B_ID, 4, false, VOUT_RECT_MOD_B.u8);
    create_bsmp_var(29, MOD_B_ID, 4, false, IOUT_RECT_MOD_B.u8);
    create_bsmp_var(30, MOD_B_ID, 4, false, TEMP_HEATSINK_MOD_B.u8);
    create_bsmp_var(31, MOD_B_ID, 4, false, TEMP_INDUCTORS_MOD_B.u8);
    create_bsmp_var(32, MOD_B_ID, 4, false, DUTY_CYCLE_MOD_B.u8);
}

/**
* @brief System configuration for FAC-2P4S AC/DC.
*
* Initialize specific parameters e configure peripherals for FAC-2P4S AC/DC
* operation.
*
*/
void fac_2s_acdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
