/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_2p_acdc_imas.h
 * @brief FAC-2P AC/DC Stage module for IMAS
 *
 * Module for control of two AC/DC modules of FAC power supplies used by IMAS
 * group on magnets characterization tests. It implements the individual
 * controllers for capacitor bank voltage of each AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 21/02/2020
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fac_2p_acdc_imas/fac_2p_acdc_imas.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Defines for module A variables
 */
#define MOD_A_ID        0x0

/// DSP Signals
#define V_CAPBANK_MOD_A                     g_controller_ctom.net_signals[0]    // HRADC0
#define IOUT_RECT_MOD_A                     g_controller_ctom.net_signals[1]    // HRADC1

#define V_CAPBANK_FILTERED_2HZ_MOD_A        g_controller_ctom.net_signals[2]
#define V_CAPBANK_FILTERED_2HZ_4HZ_MOD_A    g_controller_ctom.net_signals[3]
#define V_CAPBANK_ERROR_MOD_A               g_controller_ctom.net_signals[4]

#define IOUT_RECT_REF_MOD_A                 g_controller_ctom.net_signals[5]
#define IOUT_RECT_ERROR_MOD_A               g_controller_ctom.net_signals[6]

#define DUTY_CYCLE_MOD_A                    g_controller_ctom.output_signals[0]

/**
 * Defines for module B variables
 */
#define MOD_B_ID        0x1

/// DSP Signals
#define V_CAPBANK_MOD_B                     g_controller_ctom.net_signals[7]    // HRADC2
#define IOUT_RECT_MOD_B                     g_controller_ctom.net_signals[8]    // HRADC3

#define V_CAPBANK_FILTERED_2HZ_MOD_B        g_controller_ctom.net_signals[9]
#define V_CAPBANK_FILTERED_2HZ_4HZ_MOD_B    g_controller_ctom.net_signals[10]
#define V_CAPBANK_ERROR_MOD_B               g_controller_ctom.net_signals[11]

#define IOUT_RECT_REF_MOD_B                 g_controller_ctom.net_signals[12]
#define IOUT_RECT_ERROR_MOD_B               g_controller_ctom.net_signals[13]

#define DUTY_CYCLE_MOD_B                    g_controller_ctom.output_signals[1]

/**
 * Interlocks defines
 */

typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overcurrent,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    Module_A_Interlock,
    Module_B_Interlock,
    DCDC_Interlock
} hard_interlocks_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P AC/DC operation.
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
* Setup BSMP servers for FAC-2P AC/DC operation.
*
*/
static void bsmp_init_server(void)
{
    /**
     * Create module A specific variables
     */
    create_bsmp_var(31, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_A_ID, 4, false, V_CAPBANK_MOD_A.u8);
    create_bsmp_var(34, MOD_A_ID, 4, false, IOUT_RECT_MOD_A.u8);
    create_bsmp_var(35, MOD_A_ID, 4, false, DUTY_CYCLE_MOD_A.u8);

    /**
     * Create module B specific variables
     */

    /// Module A BSMP server already initialized
    bsmp_init(MOD_B_ID);

    /// Both modules share these variables
    modify_bsmp_var(0, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_status.u8);
    modify_bsmp_var(1, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_setpoint.u8);
    modify_bsmp_var(2, MOD_B_ID, g_ipc_ctom.ps_module[0].ps_reference.u8);

    create_bsmp_var(31, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_B_ID, 4, false, g_ipc_ctom.ps_module[MOD_B_ID].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_B_ID, 4, false, V_CAPBANK_MOD_B.u8);
    create_bsmp_var(34, MOD_B_ID, 4, false, IOUT_RECT_MOD_B.u8);
    create_bsmp_var(35, MOD_B_ID, 4, false, DUTY_CYCLE_MOD_B.u8);
}

/**
* @brief System configuration for FAC-2P AC/DC.
*
* Initialize specific parameters e configure peripherals for FAC-2P AC/DC
* operation.
*
*/
void fac_2p_acdc_imas_system_config()
{
    adcp_channel_config();
    bsmp_init_server();

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM/2, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);

    init_scope(&g_ipc_mtoc.scope[1], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[1].f,
               &(g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM/2].f),
               SIZE_BUF_SAMPLES_CTOM/2, SCOPE_SOURCE_PARAM[1].p_f,
               (void *) 0);

}
