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
 * @file fac_2p_dcdc_imas.h
 * @brief FAC-2P DC/DC Stage module for IMAS
 *
 * Module for control of two DC/DC modules of FAC power supplies used by IMAS
 * group on magnets characterization tests. It implements the controller for
 * load current.
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
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fac_2p_dcdc_imas/fac_2p_dcdc_imas.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/// DSP Net Signals
#define I_LOAD                          g_controller_ctom.net_signals[0]    // HRADC0
#define V_CAPBANK_MOD_1                 g_controller_ctom.net_signals[1]    // HRADC1
#define V_CAPBANK_MOD_2                 g_controller_ctom.net_signals[2]    // HRADC2

#define I_LOAD_ERROR                    g_controller_ctom.net_signals[3]

#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[4]
#define DUTY_REF_FF                     g_controller_ctom.net_signals[5]
#define DUTY_MEAN                       g_controller_ctom.net_signals[6]

#define I_ARMS_DIFF                     g_controller_ctom.net_signals[7]
#define DUTY_ARMS_DIFF                  g_controller_ctom.net_signals[8]

#define V_CAPBANK_MOD_1_FILTERED        g_controller_ctom.net_signals[9]
#define V_CAPBANK_MOD_2_FILTERED        g_controller_ctom.net_signals[10]

#define IN_FF_V_CAPBANK_MOD_1           g_controller_ctom.net_signals[11]
#define IN_FF_V_CAPBANK_MOD_2           g_controller_ctom.net_signals[12]

#define WFMREF_IDX                      g_controller_ctom.net_signals[30]

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define I_ARM_1                         g_controller_mtoc.net_signals[0]
#define I_ARM_2                         g_controller_mtoc.net_signals[1]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
    Arm_1_Overcurrent,
    Arm_2_Overcurrent,
    Arms_High_Difference,
    ACDC_Interlock
} hard_interlocks_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P DC/DC operation.
*/
static void adcp_channel_config(void)
{
    // Module 1 output current: 10 V = 1000 A
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 1000.0/2048.0;
    g_analog_ch_1.Value = &(I_ARM_1.f);

    // Module 2 output current: 10 V = 1000 A
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 1000.0/2048.0;
    g_analog_ch_2.Value = &(I_ARM_2.f);

    g_analog_ch_0.Enable = 0;
    g_analog_ch_3.Enable = 0;
    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FAC-2P DC/DC operation.
*
*/
static void bsmp_init_server(void)
{
    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

    create_bsmp_var(33, 0, 4, false, I_LOAD.u8);
    create_bsmp_var(34, 0, 4, false, I_LOAD_ERROR.u8);

    create_bsmp_var(35, 0, 4, false, I_ARM_1.u8);
    create_bsmp_var(36, 0, 4, false, I_ARM_2.u8);
    create_bsmp_var(37, 0, 4, false, I_ARMS_DIFF.u8);

    create_bsmp_var(38, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(39, 0, 4, false, V_CAPBANK_MOD_2.u8);

    create_bsmp_var(40, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(41, 0, 4, false, DUTY_CYCLE_MOD_2.u8);
    create_bsmp_var(42, 0, 4, false, DUTY_ARMS_DIFF.u8);
}

/**
* @brief System configuration for FAC-2P DC/DC.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2p_dcdc_imas_system_config()
{
    adcp_channel_config();
    bsmp_init_server();

    init_wfmref(&WFMREF[0], WFMREF[0].wfmref_selected.u16, WFMREF[0].sync_mode.enu,
                ISR_CONTROL_FREQ.f,
                TIMESLICER_FREQ[TIMESLICER_WFMREF].f,
                WFMREF[0].gain.f, WFMREF[0].offset.f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}
