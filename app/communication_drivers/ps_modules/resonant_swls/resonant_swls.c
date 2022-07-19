/******************************************************************************
 * Copyright (C) 2022 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file resonant_swls.h
 * @brief Resonant converter module for SWLS
 *
 * Module for control of resonant convert power supply designed for the
 * superconducting Wavelength Shifter. It implements the controller for load
 * current using frequency modulation strategy, instead of pulse-width
 * modulation (PWM).
 *
 * @author gabriel.brunheira
 * @date 18/07/2022
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/resonant_swls/resonant_swls.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD                  g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_ERROR            g_controller_ctom.net_signals[2]
#define FREQ_MODULATED          g_controller_ctom.output_signals[0]

/// ARM Net Signals

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent
} hard_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAP operation.
*
*/
static void adcp_channel_config(void)
{
    g_analog_ch_0.Enable = 0;
    g_analog_ch_0.Enable = 0;
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
    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(33, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);

    create_bsmp_var(34, 0, 4, false, I_LOAD.u8);
    create_bsmp_var(35, 0, 4, false, FREQ_MODULATED.u8);

}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void resonant_swls_system_config()
{
    adcp_channel_config();
    bsmp_init_server();

    init_wfmref(&WFMREF[0], WFMREF_SELECTED_PARAM[0].u16,
                WFMREF_SYNC_MODE_PARAM[0].u16, ISR_CONTROL_FREQ.f,
                WFMREF_FREQUENCY_PARAM[0].f, WFMREF_GAIN_PARAM[0].f,
                WFMREF_OFFSET_PARAM[0].f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}
