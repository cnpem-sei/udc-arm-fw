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
 * @file fbp_ufjf.c
 * @brief FBP for UFJF controllers
 *
 * Module for control of FBP crate adapted for usage on tests of controllers
 * developed in CNPEM-UFJF partnership
 *
 * @author gabriel.brunheira
 * @date 16/08/2022
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
#include "communication_drivers/ps_modules/fbp_ufjf/fbp_ufjf.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/// DSP Net Signals
#define I1_LOAD_CURRENT                 g_controller_ctom.net_signals[0]    // HRADC0
#define PS2_LOAD_CURRENT                g_controller_ctom.net_signals[1]    // HRADC1
#define PS3_LOAD_CURRENT                g_controller_ctom.net_signals[2]    // HRADC2
#define I2_LOAD_CURRENT                 g_controller_ctom.net_signals[3]    // HRADC3

#define I1_LOAD_ERROR                   g_controller_ctom.net_signals[4]
#define I1_LOAD_ERROR_TO_M11            g_controller_ctom.net_signals[5]
#define M11                             g_controller_ctom.net_signals[6]
#define I1_LOAD_ERROR_TO_M12            g_controller_ctom.net_signals[7]
#define M12                             g_controller_ctom.net_signals[8]

#define I2_LOAD_ERROR                   g_controller_ctom.net_signals[9]
#define I2_LOAD_ERROR_TO_M21            g_controller_ctom.net_signals[10]
#define M21                             g_controller_ctom.net_signals[11]
#define I2_LOAD_ERROR_TO_M22            g_controller_ctom.net_signals[12]
#define M22                             g_controller_ctom.net_signals[13]

#define DUTY_CYCLE_I1                   g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_I2                   g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define PS1_LOAD_VOLTAGE                g_controller_mtoc.net_signals[0]    // ANI6
#define PS2_LOAD_VOLTAGE                g_controller_mtoc.net_signals[1]    // ANI7
#define PS3_LOAD_VOLTAGE                g_controller_mtoc.net_signals[2]    // ANI3
#define PS4_LOAD_VOLTAGE                g_controller_mtoc.net_signals[3]    // ANI5

static uint8_t dummy_u8;

/**
 * Interlocks defines
 */
typedef enum
{
    I1_Load_Overcurrent,
    I2_Load_Overcurrent,
    Load_Overvoltage_Mod_1,
    Load_Overvoltage_Mod_2,
    Load_Overvoltage_Mod_3,
    Load_Overvoltage_Mod_4
} hard_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FBP operation.
*
*/
static void adcp_channel_config(void)
{
    // PS1 load voltage: 10V = 50V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 50.0/2048.0;
    g_analog_ch_6.Value = &(PS1_LOAD_VOLTAGE.f);

    // PS2 load voltage: 10V = 50V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 50.0/2048.0;
    g_analog_ch_7.Value = &(PS2_LOAD_VOLTAGE.f);

    // PS3 load voltage: 10V = 50V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 50.0/2048.0;
    g_analog_ch_3.Value = &(PS3_LOAD_VOLTAGE.f);

    // PS4 load voltage: 10V = 50V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 50.0/2048.0;
    g_analog_ch_5.Value = &(PS4_LOAD_VOLTAGE.f);

    // Unused
    g_analog_ch_0.Enable = 0;
    g_analog_ch_1.Enable = 0;
    g_analog_ch_2.Enable = 0;
    g_analog_ch_4.Enable = 0;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    uint8_t server;

    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);
    create_bsmp_var(33, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);
    create_bsmp_var(34, 0, 4, false, I1_LOAD_CURRENT.u8);
    create_bsmp_var(35, 0, 4, false, I2_LOAD_CURRENT.u8);
    create_bsmp_var(36, 0, 4, false, PS2_LOAD_CURRENT.u8);
    create_bsmp_var(37, 0, 4, false, PS3_LOAD_CURRENT.u8);
    create_bsmp_var(38, 0, 4, false, PS1_LOAD_VOLTAGE.u8);
    create_bsmp_var(39, 0, 4, false, PS2_LOAD_VOLTAGE.u8);
    create_bsmp_var(40, 0, 4, false, PS3_LOAD_VOLTAGE.u8);
    create_bsmp_var(41, 0, 4, false, PS4_LOAD_VOLTAGE.u8);
    create_bsmp_var(42, 0, 4, false, DUTY_CYCLE_I1.u8);
    create_bsmp_var(43, 0, 4, false, DUTY_CYCLE_I2.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fbp_ufjf_system_config()
{
    uint8_t i;

    adcp_channel_config();
    bsmp_init_server();

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        init_wfmref(&WFMREF[i], WFMREF_SELECTED_PARAM[i].u16,
                    WFMREF_SYNC_MODE_PARAM[i].u16, ISR_CONTROL_FREQ.f,
                    WFMREF_FREQUENCY_PARAM[i].f,
                    WFMREF_GAIN_PARAM[i].f, WFMREF_OFFSET_PARAM[i].f,
                    &g_wfmref_data.data_fbp[i][0][0].f,
                    SIZE_WFMREF_FBP, &g_ipc_ctom.ps_module[i].ps_reference.f);

        init_scope(&g_ipc_mtoc.scope[i], ISR_CONTROL_FREQ.f,
                   SCOPE_FREQ_SAMPLING_PARAM[i].f,
                   &(g_buf_samples_ctom[SIZE_BUF_SAMPLES_CTOM * i / NUM_MAX_PS_MODULES].f),
                   SIZE_BUF_SAMPLES_CTOM / NUM_MAX_PS_MODULES,
                   SCOPE_SOURCE_PARAM[i].p_f, (void *) 0);
    }
}
