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

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/ps_modules/fbp/fbp.h"
#include "communication_drivers/ps_modules/ps_modules.h"

#define PS1_ID                    0x0000

#define PS1_LOAD_CURRENT          g_controller_ctom.net_signals[0]   // HRADC0
#define PS1_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[0]   // ANI2
#define PS1_LOAD_VOLTAGE          g_controller_mtoc.net_signals[4]   // ANI6
#define PS1_TEMPERATURE           g_controller_mtoc.net_signals[8]  // I2C Add 0x48
#define PS1_DUTY_CYCLE            g_controller_ctom.output_signals[0]

#define PS2_ID                    0x0001

#define PS2_LOAD_CURRENT          g_controller_ctom.net_signals[1]    // HRADC1
#define PS2_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[1]    // ANI1
#define PS2_LOAD_VOLTAGE          g_controller_mtoc.net_signals[5]   // ANI7
#define PS2_TEMPERATURE           g_controller_mtoc.net_signals[9]   // I2C Add 0x49
#define PS2_DUTY_CYCLE            g_controller_ctom.output_signals[1]

#define PS3_ID                    0x0002

#define PS3_LOAD_CURRENT          g_controller_ctom.net_signals[2]    // HRADC2
#define PS3_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[2]    // ANI4
#define PS3_LOAD_VOLTAGE          g_controller_mtoc.net_signals[6]   // ANI3
#define PS3_TEMPERATURE           g_controller_mtoc.net_signals[10]   // I2C Add 0x4A
#define PS3_DUTY_CYCLE            g_controller_ctom.output_signals[2]

#define PS4_ID                    0x0003

#define PS4_LOAD_CURRENT          g_controller_ctom.net_signals[3]   // HRADC3
#define PS4_DCLINK_VOLTAGE        g_controller_mtoc.net_signals[3]    // ANI0
#define PS4_LOAD_VOLTAGE          g_controller_mtoc.net_signals[7]   // ANI5
#define PS4_TEMPERATURE           g_controller_mtoc.net_signals[11]   // I2C Add 0x4C
#define PS4_DUTY_CYCLE            g_controller_ctom.output_signals[3]

static uint8_t dummy_u8;

/**
* @brief Initialize IPC Parameters.
*
* Setup IPC global configurations.
*
*/
static void ipc_init_parameters(void)
{
    volatile uint8_t uiloop, uiloop2;

    for (uiloop = 0; uiloop < (uint8_t) get_param(Num_PS_Modules,0); uiloop++)
    {
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.model = FBP;
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.active = 1;
        g_ipc_mtoc.ps_module[uiloop].ps_status.bit.openloop = 1;
    }
}

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FBP operation.
*
*/
static void adcp_channel_config(void)
{

    // PS1 VdcLink: 10V = 20V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 20.0/2048.0;
    g_analog_ch_2.Value = &(PS1_DCLINK_VOLTAGE.f);

    // PS2 VdcLink: 10V = 20V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 20.0/2048.0;
    g_analog_ch_1.Value = &(PS2_DCLINK_VOLTAGE.f);

    // PS3 VdcLink: 10V = 20V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 20.0/2048.0;
    g_analog_ch_4.Value = &(PS3_DCLINK_VOLTAGE.f);

    // PS4 VdcLink: 10V = 20V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 20.0/2048.0;
    g_analog_ch_0.Value = &(PS4_DCLINK_VOLTAGE.f);

    // PS1 Vload: 10V = 20.2V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 20.2/2048.0;
    g_analog_ch_6.Value = &(PS1_LOAD_VOLTAGE.f);

    // PS2 Vload: 10V = 20.2V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 20.2/2048.0;
    g_analog_ch_7.Value = &(PS2_LOAD_VOLTAGE.f);

    // PS3 Vload: 10V = 20.2V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 20.2/2048.0;
    g_analog_ch_3.Value = &(PS3_LOAD_VOLTAGE.f);

    // PS4 Vload: 10V = 20.2V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 20.2/2048.0;
    g_analog_ch_5.Value = &(PS4_LOAD_VOLTAGE.f);
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

    // PS1 BSMP server already initialized
    bsmp_init(PS2_ID);
    bsmp_init(PS3_ID);
    bsmp_init(PS4_ID);

    create_bsmp_var(25, PS1_ID, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, PS1_ID, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, PS1_ID, 4, false, PS1_LOAD_CURRENT.u8);
    create_bsmp_var(28, PS1_ID, 4, false, PS1_LOAD_VOLTAGE.u8);
    create_bsmp_var(29, PS1_ID, 4, false, PS1_DCLINK_VOLTAGE.u8);
    create_bsmp_var(30, PS1_ID, 4, false, PS1_TEMPERATURE.u8);
    create_bsmp_var(31, PS1_ID, 4, false, PS1_DUTY_CYCLE.u8);

    create_bsmp_var(25, PS2_ID, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, PS2_ID, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, PS2_ID, 4, false, PS2_LOAD_CURRENT.u8);
    create_bsmp_var(28, PS2_ID, 4, false, PS2_LOAD_VOLTAGE.u8);
    create_bsmp_var(29, PS2_ID, 4, false, PS2_DCLINK_VOLTAGE.u8);
    create_bsmp_var(30, PS2_ID, 4, false, PS2_TEMPERATURE.u8);
    create_bsmp_var(31, PS2_ID, 4, false, PS2_DUTY_CYCLE.u8);

    create_bsmp_var(25, PS3_ID, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, PS3_ID, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, PS3_ID, 4, false, PS3_LOAD_CURRENT.u8);
    create_bsmp_var(28, PS3_ID, 4, false, PS3_LOAD_VOLTAGE.u8);
    create_bsmp_var(29, PS3_ID, 4, false, PS3_DCLINK_VOLTAGE.u8);
    create_bsmp_var(30, PS3_ID, 4, false, PS3_TEMPERATURE.u8);
    create_bsmp_var(31, PS3_ID, 4, false, PS3_DUTY_CYCLE.u8);

    create_bsmp_var(25, PS4_ID, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_soft_interlock.u8);
    create_bsmp_var(26, PS4_ID, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_hard_interlock.u8);
    create_bsmp_var(27, PS4_ID, 4, false, PS4_LOAD_CURRENT.u8);
    create_bsmp_var(28, PS4_ID, 4, false, PS4_LOAD_VOLTAGE.u8);
    create_bsmp_var(29, PS4_ID, 4, false, PS4_DCLINK_VOLTAGE.u8);
    create_bsmp_var(30, PS4_ID, 4, false, PS4_TEMPERATURE.u8);
    create_bsmp_var(31, PS4_ID, 4, false, PS4_DUTY_CYCLE.u8);

    for(server = 0; server < NUM_MAX_PS_MODULES; server++)
    {
        create_bsmp_var(32, server, 1, false, &dummy_u8);
        create_bsmp_var(33, server, 1, false, &dummy_u8);
        create_bsmp_var(34, server, 1, false, &dummy_u8);
        create_bsmp_var(35, server, 1, false, &dummy_u8);
        create_bsmp_var(36, server, 1, false, &dummy_u8);
        create_bsmp_var(37, server, 1, false, &dummy_u8);
        create_bsmp_var(38, server, 1, false, &dummy_u8);
        create_bsmp_var(39, server, 1, false, &dummy_u8);

        create_bsmp_var(40, server, 2, false, g_ipc_ctom.ps_module[PS1_ID].ps_status.u8);
        create_bsmp_var(41, server, 2, false, g_ipc_ctom.ps_module[PS2_ID].ps_status.u8);
        create_bsmp_var(42, server, 2, false, g_ipc_ctom.ps_module[PS3_ID].ps_status.u8);
        create_bsmp_var(43, server, 2, false, g_ipc_ctom.ps_module[PS4_ID].ps_status.u8);

        create_bsmp_var(44, server, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_setpoint.u8);
        create_bsmp_var(45, server, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_setpoint.u8);
        create_bsmp_var(46, server, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_setpoint.u8);
        create_bsmp_var(47, server, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_setpoint.u8);

        create_bsmp_var(48, server, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_reference.u8);
        create_bsmp_var(49, server, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_reference.u8);
        create_bsmp_var(50, server, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_reference.u8);
        create_bsmp_var(51, server, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_reference.u8);

        create_bsmp_var(52, server, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_soft_interlock.u8);
        create_bsmp_var(53, server, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_soft_interlock.u8);
        create_bsmp_var(54, server, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_soft_interlock.u8);
        create_bsmp_var(55, server, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_soft_interlock.u8);

        create_bsmp_var(56, server, 4, false, g_ipc_ctom.ps_module[PS1_ID].ps_hard_interlock.u8);
        create_bsmp_var(57, server, 4, false, g_ipc_ctom.ps_module[PS2_ID].ps_hard_interlock.u8);
        create_bsmp_var(58, server, 4, false, g_ipc_ctom.ps_module[PS3_ID].ps_hard_interlock.u8);
        create_bsmp_var(59, server, 4, false, g_ipc_ctom.ps_module[PS4_ID].ps_hard_interlock.u8);

        create_bsmp_var(60, server, 4, false, PS1_LOAD_CURRENT.u8);
        create_bsmp_var(61, server, 4, false, PS2_LOAD_CURRENT.u8);
        create_bsmp_var(62, server, 4, false, PS3_LOAD_CURRENT.u8);
        create_bsmp_var(63, server, 4, false, PS4_LOAD_CURRENT.u8);
    }

}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fbp_system_config()
{
    uint8_t i;

    adcp_channel_config();
    bsmp_init_server();

    for(i = 0; i < NUM_MAX_PS_MODULES; i++)
    {
        init_wfmref(&WFMREF[i], WFMREF[0].wfmref_selected.u16,
                    WFMREF[0].sync_mode.enu,g_ipc_mtoc.control.freq_isr_control.f,
                    g_ipc_mtoc.control.freq_timeslicer[TIMESLICER_WFMREF].f,
                    WFMREF[0].gain.f, WFMREF[0].offset.f, &g_wfmref_data.data_fbp[i][0][0].f,
                    SIZE_WFMREF_FBP, &g_ipc_ctom.ps_module[i].ps_reference.f);

        init_buffer(&g_ipc_mtoc.buf_samples[i], &(g_buf_samples_ctom[i*1024].f), SIZE_BUF_SAMPLES_CTOM/4);
    }
}
