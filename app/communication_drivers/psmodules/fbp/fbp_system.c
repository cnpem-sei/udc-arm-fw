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

#include <communication_drivers/psmodules/fbp/fbp_system.h>
#include <communication_drivers/psmodules/ps_modules.h>
#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/control/control.h"

#define APPLICATION         UVX_LINAC_RACK1

#define UVX_LINAC_RACK1     0
#define UVX_LINAC_RACK2     1
#define WEG_PILOT_BATCH_CHARACTERIZATION    2
#define ELP_FAC_CON_TESTS   3

/**
 * UVX Linac FBP Rack 1:
 *      LCH01A / LCV01A / LCH01B / LCV01B
 */
#if APPLICATION == UVX_LINAC_RACK1

    #define SIGGEN_TYPE         Sine
    #define SIGGEN_NUM_CYCLES   1
    #define SIGGEN_FREQ         1.0
    #define SIGGEN_AMP          0.0
    #define SIGGEN_OFFSET       0.0
    #define SIGGEN_AUX_PARAM_0  0.0
    #define SIGGEN_AUX_PARAM_1  0.0
    #define SIGGEN_AUX_PARAM_2  0.0
    #define SIGGEN_AUX_PARAM_3  0.0

/**
 * UVX Linac FBP Rack 2:
 *      LCH03 / LCV03 / LCH04 / LCV04
 */
#elif APPLICATION == UVX_LINAC_RACK2

    #define SIGGEN_TYPE         DampedSine
    #define SIGGEN_NUM_CYCLES   15
    #define SIGGEN_FREQ         0.25
    #define SIGGEN_AMP          10.0
    #define SIGGEN_OFFSET       0.0
    #define SIGGEN_AUX_PARAM_0  0.0
    #define SIGGEN_AUX_PARAM_1  0.0
    #define SIGGEN_AUX_PARAM_2  10.0
    #define SIGGEN_AUX_PARAM_3  0.0

/**
 *  WEG pilot batch characterization
 *
 *  Lload = 3.4 mH
 *  Rload = 0.5 Ohm
 *  Vdc-link = 7 V
 *  fbw = 1.33 kHz
 */
#elif APPLICATION == WEG_PILOT_BATCH_CHARACTERIZATION

    #define SIGGEN_TYPE         DampedSine
    #define SIGGEN_NUM_CYCLES   15
    #define SIGGEN_FREQ         0.25
    #define SIGGEN_AMP          10.0
    #define SIGGEN_OFFSET       0.0
    #define SIGGEN_AUX_PARAM_0  0.0
    #define SIGGEN_AUX_PARAM_1  0.0
    #define SIGGEN_AUX_PARAM_2  10.0
    #define SIGGEN_AUX_PARAM_3  0.0

/**
 *  Synchronization tests with ELP, CON and FAC groups
 *
 *  fbw = 1 kHz
 */
#elif APPLICATION == ELP_FAC_CON_TESTS

    #define SIGGEN_TYPE         Trapezoidal
    #define SIGGEN_NUM_CYCLES   5
    #define SIGGEN_FREQ         0.0
    #define SIGGEN_AMP          1.0
    #define SIGGEN_OFFSET       0.0
    #define SIGGEN_AUX_PARAM_0  0.28
    #define SIGGEN_AUX_PARAM_1  0.02
    #define SIGGEN_AUX_PARAM_2  0.2
    #define SIGGEN_AUX_PARAM_3  0.0

#endif


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

/**
 * Number of power supplies
 */
//static const uint8_t fbp_qtd = 4;

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
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fbp_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    //ipc_init_parameters();
}
