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
 * @file fbp_2s_ufjf.c
 * @brief Module of FPB-2S for UFJF
 *
 * Module for control of two FBP modules operating in series, used by partners
 * from UFJF to study new control strategies for Sirius.
 *
 * @author gabriel.brunheira
 * @date 01/02/2019
 *
 */

#include <communication_drivers/psmodules/fbp_2s_ufjf/fbp_2s_ufjf.h>
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
#define I_LOAD_K_1          g_controller_ctom.net_signals[1]    //-----
#define V_C_MOD1_K_1        g_controller_ctom.net_signals[2]    //  |
#define V_C_MOD3_K_1        g_controller_ctom.net_signals[3]    //  |
#define I_L_MOD1_K_1        g_controller_ctom.net_signals[4]    //  |
#define V_D_MOD1_K_1        g_controller_ctom.net_signals[5]    //  |
#define I_L_MOD3_K_1        g_controller_ctom.net_signals[6]    // STATE_OBSERVER_IN
#define V_D_MOD3_K_1        g_controller_ctom.net_signals[7]    //  |
#define M_MOD1_K_1          g_controller_ctom.net_signals[8]    //  |     -----
#define M_MOD3_K_1          g_controller_ctom.net_signals[9]    //  |       |
#define I_LOAD_K            g_controller_ctom.net_signals[10]   //  |       |
#define V_C_MOD1_K          g_controller_ctom.net_signals[11]   //  |       |
#define V_C_MOD3_K          g_controller_ctom.net_signals[12]   //-----     |
#define I_L_MOD1_K          g_controller_ctom.net_signals[13]   //          |
#define I_L_MOD3_K          g_controller_ctom.net_signals[14]   //  STATE_CONTROLLER_IN
#define V_D_MOD1_K          g_controller_ctom.net_signals[15]   //          |
#define V_D_MOD3_K          g_controller_ctom.net_signals[16]   //          |
#define REF_K               g_controller_ctom.net_signals[17]   //          |
#define Q_K_1               g_controller_ctom.net_signals[18]   //        -----

#define DUTY_CYCLE_MOD_1        g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_3        g_controller_ctom.output_signals[1]

/**
 * Auxiliary variables defines
 */
#define Q_K                 g_controller_ctom.net_signals[19]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[20]


#define MOD1_DCLINK_VOLTAGE     g_controller_mtoc.net_signals[0]    // ANI2
#define MOD1_LOAD_VOLTAGE       g_controller_mtoc.net_signals[4]    // ANI6
#define MOD1_TEMPERATURE        g_controller_mtoc.net_signals[8]    // I2C Add 0x48

#define MOD2_DCLINK_VOLTAGE     g_controller_mtoc.net_signals[1]    // ANI1
#define MOD2_LOAD_VOLTAGE       g_controller_mtoc.net_signals[5]    // ANI7
#define MOD2_TEMPERATURE        g_controller_mtoc.net_signals[9]    // I2C Add 0x49

#define MOD3_DCLINK_VOLTAGE     g_controller_mtoc.net_signals[2]    // ANI4
#define MOD3_LOAD_VOLTAGE       g_controller_mtoc.net_signals[6]    // ANI3
#define MOD3_TEMPERATURE        g_controller_mtoc.net_signals[10]   // I2C Add 0x4A

#define MOD4_DCLINK_VOLTAGE     g_controller_mtoc.net_signals[3]    // ANI0
#define MOD4_LOAD_VOLTAGE       g_controller_mtoc.net_signals[7]    // ANI5
#define MOD4_TEMPERATURE        g_controller_mtoc.net_signals[11]   // I2C Add 0x4C

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    DCLink_Relay_Fault,
    DCLink_Fuse_Fault,
    MOSFETs_Driver_Fault
} hard_interlocks_t;

typedef enum
{
    Heatsink_Overtemperature
} soft_interlocks_t;

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC ACDC operation.
*
*/
static void adcp_channel_config(void)
{
    // MOD1 VdcLink: 10V = 20V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 20.0/2048.0;
    g_analog_ch_2.Value = &(MOD1_DCLINK_VOLTAGE.f);

    // MOD2 VdcLink: 10V = 20V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 20.0/2048.0;
    g_analog_ch_1.Value = &(MOD2_DCLINK_VOLTAGE.f);

    // MOD3 VdcLink: 10V = 20V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 20.0/2048.0;
    g_analog_ch_4.Value = &(MOD3_DCLINK_VOLTAGE.f);

    // MOD4 VdcLink: 10V = 20V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 20.0/2048.0;
    g_analog_ch_0.Value = &(MOD4_DCLINK_VOLTAGE.f);

    // MOD1 Vload: 10V = 50V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 50.0/2048.0;
    g_analog_ch_6.Value = &(MOD1_LOAD_VOLTAGE.f);

    // MOD2 Vload: 10V = 50V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 50.0/2048.0;
    g_analog_ch_7.Value = &(MOD2_LOAD_VOLTAGE.f);

    // MOD3 Vload: 10V = 50V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 50.0/2048.0;
    g_analog_ch_3.Value = &(MOD3_LOAD_VOLTAGE.f);

    // MOD4 Vload: 10V = 50V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 50.0/2048.0;
    g_analog_ch_5.Value = &(MOD4_LOAD_VOLTAGE.f);
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

    create_bsmp_var(27, 0, 4, false, I_LOAD_K.u8);
    create_bsmp_var(28, 0, 4, false, I_LOAD_ERROR.u8);

    create_bsmp_var(29, 0, 4, false, MOD1_DCLINK_VOLTAGE.u8);
    create_bsmp_var(30, 0, 4, false, V_C_MOD1_K.u8);
    create_bsmp_var(31, 0, 4, false, MOD1_TEMPERATURE.u8);
    create_bsmp_var(32, 0, 4, false, DUTY_CYCLE_MOD_1.u8);

    create_bsmp_var(33, 0, 4, false, MOD3_DCLINK_VOLTAGE.u8);
    create_bsmp_var(34, 0, 4, false, V_C_MOD3_K.u8);
    create_bsmp_var(35, 0, 4, false, MOD3_TEMPERATURE.u8);
    create_bsmp_var(36, 0, 4, false, DUTY_CYCLE_MOD_3.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fbp_2s_ufjf_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
