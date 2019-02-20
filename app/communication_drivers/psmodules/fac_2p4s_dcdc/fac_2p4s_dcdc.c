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
 * @file fac_dcdc.c
 * @brief FAC DC/DC Stage module
 *
 * Module for control of DC/DC module of FAC power supplies. It implements the
 * controller for load current.
 *
 * @author gabriel.brunheira
 * @date 01/05/2018
 *
 */


#include <communication_drivers/psmodules/fac_2p4s_dcdc/fac_2p4s_dcdc.h>
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

#define I_LOAD_1            g_controller_ctom.net_signals[0]
#define I_LOAD_2            g_controller_ctom.net_signals[1]
#define I_ARM_1             g_controller_ctom.net_signals[2]
#define I_ARM_2             g_controller_ctom.net_signals[3]

#define I_LOAD_MEAN         g_controller_ctom.net_signals[5]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[6]
#define DUTY_MEAN           g_controller_ctom.net_signals[7]

#define I_ARMS_DIFF         g_controller_ctom.net_signals[8]
#define DUTY_DIFF           g_controller_ctom.net_signals[9]

#define I_LOAD_DIFF         g_controller_ctom.net_signals[10]

#define I_LOAD_1            g_controller_ctom.net_signals[0]
#define I_LOAD_2            g_controller_ctom.net_signals[1]
#define I_LOAD_MEAN         g_controller_ctom.net_signals[2]
#define I_LOAD_DIFF         g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR        g_controller_ctom.net_signals[4]

#define V_LOAD              g_controller_mtoc.net_signals[0]

#define V_CAPBANK_MOD_1     g_controller_mtoc.net_signals[1]
#define V_CAPBANK_MOD_2     g_controller_mtoc.net_signals[2]
#define V_CAPBANK_MOD_3     g_controller_mtoc.net_signals[3]
#define V_CAPBANK_MOD_4     g_controller_mtoc.net_signals[4]
#define V_CAPBANK_MOD_5     g_controller_mtoc.net_signals[5]
#define V_CAPBANK_MOD_6     g_controller_mtoc.net_signals[6]
#define V_CAPBANK_MOD_7     g_controller_mtoc.net_signals[7]
#define V_CAPBANK_MOD_8     g_controller_mtoc.net_signals[8]

#define V_OUT_MOD_1         g_controller_mtoc.net_signals[9]
#define V_OUT_MOD_2         g_controller_mtoc.net_signals[10]
#define V_OUT_MOD_3         g_controller_mtoc.net_signals[11]
#define V_OUT_MOD_4         g_controller_mtoc.net_signals[12]
#define V_OUT_MOD_5         g_controller_mtoc.net_signals[13]
#define V_OUT_MOD_6         g_controller_mtoc.net_signals[14]
#define V_OUT_MOD_7         g_controller_mtoc.net_signals[15]
#define V_OUT_MOD_8         g_controller_mtoc.net_signals[16]

//#define TEMP_INDUCTORS      g_controller_mtoc.net_signals[10]
//#define TEMP_IGBT           g_controller_mtoc.net_signals[11]

#define DUTY_CYCLE_MOD_1    g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2    g_controller_ctom.output_signals[1]
#define DUTY_CYCLE_MOD_3    g_controller_ctom.output_signals[2]
#define DUTY_CYCLE_MOD_4    g_controller_ctom.output_signals[3]
#define DUTY_CYCLE_MOD_5    g_controller_ctom.output_signals[4]
#define DUTY_CYCLE_MOD_6    g_controller_ctom.output_signals[5]
#define DUTY_CYCLE_MOD_7    g_controller_ctom.output_signals[6]
#define DUTY_CYCLE_MOD_8    g_controller_ctom.output_signals[7]

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P4S DCDC operation. Note that module 3
* and module 4 don't follow the sequence of ADCP channels, in order to keep
* cables installation on EXB more intuitive.
*/
static void adcp_channel_config(void)
{
    // Module 1 DCLink Voltage: 10 V = 300 V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 300.0/2048.0;
    g_analog_ch_0.Value = &(V_CAPBANK_MOD_1.f);

    // Module 2 DCLink Voltage: 10 V = 300 V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 300.0/2048.0;
    g_analog_ch_1.Value = &(V_CAPBANK_MOD_2.f);

    // Module 3 DCLink Voltage: 10 V = 300 V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 300.0/2048.0;
    g_analog_ch_3.Value = &(V_CAPBANK_MOD_3.f);

    // Module 4 DCLink Voltage: 10 V = 300 V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 300.0/2048.0;
    g_analog_ch_2.Value = &(V_CAPBANK_MOD_4.f);

    // Module 5 DCLink Voltage: 10 V = 300 V
    g_analog_ch_4.Enable = 1;
    g_analog_ch_4.Gain = 300.0/2048.0;
    g_analog_ch_4.Value = &(V_CAPBANK_MOD_5.f);

    // Module 6 DCLink Voltage: 10 V = 300 V
    g_analog_ch_5.Enable = 1;
    g_analog_ch_5.Gain = 300.0/2048.0;
    g_analog_ch_5.Value = &(V_CAPBANK_MOD_6.f);

    // Module 7 DCLink Voltage: 10 V = 300 V
    g_analog_ch_6.Enable = 1;
    g_analog_ch_6.Gain = 300.0/2048.0;
    g_analog_ch_6.Value = &(V_CAPBANK_MOD_7.f);

    // Module 8 DCLink Voltage: 10 V = 300 V
    g_analog_ch_7.Enable = 1;
    g_analog_ch_7.Gain = 300.0/2048.0;
    g_analog_ch_7.Value = &(V_CAPBANK_MOD_8.f);
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

    create_bsmp_var(27, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(28, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(29, 0, 4, false, I_LOAD_2.u8);

    create_bsmp_var(30, 0, 4, false, I_ARM_1.u8);
    create_bsmp_var(31, 0, 4, false, I_ARM_2.u8);

    create_bsmp_var(32, 0, 4, false, V_LOAD.u8);

    create_bsmp_var(33, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(34, 0, 4, false, V_CAPBANK_MOD_2.u8);
    create_bsmp_var(35, 0, 4, false, V_CAPBANK_MOD_3.u8);
    create_bsmp_var(36, 0, 4, false, V_CAPBANK_MOD_4.u8);
    create_bsmp_var(37, 0, 4, false, V_CAPBANK_MOD_5.u8);
    create_bsmp_var(38, 0, 4, false, V_CAPBANK_MOD_6.u8);
    create_bsmp_var(39, 0, 4, false, V_CAPBANK_MOD_7.u8);
    create_bsmp_var(40, 0, 4, false, V_CAPBANK_MOD_8.u8);

    create_bsmp_var(41, 0, 4, false, V_OUT_MOD_1.u8);
    create_bsmp_var(42, 0, 4, false, V_OUT_MOD_2.u8);
    create_bsmp_var(43, 0, 4, false, V_OUT_MOD_3.u8);
    create_bsmp_var(44, 0, 4, false, V_OUT_MOD_4.u8);
    create_bsmp_var(45, 0, 4, false, V_OUT_MOD_5.u8);
    create_bsmp_var(46, 0, 4, false, V_OUT_MOD_6.u8);
    create_bsmp_var(47, 0, 4, false, V_OUT_MOD_7.u8);
    create_bsmp_var(48, 0, 4, false, V_OUT_MOD_8.u8);

    create_bsmp_var(49, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(50, 0, 4, false, DUTY_CYCLE_MOD_2.u8);
    create_bsmp_var(51, 0, 4, false, DUTY_CYCLE_MOD_3.u8);
    create_bsmp_var(52, 0, 4, false, DUTY_CYCLE_MOD_4.u8);
    create_bsmp_var(53, 0, 4, false, DUTY_CYCLE_MOD_5.u8);
    create_bsmp_var(54, 0, 4, false, DUTY_CYCLE_MOD_6.u8);
    create_bsmp_var(55, 0, 4, false, DUTY_CYCLE_MOD_7.u8);
    create_bsmp_var(56, 0, 4, false, DUTY_CYCLE_MOD_8.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2p4s_dcdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
}
