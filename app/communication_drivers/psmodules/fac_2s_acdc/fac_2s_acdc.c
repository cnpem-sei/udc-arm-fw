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
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"

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

#define IIB_ITLK_REG_FAC_IS_1       g_controller_mtoc.net_signals[6]
#define IIB_ITLK_REG_FAC_IS_2       g_controller_mtoc.net_signals[7]
#define IIB_ITLK_REG_FAC_CMD_1      g_controller_mtoc.net_signals[8]
#define IIB_ITLK_REG_FAC_CMD_2      g_controller_mtoc.net_signals[9]

#define DUTY_CYCLE_MOD_B            g_controller_ctom.output_signals[1]

/**
 * Interlocks defines
 */
typedef enum
{
    CapBank_Overvoltage,
    Rectifier_Overcurrent,
    AC_Mains_Contactor_Fault,
    IIB_Itlk
} hard_interlocks_t;

volatile iib_input_stage_t fac_2s_acdc_is[2];
volatile iib_command_drawer_t fac_2s_acdc_cmd[2];

static void init_iib_modules();
static void handle_can_data(uint8_t *data);
static void update_iib_structure_fac_is(uint8_t iib_id, uint8_t data_id, float data_val);
static void update_iib_structure_fac_cmd(uint8_t iib_id, uint8_t data_id, float data_val);

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

    create_bsmp_var(33, 0, 4, false, fac_2s_acdc_is[0].Iin.u8);
    create_bsmp_var(34, 0, 4, false, fac_2s_acdc_is[0].VdcLink.u8);
    create_bsmp_var(35, 0, 4, false, fac_2s_acdc_is[0].TempL.u8);
    create_bsmp_var(36, 0, 4, false, fac_2s_acdc_is[0].TempHeatsink.u8);

    create_bsmp_var(37, 0, 4, false, fac_2s_acdc_is[1].Iin.u8);
    create_bsmp_var(38, 0, 4, false, fac_2s_acdc_is[1].VdcLink.u8);
    create_bsmp_var(39, 0, 4, false, fac_2s_acdc_is[1].TempL.u8);
    create_bsmp_var(40, 0, 4, false, fac_2s_acdc_is[1].TempHeatsink.u8);

    create_bsmp_var(41, 0, 4, false, fac_2s_acdc_cmd[0].Vout.u8);
    create_bsmp_var(42, 0, 4, false, fac_2s_acdc_cmd[0].VcapBank.u8);
    create_bsmp_var(43, 0, 4, false, fac_2s_acdc_cmd[0].TempL.u8);
    create_bsmp_var(44, 0, 4, false, fac_2s_acdc_cmd[0].TempHeatSink.u8);

    create_bsmp_var(45, 0, 4, false, fac_2s_acdc_cmd[1].Vout.u8);
    create_bsmp_var(46, 0, 4, false, fac_2s_acdc_cmd[1].VcapBank.u8);
    create_bsmp_var(47, 0, 4, false, fac_2s_acdc_cmd[1].TempL.u8);
    create_bsmp_var(48, 0, 4, false, fac_2s_acdc_cmd[1].TempHeatSink.u8);


    create_bsmp_var(49, 0, 4, false, IIB_ITLK_REG_FAC_IS_1.u8);
    create_bsmp_var(50, 0, 4, false, IIB_ITLK_REG_FAC_IS_2.u8);
    create_bsmp_var(51, 0, 4, false, IIB_ITLK_REG_FAC_CMD_1.u8);
    create_bsmp_var(52, 0, 4, false, IIB_ITLK_REG_FAC_CMD_2.u8);
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
    init_iib_modules();
}

static void init_iib_modules()
{
    fac_2s_acdc_is[0].CanAddress = 1;
    fac_2s_acdc_is[1].CanAddress = 2;
    fac_2s_acdc_cmd[0].CanAddress = 3;
    fac_2s_acdc_cmd[0].CanAddress = 4;

    init_iib_module(&g_iib_module, &handle_can_data);
}

static void handle_can_data(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    float_to_bytes_t converter;

    iib_address     = data[0];
    data_id         = data[1];

    converter.u8[0] = data[4];
    converter.u8[1] = data[5];
    converter.u8[2] = data[6];
    converter.u8[3] = data[7];

    if ((iib_address == 1) || (iib_address == 2)) {
        update_iib_structure_fac_is(iib_address - 1, data_id, converter.f);
    }

    if ((iib_address == 3) || (iib_address == 4)) {
        update_iib_structure_fac_cmd(iib_address - 1, data_id, converter.f);
    }
}

static void update_iib_structure_fac_is(uint8_t iib_id, uint8_t data_id, float data_val)
{
    uint8_t cmd_id;
    cmd_id = data_id;

    float_to_bytes_t converter;

    switch(cmd_id) {
        case 0:
            converter.f = data_val;
            if (iib_id == 0) IIB_ITLK_REG_FAC_IS_1.u32 = converter.u32;
            if (iib_id == 1) IIB_ITLK_REG_FAC_IS_2.u32 = converter.u32;
            set_hard_interlock(iib_id, IIB_Itlk);
            break;
        case 1:
            // TODO: Handle alarm message
            break;
        case 2:
            fac_2s_acdc_is[iib_id].Iin.f = data_val;
            break;

        case 3:
            fac_2s_acdc_is[iib_id].VdcLink.f = data_val;
            break;

        case 4:
            fac_2s_acdc_is[iib_id].TempL.f = data_val;
            break;

        case 5:
            fac_2s_acdc_is[iib_id].TempHeatsink.f = data_val;
            break;

        default:
            break;
    }
}

static void update_iib_structure_fac_cmd(uint8_t iib_id, uint8_t data_id, float data_val)
{
    uint8_t cmd_id;
    uint8_t mod_idx;
    cmd_id = data_id;

    float_to_bytes_t converter;

    if (iib_id == 2) mod_idx = 0;
    if (iib_id == 3) mod_idx = 1;

    switch(cmd_id) {
        case 0:
            converter.f = data_val;
            if (iib_id == 2) IIB_ITLK_REG_FAC_CMD_1.u32 = converter.u32;
            if (iib_id == 3) IIB_ITLK_REG_FAC_CMD_2.u32 = converter.u32;
            set_hard_interlock(iib_id, IIB_Itlk);
            break;
        case 1:
            // TODO: Handle alarm data
            break;
        case 2:
            fac_2s_acdc_cmd[mod_idx].Vout.f = data_val;
            break;

        case 3:
            fac_2s_acdc_cmd[mod_idx].VcapBank.f = data_val;
            break;

        case 4:
            fac_2s_acdc_cmd[mod_idx].TempL.f = data_val;
            break;

        case 5:
            fac_2s_acdc_cmd[mod_idx].TempHeatSink.f = data_val;
            break;

        default:
            break;
    }
}

