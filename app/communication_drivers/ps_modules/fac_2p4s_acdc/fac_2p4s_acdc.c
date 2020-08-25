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
 * @file fac_2p4s_acdc.c
 * @brief FAC-2P4S AC/DC Stage module
 *
 * Module for control of two AC/DC modules of FAC power supplies for dipoles
 * from booster. It implements the individual controllers for input current and
 * capacitor bank voltage of each AC/DC module.
 *
 * @author gabriel.brunheira
 * @date 21/07/2018
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
#include "communication_drivers/ps_modules/fac_2p4s_acdc/fac_2p4s_acdc.h"
#include "communication_drivers/ps_modules/ps_modules.h"

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
    Rectifier_Overvoltage,
    Rectifier_Undervoltage,
    Rectifier_Overcurrent,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    IGBT_Driver_Fault,
    IIB_1_Itlk,
    IIB_2_Itlk,
    IIB_3_Itlk,
    IIB_4_Itlk
} hard_interlocks_t;

static volatile iib_fac_is_t fac_is[2];
static volatile iib_fac_cmd_t fac_cmd[2];

static void init_iib();
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
    create_bsmp_var(31, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_soft_interlock.u8);
    create_bsmp_var(32, MOD_A_ID, 4, false, g_ipc_ctom.ps_module[MOD_A_ID].ps_hard_interlock.u8);
    create_bsmp_var(33, MOD_A_ID, 4, false, V_CAPBANK_MOD_A.u8);
    create_bsmp_var(34, MOD_A_ID, 4, false, VOUT_RECT_MOD_A.u8);
    create_bsmp_var(35, MOD_A_ID, 4, false, IOUT_RECT_MOD_A.u8);
    create_bsmp_var(36, MOD_A_ID, 4, false, TEMP_HEATSINK_MOD_A.u8);
    create_bsmp_var(37, MOD_A_ID, 4, false, TEMP_INDUCTORS_MOD_A.u8);
    create_bsmp_var(38, MOD_A_ID, 4, false, DUTY_CYCLE_MOD_A.u8);

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
    create_bsmp_var(34, MOD_B_ID, 4, false, VOUT_RECT_MOD_B.u8);
    create_bsmp_var(35, MOD_B_ID, 4, false, IOUT_RECT_MOD_B.u8);
    create_bsmp_var(36, MOD_B_ID, 4, false, TEMP_HEATSINK_MOD_B.u8);
    create_bsmp_var(37, MOD_B_ID, 4, false, TEMP_INDUCTORS_MOD_B.u8);
    create_bsmp_var(38, MOD_B_ID, 4, false, DUTY_CYCLE_MOD_B.u8);

    create_bsmp_var(39, MOD_A_ID, 4, false, fac_is[MOD_A_ID].Iin.u8);
    create_bsmp_var(40, MOD_A_ID, 4, false, fac_is[MOD_A_ID].Vin.u8);
    create_bsmp_var(41, MOD_A_ID, 4, false, fac_is[MOD_A_ID].TempL.u8);
    create_bsmp_var(42, MOD_A_ID, 4, false, fac_is[MOD_A_ID].TempHeatsink.u8);

    create_bsmp_var(39, MOD_B_ID, 4, false, fac_is[MOD_B_ID].Iin.u8);
    create_bsmp_var(40, MOD_B_ID, 4, false, fac_is[MOD_B_ID].Vin.u8);
    create_bsmp_var(41, MOD_B_ID, 4, false, fac_is[MOD_B_ID].TempL.u8);
    create_bsmp_var(42, MOD_B_ID, 4, false, fac_is[MOD_B_ID].TempHeatsink.u8);

    create_bsmp_var(43, MOD_A_ID, 4, false, fac_cmd[MOD_A_ID].Vout.u8);
    create_bsmp_var(44, MOD_A_ID, 4, false, fac_cmd[MOD_A_ID].VcapBank.u8);
    create_bsmp_var(45, MOD_A_ID, 4, false, fac_cmd[MOD_A_ID].TempRectInductor.u8);
    create_bsmp_var(46, MOD_A_ID, 4, false, fac_cmd[MOD_A_ID].TempRectHeatSink.u8);
    create_bsmp_var(47, MOD_A_ID, 4, false, fac_cmd[MOD_A_ID].GroundLeakage.u8);

    create_bsmp_var(43, MOD_B_ID, 4, false, fac_cmd[MOD_B_ID].Vout.u8);
    create_bsmp_var(44, MOD_B_ID, 4, false, fac_cmd[MOD_B_ID].VcapBank.u8);
    create_bsmp_var(45, MOD_B_ID, 4, false, fac_cmd[MOD_B_ID].TempRectInductor.u8);
    create_bsmp_var(46, MOD_B_ID, 4, false, fac_cmd[MOD_B_ID].TempRectHeatSink.u8);
    create_bsmp_var(47, MOD_B_ID, 4, false, fac_cmd[MOD_B_ID].GroundLeakage.u8);

    create_bsmp_var(48, MOD_A_ID, 4, false, IIB_ITLK_REG_FAC_IS_1.u8);
    create_bsmp_var(49, MOD_B_ID, 4, false, IIB_ITLK_REG_FAC_IS_2.u8);
    create_bsmp_var(48, MOD_A_ID, 4, false, IIB_ITLK_REG_FAC_CMD_1.u8);
    create_bsmp_var(49, MOD_B_ID, 4, false, IIB_ITLK_REG_FAC_CMD_2.u8);
}

/**
* @brief System configuration for FAC-2P4S AC/DC.
*
* Initialize specific parameters e configure peripherals for FAC-2P4S AC/DC
* operation.
*
*/
void fac_2p4s_acdc_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib();

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

static void init_iib()
{
    fac_is[0].CanAddress = 1;
    fac_is[1].CanAddress = 2;
    fac_cmd[0].CanAddress = 3;
    fac_cmd[1].CanAddress = 4;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(uint8_t *data)
{
    uint8_t iib_address;
    uint8_t data_id;

    convert_to_bytes_t converter;

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

    convert_to_bytes_t converter;

    switch(cmd_id) {
        case 0:
            converter.f = data_val;
            if (iib_id == 0) {
                IIB_ITLK_REG_FAC_IS_1.u32 = converter.u32;
                set_hard_interlock(0, IIB_1_Itlk);
            }
            if (iib_id == 1) {
                IIB_ITLK_REG_FAC_IS_2.u32 = converter.u32;
                set_hard_interlock(1, IIB_2_Itlk);
            }

            break;
        case 1:
            // TODO: Handle alarm message
            break;
        case 2:
            fac_is[iib_id].Iin.f = data_val;
            break;

        case 3:
            fac_is[iib_id].Vin.f = data_val;
            break;

        case 4:
            fac_is[iib_id].TempL.f = data_val;
            break;

        case 5:
            fac_is[iib_id].TempHeatsink.f = data_val;
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

    convert_to_bytes_t converter;

    if (iib_id == 2) mod_idx = 0;
    if (iib_id == 3) mod_idx = 1;

    switch(cmd_id) {
        case 0:
            converter.f = data_val;
            if (iib_id == 2) {
                IIB_ITLK_REG_FAC_CMD_1.u32 = converter.u32;
                set_hard_interlock(0, IIB_3_Itlk);
            }
            if (iib_id == 3) {
                IIB_ITLK_REG_FAC_CMD_2.u32 = converter.u32;
                set_hard_interlock(1, IIB_4_Itlk);
            }

            break;
        case 1:
            // TODO: Handle alarm data
            break;
        case 2:
            fac_cmd[mod_idx].Vout.f = data_val;
            break;

        case 3:
            fac_cmd[mod_idx].VcapBank.f = data_val;
            break;

        case 4:
            fac_cmd[mod_idx].TempRectInductor.f = data_val;
            break;

        case 5:
            fac_cmd[mod_idx].TempRectHeatSink.f = data_val;
            break;

        case 6:
            fac_cmd[mod_idx].GroundLeakage.f = data_val;
            break;

        default:
            break;
    }
}
